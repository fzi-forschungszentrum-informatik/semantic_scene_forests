// Copyright (c) 2026 FZI Forschungszentrum Informatik
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "object_attribution_worker.hpp"

#include <exception>
#include <functional>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <variant>

#include <cv_bridge/cv_bridge.hpp>
#include <minja/minja.hpp>
#include <oai_api/builder.hpp>
#include <oai_api/types.hpp>
#include <opencv2/core.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "generation_common.hpp"
#include "semantic_scene_forests_generation/object_attribution_parameters.hpp"

namespace semantic_scene_forests_generation {

ObjectAttributionWorker::~ObjectAttributionWorker()
{
  stop();
}

void ObjectAttributionWorker::initialize(
  rclcpp::Node& node,
  std::shared_ptr<fzi::semantic_scene_forests::SceneGraph> graph,
  const std::string& name,
  diagnostic_updater::Updater& diagnostics)
{
  m_graph  = std::move(graph);
  m_name   = name;
  m_logger = node.get_logger().get_child(name);
  const auto PARAMS =
    object_attribution::ParamListener(node.get_node_parameters_interface(), m_logger, name + ".")
      .get_params();
  m_poll_interval = std::chrono::duration<double>(PARAMS.poll_interval_seconds);

  m_templates   = loadTemplates(PARAMS.prompt_dir, m_logger);
  m_task_loader = std::make_unique<pluginlib::ClassLoader<IObjectTask>>(
    "semantic_scene_forests_generation", "semantic_scene_forests_generation::IObjectTask");
  m_tasks  = loadTasks(*m_task_loader, PARAMS.tasks, node, m_templates);
  m_client = std::make_unique<oai::OpenAIClient>(
    makeClient({.server_url              = PARAMS.server_url,
                .api_key                 = PARAMS.api_key,
                .extra_body              = PARAMS.extra_body,
                .model                   = PARAMS.model,
                .connect_timeout_seconds = PARAMS.llm_connect_timeout_seconds,
                .request_timeout_seconds = PARAMS.llm_request_timeout_seconds}));

  m_action_server =
    m_threads.serve(node,
                    "attribute_objects",
                    m_logger,
                    "Object attribution",
                    [this](const std::shared_ptr<WorkerThreads::GoalHandle>& goal_handle,
                           const std::function<bool()>& should_stop) {
                      return executeGoal(goal_handle, should_stop);
                    });
  diagnostics.add(name + " health", [this](diagnostic_updater::DiagnosticStatusWrapper& stat) {
    m_stats.report(stat);
    stat.add("Pending objects", pendingObjects().size());
    stat.add("Goal active", m_threads.goalActive());
  });
  RCLCPP_INFO(
    m_logger, "Object attribution with %zu tasks (%s)", m_tasks.size(), PARAMS.model.c_str());
}

void ObjectAttributionWorker::start()
{
  m_threads.startBackground([this](const std::stop_token& stop) { runBackground(stop); });
}

void ObjectAttributionWorker::stop()
{
  m_threads.stop();
}

void ObjectAttributionWorker::runBackground(const std::stop_token& stop)
{
  const auto SHOULD_STOP = [&stop] { return stop.stop_requested(); };
  while (m_threads.waitForTick(stop, m_poll_interval))
  {
    try
    {
      for (const int64_t ID : pendingObjects())
      {
        if (m_threads.goalActive() || describeObject(ID, SHOULD_STOP) == Outcome::STOPPED)
        {
          break;
        }
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(m_logger, "Background attribution failed: %s", e.what());
    }
  }
}

std::optional<std::string>
ObjectAttributionWorker::executeGoal(const std::shared_ptr<WorkerThreads::GoalHandle>& goal_handle,
                                     const std::function<bool()>& should_stop)
{
  const auto OBJECTS = m_graph->getObjectNodes();
  auto feedback      = std::make_shared<WorkerThreads::RunWorker::Feedback>();
  feedback->total    = static_cast<uint32_t>(OBJECTS.size());
  uint32_t described = 0;
  for (const auto& object : OBJECTS)
  {
    if (should_stop())
    {
      return std::nullopt;
    }
    feedback->status =
      "Describing object " + std::to_string(object.id) + " (" + object.class_name + ")";
    goal_handle->publish_feedback(feedback);
    const auto OUTCOME = describeObject(object.id, should_stop);
    if (OUTCOME == Outcome::STOPPED)
    {
      return std::nullopt;
    }
    described += OUTCOME == Outcome::DESCRIBED ? 1 : 0;
    ++feedback->current;
  }
  return "Described " + std::to_string(described) + " of " + std::to_string(OBJECTS.size()) +
         " objects";
}

std::vector<int64_t> ObjectAttributionWorker::pendingObjects() const
{
  // unprocessedBy also lists concepts: keep the object ids.
  std::unordered_set<int64_t> object_ids;
  for (const auto& object : m_graph->getObjectNodes())
  {
    object_ids.insert(object.id);
  }
  auto ids = m_graph->unprocessedBy(m_name);
  std::erase_if(ids, [&object_ids](int64_t id) { return !object_ids.contains(id); });
  return ids;
}

ObjectAttributionWorker::Outcome
ObjectAttributionWorker::describeObject(int64_t id, const std::function<bool()>& should_stop)
{
  const std::scoped_lock LOCK{m_describe_mutex};
  Outcome outcome = Outcome::FAILED;
  try
  {
    outcome = runTasks(id, should_stop);
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(m_logger, "Describing object %ld failed: %s", id, e.what());
    m_stats.failed("object " + std::to_string(id) + ": " + e.what());
  }
  catch (...)
  {
    // A pluginlib-loaded task can throw a type that is not a std::exception.
    RCLCPP_WARN(m_logger, "Describing object %ld failed: unknown exception", id);
    m_stats.failed("object " + std::to_string(id) + ": unknown exception");
  }
  if (outcome == Outcome::STOPPED || should_stop())
  {
    return Outcome::STOPPED;
  }
  if (outcome == Outcome::SKIPPED)
  {
    RCLCPP_INFO(m_logger, "Object %ld was deleted, skipped", id);
    return outcome;
  }
  // Marked even on failure so an LLM outage cannot loop on paid calls; attribute_objects retries.
  if (const auto marked = m_graph->markProcessed(id, m_name); !marked)
  {
    RCLCPP_WARN(m_logger, "Marking object %ld failed: %s", id, marked.error().message.c_str());
  }
  if (outcome == Outcome::DESCRIBED)
  {
    RCLCPP_INFO(m_logger, "Described object %ld", id);
    m_stats.succeeded();
  }
  return outcome;
}

ObjectAttributionWorker::Outcome
ObjectAttributionWorker::runTasks(int64_t id, const std::function<bool()>& should_stop)
{
  const auto NODE = m_graph->findNode(id);
  const auto* record =
    NODE ? std::get_if<fzi::semantic_scene_forests::ObjectNode>(&*NODE) : nullptr;
  if (record == nullptr)
  {
    return Outcome::SKIPPED;
  }
  auto& client = *m_client;
  std::vector<cv::Mat> crops;
  crops.reserve(record->crops.size());
  for (const auto& image : record->crops)
  {
    try
    {
      // The node stores JPEG; the LLM builder takes decoded bgr8 images.
      crops.push_back(cv_bridge::toCvCopy(*image, "bgr8")->image);
    }
    catch (const cv_bridge::Exception& e)
    {
      RCLCPP_WARN(m_logger, "cv_bridge failed for a crop of object %ld: %s", id, e.what());
    }
  }

  const auto SYSTEM_TEMPLATE = m_templates.find("instance/instance_system_prompt");
  const auto SETUP_TEMPLATE  = m_templates.find("instance/instance_object_setup");
  if (SYSTEM_TEMPLATE == m_templates.end() || SETUP_TEMPLATE == m_templates.end())
  {
    throw std::runtime_error("required prompt template is missing");
  }

  std::vector<oai::Message> history;
  auto system_message =
    oai::MessageBuilder::system()
      .text(SYSTEM_TEMPLATE->second->render(minja::Context::make(minja::Value::object())))
      .build();
  if (!system_message)
  {
    throw std::runtime_error("failed to build the system message: " + system_message.error());
  }
  history.push_back(std::move(*system_message));

  // Object setup message: text + all image crops.
  auto setup_ctx = minja::Value::object();
  setup_ctx.set("id", minja::Value(id));
  setup_ctx.set("class_name", minja::Value(record->class_name));
  const std::string SETUP_TEXT =
    SETUP_TEMPLATE->second->render(minja::Context::make(std::move(setup_ctx)));
  auto builder = oai::MessageBuilder::user().text(SETUP_TEXT);
  for (const auto& crop : crops)
  {
    builder.image(crop);
  }
  auto setup_message = builder.build();
  if (!setup_message)
  {
    RCLCPP_WARN(
      m_logger, "Image encoding failed for object %ld: %s", id, setup_message.error().c_str());
    setup_message = oai::MessageBuilder::user().text(SETUP_TEXT).build();
    if (!setup_message)
    {
      throw std::runtime_error("failed to build the object setup message: " +
                               setup_message.error());
    }
  }
  history.push_back(std::move(*setup_message));

  if (should_stop())
  {
    return Outcome::STOPPED;
  }
  auto ack = client.chat({.messages = history});
  if (!ack)
  {
    throw std::runtime_error("setup request failed: " + ack.error().message);
  }
  auto acknowledgement = oai::MessageBuilder::assistant().text(ack->text()).build();
  if (!acknowledgement)
  {
    throw std::runtime_error("failed to build the setup acknowledgement: " +
                             acknowledgement.error());
  }
  history.push_back(std::move(*acknowledgement));

  for (auto& task : m_tasks)
  {
    if (should_stop())
    {
      return Outcome::STOPPED;
    }
    if (auto task_result = task->execute(client, history, id, crops, m_graph); !task_result)
    {
      throw std::runtime_error("task failed: " + task_result.error());
    }
  }
  return Outcome::DESCRIBED;
}

} // namespace semantic_scene_forests_generation

PLUGINLIB_EXPORT_CLASS(semantic_scene_forests_generation::ObjectAttributionWorker,
                       semantic_scene_forests_manager::IWorker)
