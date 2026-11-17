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
#include "concept_generation_worker.hpp"

#include <exception>
#include <functional>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <variant>

#include <minja/minja.hpp>
#include <oai_api/builder.hpp>
#include <oai_api/types.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <semantic_scene_forests/scene_graph.hpp>

#include "generation_common.hpp"
#include "semantic_scene_forests_generation/concept_generation_parameters.hpp"

namespace semantic_scene_forests_generation {

using fzi::semantic_scene_forests::ConceptNode;

ConceptGenerationWorker::~ConceptGenerationWorker()
{
  stop();
}

void ConceptGenerationWorker::initialize(
  rclcpp::Node& node,
  std::shared_ptr<fzi::semantic_scene_forests::SceneGraph> graph,
  const std::string& name,
  diagnostic_updater::Updater& diagnostics)
{
  m_graph  = std::move(graph);
  m_name   = name;
  m_logger = node.get_logger().get_child(name);
  const auto PARAMS =
    concept_generation::ParamListener(node.get_node_parameters_interface(), m_logger, name + ".")
      .get_params();
  m_interval = std::chrono::seconds(PARAMS.interval_seconds);

  m_templates   = loadTemplates(PARAMS.prompt_dir, m_logger);
  m_task_loader = std::make_unique<pluginlib::ClassLoader<IGraphTask>>(
    "semantic_scene_forests_generation", "semantic_scene_forests_generation::IGraphTask");
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
                    "generate_concepts",
                    m_logger,
                    "Concept generation",
                    [this](const std::shared_ptr<WorkerThreads::GoalHandle>& /*goal_handle*/,
                           const std::function<bool()>& should_stop) -> std::optional<std::string> {
                      Outcome outcome{};
                      try
                      {
                        outcome = generate(false, should_stop);
                      }
                      catch (const std::exception& e)
                      {
                        m_stats.failed(e.what());
                        throw;
                      }
                      switch (outcome)
                      {
                        case Outcome::GENERATED:
                          return "Concept generation complete";
                        case Outcome::SKIPPED:
                          return "Skipped: no concepts are available";
                        case Outcome::STOPPED:
                          break;
                      }
                      return std::nullopt;
                    });
  diagnostics.add(name + " health", [this](diagnostic_updater::DiagnosticStatusWrapper& stat) {
    m_stats.report(stat);
    stat.add("Pending concepts", planInput(true).size());
    stat.add("Goal active", m_threads.goalActive());
  });
  RCLCPP_INFO(m_logger,
              "Concept generation with %zu tasks every %llds (%s)",
              m_tasks.size(),
              static_cast<long long>(m_interval.count()),
              PARAMS.model.c_str());
}

void ConceptGenerationWorker::start()
{
  m_threads.startBackground([this](const std::stop_token& stop) { runBackground(stop); });
}

void ConceptGenerationWorker::stop()
{
  m_threads.stop();
}

void ConceptGenerationWorker::runBackground(const std::stop_token& stop)
{
  const auto SHOULD_STOP = [&stop] { return stop.stop_requested(); };
  while (m_threads.waitForTick(stop, m_interval))
  {
    // A failed run keeps its concepts unprocessed for the next tick.
    try
    {
      generate(true, SHOULD_STOP);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(m_logger, "Background concept generation failed: %s", e.what());
      m_stats.failed(e.what());
    }
    catch (...)
    {
      // A pluginlib-loaded task can throw a type that is not a std::exception.
      RCLCPP_ERROR(m_logger, "Background concept generation failed: unknown exception");
      m_stats.failed("unknown exception");
    }
  }
}

ConceptGenerationWorker::Outcome
ConceptGenerationWorker::generate(bool only_unprocessed, const std::function<bool()>& should_stop)
{
  const std::scoped_lock LOCK{m_generate_mutex};
  if (should_stop())
  {
    return Outcome::STOPPED;
  }
  const auto INPUT_IDS = planInput(only_unprocessed);
  if (INPUT_IDS.empty())
  {
    return Outcome::SKIPPED;
  }
  // Concepts that exist before the run; the ones it creates count as processed afterwards.
  std::unordered_set<int64_t> existing_ids;
  for (const auto& concept_node : m_graph->getConceptNodes())
  {
    existing_ids.insert(concept_node.id);
  }
  // The input as the tasks see it: current names, concepts deleted meanwhile dropped.
  const auto read_input = [&] {
    std::vector<ConceptNode> input;
    for (const int64_t ID : INPUT_IDS)
    {
      if (const auto NODE = m_graph->findNode(ID); NODE)
      {
        if (const auto* concept_node = std::get_if<ConceptNode>(&*NODE))
        {
          input.push_back(*concept_node);
        }
      }
    }
    return input;
  };
  const auto CONCEPTS = read_input();

  const auto SYSTEM_TEMPLATE = m_templates.find("meta/meta_system_prompt");
  const auto SETUP_TEMPLATE  = m_templates.find("meta/meta_setup_prompt");
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

  auto setup_ctx    = minja::Value::object();
  auto objects_list = minja::Value::array();
  for (const auto& concept_node : CONCEPTS)
  {
    objects_list.push_back(minja::Value(concept_node.concept_name));
  }
  setup_ctx.set("input_objects", objects_list);
  auto setup_message =
    oai::MessageBuilder::user()
      .text(SETUP_TEMPLATE->second->render(minja::Context::make(std::move(setup_ctx))))
      .build();
  if (!setup_message)
  {
    throw std::runtime_error("failed to build the setup message: " + setup_message.error());
  }
  history.push_back(std::move(*setup_message));

  auto& client = *m_client;
  auto ack     = client.chat({.messages = history});
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
    if (auto task_result = task->execute(client, history, read_input(), m_graph); !task_result)
    {
      throw std::runtime_error("task failed: " + task_result.error());
    }
  }
  if (should_stop())
  {
    return Outcome::STOPPED;
  }

  markRunResults(*m_graph, m_name, INPUT_IDS, existing_ids);
  RCLCPP_INFO(m_logger, "Processed %zu concepts", CONCEPTS.size());
  m_stats.succeeded();
  return Outcome::GENERATED;
}

std::vector<int64_t> ConceptGenerationWorker::planInput(bool only_unprocessed) const
{
  std::vector<int64_t> ids;
  if (only_unprocessed)
  {
    // unprocessedBy also lists objects: keep the concept ids.
    std::unordered_set<int64_t> concept_ids;
    for (const auto& concept_node : m_graph->getConceptNodes())
    {
      concept_ids.insert(concept_node.id);
    }
    ids = m_graph->unprocessedBy(m_name);
    std::erase_if(ids, [&concept_ids](int64_t id) { return !concept_ids.contains(id); });
  }
  else
  {
    for (const auto& concept_node : m_graph->getConceptNodes())
    {
      ids.push_back(concept_node.id);
    }
  }
  return ids;
}

} // namespace semantic_scene_forests_generation

PLUGINLIB_EXPORT_CLASS(semantic_scene_forests_generation::ConceptGenerationWorker,
                       semantic_scene_forests_manager::IWorker)
