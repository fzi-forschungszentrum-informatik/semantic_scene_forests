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

#include "semantic_scene_forests_generation_node.hpp"
#include "tasks/task_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include <oai_api/builder.hpp>
#include <oai_api/types.hpp>

#include <minja/minja.hpp>

#include <nlohmann/json.hpp>

#include <semantic_scene_forests_msgs/msg/scene_graph.hpp>
#include <utility>

namespace semantic_scene_forests_generation {

using json   = nlohmann::json;
namespace fs = std::filesystem;

// ── Constructor ─────────────────────────────────────────────────────────────

SceneGraphGenerationNode::SceneGraphGenerationNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("semantic_scene_forests_generation", options)
  , m_diagnostic_updater(this)
{
  m_param_listener = std::make_shared<ParamListener>(get_node_parameters_interface());
  m_params         = m_param_listener->get_params();

  m_diagnostic_updater.setHardwareID("semantic_scene_forests_generation");
  m_diagnostic_updater.add(
    "Connectivity", this, &SceneGraphGenerationNode::onConnectivityDiagnostic);

  // Resolve prompt directory
  fs::path prompt_dir;
  if (!m_params.prompt_dir.empty())
  {
    prompt_dir = m_params.prompt_dir;
  }
  else
  {
    prompt_dir =
      fs::path(ament_index_cpp::get_package_share_directory("semantic_scene_forests_generation")) /
      "resources";
  }
  loadTemplates(prompt_dir.string());
  loadPlugins();

  // Create LLM clients
  const auto LLM_CONNECT_TIMEOUT =
    std::chrono::seconds{std::max<int64_t>(1, m_params.llm_connect_timeout_seconds)};
  const auto LLM_REQUEST_TIMEOUT =
    std::chrono::seconds{std::max<int64_t>(1, m_params.llm_request_timeout_seconds)};
  // oai-api silently ignores a non-object extra body, so reject typos at startup.
  nlohmann::json extra_body;
  if (!m_params.extra_body.empty())
  {
    extra_body = nlohmann::json::parse(m_params.extra_body, nullptr, false);
    if (!extra_body.is_object())
    {
      throw std::invalid_argument(
        "extra_body must be a JSON object (double quotes, lowercase true/false): " +
        m_params.extra_body);
    }
  }
  const auto make_client = [&](const std::string& model) {
    return oai::ClientBuilder{}
      .baseUrl(m_params.server_url)
      .apiKey(m_params.api_key)
      .model(model)
      // OpenRouter app attribution: the referer URL is the app identity, the title its name.
      .header("HTTP-Referer",
              "https://github.com/fzi-forschungszentrum-informatik/semantic_scene_forests/tree/"
              "main/semantic_scene_forests_generation")
      .header("X-OpenRouter-Title", "semantic_scene_forests_generation")
      .extraBody(extra_body)
      .connectTimeout(LLM_CONNECT_TIMEOUT)
      .requestTimeout(LLM_REQUEST_TIMEOUT)
      .build();
  };
  m_phase1_client.emplace(make_client(m_params.phase1_model));
  m_phase2_client.emplace(make_client(m_params.phase2_model));

  // QoS matching the Rust implementation
  auto qos = rclcpp::QoS(1).reliable().transient_local();

  // NOLINTNEXTLINE(performance-unnecessary-value-param): rclcpp callback requires SharedPtr by
  // value
  m_objects_sub = create_subscription<ManagedObjectArray>(
    "/managed_objects", qos, [this](const ManagedObjectArray::SharedPtr msg) {
      onManagedObjects(msg);
    });

  m_objects_freq = std::make_unique<diagnostic_updater::HeaderlessTopicDiagnostic>(
    "/managed_objects",
    m_diagnostic_updater,
    diagnostic_updater::FrequencyStatusParam(&m_min_freq, &m_max_freq, 0.1, 5));

  m_scene_graph_pub = create_publisher<SceneGraphMsg>("/scene_graph", qos);

  m_visuals_client = create_client<GetObjectVisuals>("/get_object_visuals");

  m_context_service =
    create_service<AddSceneContext>("/add_scene_context",
                                    [this](const AddSceneContext::Request::SharedPtr& request,
                                           const AddSceneContext::Response::SharedPtr& response) {
                                      onAddSceneContext(*request, *response);
                                    });

  m_action_server = rclcpp_action::create_server<UpdateSceneGraphAction>(
    this,
    "/update_scene_graph",
    [this](const auto& uuid, const auto& goal) { return handleGoal(uuid, goal); },
    [](const auto& goal_handle) { return handleCancel(goal_handle); },
    [this](const auto& goal_handle) { handleAccepted(goal_handle); });

  m_publish_timer = create_wall_timer(std::chrono::seconds(1), [this] { publishSceneGraph(); });

  m_background_worker =
    std::jthread([this](const std::stop_token& stop_token) { runBackgroundUpdates(stop_token); });

  RCLCPP_INFO(get_logger(),
              "SceneGraphGenerationNode started (phase1: %s, phase2: %s)",
              m_params.phase1_model.c_str(),
              m_params.phase2_model.c_str());
}

SceneGraphGenerationNode::~SceneGraphGenerationNode()
{
  std::jthread worker;
  {
    std::scoped_lock lock{m_goal_mutex};
    m_stopping = true;
    if (m_goal_worker.joinable())
    {
      m_goal_worker.request_stop();
      worker = std::move(m_goal_worker);
    }
  }

  // The destructor must not rely on member order: both workers use the members below.
  m_background_worker.request_stop();
  if (m_background_worker.joinable())
  {
    m_background_worker.join();
  }
  if (worker.joinable())
  {
    worker.join();
  }
}

// ── Setup ────────────────────────────────────────────────────────────────────

void SceneGraphGenerationNode::loadTemplates(const std::string& prompt_dir)
{
  RCLCPP_INFO(get_logger(), "Root template dir: %s", prompt_dir.c_str());
  const minja::Options OPTS{
    .trim_blocks = true, .lstrip_blocks = true, .keep_trailing_newline = false};
  const fs::path ROOT{prompt_dir};

  for (const auto& entry : fs::recursive_directory_iterator(ROOT))
  {
    if (!entry.is_regular_file() || entry.path().extension() != ".j2")
    {
      continue;
    }
    // Key: relative path without extension, e.g. "instance/single_attribute"
    const std::string KEY = entry.path()
                              .lexically_normal()
                              .lexically_relative(ROOT.lexically_normal())
                              .replace_extension()
                              .string();

    std::ifstream file{entry.path()};
    std::string content{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};

    m_templates[KEY] = minja::Parser::parse(content, OPTS);
    RCLCPP_INFO(get_logger(), "Loaded template: %s", KEY.c_str());
  }
  RCLCPP_INFO(get_logger(), "Loaded %zu templates from %s", m_templates.size(), prompt_dir.c_str());
}

void SceneGraphGenerationNode::loadPlugins()
{
  m_object_task_loader = std::make_shared<pluginlib::ClassLoader<IObjectTask>>(
    "semantic_scene_forests_generation", "semantic_scene_forests_generation::IObjectTask");

  m_graph_task_loader = std::make_shared<pluginlib::ClassLoader<IGraphTask>>(
    "semantic_scene_forests_generation", "semantic_scene_forests_generation::IGraphTask");

  for (const auto& name : m_params.phase1_tasks)
  {
    try
    {
      auto task = m_object_task_loader->createSharedInstance(name);
      task->initialize(this, m_templates);
      m_phase1_tasks.push_back(std::move(task));
      RCLCPP_INFO(get_logger(), "Loaded Phase 1 task: %s", name.c_str());
    }
    catch (const pluginlib::PluginlibException& e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to load Phase 1 task '%s': %s", name.c_str(), e.what());
    }
  }

  for (const auto& name : m_params.phase2_tasks)
  {
    try
    {
      auto task = m_graph_task_loader->createSharedInstance(name);
      task->initialize(this, m_templates);
      m_phase2_tasks.push_back(std::move(task));
      RCLCPP_INFO(get_logger(), "Loaded Phase 2 task: %s", name.c_str());
    }
    catch (const pluginlib::PluginlibException& e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to load Phase 2 task '%s': %s", name.c_str(), e.what());
    }
  }
}

// ── Subscription callback ─────────────────────────────────────────────────────

void SceneGraphGenerationNode::onManagedObjects(const ManagedObjectArray::SharedPtr& msg)
{
  m_objects_freq->tick();

  // The array is authoritative: rebuild the cache from its rows. Legacy
  // managers may still publish rows marked blocked; treat those as absent.
  std::unordered_map<int64_t, semantic_scene_forests_object_msgs::msg::ManagedObject> next_objects;
  for (const auto& obj : msg->objects)
  {
    if (obj.is_blocked)
    {
      continue;
    }
    next_objects.emplace(obj.id, obj);
  }

  std::unordered_set<int64_t> active_ids;
  for (const auto& [id, object] : next_objects)
  {
    (void)object;
    active_ids.insert(id);
  }

  bool membership_changed = false;
  {
    std::scoped_lock lock{m_objects_mutex, m_graph_mutex};
    membership_changed = m_cached_objects.size() != next_objects.size();
    if (!membership_changed)
    {
      for (const auto& [id, object] : m_cached_objects)
      {
        (void)object;
        if (!next_objects.contains(id))
        {
          membership_changed = true;
          break;
        }
      }
    }

    m_cached_objects = std::move(next_objects);
    if (membership_changed)
    {
      // Eagerly remove absent objects and everything that became unreachable
      // from the retained objects; pose-only refreshes never enter here.
      if (auto retained = m_scene_graph.retainObjectNodes(active_ids); !retained)
      {
        // TODO: a failed live reconcile leaves removed nodes published until the
        // next membership change retries it; commit-time reconciliation still keeps
        // removed objects out of the live graph.
        RCLCPP_ERROR(get_logger(),
                     "Failed to reconcile the live graph with the active objects: %s",
                     retained.error().message.c_str());
      }
    }
  }
  if (membership_changed)
  {
    {
      std::scoped_lock lock{m_goal_mutex};
      const auto inactive = [&active_ids](int64_t id) { return !active_ids.contains(id); };
      std::erase_if(m_phase1_queue, inactive);
      std::erase_if(m_phase1_seen, inactive);
      for (const int64_t ID : active_ids)
      {
        if (m_phase1_seen.insert(ID).second)
        {
          m_phase1_queue.push_back(ID);
        }
      }
    }
    m_background_wake.notify_all();

    // Removals must become visible immediately, not at the next 1 Hz tick.
    publishSceneGraph();
  }
}

void SceneGraphGenerationNode::onAddSceneContext(const AddSceneContext::Request& request,
                                                 AddSceneContext::Response& response)
{
  {
    // An update in flight would overwrite this context when it commits its older snapshot.
    const std::unique_lock update_lock{m_update_mutex, std::try_to_lock};
    std::scoped_lock lock{m_goal_mutex, m_objects_mutex, m_graph_mutex};
    if (!update_lock.owns_lock() || m_stopping)
    {
      response.message = "Scene graph generation is busy or stopping";
      return;
    }
    const auto SOURCE = m_scene_graph.findNode(request.node_id);
    if (!SOURCE)
    {
      response.message = "Source node does not exist";
      return;
    }
    if (std::holds_alternative<fzi::semantic_scene_forests::ObjectNode>(*SOURCE) &&
        !m_cached_objects.contains(request.node_id))
    {
      response.message = "Source object is no longer active";
      return;
    }
    auto added = appendSceneContext(m_scene_graph, request.node_id, request.context);
    if (!added)
    {
      response.message = added.error();
      return;
    }
    response.success         = true;
    response.context_node_id = added->context_node_id;
    response.created         = added->created;
    response.message         = added->created ? "Context added" : "Context already exists";
  }
  publishSceneGraph();
}

// ── Timer callback ────────────────────────────────────────────────────────────

void SceneGraphGenerationNode::publishSceneGraph()
{
  std::unique_lock lock{m_graph_mutex};
  auto msg = m_scene_graph.toMsg();
  lock.unlock();

  msg->header.stamp    = now();
  msg->header.frame_id = m_params.frame_id;
  m_scene_graph_pub->publish(*msg);
}

// ── Action server ─────────────────────────────────────────────────────────────

rclcpp_action::GoalResponse SceneGraphGenerationNode::handleGoal(
  const rclcpp_action::GoalUUID& /*uuid*/,
  const std::shared_ptr<const UpdateSceneGraphAction::Goal>& goal)
{
  const uint8_t PHASE = goal->phase;
  if (PHASE != UpdateSceneGraphAction::Goal::PHASE_1 &&
      PHASE != UpdateSceneGraphAction::Goal::PHASE_2)
  {
    RCLCPP_WARN(get_logger(), "Rejecting goal with unknown phase: %d", PHASE);
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::scoped_lock lock{m_goal_mutex};
  if (m_stopping || m_goal_active)
  {
    RCLCPP_WARN(get_logger(), "Rejecting scene graph update because another update is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  m_goal_active = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
SceneGraphGenerationNode::handleCancel(const std::shared_ptr<GoalHandle>& /*goal_handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SceneGraphGenerationNode::handleAccepted(const std::shared_ptr<GoalHandle>& goal_handle)
{
  // Join the preceding worker outside the action mutex before replacing it.
  std::jthread completed_worker;
  {
    std::scoped_lock lock{m_goal_mutex};
    if (!m_stopping)
    {
      completed_worker = std::move(m_goal_worker);
    }
  }
  if (completed_worker.joinable())
  {
    completed_worker.join();
  }

  // Final locked stop check before installing a worker: a goal caught during
  // teardown clears its reservation and receives no result.
  std::scoped_lock lock{m_goal_mutex};
  if (m_stopping)
  {
    m_goal_active = false;
    return;
  }
  m_goal_worker = std::jthread(
    [this, goal_handle](std::stop_token stop_token) { executeGoal(goal_handle, stop_token); });
}

void SceneGraphGenerationNode::executeGoal(const std::shared_ptr<GoalHandle>& goal_handle,
                                           const std::stop_token& stop_token)
{
  try
  {
    if (stop_token.stop_requested() || goal_handle->is_canceling())
    {
      stopGoal(goal_handle, stop_token);
    }
    else
    {
      const uint8_t PHASE = goal_handle->get_goal()->phase;
      RCLCPP_INFO(get_logger(), "Executing Phase %d", PHASE);
      if (PHASE == UpdateSceneGraphAction::Goal::PHASE_1)
      {
        executePhase1(goal_handle, stop_token);
      }
      else if (PHASE == UpdateSceneGraphAction::Goal::PHASE_2)
      {
        executePhase2(goal_handle, stop_token);
      }
      else
      {
        throw std::runtime_error("goal has an unknown phase");
      }
    }
  }
  catch (const std::exception& e)
  {
    // A computation exception while running must still abort with a failure
    // result; guardedTerminal contains teardown races without retrying.
    const std::string message = "Scene graph update failed: " + std::string(e.what());
    m_llm_errors.increment(message);
    abortGoal(goal_handle, message);
  }
  catch (...)
  {
    // A pluginlib-loaded task can throw a type that is not a std::exception.
    const std::string message = "Scene graph update failed: unknown exception";
    m_llm_errors.increment(message);
    abortGoal(goal_handle, message);
  }
  clearActiveGoal();
}

// ── Terminal goal transitions ────────────────────────────────────────────────

void SceneGraphGenerationNode::guardedTerminal(const std::function<void()>& terminal)
{
  // Do not publish to an already-disposed context; contain the rclcpp_action
  // exception when teardown races the call ("Asked to publish result for goal
  // that does not exist").
  if (!rclcpp::ok(get_node_base_interface()->get_context()))
  {
    return;
  }
  try
  {
    terminal();
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(get_logger(), "terminal goal transition failed: %s", e.what());
  }
}

void SceneGraphGenerationNode::abortGoal(const std::shared_ptr<GoalHandle>& goal_handle,
                                         const std::string& message)
{
  auto result     = std::make_shared<UpdateSceneGraphAction::Result>();
  result->success = false;
  result->message = message;
  guardedTerminal([&] { goal_handle->abort(result); });
  RCLCPP_ERROR(get_logger(), "%s", message.c_str());
}

void SceneGraphGenerationNode::succeedGoal(const std::shared_ptr<GoalHandle>& goal_handle,
                                           const std::string& message)
{
  auto result     = std::make_shared<UpdateSceneGraphAction::Result>();
  result->success = true;
  result->message = message;
  guardedTerminal([&] { goal_handle->succeed(result); });
  RCLCPP_INFO(get_logger(), "%s", message.c_str());
}

void SceneGraphGenerationNode::stopGoal(const std::shared_ptr<GoalHandle>& goal_handle,
                                        const std::stop_token& stop_token)
{
  // A requested stop token means teardown: no result delivery, even for a
  // client cancellation. Otherwise only an actual accepted client
  // cancellation may terminalize as canceled.
  if (stop_token.stop_requested() || !rclcpp::ok(get_node_base_interface()->get_context()))
  {
    return;
  }
  if (!goal_handle->is_canceling())
  {
    return;
  }
  auto result     = std::make_shared<UpdateSceneGraphAction::Result>();
  result->success = false;
  result->message = "Cancelled";
  guardedTerminal([&] { goal_handle->canceled(result); });
}

// ── Background updates ───────────────────────────────────────────────────────

void SceneGraphGenerationNode::runBackgroundUpdates(const std::stop_token& stop_token)
{
  const auto PHASE2_INTERVAL =
    std::chrono::seconds{std::max<int64_t>(1, m_params.phase2_interval_seconds)};
  const auto should_stop = [&stop_token]() { return stop_token.stop_requested(); };
  auto next_phase2       = std::chrono::steady_clock::now() + PHASE2_INTERVAL;

  while (!stop_token.stop_requested())
  {
    std::optional<int64_t> object_id;
    {
      std::unique_lock lock{m_goal_mutex};
      // A manual goal owns the pipeline: idle until it finishes, even if Phase 2 is due.
      m_background_wake.wait(lock, stop_token, [this]() { return !m_goal_active; });
      m_background_wake.wait_until(lock, stop_token, next_phase2, [this]() {
        return m_goal_active || !m_phase1_queue.empty();
      });
      if (stop_token.stop_requested() || m_goal_active)
      {
        continue;
      }
      if (std::chrono::steady_clock::now() < next_phase2)
      {
        object_id = m_phase1_queue.front();
        m_phase1_queue.pop_front();
      }
    }

    // A failed object is not requeued, so an LLM outage cannot loop on paid calls; a
    // manual Phase 1 retries it. A failed Phase 2 keeps its concepts for the next tick.
    try
    {
      const auto outcome =
        object_id ? describeObject(*object_id, should_stop) : runPhase2(true, should_stop);
      if (!outcome)
      {
        RCLCPP_WARN(get_logger(), "Background update failed: %s", outcome.error().c_str());
      }
    }
    catch (const std::exception& e)
    {
      const std::string message = "Background update failed: " + std::string(e.what());
      m_llm_errors.increment(message);
      RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    }
    catch (...)
    {
      // A pluginlib-loaded task can throw a type that is not a std::exception.
      const std::string message = "Background update failed: unknown exception";
      m_llm_errors.increment(message);
      RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    }

    if (!object_id)
    {
      next_phase2 = std::chrono::steady_clock::now() + PHASE2_INTERVAL;
    }
  }
}

// ── Phase 1 ───────────────────────────────────────────────────────────────────

void SceneGraphGenerationNode::executePhase1(const std::shared_ptr<GoalHandle>& goal_handle,
                                             const std::stop_token& stop_token)
{
  const auto should_stop = [&goal_handle, &stop_token]() {
    return stop_token.stop_requested() || goal_handle->is_canceling();
  };

  auto initial_state = snapshotState();
  if (!initial_state)
  {
    abortGoal(goal_handle, "Phase 1 failed to snapshot the scene state: " + initial_state.error());
    return;
  }

  auto feedback           = std::make_shared<UpdateSceneGraphAction::Feedback>();
  feedback->total_objects = static_cast<uint32_t>(initial_state->objects.size());
  uint32_t processed      = 0;
  uint32_t updated        = 0;

  for (const auto& obj : initial_state->objects)
  {
    if (should_stop())
    {
      stopGoal(goal_handle, stop_token);
      return;
    }

    feedback->current_object = processed;
    feedback->status = "Processing object " + std::to_string(obj.id) + " (" + obj.class_name + ")";
    goal_handle->publish_feedback(feedback);

    // Dequeue per object: a canceled rerun leaves the rest for the background worker.
    {
      std::scoped_lock lock{m_goal_mutex};
      std::erase(m_phase1_queue, obj.id);
    }

    const auto outcome = describeObject(obj.id, should_stop);
    if (!outcome)
    {
      abortGoal(goal_handle, outcome.error());
      return;
    }
    if (*outcome == UpdateOutcome::STOPPED)
    {
      stopGoal(goal_handle, stop_token);
      return;
    }
    ++processed;
    if (*outcome == UpdateOutcome::UPDATED)
    {
      ++updated;
    }
  }

  succeedGoal(goal_handle,
              "Phase 1 complete: updated " + std::to_string(updated) + " of " +
                std::to_string(processed) + " objects");
}

std::expected<SceneGraphGenerationNode::UpdateOutcome, std::string>
SceneGraphGenerationNode::describeObject(int64_t object_id,
                                         const std::function<bool()>& should_stop)
{
  std::scoped_lock update_lock{m_update_mutex};

  const auto system_template = m_templates.find("instance/instance_system_prompt");
  const auto setup_template  = m_templates.find("instance/instance_object_setup");
  if (system_template == m_templates.end() || setup_template == m_templates.end())
  {
    return std::unexpected("Phase 1 failed: required prompt template is missing");
  }
  const auto system_context       = minja::Context::make(minja::Value::object());
  const std::string system_prompt = system_template->second->render(system_context);
  if (!m_phase1_client)
  {
    return std::unexpected("Phase 1 failed: LLM client is not initialized");
  }
  auto& phase1_client = m_phase1_client.value();

  const auto visuals_timeout =
    std::chrono::seconds{std::max<int64_t>(1, m_params.visuals_request_timeout_seconds)};

  // Snapshot per object so earlier commits and live removals are included.
  auto state_snapshot = snapshotState();
  if (!state_snapshot)
  {
    return std::unexpected("Phase 1 failed to snapshot the scene state: " + state_snapshot.error());
  }
  const auto object_it = std::ranges::find_if(
    state_snapshot->objects, [object_id](const auto& active) { return active.id == object_id; });
  if (object_it == state_snapshot->objects.end())
  {
    return UpdateOutcome::SKIPPED; // Removed before its turn; nothing to describe.
  }
  const auto& obj = *object_it;

  // ── Fetch image crops ────────────────────────────────────────────────
  auto request        = std::make_shared<GetObjectVisuals::Request>();
  request->object_ids = {obj.id};

  std::vector<cv::Mat> crops;
  try
  {
    auto future         = m_visuals_client->async_send_request(request);
    const auto deadline = std::chrono::steady_clock::now() + visuals_timeout;
    while (future.wait_for(std::chrono::milliseconds{100}) != std::future_status::ready)
    {
      if (should_stop())
      {
        (void)m_visuals_client->remove_pending_request(future);
        return UpdateOutcome::STOPPED;
      }
      if (std::chrono::steady_clock::now() >= deadline)
      {
        (void)m_visuals_client->remove_pending_request(future);
        throw std::runtime_error("GetObjectVisuals timed out");
      }
    }

    const auto visuals_response = future.get();
    if (!visuals_response)
    {
      throw std::runtime_error("GetObjectVisuals returned an empty response");
    }
    if (!visuals_response->visuals.empty())
    {
      for (const auto& image_msg : visuals_response->visuals.front().image_crops)
      {
        try
        {
          auto cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
          crops.push_back(cv_ptr->image);
        }
        catch (const cv_bridge::Exception& e)
        {
          RCLCPP_WARN(get_logger(), "cv_bridge failed for object %ld: %s", obj.id, e.what());
        }
      }
    }
  }
  catch (const std::exception& e)
  {
    const std::string message =
      "GetObjectVisuals failed for object " + std::to_string(obj.id) + ": " + e.what();
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
    m_service_errors.increment(message);
    return UpdateOutcome::SKIPPED;
  }

  // ── Build chat history for this object ───────────────────────────────
  std::vector<oai::Message> history;
  auto system_message = oai::MessageBuilder::system().text(system_prompt).build();
  if (!system_message)
  {
    return std::unexpected("Phase 1 failed to build the system message: " + system_message.error());
  }
  history.push_back(std::move(*system_message));

  // Object setup message: text + all image crops
  {
    auto setup_ctx = minja::Value::object();
    setup_ctx.set("id", minja::Value(obj.id));
    setup_ctx.set("class_name", minja::Value(obj.class_name));
    const auto setup_context     = minja::Context::make(std::move(setup_ctx));
    const std::string setup_text = setup_template->second->render(setup_context);

    auto builder = oai::MessageBuilder::user().text(setup_text);
    for (const auto& crop : crops)
    {
      builder.image(crop);
    }
    auto msg = builder.build();
    if (!msg)
    {
      RCLCPP_WARN(
        get_logger(), "Image encoding failed for object %ld: %s", obj.id, msg.error().c_str());
      auto fallback = oai::MessageBuilder::user().text(setup_text).build();
      if (!fallback)
      {
        return std::unexpected("Phase 1 failed to build the object setup message: " +
                               fallback.error());
      }
      history.push_back(std::move(*fallback));
    }
    else
    {
      history.push_back(std::move(*msg));
    }
  }

  // Send setup and get acknowledgement
  auto ack = phase1_client.chat({.messages = history});
  if (!ack)
  {
    m_llm_errors.increment(ack.error().message);
    return std::unexpected("Phase 1 setup request failed for object " + std::to_string(obj.id) +
                           ": " + ack.error().message);
  }
  auto acknowledgement = oai::MessageBuilder::assistant().text(ack->text()).build();
  if (!acknowledgement)
  {
    return std::unexpected("Phase 1 failed to build the setup acknowledgement: " +
                           acknowledgement.error());
  }
  history.push_back(std::move(*acknowledgement));

  auto& graph = *state_snapshot->graph;
  if (const auto existing = graph.findNode(obj.id); existing)
  {
    if (!std::holds_alternative<fzi::semantic_scene_forests::ObjectNode>(*existing))
    {
      return std::unexpected("Phase 1 found an object id that is already used by a concept node");
    }
  }
  else
  {
    auto detection = std::make_shared<vision_msgs::msg::Detection3D>(obj.detection);
    std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals{};
    if (auto add_result = graph.addObjectNodeWithId(obj.id, obj.class_name, detection, visuals);
        !add_result)
    {
      return std::unexpected("Phase 1 failed to add object " + std::to_string(obj.id) + ": " +
                             add_result.error().message);
    }
  }

  // ── Run each Phase 1 task ────────────────────────────────────────────
  for (auto& task : m_phase1_tasks)
  {
    if (should_stop())
    {
      return UpdateOutcome::STOPPED;
    }
    auto task_result = task->execute(phase1_client, history, obj.id, crops, graph);
    if (!task_result)
    {
      const std::string message =
        "Phase 1 task failed for object " + std::to_string(obj.id) + ": " + task_result.error();
      m_llm_errors.increment(message);
      return std::unexpected(message);
    }
  }

  if (should_stop())
  {
    return UpdateOutcome::STOPPED;
  }
  if (auto committed = commitGraph(graph); !committed)
  {
    return std::unexpected("Phase 1 failed to commit the scene graph: " + committed.error());
  }
  RCLCPP_INFO(get_logger(), "Described object %ld (%s)", object_id, obj.class_name.c_str());
  return UpdateOutcome::UPDATED;
}

// ── Phase 2 ───────────────────────────────────────────────────────────────────

void SceneGraphGenerationNode::executePhase2(const std::shared_ptr<GoalHandle>& goal_handle,
                                             const std::stop_token& stop_token)
{
  const auto outcome = runPhase2(false, [&goal_handle, &stop_token]() {
    return stop_token.stop_requested() || goal_handle->is_canceling();
  });
  if (!outcome)
  {
    abortGoal(goal_handle, outcome.error());
  }
  else if (*outcome == UpdateOutcome::STOPPED)
  {
    stopGoal(goal_handle, stop_token);
  }
  else if (*outcome == UpdateOutcome::SKIPPED)
  {
    succeedGoal(goal_handle, "Phase 2 skipped: no concept nodes are available");
  }
  else
  {
    succeedGoal(goal_handle, "Phase 2 complete");
  }
}

std::expected<SceneGraphGenerationNode::UpdateOutcome, std::string>
SceneGraphGenerationNode::runPhase2(bool only_new_concepts,
                                    const std::function<bool()>& should_stop)
{
  std::scoped_lock update_lock{m_update_mutex};

  if (should_stop())
  {
    return UpdateOutcome::STOPPED;
  }
  if (!m_phase2_client)
  {
    return std::unexpected("Phase 2 failed: LLM client is not initialized");
  }
  auto& phase2_client = m_phase2_client.value();

  const auto system_template = m_templates.find("meta/meta_system_prompt");
  const auto setup_template  = m_templates.find("meta/meta_setup_prompt");
  if (system_template == m_templates.end() || setup_template == m_templates.end())
  {
    return std::unexpected("Phase 2 failed: required prompt template is missing");
  }

  auto state_snapshot = snapshotState();
  if (!state_snapshot)
  {
    return std::unexpected("Phase 2 failed to snapshot the scene state: " + state_snapshot.error());
  }
  auto& graph = *state_snapshot->graph;

  // Tasks read their input through getConceptNodes() only, so a graph holding just the
  // unprocessed concepts narrows the input while results still land in the full graph.
  fzi::semantic_scene_forests::SceneGraph new_concepts;
  if (only_new_concepts)
  {
    for (const auto& concept_node : graph.getConceptNodes())
    {
      if (m_phase2_processed_concepts.contains(concept_node.id))
      {
        continue;
      }
      if (auto added =
            new_concepts.addConceptNodeWithId(concept_node.id, concept_node.concept_name);
          !added)
      {
        return std::unexpected("Phase 2 failed to collect new concepts: " + added.error().message);
      }
    }
  }
  const auto& input_graph = only_new_concepts ? new_concepts : graph;

  const auto CONCEPTS = input_graph.getConceptNodes();
  if (CONCEPTS.empty())
  {
    return UpdateOutcome::SKIPPED;
  }

  const auto system_context       = minja::Context::make(minja::Value::object());
  const std::string system_prompt = system_template->second->render(system_context);

  std::vector<oai::Message> history;
  auto system_message = oai::MessageBuilder::system().text(system_prompt).build();
  if (!system_message)
  {
    return std::unexpected("Phase 2 failed to build the system message: " + system_message.error());
  }
  history.push_back(std::move(*system_message));

  // Build concept list for meta setup
  auto setup_context = minja::Value::object();
  auto objects_list  = minja::Value::array();
  for (const auto& concept_node : CONCEPTS)
  {
    objects_list.push_back(minja::Value(concept_node.concept_name));
  }
  setup_context.set("input_objects", objects_list);
  const auto meta_context      = minja::Context::make(std::move(setup_context));
  const std::string setup_text = setup_template->second->render(meta_context);

  auto setup_message = oai::MessageBuilder::user().text(setup_text).build();
  if (!setup_message)
  {
    return std::unexpected("Phase 2 failed to build the setup message: " + setup_message.error());
  }
  history.push_back(std::move(*setup_message));

  auto ack = phase2_client.chat({.messages = history});
  if (!ack)
  {
    m_llm_errors.increment(ack.error().message);
    return std::unexpected("Phase 2 setup request failed: " + ack.error().message);
  }
  auto acknowledgement = oai::MessageBuilder::assistant().text(ack->text()).build();
  if (!acknowledgement)
  {
    return std::unexpected("Phase 2 failed to build the setup acknowledgement: " +
                           acknowledgement.error());
  }
  history.push_back(std::move(*acknowledgement));

  // Run each Phase 2 task
  for (auto& task : m_phase2_tasks)
  {
    if (should_stop())
    {
      return UpdateOutcome::STOPPED;
    }
    auto task_result = task->execute(phase2_client, history, input_graph, graph);
    if (!task_result)
    {
      const std::string message = "Phase 2 task failed: " + task_result.error();
      m_llm_errors.increment(message);
      return std::unexpected(message);
    }
  }

  if (should_stop())
  {
    return UpdateOutcome::STOPPED;
  }

  // Concepts Phase 2 just created count as processed, or the next tick would
  // categorize its own categories.
  std::unordered_set<int64_t> processed_concepts;
  for (const auto& concept_node : graph.getConceptNodes())
  {
    processed_concepts.insert(concept_node.id);
  }
  if (auto committed = commitGraph(graph); !committed)
  {
    return std::unexpected("Phase 2 failed to commit the scene graph: " + committed.error());
  }
  m_phase2_processed_concepts = std::move(processed_concepts);
  RCLCPP_INFO(get_logger(), "Phase 2 processed %zu concepts", CONCEPTS.size());
  return UpdateOutcome::UPDATED;
}

std::expected<SceneGraphGenerationNode::StateSnapshot, std::string>
SceneGraphGenerationNode::snapshotState()
{
  std::scoped_lock lock{m_objects_mutex, m_graph_mutex};

  auto snapshot = m_scene_graph.clone();
  if (!snapshot)
  {
    return std::unexpected(snapshot.error().message);
  }

  StateSnapshot state;
  state.graph = std::move(*snapshot);
  state.objects.reserve(m_cached_objects.size());
  for (const auto& [id, object] : m_cached_objects)
  {
    (void)id;
    state.objects.push_back(object);
  }
  return state;
}

std::expected<void, std::string>
SceneGraphGenerationNode::commitGraph(fzi::semantic_scene_forests::SceneGraph& candidate)
{
  std::scoped_lock lock{m_objects_mutex, m_graph_mutex};

  std::unordered_set<int64_t> active_ids;
  active_ids.reserve(m_cached_objects.size());
  for (const auto& [id, object] : m_cached_objects)
  {
    (void)object;
    active_ids.insert(id);
  }
  if (auto retained = candidate.retainObjectNodes(active_ids); !retained)
  {
    return std::unexpected("failed to reconcile the candidate against the active objects: " +
                           retained.error().message);
  }

  m_scene_graph.swap(candidate);
  return {};
}

void SceneGraphGenerationNode::clearActiveGoal()
{
  {
    std::scoped_lock lock{m_goal_mutex};
    m_goal_active = false;
  }
  m_background_wake.notify_all();
}

void SceneGraphGenerationNode::onConnectivityDiagnostic(
  diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  const bool service_ready = m_visuals_client->service_is_ready();
  stat.add("GetObjectVisuals ready", service_ready);
  m_service_errors.report(stat, "Service");
  m_llm_errors.report(stat, "LLM");
  {
    std::scoped_lock lock{m_graph_mutex};
    stat.add("Graph nodes",
             m_scene_graph.getObjectNodes().size() + m_scene_graph.getConceptNodes().size());
    stat.add("Graph edges", m_scene_graph.getEdges().size());
  }

  stat.add("LLM endpoint", m_params.server_url);
  bool llm_ok = false;
  if (m_phase1_client)
  {
    const auto HEALTH = m_phase1_client->isHealthy();
    llm_ok            = HEALTH.has_value();
    stat.add("LLM reachable", llm_ok);
    if (!HEALTH)
    {
      stat.add("LLM error", HEALTH.error().message);
    }
  }
  else
  {
    stat.add("LLM reachable", false);
    stat.add("LLM error", "client not initialized");
  }

  if (!llm_ok)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                 "LLM endpoint unreachable: " + m_params.server_url);
  }
  else if (!service_ready)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                 "GetObjectVisuals service not available");
  }
  else
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Connectivity nominal");
  }
}

} // namespace semantic_scene_forests_generation

RCLCPP_COMPONENTS_REGISTER_NODE(semantic_scene_forests_generation::SceneGraphGenerationNode)
