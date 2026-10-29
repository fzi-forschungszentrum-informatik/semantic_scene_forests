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

#include "semantic_scene_forests_query_node.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <iterator>
#include <limits>
#include <thread>
#include <utility>

#include <fmt/format.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include <nlohmann/json.hpp>

#include <oai_api/builder.hpp>
#include <oai_api/types.hpp>

namespace semantic_scene_forests_query {

using json   = nlohmann::json;
namespace fs = std::filesystem;

namespace {

// Hardcoded ROS interface names (intentionally NOT parameters).
constexpr const char* K_SCENE_GRAPH_TOPIC     = "scene_graph";
constexpr const char* K_MANAGED_OBJECTS_TOPIC = "managed_objects";
constexpr const char* K_GET_OBJECT_VISUALS    = "get_object_visuals";
constexpr const char* K_FIND_OBJECT_ACTION    = "find_object";
constexpr const char* K_PROMPT_LLM_ACTION     = "prompt_llm";

// Extract the first balanced {...} block from arbitrary model text (handles
// fenced ```json blocks and surrounding prose). Returns the substring or "".
std::string extractJsonObject(const std::string& text)
{
  const auto FIRST = text.find('{');
  const auto LAST  = text.rfind('}');
  if (FIRST == std::string::npos || LAST == std::string::npos || LAST < FIRST)
  {
    return {};
  }
  return text.substr(FIRST, LAST - FIRST + 1);
}

} // namespace

// ── Constructor ─────────────────────────────────────────────────────────────

SceneGraphQueryNode::SceneGraphQueryNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("semantic_scene_forests_query", options)
  , m_diagnostic_updater(this)
{
  m_param_listener = std::make_shared<ParamListener>(get_node_parameters_interface());
  m_params         = m_param_listener->get_params();

  m_diagnostic_updater.setHardwareID("semantic_scene_forests_query");
  m_diagnostic_updater.add("Connectivity", this, &SceneGraphQueryNode::onConnectivityDiagnostic);

  fs::path prompt_dir;
  if (!m_params.prompt_dir.empty())
  {
    prompt_dir = m_params.prompt_dir;
  }
  else
  {
    prompt_dir =
      fs::path(ament_index_cpp::get_package_share_directory("semantic_scene_forests_query")) /
      "resources";
  }
  loadTemplates(prompt_dir.string());

  // Match the semantic_scene_forests_generation publisher so the latest graph is latched.
  const auto QOS = rclcpp::QoS(1).reliable().transient_local();

  // NOLINTNEXTLINE(performance-unnecessary-value-param): rclcpp delivers SharedPtr by value.
  m_graph_sub = create_subscription<SceneGraphMsg>(
    K_SCENE_GRAPH_TOPIC, QOS, [this](const SceneGraphMsg::SharedPtr msg) { onSceneGraph(msg); });

  // NOLINTNEXTLINE(performance-unnecessary-value-param)
  m_objects_sub = create_subscription<ManagedObjectArray>(
    K_MANAGED_OBJECTS_TOPIC, QOS, [this](const ManagedObjectArray::SharedPtr msg) {
      onManagedObjects(msg);
    });

  m_graph_freq = std::make_unique<diagnostic_updater::HeaderlessTopicDiagnostic>(
    K_SCENE_GRAPH_TOPIC,
    m_diagnostic_updater,
    diagnostic_updater::FrequencyStatusParam(&m_min_freq, &m_max_freq, 0.1, 5));
  m_objects_freq = std::make_unique<diagnostic_updater::HeaderlessTopicDiagnostic>(
    K_MANAGED_OBJECTS_TOPIC,
    m_diagnostic_updater,
    diagnostic_updater::FrequencyStatusParam(&m_min_freq, &m_max_freq, 0.1, 5));

  m_visuals_client = create_client<GetObjectVisuals>(K_GET_OBJECT_VISUALS);

  m_find_object_server = rclcpp_action::create_server<FindObjectAction>(
    this,
    K_FIND_OBJECT_ACTION,
    [this](const auto& uuid, auto goal) { return handleFindObjectGoal(uuid, std::move(goal)); },
    [this](auto goal_handle) { return handleFindObjectCancel(std::move(goal_handle)); },
    [this](auto goal_handle) { handleFindObjectAccepted(std::move(goal_handle)); });

  m_prompt_llm_server = rclcpp_action::create_server<PromptLLMAction>(
    this,
    K_PROMPT_LLM_ACTION,
    [this](const auto& uuid, auto goal) { return handlePromptLLMGoal(uuid, std::move(goal)); },
    [this](auto goal_handle) { return handlePromptLLMCancel(std::move(goal_handle)); },
    [this](auto goal_handle) { handlePromptLLMAccepted(std::move(goal_handle)); });

  RCLCPP_INFO(get_logger(),
              "SceneGraphQueryNode started (model: %s, server: %s)",
              m_params.model.c_str(),
              m_params.server_url.c_str());
}

SceneGraphQueryNode::~SceneGraphQueryNode()
{
  // Mark both action kinds stopping and request BOTH workers to stop before
  // joining either, so a goal blocked in one worker cannot delay the other's
  // teardown. Joins happen outside the action mutexes.
  std::jthread find_object_worker;
  {
    const std::scoped_lock LOCK{m_find_object_mutex};
    m_find_object_stopping = true;
    if (m_find_object_worker.joinable())
    {
      m_find_object_worker.request_stop();
      find_object_worker = std::move(m_find_object_worker);
    }
  }
  std::jthread prompt_llm_worker;
  {
    const std::scoped_lock LOCK{m_prompt_llm_mutex};
    m_prompt_llm_stopping = true;
    if (m_prompt_llm_worker.joinable())
    {
      m_prompt_llm_worker.request_stop();
      prompt_llm_worker = std::move(m_prompt_llm_worker);
    }
  }
  if (find_object_worker.joinable())
  {
    find_object_worker.join();
  }
  if (prompt_llm_worker.joinable())
  {
    prompt_llm_worker.join();
  }
}

// ── Setup ─────────────────────────────────────────────────────────────────────

void SceneGraphQueryNode::loadTemplates(const std::string& prompt_dir)
{
  const minja::Options OPTS{
    .trim_blocks = true, .lstrip_blocks = true, .keep_trailing_newline = false};
  const fs::path ROOT{prompt_dir};

  if (!fs::exists(ROOT))
  {
    RCLCPP_WARN(
      get_logger(), "Prompt dir '%s' does not exist; using built-in prompts", prompt_dir.c_str());
    return;
  }

  for (const auto& entry : fs::recursive_directory_iterator(ROOT))
  {
    if (!entry.is_regular_file() || entry.path().extension() != ".j2")
    {
      continue;
    }
    const std::string KEY = entry.path()
                              .lexically_normal()
                              .lexically_relative(ROOT.lexically_normal())
                              .replace_extension()
                              .string();
    std::ifstream file{entry.path()};
    const std::string CONTENT{std::istreambuf_iterator<char>(file),
                              std::istreambuf_iterator<char>()};
    m_templates[KEY] = minja::Parser::parse(CONTENT, OPTS);
    RCLCPP_DEBUG(get_logger(), "Loaded template: %s", KEY.c_str());
  }
  RCLCPP_INFO(get_logger(), "Loaded %zu templates from %s", m_templates.size(), prompt_dir.c_str());
}

// ── Subscription callbacks ──────────────────────────────────────────────────--

void SceneGraphQueryNode::onSceneGraph(const SceneGraphMsg::SharedPtr& msg)
{
  m_graph_freq->tick();
  const std::scoped_lock LOCK{m_graph_mutex};
  m_last_graph = *msg;
}

void SceneGraphQueryNode::onManagedObjects(const ManagedObjectArray::SharedPtr& msg)
{
  m_objects_freq->tick();
  // Complete managed-object arrays are authoritative. Blocked legacy rows are
  // not active. The membership epoch changes only when the active id set
  // changes, so in-flight query results can be checked for staleness.
  std::unordered_map<int64_t, ManagedObject> next;
  for (const auto& obj : msg->objects)
  {
    if (!obj.is_blocked)
    {
      next[obj.id] = obj;
    }
  }
  const std::scoped_lock LOCK{m_objects_mutex};
  bool changed = next.size() != m_cached_objects.size();
  if (!changed)
  {
    for (const auto& [id, obj] : next)
    {
      if (!m_cached_objects.contains(id))
      {
        changed = true;
        break;
      }
    }
  }
  if (changed)
  {
    ++m_objects_epoch;
  }
  m_cached_objects   = std::move(next);
  m_objects_received = true;
}

// ── Shared helpers ──────────────────────────────────────────────────────────--

std::expected<SceneGraphQueryNode::SceneSnapshot, std::string>
SceneGraphQueryNode::snapshotGraph() const
{
  // Consistent capture: graph message and managed-object state are copied
  // together under one scoped_lock, then the locks are released before any
  // conversion, sanitization or IO below.
  SceneGraphMsg msg;
  std::unordered_set<int64_t> active_ids;
  bool received{false};
  std::uint64_t epoch{0};
  {
    const std::scoped_lock LOCK{m_objects_mutex, m_graph_mutex};
    if (!m_last_graph)
    {
      return std::unexpected{"no scene graph received yet"};
    }
    msg      = *m_last_graph;
    received = m_objects_received;
    epoch    = m_objects_epoch;
    active_ids.reserve(m_cached_objects.size());
    for (const auto& [id, obj] : m_cached_objects)
    {
      active_ids.insert(id);
    }
  }
  // Queries require an authoritative managed-object snapshot: fail clearly
  // until one arrives. A confirmed empty array is a valid empty scene.
  if (!received)
  {
    return std::unexpected{"no authoritative managed-object state received yet"};
  }

  auto graph = fzi::semantic_scene_forests::SceneGraph::fromMsg(msg);
  if (!graph)
  {
    return std::unexpected{
      fmt::format("failed to build graph snapshot: {}", graph.error().message)};
  }
  // Drop objects (and concepts unreachable from the retained ones) that are
  // no longer active, so removals take effect even for a latched graph.
  if (auto retained = (*graph)->retainObjectNodes(active_ids); !retained)
  {
    return std::unexpected{
      fmt::format("failed to sanitize graph snapshot: {}", retained.error().message)};
  }

  std::unordered_set<int64_t> object_ids;
  for (const auto& node : (*graph)->getObjectNodes())
  {
    object_ids.insert(node.id);
  }
  return SceneSnapshot{
    .graph = std::move(*graph), .object_ids = std::move(object_ids), .epoch = epoch};
}

std::optional<ObjectInfo> SceneGraphQueryNode::lookupObject(int64_t id) const
{
  const std::scoped_lock LOCK{m_objects_mutex};
  const auto IT = m_cached_objects.find(id);
  if (IT == m_cached_objects.end())
  {
    return std::nullopt;
  }
  const auto& OBJ  = IT->second;
  const auto& BBOX = OBJ.detection.bbox;
  return ObjectInfo{.class_name = OBJ.class_name,
                    .position_x = BBOX.center.position.x,
                    .position_y = BBOX.center.position.y,
                    .position_z = BBOX.center.position.z,
                    .size_x     = BBOX.size.x,
                    .size_y     = BBOX.size.y,
                    .size_z     = BBOX.size.z};
}

std::expected<std::string, std::string> SceneGraphQueryNode::inspectObjectVisual(
  int64_t id, std::string_view focus, const StopCheck& should_stop)
{
  auto request        = std::make_shared<GetObjectVisuals::Request>();
  request->object_ids = {id};

  // Poll the service future so the wait is bounded by the visuals timeout and
  // an interruption or a timeout cannot leak a pending request.
  const auto TIMEOUT =
    std::chrono::seconds{std::max<int64_t>(1, m_params.visuals_request_timeout_seconds)};
  auto future         = m_visuals_client->async_send_request(request);
  const auto DEADLINE = std::chrono::steady_clock::now() + TIMEOUT;
  while (future.wait_for(std::chrono::milliseconds{100}) != std::future_status::ready)
  {
    if (should_stop && should_stop())
    {
      (void)m_visuals_client->remove_pending_request(future);
      return std::unexpected{fmt::format("visual inspection of object {} interrupted", id)};
    }
    if (std::chrono::steady_clock::now() >= DEADLINE)
    {
      (void)m_visuals_client->remove_pending_request(future);
      return std::unexpected{fmt::format("/get_object_visuals timed out for object {}", id)};
    }
  }
  auto response = future.get();
  if (response->visuals.empty() || response->visuals.front().image_crops.empty())
  {
    return std::unexpected{fmt::format("no image crops available for object {}", id)};
  }

  cv::Mat crop;
  try
  {
    // cv_bridge throws on conversion failure; convert that to std::expected here.
    crop = cv_bridge::toCvCopy(response->visuals.front().image_crops.front(), "bgr8")->image;
  }
  catch (const std::exception& e)
  {
    return std::unexpected{fmt::format("cv_bridge failed for object {}: {}", id, e.what())};
  }

  const std::string PROMPT =
    focus.empty() ? "Describe this object in detail (appearance, colour, material, function)."
                  : fmt::format("Describe this object, focusing on: {}", focus);

  auto user_msg = oai::MessageBuilder::user().text(PROMPT).image(crop).build();
  if (!user_msg)
  {
    return std::unexpected{
      fmt::format("failed to encode crop for object {}: {}", id, user_msg.error())};
  }

  auto client    = makeClient();
  auto response2 = client.complete(
    {oai::MessageBuilder::system().text("You are a precise vision assistant.").build().value(),
     std::move(*user_msg)});
  if (!response2)
  {
    m_llm_errors.increment(response2.error().message);
    return std::unexpected{fmt::format("vision completion failed ({}): {}",
                                       response2.error().http_status,
                                       response2.error().message)};
  }
  return response2->choice.content.value_or("");
}

std::string SceneGraphQueryNode::renderSystemPrompt(const std::string& template_key) const
{
  if (const auto IT = m_templates.find(template_key); IT != m_templates.end())
  {
    auto ctx = minja::Context::make(minja::Value::object());
    return IT->second->render(ctx);
  }
  return "You are an assistant that answers questions about a robot's scene graph. "
         "Use the provided tools to inspect objects, concepts and their relationships before "
         "answering.";
}

std::string
SceneGraphQueryNode::renderObjectCatalog(const fzi::semantic_scene_forests::SceneGraph& graph) const
{
  auto objects = minja::Value::array();
  for (const auto& obj : graph.getObjectNodes())
  {
    std::string class_name;
    if (const auto INFO = lookupObject(obj.id))
    {
      class_name = INFO->class_name;
    }
    auto entry = minja::Value::object();
    entry.set("id", minja::Value(obj.id));
    entry.set("class_name", minja::Value(class_name));
    objects.push_back(entry);
  }

  if (const auto IT = m_templates.find("common/object_catalog"); IT != m_templates.end())
  {
    auto ctx_obj = minja::Value::object();
    ctx_obj.set("objects", objects);
    auto ctx = minja::Context::make(std::move(ctx_obj));
    return IT->second->render(ctx);
  }

  // Fallback plain-text catalog.
  std::string out = "Objects currently in the scene graph (id: class_name):\n";
  for (const auto& obj : graph.getObjectNodes())
  {
    std::string class_name;
    if (const auto INFO = lookupObject(obj.id))
    {
      class_name = INFO->class_name;
    }
    out += fmt::format("- {}: {}\n", obj.id, class_name);
  }
  return out;
}

oai::OpenAIClient SceneGraphQueryNode::makeClient() const
{
  // Production defaults: 5 s connect, 60 s request (see parameter schema).
  const auto CONNECT_TIMEOUT =
    std::chrono::seconds{std::max<int64_t>(1, m_params.llm_connect_timeout_seconds)};
  const auto REQUEST_TIMEOUT =
    std::chrono::seconds{std::max<int64_t>(1, m_params.llm_request_timeout_seconds)};
  return oai::ClientBuilder{}
    .baseUrl(m_params.server_url)
    .apiKey(m_params.api_key)
    .model(m_params.model)
    .temperature(static_cast<float>(m_params.temperature))
    .maxTokens(m_params.max_tokens)
    .connectTimeout(CONNECT_TIMEOUT)
    .requestTimeout(REQUEST_TIMEOUT)
    .build();
}

std::expected<SceneGraphQueryNode::AgentOutcome, std::string>
SceneGraphQueryNode::runAgent(const std::string& system_prompt_key,
                              std::string_view request,
                              const ExpandCallback& on_expand,
                              const StopCheck& should_stop)
{
  auto snapshot = snapshotGraph();
  if (!snapshot)
  {
    return std::unexpected{snapshot.error()};
  }

  const std::string CATALOG = renderObjectCatalog(*snapshot->graph);

  GraphAgentSession session(
    std::move(snapshot->graph),
    [this](int64_t id) { return lookupObject(id); },
    [this, should_stop](int64_t id, std::string_view focus) {
      return inspectObjectVisual(id, focus, should_stop);
    },
    on_expand,
    should_stop,
    [this](const std::string& line) { RCLCPP_INFO(get_logger(), "[agent] %s", line.c_str()); });

  auto agent = oai::AgentBuilder{}
                 .systemPrompt(renderSystemPrompt(system_prompt_key))
                 .maxSteps(m_params.max_steps)
                 .client(makeClient())
                 .tools(session.buildTools())
                 .build();

  const std::string TASK = fmt::format("{}\n\nUser request: {}", CATALOG, request);
  auto answer            = agent.run(TASK);
  if (!answer)
  {
    m_llm_errors.increment(answer.error().message);
    RCLCPP_WARN(get_logger(), "[agent] run failed: %s", answer.error().message.c_str());
    return std::unexpected{
      fmt::format("LLM error ({}): {}", answer.error().http_status, answer.error().message)};
  }
  RCLCPP_INFO(get_logger(), "[agent] final answer: %s", answer->c_str());
  return AgentOutcome{.text                = std::move(*answer),
                      .expanded            = session.expandedNodes(),
                      .snapshot_object_ids = std::move(snapshot->object_ids),
                      .membership_epoch    = snapshot->epoch};
}

void SceneGraphQueryNode::guardedTerminal(const std::function<void()>& terminal)
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

// ── FindObject action ─────────────────────────────────────────────────────────

rclcpp_action::GoalResponse
SceneGraphQueryNode::handleFindObjectGoal(const rclcpp_action::GoalUUID& /*uuid*/,
                                          std::shared_ptr<const FindObjectAction::Goal> goal)
{
  if (goal->object_description.empty())
  {
    RCLCPP_WARN(get_logger(), "Rejecting FindObject goal: empty object_description");
    return rclcpp_action::GoalResponse::REJECT;
  }
  {
    const std::scoped_lock LOCK{m_graph_mutex};
    if (!m_last_graph)
    {
      RCLCPP_WARN(get_logger(), "Rejecting FindObject goal: no scene graph received yet");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  const std::scoped_lock LOCK{m_find_object_mutex};
  if (m_find_object_stopping || m_find_object_active)
  {
    RCLCPP_WARN(get_logger(), "Rejecting FindObject goal: another FindObject goal is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  m_find_object_active = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
SceneGraphQueryNode::handleFindObjectCancel(const std::shared_ptr<FindObjectGoal>& /*goal_handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SceneGraphQueryNode::handleFindObjectAccepted(
  const std::shared_ptr<FindObjectGoal>& goal_handle)
{
  // Join the preceding worker outside the action mutex before replacing it.
  std::jthread completed_worker;
  {
    const std::scoped_lock LOCK{m_find_object_mutex};
    if (!m_find_object_stopping)
    {
      completed_worker = std::move(m_find_object_worker);
    }
  }
  if (completed_worker.joinable())
  {
    completed_worker.join();
  }

  // Final locked stop check before replacement: a goal caught during teardown
  // clears the active flag and receives no result.
  const std::scoped_lock LOCK{m_find_object_mutex};
  if (m_find_object_stopping)
  {
    m_find_object_active = false;
    return;
  }
  m_find_object_worker = std::jthread([this, goal_handle](std::stop_token stop) {
    executeFindObject(goal_handle, std::move(stop));
    const std::scoped_lock LOCK{m_find_object_mutex};
    m_find_object_active = false;
  });
}

void SceneGraphQueryNode::executeFindObject(const std::shared_ptr<FindObjectGoal>& goal_handle,
                                            std::stop_token stop)
{
  const auto GOAL = goal_handle->get_goal();
  RCLCPP_INFO(get_logger(), "FindObject: '%s'", GOAL->object_description.c_str());

  const ExpandCallback ON_EXPAND = [goal_handle](const std::vector<int64_t>& ids) {
    auto fb               = std::make_shared<FindObjectAction::Feedback>();
    fb->expanded_node_ids = ids;
    fb->status            = fmt::format("Inspected {} node(s)", ids.size());
    goal_handle->publish_feedback(fb);
  };

  // Both a client cancellation and a shutdown request interrupt the agent at
  // the next tool boundary; only an accepted cancellation may end as canceled.
  const StopCheck SHOULD_STOP = [goal_handle, &stop] {
    return goal_handle->is_canceling() || stop.stop_requested();
  };

  // Explicit completion outcome: distinguishes an abort from a valid
  // not-found answer, which still SUCCEEDS with success=false and object_id=-1.
  enum class Completion : std::uint8_t
  {
    SUCCEED,
    ABORT
  };

  auto result       = std::make_shared<FindObjectAction::Result>();
  result->object_id = -1;

  Completion completion = Completion::ABORT;
  std::string abort_reason;

  try
  {
    // Cheap pre-flight: skip the paid LLM call when already cancelled/stopping.
    if (!SHOULD_STOP())
    {
      auto outcome =
        runAgent("find_object/system_prompt", GOAL->object_description, ON_EXPAND, SHOULD_STOP);
      if (outcome)
      {
        auto validated = validateFindObjectAnswer(*outcome);
        if (validated)
        {
          result     = *validated;
          completion = Completion::SUCCEED;
        }
        else
        {
          abort_reason = validated.error();
        }
      }
      else
      {
        m_action_errors.increment(outcome.error());
        abort_reason = outcome.error();
      }
    }
  }
  catch (const AgentStopped&)
  {
    // The agent stopped at a tool boundary; the completion point below decides
    // whether that is a client cancellation or a silent teardown.
    abort_reason = "Interrupted";
  }
  catch (const std::exception& e)
  {
    abort_reason = std::string{"internal error: "} + e.what();
  }
  catch (...)
  {
    abort_reason = "unknown internal error";
  }

  // Single completion point: shutdown/context-gone delivers no result, an
  // accepted client cancellation ends as canceled, a computation or validation
  // failure aborts, and otherwise the answer succeeds.
  if (stop.stop_requested() || !rclcpp::ok(get_node_base_interface()->get_context()))
  {
    return;
  }
  if (goal_handle->is_canceling())
  {
    // Uniform cancel payload: no computed-success fields may leak into a
    // CANCELED result.
    result->success   = false;
    result->object_id = -1;
    result->message   = "Cancelled";
    guardedTerminal([&] { goal_handle->canceled(result); });
    return;
  }
  if (completion == Completion::ABORT)
  {
    result->message = abort_reason;
    guardedTerminal([&] { goal_handle->abort(result); });
    RCLCPP_WARN(get_logger(), "FindObject aborted: %s", abort_reason.c_str());
    return;
  }
  guardedTerminal([&] { goal_handle->succeed(result); });
  RCLCPP_INFO(get_logger(),
              "FindObject done: success=%d object_id=%ld",
              static_cast<int>(result->success),
              result->object_id);
}

std::expected<SceneGraphQueryNode::FindObjectAction::Result::SharedPtr, std::string>
SceneGraphQueryNode::validateFindObjectAnswer(const AgentOutcome& outcome) const
{
  // The model is instructed to answer with {"found": bool, "object_id": int,
  // "message": str}. Validate strictly: malformed, fabricated or stale answers
  // are ABORTED, never successful parses with defaulted fields.
  const json PARSED = json::parse(extractJsonObject(outcome.text), nullptr, false);
  if (PARSED.is_discarded() || !PARSED.is_object())
  {
    return std::unexpected{"could not parse model output: " + outcome.text};
  }

  const auto FOUND_FIELD = PARSED.find("found");
  if (FOUND_FIELD == PARSED.end() || !FOUND_FIELD->is_boolean())
  {
    return std::unexpected{"model output is missing a boolean 'found' field: " + outcome.text};
  }
  const bool FOUND = FOUND_FIELD->get<bool>();

  // Receipt + epoch gate for BOTH positive and negative answers: any result
  // computed against outdated membership is stale and must not terminalize.
  {
    const std::scoped_lock LOCK{m_objects_mutex};
    if (!m_objects_received)
    {
      return std::unexpected{"no authoritative managed-object state received yet"};
    }
    if (m_objects_epoch != outcome.membership_epoch)
    {
      return std::unexpected{"object membership changed during the query (stale result)"};
    }
  }

  auto result       = std::make_shared<FindObjectAction::Result>();
  result->object_id = -1;
  result->success   = false;
  result->message   = outcome.text;
  if (const auto MSG_FIELD = PARSED.find("message"); MSG_FIELD != PARSED.end())
  {
    if (!MSG_FIELD->is_string())
    {
      return std::unexpected{"model output has a non-string 'message' field: " + outcome.text};
    }
    result->message = MSG_FIELD->get<std::string>();
  }

  if (!FOUND)
  {
    // A genuine negative on unchanged membership is a completed, unsuccessful
    // lookup.
    return result;
  }

  const auto ID_FIELD = PARSED.find("object_id");
  if (ID_FIELD == PARSED.end() || !ID_FIELD->is_number_integer())
  {
    return std::unexpected{"positive match without an integral 'object_id': " + outcome.text};
  }
  int64_t object_id = -1;
  if (ID_FIELD->is_number_unsigned())
  {
    // Check the unsigned range BEFORE casting to a signed integer.
    const auto AS_UNSIGNED = ID_FIELD->get<uint64_t>();
    if (AS_UNSIGNED > static_cast<uint64_t>(std::numeric_limits<int64_t>::max()))
    {
      return std::unexpected{"'object_id' is not representable as a signed 64-bit integer"};
    }
    object_id = static_cast<int64_t>(AS_UNSIGNED);
  }
  else
  {
    object_id = ID_FIELD->get<int64_t>();
  }

  // The selected id must have been an object in the captured snapshot and
  // must still be active now (re-check under the objects mutex closes the
  // snapshot-to-validation window).
  if (!outcome.snapshot_object_ids.contains(object_id))
  {
    return std::unexpected{
      fmt::format("model reported object {} which is not in the scene snapshot", object_id)};
  }
  {
    const std::scoped_lock LOCK{m_objects_mutex};
    if (!m_cached_objects.contains(object_id))
    {
      return std::unexpected{
        fmt::format("model reported object {} which is no longer active", object_id)};
    }
  }

  result->success   = true;
  result->object_id = object_id;
  return result;
}

// ── PromptLLM action ──────────────────────────────────────────────────────────

rclcpp_action::GoalResponse
SceneGraphQueryNode::handlePromptLLMGoal(const rclcpp_action::GoalUUID& /*uuid*/,
                                         std::shared_ptr<const PromptLLMAction::Goal> goal)
{
  if (goal->query.empty())
  {
    RCLCPP_WARN(get_logger(), "Rejecting PromptLLM goal: empty query");
    return rclcpp_action::GoalResponse::REJECT;
  }
  {
    const std::scoped_lock LOCK{m_graph_mutex};
    if (!m_last_graph)
    {
      RCLCPP_WARN(get_logger(), "Rejecting PromptLLM goal: no scene graph received yet");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  const std::scoped_lock LOCK{m_prompt_llm_mutex};
  if (m_prompt_llm_stopping || m_prompt_llm_active)
  {
    RCLCPP_WARN(get_logger(), "Rejecting PromptLLM goal: another PromptLLM goal is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  m_prompt_llm_active = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
SceneGraphQueryNode::handlePromptLLMCancel(const std::shared_ptr<PromptLLMGoal>& /*goal_handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SceneGraphQueryNode::handlePromptLLMAccepted(const std::shared_ptr<PromptLLMGoal>& goal_handle)
{
  // Join the preceding worker outside the action mutex before replacing it.
  std::jthread completed_worker;
  {
    const std::scoped_lock LOCK{m_prompt_llm_mutex};
    if (!m_prompt_llm_stopping)
    {
      completed_worker = std::move(m_prompt_llm_worker);
    }
  }
  if (completed_worker.joinable())
  {
    completed_worker.join();
  }

  // Final locked stop check before replacement: a goal caught during teardown
  // clears the active flag and receives no result.
  const std::scoped_lock LOCK{m_prompt_llm_mutex};
  if (m_prompt_llm_stopping)
  {
    m_prompt_llm_active = false;
    return;
  }
  m_prompt_llm_worker = std::jthread([this, goal_handle](std::stop_token stop) {
    executePromptLLM(goal_handle, std::move(stop));
    const std::scoped_lock LOCK{m_prompt_llm_mutex};
    m_prompt_llm_active = false;
  });
}

void SceneGraphQueryNode::executePromptLLM(const std::shared_ptr<PromptLLMGoal>& goal_handle,
                                           std::stop_token stop)
{
  const auto GOAL = goal_handle->get_goal();
  RCLCPP_INFO(get_logger(), "PromptLLM: '%s'", GOAL->query.c_str());

  const ExpandCallback ON_EXPAND = [goal_handle](const std::vector<int64_t>& ids) {
    auto fb               = std::make_shared<PromptLLMAction::Feedback>();
    fb->expanded_node_ids = ids;
    fb->status            = fmt::format("Inspected {} node(s)", ids.size());
    goal_handle->publish_feedback(fb);
  };

  // Same interruption semantics as FindObject: cancel and shutdown both stop
  // the agent; only an accepted cancellation may terminalize as canceled.
  const StopCheck SHOULD_STOP = [goal_handle, &stop] {
    return goal_handle->is_canceling() || stop.stop_requested();
  };

  // Explicit completion outcome (see executeFindObject): computing the answer
  // is separate from the one terminal decision at the completion point below.
  enum class Completion : std::uint8_t
  {
    SUCCEED,
    ABORT
  };

  auto result = std::make_shared<PromptLLMAction::Result>();

  Completion completion = Completion::ABORT;
  std::string abort_reason;

  try
  {
    // Cheap pre-flight: skip the paid LLM call when already cancelled/stopping.
    if (!SHOULD_STOP())
    {
      auto outcome = runAgent("prompt_llm/system_prompt", GOAL->query, ON_EXPAND, SHOULD_STOP);
      if (outcome)
      {
        // Reject free-form answers computed against outdated membership.
        std::uint64_t epoch_now = 0;
        {
          const std::scoped_lock LOCK{m_objects_mutex};
          epoch_now = m_objects_epoch;
        }
        if (epoch_now == outcome->membership_epoch)
        {
          result->success  = true;
          result->message  = "ok";
          result->response = outcome->text;
          completion       = Completion::SUCCEED;
        }
        else
        {
          abort_reason = "object membership changed during the query (stale result)";
        }
      }
      else
      {
        m_action_errors.increment(outcome.error());
        abort_reason = outcome.error();
      }
    }
  }
  catch (const AgentStopped&)
  {
    // The agent stopped at a tool boundary; the completion point below decides
    // whether that is a client cancellation or a silent teardown.
    abort_reason = "Interrupted";
  }
  catch (const std::exception& e)
  {
    abort_reason = std::string{"internal error: "} + e.what();
  }
  catch (...)
  {
    abort_reason = "unknown internal error";
  }

  // Single completion point (see executeFindObject).
  if (stop.stop_requested() || !rclcpp::ok(get_node_base_interface()->get_context()))
  {
    return;
  }
  if (goal_handle->is_canceling())
  {
    // Uniform cancel payload (see executeFindObject): no computed-success
    // fields may leak into a CANCELED result.
    result->success = false;
    result->response.clear();
    result->message = "Cancelled";
    guardedTerminal([&] { goal_handle->canceled(result); });
    return;
  }
  if (completion == Completion::ABORT)
  {
    result->message = abort_reason;
    guardedTerminal([&] { goal_handle->abort(result); });
    RCLCPP_WARN(get_logger(), "PromptLLM aborted: %s", abort_reason.c_str());
    return;
  }
  guardedTerminal([&] { goal_handle->succeed(result); });
  RCLCPP_INFO(get_logger(), "PromptLLM done");
}

void SceneGraphQueryNode::onConnectivityDiagnostic(
  diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  const bool service_ready = m_visuals_client->service_is_ready();

  bool has_graph = false;
  {
    const std::scoped_lock LOCK{m_graph_mutex};
    has_graph = m_last_graph.has_value();
  }
  size_t cached = 0;
  {
    const std::scoped_lock LOCK{m_objects_mutex};
    cached = m_cached_objects.size();
  }

  stat.add("Has scene graph", has_graph);
  stat.add("Cached objects", cached);
  stat.add("GetObjectVisuals ready", service_ready);
  m_llm_errors.report(stat, "LLM");
  m_service_errors.report(stat, "Service");
  m_action_errors.report(stat, "Action");

  stat.add("LLM endpoint", m_params.server_url);
  const auto HEALTH = makeClient().isHealthy();
  stat.add("LLM reachable", HEALTH.has_value());
  if (!HEALTH)
  {
    stat.add("LLM error", HEALTH.error().message);
  }

  if (!HEALTH)
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

} // namespace semantic_scene_forests_query

RCLCPP_COMPONENTS_REGISTER_NODE(semantic_scene_forests_query::SceneGraphQueryNode)
