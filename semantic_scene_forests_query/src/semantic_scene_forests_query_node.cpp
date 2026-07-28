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
using namespace std::chrono_literals;

namespace {

// Hardcoded ROS interface names (intentionally NOT parameters).
constexpr const char* K_SCENE_GRAPH_TOPIC     = "scene_graph";
constexpr const char* K_MANAGED_OBJECTS_TOPIC = "managed_objects";
constexpr const char* K_GET_OBJECT_VISUALS    = "get_object_visuals";
constexpr const char* K_FIND_OBJECT_ACTION    = "find_object";
constexpr const char* K_PROMPT_LLM_ACTION     = "prompt_llm";

constexpr auto K_VISUALS_TIMEOUT = 10s;

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
  const std::scoped_lock LOCK{m_objects_mutex};
  m_cached_objects.clear();
  for (const auto& obj : msg->objects)
  {
    m_cached_objects[obj.id] = obj;
  }
}

// ── Shared helpers ──────────────────────────────────────────────────────────--

std::expected<std::unique_ptr<fzi::semantic_scene_forests::SceneGraph>, std::string>
SceneGraphQueryNode::snapshotGraph() const
{
  SceneGraphMsg msg;
  {
    const std::scoped_lock LOCK{m_graph_mutex};
    if (!m_last_graph)
    {
      return std::unexpected{"no scene graph received yet"};
    }
    msg = *m_last_graph;
  }

  auto graph = fzi::semantic_scene_forests::SceneGraph::fromMsg(msg);
  if (!graph)
  {
    return std::unexpected{
      fmt::format("failed to build graph snapshot: {}", graph.error().message)};
  }
  return std::move(*graph);
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

std::expected<std::string, std::string>
SceneGraphQueryNode::inspectObjectVisual(int64_t id, std::string_view focus)
{
  auto request        = std::make_shared<GetObjectVisuals::Request>();
  request->object_ids = {id};

  auto future = m_visuals_client->async_send_request(request);
  // NOTE: requires a multi-threaded executor (see launch file) — this blocks the
  // action thread until the service responds.
  if (future.wait_for(K_VISUALS_TIMEOUT) != std::future_status::ready)
  {
    return std::unexpected{fmt::format("/get_object_visuals timed out for object {}", id)};
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
  return oai::ClientBuilder{}
    .baseUrl(m_params.server_url)
    .apiKey(m_params.api_key)
    .model(m_params.model)
    .temperature(static_cast<float>(m_params.temperature))
    .maxTokens(m_params.max_tokens)
    .build();
}

std::expected<SceneGraphQueryNode::AgentOutcome, std::string> SceneGraphQueryNode::runAgent(
  const std::string& system_prompt_key, std::string_view request, const ExpandCallback& on_expand)
{
  auto snapshot = snapshotGraph();
  if (!snapshot)
  {
    return std::unexpected{snapshot.error()};
  }

  const std::string CATALOG = renderObjectCatalog(**snapshot);

  GraphAgentSession session(
    std::move(*snapshot),
    [this](int64_t id) { return lookupObject(id); },
    [this](int64_t id, std::string_view focus) { return inspectObjectVisual(id, focus); },
    on_expand,
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
  return AgentOutcome{.text = std::move(*answer), .expanded = session.expandedNodes()};
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
  const std::scoped_lock LOCK{m_graph_mutex};
  if (!m_last_graph)
  {
    RCLCPP_WARN(get_logger(), "Rejecting FindObject goal: no scene graph received yet");
    return rclcpp_action::GoalResponse::REJECT;
  }
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
  std::thread{[this, goal_handle] { executeFindObject(goal_handle); }}.detach();
}

void SceneGraphQueryNode::executeFindObject(const std::shared_ptr<FindObjectGoal>& goal_handle)
{
  const auto GOAL = goal_handle->get_goal();
  RCLCPP_INFO(get_logger(), "FindObject: '%s'", GOAL->object_description.c_str());

  const ExpandCallback ON_EXPAND = [goal_handle](const std::vector<int64_t>& ids) {
    auto fb               = std::make_shared<FindObjectAction::Feedback>();
    fb->expanded_node_ids = ids;
    fb->status            = fmt::format("Inspected {} node(s)", ids.size());
    goal_handle->publish_feedback(fb);
  };

  auto outcome = runAgent("find_object/system_prompt", GOAL->object_description, ON_EXPAND);

  auto result = std::make_shared<FindObjectAction::Result>();
  if (!outcome)
  {
    m_action_errors.increment(outcome.error());
    result->success   = false;
    result->message   = outcome.error();
    result->object_id = -1;
    goal_handle->abort(result);
    RCLCPP_ERROR(get_logger(), "FindObject failed: %s", outcome.error().c_str());
    return;
  }

  if (goal_handle->is_canceling())
  {
    result->success   = false;
    result->message   = "Cancelled";
    result->object_id = -1;
    goal_handle->canceled(result);
    return;
  }

  // The model is instructed to answer with {"found": bool, "object_id": int, "message": str}.
  const json PARSED = json::parse(extractJsonObject(outcome->text), nullptr, false);
  if (PARSED.is_discarded() || !PARSED.is_object())
  {
    result->success   = false;
    result->object_id = -1;
    result->message   = "Could not parse model output: " + outcome->text;
  }
  else
  {
    const bool FOUND = PARSED.value("found", false);
    result->success  = FOUND;
    result->object_id =
      FOUND ? PARSED.value("object_id", static_cast<int64_t>(-1)) : static_cast<int64_t>(-1);
    result->message = PARSED.value("message", outcome->text);
  }
  goal_handle->succeed(result);
  RCLCPP_INFO(get_logger(),
              "FindObject done: success=%d object_id=%ld",
              static_cast<int>(result->success),
              result->object_id);
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
  const std::scoped_lock LOCK{m_graph_mutex};
  if (!m_last_graph)
  {
    RCLCPP_WARN(get_logger(), "Rejecting PromptLLM goal: no scene graph received yet");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
SceneGraphQueryNode::handlePromptLLMCancel(const std::shared_ptr<PromptLLMGoal>& /*goal_handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SceneGraphQueryNode::handlePromptLLMAccepted(const std::shared_ptr<PromptLLMGoal>& goal_handle)
{
  std::thread{[this, goal_handle] { executePromptLLM(goal_handle); }}.detach();
}

void SceneGraphQueryNode::executePromptLLM(const std::shared_ptr<PromptLLMGoal>& goal_handle)
{
  const auto GOAL = goal_handle->get_goal();
  RCLCPP_INFO(get_logger(), "PromptLLM: '%s'", GOAL->query.c_str());

  const ExpandCallback ON_EXPAND = [goal_handle](const std::vector<int64_t>& ids) {
    auto fb               = std::make_shared<PromptLLMAction::Feedback>();
    fb->expanded_node_ids = ids;
    fb->status            = fmt::format("Inspected {} node(s)", ids.size());
    goal_handle->publish_feedback(fb);
  };

  auto outcome = runAgent("prompt_llm/system_prompt", GOAL->query, ON_EXPAND);

  auto result = std::make_shared<PromptLLMAction::Result>();
  if (!outcome)
  {
    m_action_errors.increment(outcome.error());
    result->success = false;
    result->message = outcome.error();
    goal_handle->abort(result);
    RCLCPP_ERROR(get_logger(), "PromptLLM failed: %s", outcome.error().c_str());
    return;
  }

  if (goal_handle->is_canceling())
  {
    result->success = false;
    result->message = "Cancelled";
    goal_handle->canceled(result);
    return;
  }

  result->success  = true;
  result->message  = "ok";
  result->response = outcome->text;
  goal_handle->succeed(result);
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
