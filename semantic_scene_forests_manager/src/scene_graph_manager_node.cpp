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

#include "scene_graph_manager_node.hpp"

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <ranges>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp_components/register_node_macro.hpp>

namespace semantic_scene_forests_manager {

using SceneGraphMsg    = semantic_scene_forests_msgs::msg::SceneGraph;
using AddSceneContext  = semantic_scene_forests_msgs::srv::AddSceneContext;
using ManagedObject    = semantic_scene_forests_object_msgs::msg::ManagedObject;
using ObjectVisuals    = semantic_scene_forests_object_msgs::msg::ObjectVisuals;
using GetObjects       = semantic_scene_forests_object_msgs::srv::GetObjects;
using GetObjectVisuals = semantic_scene_forests_object_msgs::srv::GetObjectVisuals;
using BlockObject      = semantic_scene_forests_object_msgs::srv::BlockObject;
using UnblockObject    = semantic_scene_forests_object_msgs::srv::UnblockObject;
using EditObject       = semantic_scene_forests_object_msgs::srv::EditObject;

namespace {

//! Crops of the requested objects; all visible objects if `ids` is empty. Unknown and blocked
//! ids are skipped.
std::vector<ObjectVisuals> visibleCrops(const Tx& tx, std::vector<int64_t> ids)
{
  if (ids.empty())
  {
    for (const auto& record : tx.visibleObjects())
    {
      ids.push_back(record.id);
    }
  }
  std::vector<ObjectVisuals> visuals;
  for (const int64_t ID : ids)
  {
    if (const auto RECORD = tx.object(ID); RECORD && !RECORD->blocked)
    {
      ObjectVisuals entry;
      entry.object_id   = ID;
      entry.image_crops = tx.crops(ID);
      visuals.push_back(std::move(entry));
    }
  }
  return visuals;
}

// Never destroyed, not a member: the node's callback groups keep weak references to entities the
// workers created, and ~Node releases them after all members are gone. Unloading the worker
// libraries before that left those references pointing into unmapped code (segfault on shutdown);
// a static loader instead corrupted the heap during exit-time destruction. The worker libraries
// stay loaded until the process exits.
pluginlib::ClassLoader<IWorker>& workerLoader()
{
  static auto* const LOADER_ = new pluginlib::ClassLoader<IWorker>(
    "semantic_scene_forests_manager", "semantic_scene_forests_manager::IWorker");
  return *LOADER_;
}

} // namespace

SceneGraphManagerNode::SceneGraphManagerNode(const rclcpp::NodeOptions& options)
  : Node("scene_graph_manager", options)
  , m_params(scene_graph_manager::ParamListener(get_node_parameters_interface()).get_params())
  , m_diagnostics(this)
{
  m_diagnostics.setHardwareID("scene_graph_manager");
  m_diagnostics.add("database", this, &SceneGraphManagerNode::onDiagnostic);

  m_scene_graph_pub =
    create_publisher<SceneGraphMsg>("scene_graph", rclcpp::QoS(1).reliable().transient_local());

  m_read_group      = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  m_get_objects_srv = create_service<GetObjects>(
    "get_objects",
    [this](const GetObjects::Request::SharedPtr& /*request*/,
           const GetObjects::Response::SharedPtr& response) {
      for (auto& record : m_db.read([](const Tx& tx) { return tx.visibleObjects(); }))
      {
        ManagedObject object;
        object.id              = record.id;
        object.class_name      = std::move(record.class_name);
        object.detection       = std::move(record.detection);
        object.header.stamp    = record.last_updated;
        object.header.frame_id = m_params.frame_id;
        object.source          = record.source;
        object.last_updated    = record.last_updated;
        object.last_seen       = record.last_seen;
        response->objects.push_back(std::move(object));
      }
    },
    rclcpp::ServicesQoS(),
    m_read_group);

  m_get_object_visuals_srv = create_service<GetObjectVisuals>(
    "get_object_visuals",
    [this](const GetObjectVisuals::Request::SharedPtr& request,
           const GetObjectVisuals::Response::SharedPtr& response) {
      response->visuals =
        m_db.read([&](const Tx& tx) { return visibleCrops(tx, request->object_ids); });
    },
    rclcpp::ServicesQoS(),
    m_read_group);

  m_block_object_srv = create_service<BlockObject>(
    "block_object",
    [this](const BlockObject::Request::SharedPtr& request,
           const BlockObject::Response::SharedPtr& response) {
      response->success = m_db.write([&](Tx& tx) { return tx.block(request->object_id); });
      response->message = response->success ? "Object blocked" : "Object ID not found";
      RCLCPP_INFO(
        get_logger(), "BlockObject id=%ld: %s", request->object_id, response->message.c_str());
    });

  m_unblock_object_srv = create_service<UnblockObject>(
    "unblock_object",
    [this](const UnblockObject::Request::SharedPtr& request,
           const UnblockObject::Response::SharedPtr& response) {
      response->success = m_db.write([&](Tx& tx) { return tx.unblock(request->object_id); });
      response->message = response->success ? "Object unblocked" : "Object ID not blocked";
      RCLCPP_INFO(
        get_logger(), "UnblockObject id=%ld: %s", request->object_id, response->message.c_str());
    });

  m_edit_object_srv = create_service<EditObject>(
    "edit_object",
    [this](const EditObject::Request::SharedPtr& request,
           const EditObject::Response::SharedPtr& response) {
      const auto NOW     = now();
      const auto FAILURE = m_db.write([&](Tx& tx) { return tx.edit(*request, NOW); });
      response->success  = FAILURE == EditFailure::SUCCESS;
      switch (FAILURE)
      {
        case EditFailure::SUCCESS:
          response->message = "Object edited successfully";
          break;
        case EditFailure::UNKNOWN_OBJECT:
          response->message = "Object ID not found or blocked";
          break;
        case EditFailure::INVALID_CROP_INDEX:
          response->message = "Invalid crop index in remove_crop_indices; edit rejected";
          break;
      }
      RCLCPP_INFO(
        get_logger(), "EditObject id=%ld: %s", request->object_id, response->message.c_str());
    });

  m_add_scene_context_srv = create_service<AddSceneContext>(
    "add_scene_context",
    [this](const AddSceneContext::Request::SharedPtr& request,
           const AddSceneContext::Response::SharedPtr& response) {
      const auto [ID, CREATED] =
        m_db.write([&](Tx& tx) { return tx.addContext(request->node_id, request->context); });
      response->success = ID != 0;
      if (!response->success)
      {
        response->message = "Node does not exist, is a blocked object, or the context is invalid";
        return;
      }
      response->context_node_id = ID;
      response->created         = CREATED;
      response->message         = CREATED ? "Context added" : "Context already exists";
    });

  // Load and initialize every worker before starting any.
  for (const auto& name : m_params.workers)
  {
    const auto TYPE = declare_parameter<std::string>("worker_types." + name);
    m_workers.push_back(workerLoader().createUniqueInstance(TYPE));
    m_workers.back()->initialize(*this, m_db, name, m_diagnostics);
    RCLCPP_INFO(get_logger(), "Loaded worker '%s' (%s)", name.c_str(), TYPE.c_str());
  }
  for (auto& worker : m_workers)
  {
    worker->start();
  }

  m_publish_thread = std::jthread([this](const std::stop_token& stop) { publishLoop(stop); });
}

SceneGraphManagerNode::~SceneGraphManagerNode()
{
  m_publish_thread.request_stop();
  for (auto& worker : std::views::reverse(m_workers))
  {
    worker->stop();
  }
}

void SceneGraphManagerNode::publishLoop(const std::stop_token& stop)
{
  const auto PERIOD = std::chrono::duration<double>(1.0 / m_params.publish_max_rate_hz);
  std::mutex mutex;
  std::condition_variable_any wake;
  std::optional<int64_t> published_count;
  while (!stop.stop_requested())
  {
    // Read the count before building: a change made meanwhile triggers the next publish.
    if (const int64_t COUNT = m_db.changeCount(); COUNT != published_count)
    {
      published_count = COUNT;
      // An escaping exception would terminate the process; a failed publish waits for the next
      // change.
      std::string error;
      try
      {
        publishSceneGraph();
      }
      catch (const std::exception& e)
      {
        error = e.what();
      }
      catch (...)
      {
        error = "unknown exception";
      }
      if (!error.empty())
      {
        RCLCPP_ERROR(get_logger(), "Publishing the scene graph failed: %s", error.c_str());
      }
      const std::scoped_lock LOCK{m_publish_stats_mutex};
      m_publish_count += error.empty() ? 1 : 0;
      m_publish_error = std::move(error);
    }
    std::unique_lock lock{mutex};
    wake.wait_for(lock, stop, PERIOD, [] { return false; });
  }
}

void SceneGraphManagerNode::publishSceneGraph()
{
  auto [graph, objects] =
    m_db.read([](const Tx& tx) { return std::pair{tx.visibleGraph(), tx.visibleObjects()}; });
  auto msg = graph.toMsg();

  std::unordered_map<int64_t, const ObjectRecord*> by_id;
  for (const auto& record : objects)
  {
    by_id.emplace(record.id, &record);
  }
  for (auto& node : msg->object_nodes)
  {
    if (const auto IT = by_id.find(node.id); IT != by_id.end())
    {
      node.source       = IT->second->source;
      node.last_updated = IT->second->last_updated;
      node.last_seen    = IT->second->last_seen;
    }
  }
  msg->header.stamp    = now();
  msg->header.frame_id = m_params.frame_id;
  m_scene_graph_pub->publish(std::move(msg));
}

void SceneGraphManagerNode::onDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  const auto COUNTS = m_db.read([](const Tx& tx) { return tx.counts(); });
  stat.add("Objects", COUNTS.objects);
  stat.add("Blocked objects", COUNTS.blocked_objects);
  stat.add("Concepts", COUNTS.concepts);
  stat.add("Edges", COUNTS.edges);
  stat.add("Changed rows", m_db.changeCount());
  stat.add("Workers", m_workers.size());
  const std::scoped_lock LOCK{m_publish_stats_mutex};
  stat.add("Published graphs", m_publish_count);
  stat.add("Subscribers", m_scene_graph_pub->get_subscription_count());
  if (!m_publish_error.empty())
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                 "Publishing the scene graph failed: " + m_publish_error);
    return;
  }
  stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::OK,
                "%ld objects (%ld blocked), %ld concepts, %ld edges",
                COUNTS.objects,
                COUNTS.blocked_objects,
                COUNTS.concepts,
                COUNTS.edges);
}

} // namespace semantic_scene_forests_manager

RCLCPP_COMPONENTS_REGISTER_NODE(semantic_scene_forests_manager::SceneGraphManagerNode)
