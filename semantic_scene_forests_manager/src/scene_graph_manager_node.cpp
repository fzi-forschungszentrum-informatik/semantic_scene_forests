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
#include <ranges>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace semantic_scene_forests_manager {

using fzi::semantic_scene_forests::ObjectNode;
using fzi::semantic_scene_forests::SceneGraph;
using SceneGraphMsg    = semantic_scene_forests_msgs::msg::SceneGraph;
using AddConcept       = semantic_scene_forests_msgs::srv::AddConcept;
using AddEdge          = semantic_scene_forests_msgs::srv::AddEdge;
using ObjectVisuals    = semantic_scene_forests_object_msgs::msg::ObjectVisuals;
using GetObjects       = semantic_scene_forests_object_msgs::srv::GetObjects;
using GetObjectVisuals = semantic_scene_forests_object_msgs::srv::GetObjectVisuals;
using DeleteObject     = semantic_scene_forests_object_msgs::srv::DeleteObject;
using EditObject       = semantic_scene_forests_object_msgs::srv::EditObject;

namespace {

//! Crops of the requested objects as raw images; all objects if `ids` is empty. Unknown ids are
//! skipped, undecodable crops are logged and skipped.
std::vector<ObjectVisuals>
objectCrops(const SceneGraph& graph, const std::vector<int64_t>& ids, const rclcpp::Logger& logger)
{
  std::vector<ObjectVisuals> visuals;
  for (const auto& object : graph.getObjectNodes())
  {
    if (!ids.empty() && std::ranges::find(ids, object.id) == ids.end())
    {
      continue;
    }
    ObjectVisuals entry;
    entry.object_id = object.id;
    for (const auto& crop : object.crops)
    {
      try
      {
        entry.image_crops.push_back(*cv_bridge::toCvCopy(*crop, "bgr8")->toImageMsg());
      }
      catch (const cv_bridge::Exception& e)
      {
        RCLCPP_WARN(logger, "Object %ld: skipping undecodable crop: %s", object.id, e.what());
      }
    }
    visuals.push_back(std::move(entry));
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
  , m_graph(std::make_shared<SceneGraph>())
  , m_params(scene_graph_manager::ParamListener(get_node_parameters_interface()).get_params())
  , m_diagnostics(this)
{
  m_diagnostics.setHardwareID("scene_graph_manager");
  m_diagnostics.add("scene_graph", this, &SceneGraphManagerNode::onDiagnostic);

  m_scene_graph_pub =
    create_publisher<SceneGraphMsg>("scene_graph", rclcpp::QoS(1).reliable().transient_local());

  // Read services get their own group so they don't queue behind writes.
  m_read_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  m_services.push_back(create_service<GetObjects>(
    "get_objects",
    [this](const GetObjects::Request::SharedPtr& /*request*/,
           const GetObjects::Response::SharedPtr& response) {
      response->objects = std::move(m_graph->toMsg()->object_nodes);
    },
    rclcpp::ServicesQoS(),
    m_read_group));

  m_services.push_back(create_service<GetObjectVisuals>(
    "get_object_visuals",
    [this](const GetObjectVisuals::Request::SharedPtr& request,
           const GetObjectVisuals::Response::SharedPtr& response) {
      response->visuals = objectCrops(*m_graph, request->object_ids, get_logger());
    },
    rclcpp::ServicesQoS(),
    m_read_group));

  m_services.push_back(create_service<DeleteObject>(
    "delete_object",
    [this](const DeleteObject::Request::SharedPtr& request,
           const DeleteObject::Response::SharedPtr& response) {
      // removeNode takes any node id; this service only deletes objects.
      const auto NODE = m_graph->findNode(request->object_id);
      if (!NODE || !std::holds_alternative<ObjectNode>(*NODE))
      {
        response->message = "Object ID not found";
      }
      else if (const auto RESULT = m_graph->removeNode(request->object_id); !RESULT)
      {
        response->message = RESULT.error().message;
      }
      else
      {
        response->success = true;
        response->message = "Object deleted";
      }
      RCLCPP_INFO(
        get_logger(), "DeleteObject id=%ld: %s", request->object_id, response->message.c_str());
    }));

  m_services.push_back(create_service<EditObject>(
    "edit_object",
    [this](const EditObject::Request::SharedPtr& request,
           const EditObject::Response::SharedPtr& response) {
      const auto RESULT = m_graph->setObjectClass(request->object_id, request->class_name);
      response->success = RESULT.has_value();
      response->message = RESULT ? "Object edited successfully" : RESULT.error().message;
      RCLCPP_INFO(
        get_logger(), "EditObject id=%ld: %s", request->object_id, response->message.c_str());
    }));

  m_services.push_back(create_service<AddConcept>(
    "add_concept",
    [this](const AddConcept::Request::SharedPtr& request,
           const AddConcept::Response::SharedPtr& response) {
      const auto RESULT =
        m_graph->addConceptEdge(request->source_id, request->relation, request->label);
      response->success = RESULT.has_value();
      if (!RESULT)
      {
        response->message = RESULT.error().message;
        return;
      }
      response->concept_id = RESULT->id;
      response->created    = RESULT->created;
      response->message    = RESULT->created ? "Concept added" : "Concept already exists";
    }));

  m_services.push_back(create_service<AddEdge>(
    "add_edge",
    [this](const AddEdge::Request::SharedPtr& request,
           const AddEdge::Response::SharedPtr& response) {
      const auto RESULT = m_graph->addEdge(request->source_id, request->target_id, request->label);
      response->success = RESULT.has_value();
      response->message = RESULT ? "Edge added" : RESULT.error().message;
    }));

  // Load and initialize every worker before starting any.
  for (const auto& name : m_params.workers)
  {
    const auto TYPE = declare_parameter<std::string>("worker_types." + name);
    m_workers.push_back(workerLoader().createUniqueInstance(TYPE));
    m_workers.back()->initialize(*this, m_graph, name, m_diagnostics);
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
  while (!stop.stop_requested())
  {
    // An escaping exception would terminate the process; a failed publish waits for the next tick.
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
    {
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
  auto msg             = m_graph->toMsg();
  msg->header.stamp    = now();
  msg->header.frame_id = m_params.frame_id;
  m_scene_graph_pub->publish(std::move(msg));
}

void SceneGraphManagerNode::onDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  const auto OBJECTS  = m_graph->getObjectNodes().size();
  const auto CONCEPTS = m_graph->getConceptNodes().size();
  const auto EDGES    = m_graph->getEdges().size();
  stat.add("Objects", OBJECTS);
  stat.add("Concepts", CONCEPTS);
  stat.add("Edges", EDGES);
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
                "%zu objects, %zu concepts, %zu edges",
                OBJECTS,
                CONCEPTS,
                EDGES);
}

} // namespace semantic_scene_forests_manager

RCLCPP_COMPONENTS_REGISTER_NODE(semantic_scene_forests_manager::SceneGraphManagerNode)
