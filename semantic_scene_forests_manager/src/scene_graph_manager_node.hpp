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

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <semantic_scene_forests_msgs/msg/scene_graph.hpp>
#include <semantic_scene_forests_msgs/srv/add_concept.hpp>
#include <semantic_scene_forests_msgs/srv/add_edge.hpp>
#include <semantic_scene_forests_object_msgs/srv/delete_object.hpp>
#include <semantic_scene_forests_object_msgs/srv/edit_object.hpp>
#include <semantic_scene_forests_object_msgs/srv/get_object_visuals.hpp>
#include <semantic_scene_forests_object_msgs/srv/get_objects.hpp>

#include "semantic_scene_forests/scene_graph.hpp"
#include "semantic_scene_forests_manager/scene_graph_manager_parameters.hpp"
#include "semantic_scene_forests_manager/worker_interface.hpp"

namespace semantic_scene_forests_manager {

//! Owns the shared SceneGraph, serves the graph services, publishes `scene_graph` at
//! `publish_max_rate_hz` and hosts the configured worker plugins.
class SceneGraphManagerNode : public rclcpp::Node
{
public:
  explicit SceneGraphManagerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~SceneGraphManagerNode() override;
  SceneGraphManagerNode(const SceneGraphManagerNode&)            = delete;
  SceneGraphManagerNode& operator=(const SceneGraphManagerNode&) = delete;
  SceneGraphManagerNode(SceneGraphManagerNode&&)                 = delete;
  SceneGraphManagerNode& operator=(SceneGraphManagerNode&&)      = delete;

  //! Test-only access to the shared graph.
  const std::shared_ptr<fzi::semantic_scene_forests::SceneGraph>& graph() const { return m_graph; }

private:
  void publishLoop(const std::stop_token& stop);
  void publishSceneGraph();
  void onDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);

  // Declared first: destroyed after the workers and the publisher thread that use it.
  std::shared_ptr<fzi::semantic_scene_forests::SceneGraph> m_graph;

  scene_graph_manager::Params m_params;

  // Created by workerLoader() in the .cpp, which outlives this node.
  std::vector<pluginlib::UniquePtr<IWorker>> m_workers;

  // Declared after the workers: destroyed first, so no diagnostic callback outlives a worker.
  diagnostic_updater::Updater m_diagnostics;
  std::mutex m_publish_stats_mutex;
  int64_t m_publish_count{0};
  //! Empty after a successful publish.
  std::string m_publish_error;

  rclcpp::Publisher<semantic_scene_forests_msgs::msg::SceneGraph>::SharedPtr m_scene_graph_pub;
  // Read services get their own group so they don't queue behind writes.
  rclcpp::CallbackGroup::SharedPtr m_read_group;
  std::vector<rclcpp::ServiceBase::SharedPtr> m_services;

  // Declared last: joined first on destruction.
  std::jthread m_publish_thread;
};

} // namespace semantic_scene_forests_manager
