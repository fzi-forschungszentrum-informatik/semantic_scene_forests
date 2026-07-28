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

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <semantic_scene_forests_object_msgs/msg/managed_object_array.hpp>
#include <semantic_scene_forests_object_msgs/msg/object_candidate.hpp>
#include <semantic_scene_forests_object_msgs/srv/block_object.hpp>
#include <semantic_scene_forests_object_msgs/srv/edit_object.hpp>
#include <semantic_scene_forests_object_msgs/srv/get_object_visuals.hpp>
#include <semantic_scene_forests_object_msgs/srv/get_objects.hpp>
#include <semantic_scene_forests_object_msgs/srv/unblock_object.hpp>
#include <semantic_scene_forests_utils/diagnostic_utils.hpp>

#include "semantic_scene_forests_object_manager/object_manager.hpp"
#include "semantic_scene_forests_object_manager/object_manager_parameters.hpp"

namespace semantic_scene_forests_object_manager {

class ObjectManagerNode : public rclcpp::Node
{
public:
  explicit ObjectManagerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void onObjectCandidate(
    const semantic_scene_forests_object_msgs::msg::ObjectCandidate::ConstSharedPtr& candidate);

  void onGetObjects(
    const semantic_scene_forests_object_msgs::srv::GetObjects::Request::SharedPtr& request,
    const semantic_scene_forests_object_msgs::srv::GetObjects::Response::SharedPtr& response);

  void onBlockObject(
    const semantic_scene_forests_object_msgs::srv::BlockObject::Request::SharedPtr& request,
    const semantic_scene_forests_object_msgs::srv::BlockObject::Response::SharedPtr& response);

  void onEditObject(
    const semantic_scene_forests_object_msgs::srv::EditObject::Request::SharedPtr& request,
    const semantic_scene_forests_object_msgs::srv::EditObject::Response::SharedPtr& response);

  void onUnblockObject(
    const semantic_scene_forests_object_msgs::srv::UnblockObject::Request::SharedPtr& request,
    const semantic_scene_forests_object_msgs::srv::UnblockObject::Response::SharedPtr& response);

  void onGetObjectVisuals(
    const semantic_scene_forests_object_msgs::srv::GetObjectVisuals::Request::SharedPtr& request,
    const semantic_scene_forests_object_msgs::srv::GetObjectVisuals::Response::SharedPtr& response);

  void publishManagedObjects();

  void onHealthDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);

  ObjectManager m_manager;

  std::shared_ptr<object_manager::ParamListener> m_param_listener;
  object_manager::Params m_params;

  rclcpp::Subscription<semantic_scene_forests_object_msgs::msg::ObjectCandidate>::SharedPtr
    m_candidate_sub;
  rclcpp::Publisher<semantic_scene_forests_object_msgs::msg::ManagedObjectArray>::SharedPtr
    m_managed_pub;

  rclcpp::Service<semantic_scene_forests_object_msgs::srv::GetObjects>::SharedPtr m_get_objects_srv;
  rclcpp::Service<semantic_scene_forests_object_msgs::srv::BlockObject>::SharedPtr
    m_block_object_srv;
  rclcpp::Service<semantic_scene_forests_object_msgs::srv::EditObject>::SharedPtr m_edit_object_srv;
  rclcpp::Service<semantic_scene_forests_object_msgs::srv::UnblockObject>::SharedPtr
    m_unblock_object_srv;
  rclcpp::Service<semantic_scene_forests_object_msgs::srv::GetObjectVisuals>::SharedPtr
    m_get_object_visuals_srv;

  rclcpp::TimerBase::SharedPtr m_publish_timer;

  diagnostic_updater::Updater m_diagnostic_updater;
  double m_min_freq{0.1};
  double m_max_freq{1000.0};
  std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> m_candidate_freq;
};

} // namespace semantic_scene_forests_object_manager
