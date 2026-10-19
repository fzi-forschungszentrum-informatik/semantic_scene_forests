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

#include <utility>

#include "semantic_scene_forests_object_manager/object_manager_node.hpp"

namespace semantic_scene_forests_object_manager {

using ObjectCandidate    = semantic_scene_forests_object_msgs::msg::ObjectCandidate;
using ManagedObjectArray = semantic_scene_forests_object_msgs::msg::ManagedObjectArray;
using GetObjects         = semantic_scene_forests_object_msgs::srv::GetObjects;
using GetObjectVisuals   = semantic_scene_forests_object_msgs::srv::GetObjectVisuals;
using BlockObject        = semantic_scene_forests_object_msgs::srv::BlockObject;
using EditObject         = semantic_scene_forests_object_msgs::srv::EditObject;
using UnblockObject      = semantic_scene_forests_object_msgs::srv::UnblockObject;

ObjectManagerNode::ObjectManagerNode(const rclcpp::NodeOptions& options)
  : Node("object_manager", options)
  , m_diagnostic_updater(this)
{
  m_param_listener =
    std::make_shared<object_manager::ParamListener>(this->get_node_parameters_interface());
  m_params = m_param_listener->get_params();

  m_diagnostic_updater.setHardwareID("object_manager");
  m_diagnostic_updater.add("Health", this, &ObjectManagerNode::onHealthDiagnostic);

  auto pub_qos  = rclcpp::QoS(1).reliable().transient_local();
  m_managed_pub = this->create_publisher<ManagedObjectArray>("managed_objects", pub_qos);

  m_candidate_sub = this->create_subscription<ObjectCandidate>(
    "object_candidates",
    rclcpp::SensorDataQoS(),
    [this](const ObjectCandidate::ConstSharedPtr& candidate) { onObjectCandidate(candidate); });

  m_candidate_freq = std::make_unique<diagnostic_updater::HeaderlessTopicDiagnostic>(
    "object_candidates",
    m_diagnostic_updater,
    diagnostic_updater::FrequencyStatusParam(&m_min_freq, &m_max_freq, 0.1, 5));

  m_get_objects_srv =
    this->create_service<GetObjects>("get_objects",
                                     [this](const GetObjects::Request::SharedPtr& request,
                                            GetObjects::Response::SharedPtr response) {
                                       onGetObjects(request, std::move(response));
                                     });

  m_block_object_srv =
    this->create_service<BlockObject>("block_object",
                                      [this](const BlockObject::Request::SharedPtr& request,
                                             BlockObject::Response::SharedPtr response) {
                                        onBlockObject(request, std::move(response));
                                      });

  m_edit_object_srv =
    this->create_service<EditObject>("edit_object",
                                     [this](const EditObject::Request::SharedPtr& request,
                                            EditObject::Response::SharedPtr response) {
                                       onEditObject(request, std::move(response));
                                     });

  m_unblock_object_srv =
    this->create_service<UnblockObject>("unblock_object",
                                        [this](const UnblockObject::Request::SharedPtr& request,
                                               UnblockObject::Response::SharedPtr response) {
                                          onUnblockObject(request, std::move(response));
                                        });

  m_get_object_visuals_srv = this->create_service<GetObjectVisuals>(
    "get_object_visuals",
    [this](const GetObjectVisuals::Request::SharedPtr& request,
           GetObjectVisuals::Response::SharedPtr response) {
      onGetObjectVisuals(request, std::move(response));
    });

  m_publish_timer =
    this->create_wall_timer(std::chrono::duration<double>(1.0 / m_params.publish_frequency_hz),
                            [this]() { publishManagedObjects(); });

  RCLCPP_INFO(this->get_logger(),
              "ObjectManagerNode started. frame_id=%s, publish_frequency=%.1f Hz",
              m_params.frame_id.c_str(),
              m_params.publish_frequency_hz);
}

void ObjectManagerNode::onObjectCandidate(const ObjectCandidate::ConstSharedPtr& candidate)
{
  m_candidate_freq->tick();

  const bool ACCEPTED = m_manager.processObjectCandidate(*candidate, this->now());
  if (ACCEPTED)
  {
    RCLCPP_DEBUG(
      this->get_logger(), "Updated object %ld (%s)", candidate->id, candidate->class_name.c_str());
  }
  else
  {
    RCLCPP_DEBUG(this->get_logger(), "Dropped blocked object %ld", candidate->id);
  }
}

void ObjectManagerNode::onGetObjects(const GetObjects::Request::SharedPtr& /*request*/,
                                     const GetObjects::Response::SharedPtr& response)
{
  response->objects = m_manager.getManagedObjects(m_params.frame_id);
  RCLCPP_DEBUG(this->get_logger(), "GetObjects: returned %zu objects", response->objects.size());
}

void ObjectManagerNode::onBlockObject(const BlockObject::Request::SharedPtr& request,
                                      const BlockObject::Response::SharedPtr& response)
{
  RCLCPP_INFO(this->get_logger(), "BlockObject: id=%ld", request->object_id);
  if (m_manager.blockObject(request->object_id))
  {
    response->success = true;
    response->message = "Object blocked";
    publishManagedObjects();
  }
  else
  {
    response->success = false;
    response->message = "Object ID not found";
    RCLCPP_WARN(this->get_logger(), "BlockObject failed: id=%ld not found", request->object_id);
  }
}

void ObjectManagerNode::onEditObject(const EditObject::Request::SharedPtr& request,
                                     const EditObject::Response::SharedPtr& response)
{
  RCLCPP_INFO(this->get_logger(), "EditObject: id=%ld", request->object_id);
  ObjectManager::EditFailure failure;
  if (m_manager.editObject(*request, this->now(), failure))
  {
    response->success = true;
    response->message = "Object edited successfully";
    publishManagedObjects();
    return;
  }

  response->success = false;
  switch (failure)
  {
    case ObjectManager::EditFailure::UNKNOWN_OBJECT:
      response->message = "Object ID not found or blocked";
      break;
    case ObjectManager::EditFailure::INVALID_CROP_INDEX:
      response->message = "Invalid crop index in remove_crop_indices; edit rejected";
      break;
    case ObjectManager::EditFailure::SUCCESS:
      break;
  }
  RCLCPP_WARN(this->get_logger(),
              "EditObject failed: id=%ld (%s)",
              request->object_id,
              response->message.c_str());
}

void ObjectManagerNode::onUnblockObject(const UnblockObject::Request::SharedPtr& request,
                                        const UnblockObject::Response::SharedPtr& response)
{
  RCLCPP_INFO(this->get_logger(), "UnblockObject: id=%ld", request->object_id);
  if (m_manager.unblockObject(request->object_id))
  {
    response->success = true;
    response->message = "Object unblocked";
    publishManagedObjects();
  }
  else
  {
    response->success = false;
    response->message = "Object ID not blocked";
    RCLCPP_WARN(this->get_logger(), "UnblockObject failed: id=%ld not blocked", request->object_id);
  }
}

void ObjectManagerNode::onGetObjectVisuals(const GetObjectVisuals::Request::SharedPtr& request,
                                           const GetObjectVisuals::Response::SharedPtr& response)
{
  response->visuals = m_manager.getObjectVisuals(request->object_ids);
  RCLCPP_DEBUG(
    this->get_logger(), "GetObjectVisuals: returned %zu entries", response->visuals.size());
}

void ObjectManagerNode::publishManagedObjects()
{
  auto msg =
    std::make_unique<ManagedObjectArray>(m_manager.buildMessage(m_params.frame_id, this->now()));
  m_managed_pub->publish(std::move(msg));
}

void ObjectManagerNode::onHealthDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  const auto OBJECTS = m_manager.getManagedObjects(m_params.frame_id);

  stat.add("Managed objects", OBJECTS.size());
  stat.add("Blocked objects", m_manager.getBlockedObjectCount());
  stat.add("Publish frequency (Hz)", m_params.publish_frequency_hz);
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
               "Managing " + std::to_string(OBJECTS.size()) + " objects");
}

} // namespace semantic_scene_forests_object_manager

RCLCPP_COMPONENTS_REGISTER_NODE(semantic_scene_forests_object_manager::ObjectManagerNode)
