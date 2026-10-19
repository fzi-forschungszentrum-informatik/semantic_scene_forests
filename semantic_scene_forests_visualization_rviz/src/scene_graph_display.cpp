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

#include "semantic_scene_forests_visualization_rviz/scene_graph_display.hpp"

#include <algorithm>
#include <functional>
#include <map>
#include <string>

#include <QMenu>
#include <QMetaObject>
#include <QMouseEvent>
#include <QVBoxLayout>

#include <OgreEntity.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/interaction/handler_manager_iface.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/interaction/selection_manager_iface.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_rendering/render_window.hpp"
#include "semantic_scene_forests_msgs/msg/scene_graph.hpp"
#include "semantic_scene_forests_object_msgs/msg/managed_object.hpp"
#include "semantic_scene_forests_visualization_rviz/image_viewer_dialog.hpp"
#include "semantic_scene_forests_visualization_rviz/scene_graph_object_selection_handler.hpp"

namespace semantic_scene_forests_visualization_rviz {

SceneGraphDisplay::SceneGraphDisplay() = default;

SceneGraphDisplay::~SceneGraphDisplay()
{
  // Remove event filter from render panel
  if (m_render_panel)
  {
    m_render_panel->getRenderWindow()->removeEventFilter(this);
  }

  // Clear visuals first (selection handlers, shapes, labels)
  clearVisuals();

  // Properly destroy scene node hierarchy using removeAndDestroyChild()
  // Child nodes must be destroyed by their parent
  if (m_scene_graph_node)
  {
    auto* parent = m_scene_graph_node->getParentSceneNode();
    if (parent)
    {
      parent->removeAndDestroyChild(m_scene_graph_node);
    }
    m_scene_graph_node = nullptr;
  }
}

void SceneGraphDisplay::onInitialize()
{
  rviz_common::Display::onInitialize();

  // The MessageFilter will drop messages with empty header.frame_id.
  // We handle TF transforms per-object internally, so we don't need
  // the MessageFilter's automatic transformation. Messages will be
  // processed regardless of their header.

  m_scene_graph_node  = scene_node_->createChildSceneNode();
  m_object_nodes_node = m_scene_graph_node->createChildSceneNode();
  m_labels_node       = m_scene_graph_node->createChildSceneNode();

  m_scene_graph_node->setVisible(true);
  m_object_nodes_node->setVisible(true);
  m_labels_node->setVisible(true);

  // Topic properties (first)
  m_managed_objects_topic_property = new rviz_common::properties::RosTopicProperty(
    "Managed Objects Topic",
    "managed_objects", // Default value
    rosidl_generator_traits::data_type<
      semantic_scene_forests_object_msgs::msg::ManagedObjectArray>(),
    "Topic containing managed objects from the Object Manager",
    this,
    SLOT(updateManagedObjectsTopic()));

  // Create QoS profile property as child of topic property
  m_qos_profile_property = new rviz_common::properties::QosProfileProperty(
    m_managed_objects_topic_property,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

  // Visualization group
  auto* visualization_property = new rviz_common::properties::Property(
    "Visualization", QVariant(), "Visualization settings for the scene graph", this);

  m_show_labels_property =
    new rviz_common::properties::BoolProperty("Show Labels",
                                              true,
                                              "Show class name labels on object nodes",
                                              visualization_property,
                                              SLOT(updateLabels()));

  m_object_color_property =
    new rviz_common::properties::ColorProperty("Object Color",
                                               QColor(0, 200, 0),
                                               "Color for object node bounding boxes",
                                               visualization_property,
                                               SLOT(updateObjectColor()));

  m_object_scale_property = new rviz_common::properties::FloatProperty(
    "Object Scale", 1.0F, "Scale multiplier for object bounding boxes", visualization_property);
  m_object_scale_property->setMin(0.01F);

  m_control_widget = createControlWidget();
  setAssociatedWidget(m_control_widget);

  // Store ROS node abstraction for topic property initialization
  m_rviz_ros_node = context_->getRosNodeAbstraction();

  // Initialize the topic property with the node abstraction (CRITICAL for RosTopicProperty!)
  m_managed_objects_topic_property->initialize(m_rviz_ros_node);

  // Initialize QoS property with callback
  m_qos_profile_property->initialize([this](rclcpp::QoS profile) {
    this->m_qos_profile = profile;
    updateManagedObjectsTopic();
  });

  m_scene_graph_topic_property = new rviz_common::properties::RosTopicProperty(
    "Scene Graph Topic",
    "scene_graph",
    rosidl_generator_traits::data_type<semantic_scene_forests_msgs::msg::SceneGraph>(),
    "Topic containing the scene graph (node/edge statistics)",
    this,
    SLOT(updateSceneGraphTopic()));

  m_scene_graph_qos_profile_property = new rviz_common::properties::QosProfileProperty(
    m_scene_graph_topic_property, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

  m_scene_graph_topic_property->initialize(m_rviz_ros_node);

  m_scene_graph_qos_profile_property->initialize([this](rclcpp::QoS profile) {
    m_scene_graph_qos_profile = profile;
    updateSceneGraphTopic();
  });

  auto ros_node   = context_->getRosNodeAbstraction().lock()->get_raw_node();
  m_action_client = rclcpp_action::create_client<UpdateSceneGraph>(ros_node, "update_scene_graph");
  m_block_object_client =
    ros_node->create_client<semantic_scene_forests_object_msgs::srv::BlockObject>("block_object");

  // Create managed objects subscription with QoS from property
  m_managed_objects_sub =
    ros_node->create_subscription<semantic_scene_forests_object_msgs::msg::ManagedObjectArray>(
      m_managed_objects_topic_property->getTopicStd(),
      m_qos_profile,
      std::bind(&SceneGraphDisplay::onManagedObjects, this, std::placeholders::_1));

  RCLCPP_INFO(
    ros_node->get_logger(),
    "Subscribed to %s topic with QoS: reliability=%s, durability=%s, depth=%zu",
    m_managed_objects_topic_property->getTopicStd().c_str(),
    m_qos_profile.reliability() == rclcpp::ReliabilityPolicy::Reliable ? "Reliable" : "Best Effort",
    m_qos_profile.durability() == rclcpp::DurabilityPolicy::TransientLocal ? "Transient Local"
                                                                           : "Volatile",
    m_qos_profile.depth());

  m_scene_graph_sub = ros_node->create_subscription<semantic_scene_forests_msgs::msg::SceneGraph>(
    m_scene_graph_topic_property->getTopicStd(),
    m_scene_graph_qos_profile,
    std::bind(&SceneGraphDisplay::onSceneGraph, this, std::placeholders::_1));

  // Create service client for requesting object visuals
  m_get_visuals_client =
    ros_node->create_client<semantic_scene_forests_object_msgs::srv::GetObjectVisuals>(
      "get_object_visuals");

  m_render_panel = context_->getViewManager()->getRenderPanel();
  if (m_render_panel)
  {
    // Install on the QWindow (RenderWindow) directly — mouse events are delivered there
    // via QGuiApplication::notify(), not via the QWidget event chain on m_render_panel_.
    m_render_panel->getRenderWindow()->installEventFilter(this);
  }
}

void SceneGraphDisplay::reset()
{
  rviz_common::Display::reset();
  clearVisuals();
  // Note: We do NOT reset action state here — reset() fires on every topic/frame change
  // and would wipe the progress bar mid-action. Action state is only reset when
  // an action completes or fails.
}


void SceneGraphDisplay::clearVisuals()
{
  auto ros_node = context_->getRosNodeAbstraction().lock();
  if (!ros_node)
  {
    // Context already destroyed, skip cleanup that requires ROS node
    return;
  }

  // Selection handlers are shared_ptr and will be cleaned up automatically when m_object_visuals is
  // cleared The selection manager will automatically deregister handlers when they are destroyed

  // Clear shapes (unique_ptr automatically destroys Ogre entities)
  m_object_shapes.clear();

  // Properly clean up label nodes and attached MovableText
  for (auto& [id, label] : m_object_labels)
  {
    if (label.node)
    {
      if (label.text)
      {
        label.node->detachObject(label.text);
      }
      auto* parent = label.node->getParentSceneNode();
      if (parent)
      {
        parent->removeAndDestroyChild(label.node);
      }
    }
  }
  m_object_labels.clear();

  // Clear object visuals map (selection handlers are shared_ptr, will be cleaned up)
  const std::scoped_lock LOCK{m_object_visuals_mutex};
  m_object_visuals.clear();
}

void SceneGraphDisplay::updateObjectNodes(
  const std::vector<semantic_scene_forests_object_msgs::msg::ManagedObject>& objects)
{
  // Check if context still exists
  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction)
  {
    return; // Display already destroyed
  }

  auto ros_node = ros_node_abstraction->get_raw_node();

  // Check if scene nodes still exist
  if (!m_object_nodes_node || !m_labels_node)
  {
    RCLCPP_ERROR(ros_node->get_logger(), "Scene nodes not initialized");
    return;
  }

  const auto COLOR       = m_object_color_property->getOgreColor();
  const float SCALE      = m_object_scale_property->getFloat();
  const bool SHOW_LABELS = m_show_labels_property->getBool();

  RCLCPP_DEBUG(ros_node->get_logger(), "Processing %zu object nodes", objects.size());

  size_t success_count = 0;
  size_t failure_count = 0;
  std::map<std::string, size_t> failure_by_frame;

  for (const auto& obj : objects)
  {
    Ogre::Vector3 frame_pos;
    Ogre::Quaternion frame_orient;
    const std::string& frame_id = obj.detection.header.frame_id;

    if (frame_id.empty())
    {
      RCLCPP_WARN(ros_node->get_logger(), "Object %ld has empty frame_id, skipping", obj.id);
      failure_count++;
      failure_by_frame["<empty>"]++;
      continue;
    }

    std::string transform_error;
    if (context_->getFrameManager()->transformHasProblems(frame_id, transform_error))
    {
      RCLCPP_WARN(ros_node->get_logger(),
                  "Transform failed for object %ld (frame '%s' → '%s'): %s",
                  obj.id,
                  frame_id.c_str(),
                  fixed_frame_.toStdString().c_str(),
                  transform_error.c_str());
      failure_count++;
      failure_by_frame[frame_id]++;
      continue;
    }

    if (!context_->getFrameManager()->getTransform(obj.detection.header, frame_pos, frame_orient))
    {
      RCLCPP_WARN(ros_node->get_logger(),
                  "Transform failed for object %ld (frame '%s' → '%s'), skipping",
                  obj.id,
                  frame_id.c_str(),
                  fixed_frame_.toStdString().c_str());

      failure_count++;
      failure_by_frame[frame_id]++;
      continue;
    }

    // Transform bbox center position from detection frame → fixed frame.
    const auto& bp = obj.detection.bbox.center.position;
    const Ogre::Vector3 LOCAL_POS(
      static_cast<float>(bp.x), static_cast<float>(bp.y), static_cast<float>(bp.z));
    const Ogre::Vector3 WORLD_POS = frame_pos + frame_orient * LOCAL_POS;

    RCLCPP_DEBUG(ros_node->get_logger(),
                 "Object %ld ('%s'): frame_pos=(%.2f,%.2f,%.2f), "
                 "local_pos=(%.2f,%.2f,%.2f), world_pos=(%.2f,%.2f,%.2f)",
                 obj.id,
                 obj.class_name.c_str(),
                 frame_pos.x,
                 frame_pos.y,
                 frame_pos.z,
                 LOCAL_POS.x,
                 LOCAL_POS.y,
                 LOCAL_POS.z,
                 WORLD_POS.x,
                 WORLD_POS.y,
                 WORLD_POS.z);

    const auto& sz = obj.detection.bbox.size;
    const Ogre::Vector3 BBOX_SCALE(static_cast<float>(std::max(sz.x, 0.01) * SCALE),
                                   static_cast<float>(std::max(sz.y, 0.01) * SCALE),
                                   static_cast<float>(std::max(sz.z, 0.01) * SCALE));

    auto shape = std::make_unique<rviz_rendering::Shape>(
      rviz_rendering::Shape::Cube, scene_manager_, m_object_nodes_node);
    shape->setPosition(WORLD_POS);
    shape->setScale(BBOX_SCALE);
    shape->setColor(COLOR);
    m_object_shapes[obj.id] = std::move(shape);

    // Register selection handler so right-click can identify this object
    ObjectVisual ov;
    ov.class_name = obj.class_name;
    // Don't store images locally - they will be requested via service on-demand
    ov.selection_handler =
      rviz_common::interaction::createSelectionHandler<SceneGraphObjectSelectionHandler>(obj.id,
                                                                                         context_);
    ov.selection_handler->addTrackedObject(m_object_shapes[obj.id]->getEntity());
    // Register the pick color on the Ogre entity so pick() can identify it
    rviz_common::interaction::SelectionManager::setPickHandle(ov.selection_handler->getHandle(),
                                                              m_object_shapes[obj.id]->getEntity());
    {
      const std::scoped_lock LOCK{m_object_visuals_mutex};
      m_object_visuals[obj.id] = std::move(ov);
    }

    success_count++;

    if (SHOW_LABELS)
    {
      Ogre::SceneNode* label_node = m_labels_node->createChildSceneNode();
      const float Z_OFFSET        = static_cast<float>(sz.z) * SCALE / 2.0F + 0.15F;
      label_node->setPosition(WORLD_POS + Ogre::Vector3(0.0F, 0.0F, Z_OFFSET));

      auto* text = new rviz_rendering::MovableText(obj.class_name, "Liberation Sans", 0.2F, COLOR);
      text->setTextAlignment(rviz_rendering::MovableText::H_CENTER,
                             rviz_rendering::MovableText::V_ABOVE);
      text->showOnTop(true);
      label_node->attachObject(text);
      m_object_labels[obj.id] = {label_node, text};
    }
  }

  // Log summary
  RCLCPP_DEBUG(ros_node->get_logger(),
               "Object visualization complete: %zu shown, %zu skipped",
               success_count,
               failure_count);

  if (failure_count > 0)
  {
    std::string status_msg = std::to_string(failure_count) + " objects skipped (TF failures: ";
    bool first             = true;
    for (const auto& [frame, count] : failure_by_frame)
    {
      if (!first)
      {
        status_msg += ", ";
      }
      status_msg += frame + ":" + std::to_string(count);
      first = false;
    }
    status_msg += ")";

    setStatusStd(rviz_common::properties::StatusProperty::Warn, "Transform", status_msg);

    for (const auto& [frame, count] : failure_by_frame)
    {
      RCLCPP_WARN(
        ros_node->get_logger(), "  - Frame '%s': %zu objects skipped", frame.c_str(), count);
    }
  }
}


QWidget* SceneGraphDisplay::createControlWidget()
{
  auto* widget = new QWidget();
  widget->setObjectName("SceneGraphControlWidget");

  auto* layout = new QVBoxLayout();
  layout->setSpacing(8);
  layout->setContentsMargins(8, 8, 8, 8);

  m_phase1_button = new QPushButton("Update Instance Graph");
  m_phase1_button->setObjectName("Phase1Button");
  m_phase1_button->setToolTip(
    "Trigger PHASE_1: Update instance-level scene graph with current detections");
  m_phase1_button->setMinimumHeight(35);
  connect(m_phase1_button,
          &QPushButton::clicked,
          this,
          &SceneGraphDisplay::sendPhase1Goal,
          Qt::QueuedConnection); // Use queued connection for thread safety
  layout->addWidget(m_phase1_button);

  m_phase2_button = new QPushButton("Update Meta Graph");
  m_phase2_button->setObjectName("Phase2Button");
  m_phase2_button->setToolTip(
    "Trigger PHASE_2: Update meta-level scene graph with semantic relationships");
  m_phase2_button->setMinimumHeight(35);
  connect(m_phase2_button,
          &QPushButton::clicked,
          this,
          &SceneGraphDisplay::sendPhase2Goal,
          Qt::QueuedConnection); // Use queued connection for thread safety
  layout->addWidget(m_phase2_button);

  layout->addSpacing(8);

  m_node_count_label = new QLabel("Objects: 0");
  m_node_count_label->setObjectName("NodeCountLabel");
  m_node_count_label->setStyleSheet("color: black; font-weight: bold;");
  layout->addWidget(m_node_count_label);

  m_scene_graph_info_label = new QLabel("Graph: \xe2\x80\x94");
  m_scene_graph_info_label->setObjectName("SceneGraphInfoLabel");
  m_scene_graph_info_label->setStyleSheet("color: black; font-weight: bold;");
  layout->addWidget(m_scene_graph_info_label);

  layout->addSpacing(4);

  m_progress_bar = new QProgressBar();
  m_progress_bar->setObjectName("ActionProgressBar");
  m_progress_bar->setRange(0, 100);
  m_progress_bar->setValue(0);
  m_progress_bar->setFormat("Not active");
  m_progress_bar->setVisible(false);
  layout->addWidget(m_progress_bar);

  m_status_label = new QLabel("Ready");
  m_status_label->setObjectName("ActionStatusLabel");
  m_status_label->setStyleSheet("color: gray; font-weight: bold;");
  m_status_label->setWordWrap(true);
  m_status_label->setMinimumHeight(40);
  layout->addWidget(m_status_label);

  layout->addStretch();
  widget->setLayout(layout);
  widget->setMinimumWidth(220);
  widget->setMaximumWidth(300);

  return widget;
}

void SceneGraphDisplay::sendPhase1Goal()
{
  sendGoal(UpdateSceneGraph::Goal::PHASE_1);
}

void SceneGraphDisplay::sendPhase2Goal()
{
  sendGoal(UpdateSceneGraph::Goal::PHASE_2);
}

void SceneGraphDisplay::sendGoal(uint8_t phase)
{
  // Check if action already in progress
  if (m_action_in_progress)
  {
    if (m_status_label)
    {
      m_status_label->setText("Action already in progress");
      m_status_label->setStyleSheet("color: orange; font-weight: bold;");
    }
    return;
  }

  // Check if action client exists
  if (!m_action_client)
  {
    if (m_status_label)
    {
      m_status_label->setText("Error: Action client not initialized");
      m_status_label->setStyleSheet("color: red; font-weight: bold;");
    }
    return;
  }

  // Wait for action server with timeout
  if (!m_action_client->wait_for_action_server(std::chrono::seconds(2)))
  {
    if (m_status_label)
    {
      m_status_label->setText("Error: Action server not available");
      m_status_label->setStyleSheet("color: red; font-weight: bold;");
    }
    setStatus(rviz_common::properties::StatusProperty::Error,
              "Action",
              "UpdateSceneGraph action server not available");
    RCLCPP_ERROR(context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger(),
                 "Action server not available");
    return;
  }

  UpdateSceneGraph::Goal goal;
  goal.phase = phase;

  rclcpp_action::Client<UpdateSceneGraph>::SendGoalOptions options;
  options.goal_response_callback = [this](auto&& goal_handle) {
    handleGoalResponse(std::forward<decltype(goal_handle)>(goal_handle));
  };
  options.feedback_callback = [this](auto&& goal_handle, auto&& feedback) {
    handleFeedback(std::forward<decltype(goal_handle)>(goal_handle),
                   std::forward<decltype(feedback)>(feedback));
  };
  options.result_callback = [this](auto&& result) {
    handleResult(std::forward<decltype(result)>(result));
  };

  m_action_client->async_send_goal(goal, options);

  m_action_in_progress = true;

  // Update UI state with null checks
  if (m_phase1_button)
    m_phase1_button->setEnabled(false);
  if (m_phase2_button)
    m_phase2_button->setEnabled(false);
  if (m_progress_bar)
  {
    m_progress_bar->setVisible(true);
    m_progress_bar->setValue(0);
    m_progress_bar->setFormat("0%");
  }
  if (m_status_label)
  {
    m_status_label->setText(QString("Sending Phase %1 goal...").arg(static_cast<int>(phase)));
    m_status_label->setStyleSheet("color: blue; font-weight: bold;");
  }
}

void SceneGraphDisplay::handleGoalResponse(const GoalHandle::SharedPtr& goal_handle)
{
  QMetaObject::invokeMethod(
    this,
    [this, goal_handle]() {
      // Check if widgets still exist (QPointer will be null if destroyed)
      if (!m_status_label)
      {
        return;
      }

      if (!goal_handle)
      {
        m_status_label->setText("Goal rejected by server");
        m_status_label->setStyleSheet("color: red; font-weight: bold;");
        setStatus(rviz_common::properties::StatusProperty::Error,
                  "Action",
                  "Goal rejected by action server");
        resetActionState();
      }
      else
      {
        m_status_label->setText("Goal accepted, executing...");
        m_status_label->setStyleSheet("color: green; font-weight: bold;");
        setStatus(rviz_common::properties::StatusProperty::Ok, "Action", "Goal accepted");
      }
    },
    Qt::QueuedConnection);
}

void SceneGraphDisplay::handleFeedback(
  const GoalHandle::SharedPtr& /* goal_handle */,
  const std::shared_ptr<const UpdateSceneGraph::Feedback>& feedback)
{
  QMetaObject::invokeMethod(
    this,
    [this, feedback]() {
      // Check if widgets still exist (QPointer will be null if destroyed)
      if (!m_progress_bar || !m_status_label)
      {
        return;
      }

      const uint32_t CURRENT = feedback->current_object;
      const uint32_t TOTAL   = feedback->total_objects;
      const int PCT          = (TOTAL > 0) ? static_cast<int>(CURRENT * 100U / TOTAL) : 0;

      m_progress_bar->setValue(PCT);
      m_progress_bar->setFormat(QString::number(PCT) + "%");
      m_status_label->setText(QString("Processing: %1/%2 - %3")
                                .arg(CURRENT)
                                .arg(TOTAL)
                                .arg(QString::fromStdString(feedback->status)));
      m_status_label->setStyleSheet("color: blue; font-weight: bold;");
    },
    Qt::QueuedConnection);
}

void SceneGraphDisplay::handleResult(const GoalHandle::WrappedResult& result)
{
  const bool SUCCESS = (result.code == rclcpp_action::ResultCode::SUCCEEDED) &&
                       result.result != nullptr && result.result->success;
  const std::string MESSAGE =
    result.result != nullptr ? result.result->message : "Action finished without a result";

  QMetaObject::invokeMethod(
    this,
    [this, SUCCESS, MESSAGE]() {
      // Check if widgets still exist (QPointer will be null if destroyed)
      if (!m_progress_bar || !m_status_label)
      {
        return;
      }

      if (SUCCESS)
      {
        m_progress_bar->setValue(100);
        m_progress_bar->setFormat("Complete");
        m_status_label->setText("Success: " + QString::fromStdString(MESSAGE));
        m_status_label->setStyleSheet("color: green; font-weight: bold;");
        setStatusStd(rviz_common::properties::StatusProperty::Ok, "Action", MESSAGE);
      }
      else
      {
        m_progress_bar->setFormat("Failed");
        m_status_label->setText("Failed: " + QString::fromStdString(MESSAGE));
        m_status_label->setStyleSheet("color: red; font-weight: bold;");
        setStatusStd(rviz_common::properties::StatusProperty::Error, "Action", MESSAGE);
      }
      m_action_in_progress = false;
      QTimer::singleShot(5000, this, [this]() { resetActionState(); });
    },
    Qt::QueuedConnection);
}

void SceneGraphDisplay::resetActionState()
{
  m_action_in_progress = false;

  // QPointer will be null if widgets are destroyed, so checks are safe
  if (m_phase1_button)
  {
    m_phase1_button->setEnabled(true);
  }
  if (m_phase2_button)
  {
    m_phase2_button->setEnabled(true);
  }
  if (m_progress_bar)
  {
    m_progress_bar->setVisible(false);
    m_progress_bar->setValue(0);
    m_progress_bar->setFormat("Not active");
  }
  if (m_status_label)
  {
    m_status_label->setText("Ready");
    m_status_label->setStyleSheet("color: gray; font-weight: bold;");
  }
}

void SceneGraphDisplay::updateObjectColor()
{
  const auto COLOR = m_object_color_property->getOgreColor();
  for (auto& [id, shape] : m_object_shapes)
  {
    shape->setColor(COLOR);
  }
  for (auto& [id, label] : m_object_labels)
  {
    if (label.text)
    {
      label.text->setColor(COLOR);
    }
  }
}


void SceneGraphDisplay::updateLabels()
{
  if (m_labels_node)
  {
    m_labels_node->setVisible(m_show_labels_property->getBool());
  }
}


void SceneGraphDisplay::updateManagedObjectsTopic()
{
  auto ros_node_abstraction = m_rviz_ros_node.lock();
  if (!ros_node_abstraction)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("SceneGraphDisplay"),
                 "ROS node abstraction no longer available");
    return;
  }

  auto ros_node = ros_node_abstraction->get_raw_node();

  std::string topic_name_str = m_managed_objects_topic_property->getTopicStd();
  if (topic_name_str.empty())
  {
    RCLCPP_DEBUG(ros_node->get_logger(), "Managed objects topic is empty");
    return;
  }

  // Check if topic actually changed
  if (m_managed_objects_sub && m_managed_objects_sub->get_topic_name() == topic_name_str)
  {
    return;
  }

  // Unsubscribe from old topic
  m_managed_objects_sub.reset();
  clearVisuals();

  // Get current QoS from property - it will be updated via callback when changed
  // The callback will call updateManagedObjectsTopic() again, but the topic check will prevent
  // infinite loop

  // Create new subscription with new topic and current QoS
  try
  {
    m_managed_objects_sub =
      ros_node->create_subscription<semantic_scene_forests_object_msgs::msg::ManagedObjectArray>(
        topic_name_str,
        m_qos_profile,
        std::bind(&SceneGraphDisplay::onManagedObjects, this, std::placeholders::_1));

    RCLCPP_INFO(
      ros_node->get_logger(),
      "Subscribed to managed objects topic: %s with QoS: reliability=%s, durability=%s, depth=%zu",
      topic_name_str.c_str(),
      m_qos_profile.reliability() == rclcpp::ReliabilityPolicy::Reliable ? "Reliable"
                                                                         : "Best Effort",
      m_qos_profile.durability() == rclcpp::DurabilityPolicy::TransientLocal ? "Transient Local"
                                                                             : "Volatile",
      m_qos_profile.depth());
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(ros_node->get_logger(),
                 "Failed to subscribe to topic '%s': %s",
                 topic_name_str.c_str(),
                 e.what());
    setStatusStd(rviz_common::properties::StatusProperty::Error,
                 "Subscription",
                 std::string("Failed to subscribe: ") + e.what());
  }
}

void SceneGraphDisplay::onManagedObjects(
  semantic_scene_forests_object_msgs::msg::ManagedObjectArray::ConstSharedPtr msg)
{
  // Check if context still exists
  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction)
  {
    return; // Display already destroyed
  }

  auto ros_node = ros_node_abstraction->get_raw_node();

  RCLCPP_DEBUG(
    ros_node->get_logger(), "Received managed objects: %zu objects", msg->objects.size());

  QMetaObject::invokeMethod(
    this,
    [this, objects = msg->objects]() {
      clearVisuals();
      updateObjectNodes(objects);
    },
    Qt::QueuedConnection);

  const size_t object_count = msg->objects.size();
  QMetaObject::invokeMethod(
    this,
    [this, object_count]() {
      if (m_node_count_label)
      {
        m_node_count_label->setText(QString("Objects: %1").arg(static_cast<int>(object_count)));
      }
    },
    Qt::QueuedConnection);

  setStatusStd(rviz_common::properties::StatusProperty::Ok,
               "Managed Objects",
               "Objects: " + std::to_string(object_count));
}

void SceneGraphDisplay::updateSceneGraphTopic()
{
  auto ros_node_abstraction = m_rviz_ros_node.lock();
  if (!ros_node_abstraction)
  {
    return;
  }

  auto ros_node = ros_node_abstraction->get_raw_node();

  std::string topic_name_str = m_scene_graph_topic_property->getTopicStd();
  if (topic_name_str.empty())
  {
    return;
  }

  if (m_scene_graph_sub && m_scene_graph_sub->get_topic_name() == topic_name_str)
  {
    return;
  }

  m_scene_graph_sub.reset();

  try
  {
    m_scene_graph_sub = ros_node->create_subscription<semantic_scene_forests_msgs::msg::SceneGraph>(
      topic_name_str,
      m_scene_graph_qos_profile,
      std::bind(&SceneGraphDisplay::onSceneGraph, this, std::placeholders::_1));

    RCLCPP_INFO(
      ros_node->get_logger(), "Subscribed to scene graph topic: %s", topic_name_str.c_str());
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(ros_node->get_logger(),
                 "Failed to subscribe to scene graph topic '%s': %s",
                 topic_name_str.c_str(),
                 e.what());
    setStatusStd(rviz_common::properties::StatusProperty::Error,
                 "Scene Graph",
                 std::string("Failed to subscribe: ") + e.what());
  }
}

void SceneGraphDisplay::onSceneGraph(
  semantic_scene_forests_msgs::msg::SceneGraph::ConstSharedPtr msg)
{
  if (!context_->getRosNodeAbstraction().lock())
  {
    return;
  }

  const size_t obj_count     = msg->object_nodes.size();
  const size_t concept_count = msg->concept_nodes.size();
  const size_t edge_count    = msg->edges.size();

  QMetaObject::invokeMethod(
    this,
    [this, obj_count, concept_count, edge_count]() {
      if (m_scene_graph_info_label)
      {
        m_scene_graph_info_label->setText(QString("Graph: %1 obj, %2 concept, %3 edges")
                                            .arg(static_cast<int>(obj_count))
                                            .arg(static_cast<int>(concept_count))
                                            .arg(static_cast<int>(edge_count)));
      }
    },
    Qt::QueuedConnection);

  setStatusStd(rviz_common::properties::StatusProperty::Ok,
               "Scene Graph",
               "Nodes: " + std::to_string(obj_count + concept_count) +
                 ", Edges: " + std::to_string(edge_count));
}

void SceneGraphDisplay::requestObjectVisuals(int64_t object_id)
{
  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction)
  {
    return;
  }
  auto ros_node = ros_node_abstraction->get_raw_node();

  if (!m_get_visuals_client->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_WARN(ros_node->get_logger(), "Service /get_object_visuals not available");
    return;
  }

  auto request =
    std::make_shared<semantic_scene_forests_object_msgs::srv::GetObjectVisuals::Request>();
  request->object_ids.push_back(object_id);

  auto result = m_get_visuals_client->async_send_request(
    request,
    [this](rclcpp::Client<semantic_scene_forests_object_msgs::srv::GetObjectVisuals>::SharedFuture
             future) { onObjectVisualsResponse(future); });
}

void SceneGraphDisplay::onObjectVisualsResponse(
  rclcpp::Client<semantic_scene_forests_object_msgs::srv::GetObjectVisuals>::SharedFuture result)
{
  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction)
  {
    return;
  }
  auto ros_node = ros_node_abstraction->get_raw_node();

  try
  {
    auto response = result.get();
    if (response->visuals.empty())
    {
      RCLCPP_WARN(ros_node->get_logger(), "No visuals received");
      return;
    }

    // Find the object in our cache and update it
    for (const auto& visual : response->visuals)
    {
      const std::scoped_lock LOCK{m_object_visuals_mutex};
      auto it = m_object_visuals.find(visual.object_id);
      if (it != m_object_visuals.end())
      {
        // Update images in the existing ObjectVisual
        it->second.images.assign(visual.image_crops.begin(), visual.image_crops.end());

        // Show the image dialog if we have images
        if (!it->second.images.empty())
        {
          QMetaObject::invokeMethod(
            this,
            [this, id = visual.object_id, visual_copy = it->second]() {
              showImageDialog(id, visual_copy);
            },
            Qt::QueuedConnection);
        }
      }
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(ros_node->get_logger(), "Error receiving visuals: %s", e.what());
  }
}

bool SceneGraphDisplay::eventFilter(QObject* obj, QEvent* event)
{
  // Mouse events are delivered to the RenderWindow (QWindow) via QGuiApplication::notify(),
  // not via the QWidget event chain on m_render_panel_.

  // Check if render panel still exists (QPointer will be null if destroyed)
  if (!m_render_panel)
  {
    return QObject::eventFilter(obj, event);
  }

  if (obj == m_render_panel->getRenderWindow() && event->type() == QEvent::MouseButtonRelease)
  {
    auto* me = static_cast<QMouseEvent*>(event);
    if (me->button() == Qt::RightButton)
    {
      // Coordinates are in QWindow-local pixels, which match pick()'s expected space.
      if (handleContextMenu(me->pos()))
      {
        return true; // consume only when we showed a menu
      }
    }
  }
  return QObject::eventFilter(obj, event);
}

bool SceneGraphDisplay::handleContextMenu(const QPoint& pos)
{
  using namespace rviz_common::interaction;

  // Check if render panel still exists
  if (!m_render_panel)
  {
    return false;
  }

  M_Picked results;
  auto selection_manager = context_->getSelectionManager();
  if (!selection_manager)
  {
    return false;
  }

  selection_manager->pick(
    m_render_panel->getRenderWindow(), pos.x(), pos.y(), pos.x(), pos.y(), results);
  if (results.empty())
  {
    return false;
  }

  const CollObjectHandle HANDLE = results.begin()->first;
  auto handler_manager          = context_->getHandlerManager();
  if (!handler_manager)
  {
    return false;
  }

  auto handler = handler_manager->getHandler(HANDLE);
  if (!handler)
  {
    return false;
  }

  auto* sg_handler = dynamic_cast<SceneGraphObjectSelectionHandler*>(handler.get());
  if (!sg_handler)
  {
    return false;
  }

  const int64_t object_id = sg_handler->getObjectId();
  ObjectVisual visual;
  {
    const std::scoped_lock LOCK{m_object_visuals_mutex};
    auto it = m_object_visuals.find(object_id);
    if (it == m_object_visuals.end())
    {
      return false;
    }
    visual = it->second;
  }

  auto menu = std::make_shared<QMenu>();

  // Image actions
  if (!visual.images.empty())
  {
    QAction* action =
      menu->addAction(QString("Show Images (%1)").arg(static_cast<int>(visual.images.size())));
    connect(action, &QAction::triggered, [this, object_id, visual]() {
      showImageDialog(object_id, visual);
    });
  }
  else
  {
    QAction* action = menu->addAction("Load Images");
    action->setToolTip("Request object images from Object Manager");
    connect(action, &QAction::triggered, [this, object_id]() { requestObjectVisuals(object_id); });
  }

  menu->addSeparator();

  // Block object action — shows images for confirmation and calls the service
  QAction* block_action = menu->addAction(QString("Block Object %1").arg(object_id));
  block_action->setToolTip("Block this object and show its images");
  connect(block_action, &QAction::triggered, [this, object_id, visual]() {
    blockObject(object_id);
    if (!visual.images.empty())
    {
      showImageDialog(object_id, visual);
    }
    else
    {
      requestObjectVisuals(object_id);
    }
  });

  m_render_panel->showContextMenu(menu);
  return true;
}

void SceneGraphDisplay::blockObject(int64_t object_id)
{
  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction)
  {
    return;
  }
  auto ros_node = ros_node_abstraction->get_raw_node();

  if (!m_block_object_client->wait_for_service(std::chrono::milliseconds(500)))
  {
    RCLCPP_WARN(ros_node->get_logger(), "block_object service not available");
    setStatusStd(
      rviz_common::properties::StatusProperty::Warn, "Block", "block_object service not available");
    return;
  }

  auto req = std::make_shared<semantic_scene_forests_object_msgs::srv::BlockObject::Request>();
  req->object_id = object_id;

  m_block_object_client->async_send_request(
    req,
    [this, object_id](
      rclcpp::Client<semantic_scene_forests_object_msgs::srv::BlockObject>::SharedFuture future) {
      try
      {
        auto resp                 = future.get();
        const bool success        = resp->success;
        const std::string message = resp->message;
        QMetaObject::invokeMethod(
          this,
          [this, object_id, success, message]() {
            if (success)
            {
              setStatusStd(rviz_common::properties::StatusProperty::Ok,
                           "Block",
                           "Blocked object " + std::to_string(object_id) + ": " + message);
            }
            else
            {
              setStatusStd(rviz_common::properties::StatusProperty::Warn, "Block", message);
            }
          },
          Qt::QueuedConnection);
      }
      catch (const std::exception& e)
      {
        auto node = context_->getRosNodeAbstraction().lock();
        if (node)
        {
          RCLCPP_ERROR(
            node->get_raw_node()->get_logger(), "block_object call failed: %s", e.what());
        }
      }
    });
}

void SceneGraphDisplay::showImageDialog(int64_t id, const ObjectVisual& visual)
{
  if (visual.images.empty())
  {
    auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
    if (ros_node_abstraction)
    {
      RCLCPP_WARN(ros_node_abstraction->get_raw_node()->get_logger(),
                  "No images available for object %ld",
                  id);
    }
    return;
  }

  auto* dlg = new ImageViewerDialog(id, visual.class_name, visual.images, m_render_panel);
  dlg->setAttribute(Qt::WA_DeleteOnClose);
  dlg->show();
}

} // namespace semantic_scene_forests_visualization_rviz
