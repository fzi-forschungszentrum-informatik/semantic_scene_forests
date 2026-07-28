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

#ifndef SEMANTIC_SCENE_FORESTS_VISUALIZATION_RVIZ__SCENE_GRAPH_DISPLAY_HPP_
#define SEMANTIC_SCENE_FORESTS_VISUALIZATION_RVIZ__SCENE_GRAPH_DISPLAY_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#ifndef Q_MOC_RUN
#  include "rviz_common/interaction/selection_handler.hpp"
#  include "rviz_common/render_panel.hpp"
#  include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#  include "sensor_msgs/msg/image.hpp"
#  include <OgreColourValue.h>
#  include <OgreVector.h>
#endif

#include <QLabel>
#include <QPointer>
#include <QProgressBar>
#include <QPushButton>
#include <QTimer>
#include <QWidget>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_rendering/objects/movable_text.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "semantic_scene_forests_msgs/action/update_scene_graph.hpp"
#include "semantic_scene_forests_msgs/msg/scene_graph.hpp"
#include "semantic_scene_forests_object_msgs/msg/managed_object_array.hpp"
#include "semantic_scene_forests_object_msgs/srv/block_object.hpp"
#include "semantic_scene_forests_object_msgs/srv/get_object_visuals.hpp"

namespace Ogre {
class SceneNode;
}

namespace semantic_scene_forests_visualization_rviz {

class SceneGraphDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  using UpdateSceneGraph = semantic_scene_forests_msgs::action::UpdateSceneGraph;
  using GoalHandle       = rclcpp_action::ClientGoalHandle<UpdateSceneGraph>;

  SceneGraphDisplay();
  ~SceneGraphDisplay() override;

  void onInitialize() override;
  void reset() override;

private:
  // Nested type used by showImageDialog — must be declared before method signatures
  struct ObjectVisual
  {
    std::string class_name;
    std::vector<sensor_msgs::msg::Image> images;
    rviz_common::interaction::SelectionHandlerPtr selection_handler;
  };

  void clearVisuals();
  void updateObjectNodes(
    const std::vector<semantic_scene_forests_object_msgs::msg::ManagedObject>& objects);
  bool eventFilter(QObject* obj, QEvent* event) override;
  bool handleContextMenu(const QPoint& pos);
  void showImageDialog(int64_t object_id, const ObjectVisual& visual);
  void blockObject(int64_t object_id);

  void sendGoal(uint8_t phase);
  void handleGoalResponse(const GoalHandle::SharedPtr& goal_handle);
  void handleFeedback(const GoalHandle::SharedPtr& goal_handle,
                      const std::shared_ptr<const UpdateSceneGraph::Feedback>& feedback);
  void handleResult(const GoalHandle::WrappedResult& result);
  void resetActionState();
  QWidget* createControlWidget();

  void
  onManagedObjects(semantic_scene_forests_object_msgs::msg::ManagedObjectArray::ConstSharedPtr msg);
  void onSceneGraph(semantic_scene_forests_msgs::msg::SceneGraph::ConstSharedPtr msg);
  void requestObjectVisuals(int64_t object_id);
  void onObjectVisualsResponse(
    rclcpp::Client<semantic_scene_forests_object_msgs::srv::GetObjectVisuals>::SharedFuture result);

  // Scene nodes (raw pointers — owned by Ogre scene manager)
  Ogre::SceneNode* m_scene_graph_node{nullptr};
  Ogre::SceneNode* m_object_nodes_node{nullptr};
  Ogre::SceneNode* m_labels_node{nullptr};

  // Visual storage (unique_ptr for RAII on Shape)
  std::map<int64_t, std::unique_ptr<rviz_rendering::Shape>> m_object_shapes;

  // Labels: raw pointers — MovableText (Ogre::SimpleRenderable) requires manual cleanup
  struct LabelVisual
  {
    Ogre::SceneNode* node{nullptr};
    rviz_rendering::MovableText* text{nullptr};
  };
  std::map<int64_t, LabelVisual> m_object_labels;

  // Per-object visual data and selection handlers for right-click context menu
  std::map<int64_t, ObjectVisual> m_object_visuals;

  // Render panel (QPointer for safe access — not owned)
  QPointer<rviz_common::RenderPanel> m_render_panel;

  // Properties (raw pointers — RViz property system owns via QObject parent-child)
  rviz_common::properties::ColorProperty* m_object_color_property{nullptr};
  rviz_common::properties::FloatProperty* m_object_scale_property{nullptr};
  rviz_common::properties::BoolProperty* m_show_labels_property{nullptr};
  rviz_common::properties::RosTopicProperty* m_managed_objects_topic_property{nullptr};
  rviz_common::properties::QosProfileProperty* m_qos_profile_property{nullptr};
  rclcpp::QoS m_qos_profile{5};

  rviz_common::properties::RosTopicProperty* m_scene_graph_topic_property{nullptr};
  rviz_common::properties::QosProfileProperty* m_scene_graph_qos_profile_property{nullptr};
  rclcpp::QoS m_scene_graph_qos_profile{5};

  // Action client
  rclcpp_action::Client<UpdateSceneGraph>::SharedPtr m_action_client;

  // Service client for blocking objects
  rclcpp::Client<semantic_scene_forests_object_msgs::srv::BlockObject>::SharedPtr
    m_block_object_client;

  // ROS node abstraction for topic property initialization
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr m_rviz_ros_node;

  // Managed objects subscription
  rclcpp::Subscription<semantic_scene_forests_object_msgs::msg::ManagedObjectArray>::SharedPtr
    m_managed_objects_sub;

  // Scene graph subscription
  rclcpp::Subscription<semantic_scene_forests_msgs::msg::SceneGraph>::SharedPtr m_scene_graph_sub;

  // Service client for requesting object visuals
  rclcpp::Client<semantic_scene_forests_object_msgs::srv::GetObjectVisuals>::SharedPtr
    m_get_visuals_client;

  // Associated widget (QPointer for safe access after destruction)
  QPointer<QWidget> m_control_widget;
  QPointer<QPushButton> m_phase1_button;
  QPointer<QPushButton> m_phase2_button;
  QPointer<QLabel> m_node_count_label;
  QPointer<QLabel> m_scene_graph_info_label;
  QPointer<QProgressBar> m_progress_bar;
  QPointer<QLabel> m_status_label;
  bool m_action_in_progress{false};


  void sendPhase1Goal();
  void sendPhase2Goal();

private Q_SLOTS:
  void updateObjectColor();
  void updateLabels();
  void updateManagedObjectsTopic();
  void updateSceneGraphTopic();
};

} // namespace semantic_scene_forests_visualization_rviz

#endif // SEMANTIC_SCENE_FORESTS_VISUALIZATION_RVIZ__SCENE_GRAPH_DISPLAY_HPP_
