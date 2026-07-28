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

#include <cstdint>
#include <memory>
#include <unordered_set>
#include <vector>

#include <QObject>
#include <QString>
#include <QVector>

// rqt_gui_cpp renamed its public header from plugin.h to plugin.hpp after
// Jazzy (rqt_gui_cpp >= ~1.7). Support both so the plugin builds across distros.
#if __has_include(<rqt_gui_cpp/plugin.hpp>)
#  include <rqt_gui_cpp/plugin.hpp>
#else
#  include <rqt_gui_cpp/plugin.h>
#endif

#include <rclcpp_action/client.hpp>
#include <rclcpp_action/create_client.hpp>

#include <semantic_scene_forests_msgs/action/update_scene_graph.hpp>
#include <semantic_scene_forests_msgs/msg/scene_graph.hpp>
#include <semantic_scene_forests_object_msgs/msg/managed_object_array.hpp>
#include <semantic_scene_forests_object_msgs/srv/get_object_visuals.hpp>
#include <semantic_scene_forests_planning_msgs/action/find_object.hpp>
#include <semantic_scene_forests_planning_msgs/action/prompt_llm.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "semantic_scene_forests_visualization_rqt/graph_view.hpp"

namespace fzi::semantic_scene_forests {
class SceneGraph;
} // namespace fzi::semantic_scene_forests

class QWidget;

namespace semantic_scene_forests_visualization_rqt {

class GraphView;
class ImagePanel;
class SidePanel;

class SceneGraphPlugin : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  SceneGraphPlugin();
  ~SceneGraphPlugin() override;

  void initPlugin(qt_gui_cpp::PluginContext& context) override;
  void shutdownPlugin() override;
  void saveSettings(qt_gui_cpp::Settings& plugin_settings,
                    qt_gui_cpp::Settings& instance_settings) const override;
  void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                       const qt_gui_cpp::Settings& instance_settings) override;

signals:
  void sceneGraphReceived();
  void managedObjectsReceived();
  void visualsReceived(qint64 object_id);
  void actionFeedback(unsigned current, unsigned total, QString status);
  void actionFinished(bool success, QString message);
  void queryInspected(QVector<qint64> ids, QString status);
  void queryFinished(bool success, QString message, qint64 object_id);

private slots:
  void onSceneGraphReceived();
  void onManagedObjectsReceived();
  void onVisualsReceived(qint64 object_id);
  void onActionFeedback(unsigned current, unsigned total, QString status);
  void onActionFinished(bool success, QString message);
  void onNodeSelected(qint64 node_id, bool is_object);
  void onSelectionCleared();
  void onViewModeChanged(ViewMode mode);
  void onRefreshRequested();
  void sendActionGoal(std::uint8_t phase);
  void onQueryRequested(int kind, QString text);
  void onQueryCancel();
  void onQueryInspected(QVector<qint64> ids, QString status);
  void onQueryFinished(bool success, QString message, qint64 object_id);
  void onShowInspectedToggled(bool enabled);

  // NOLINTNEXTLINE(readability-redundant-access-specifiers) — Qt slots section.
private:
  using UpdateSceneGraph = semantic_scene_forests_msgs::action::UpdateSceneGraph;
  using GoalHandle       = rclcpp_action::ClientGoalHandle<UpdateSceneGraph>;
  using PromptLLM        = semantic_scene_forests_planning_msgs::action::PromptLLM;
  using FindObject       = semantic_scene_forests_planning_msgs::action::FindObject;

  QWidget* m_widget{nullptr};
  GraphView* m_graph_view{nullptr};
  SidePanel* m_side_panel{nullptr};
  ImagePanel* m_image_panel{nullptr};

  rclcpp::Subscription<semantic_scene_forests_msgs::msg::SceneGraph>::SharedPtr m_scene_graph_sub;
  rclcpp::Subscription<semantic_scene_forests_object_msgs::msg::ManagedObjectArray>::SharedPtr
    m_managed_objects_sub;
  rclcpp_action::Client<UpdateSceneGraph>::SharedPtr m_action_client;
  rclcpp_action::Client<PromptLLM>::SharedPtr m_prompt_client;
  rclcpp_action::Client<FindObject>::SharedPtr m_find_client;
  rclcpp::Client<semantic_scene_forests_object_msgs::srv::GetObjectVisuals>::SharedPtr
    m_visuals_client;

  // Active query goal handles for cancellation — set/cleared from ROS callbacks under m_mutex.
  rclcpp_action::ClientGoalHandle<PromptLLM>::SharedPtr m_prompt_goal;
  rclcpp_action::ClientGoalHandle<FindObject>::SharedPtr m_find_goal;

  // Latched data — written from ROS callbacks under m_mutex, read on the UI thread.
  mutable std::mutex m_mutex;
  semantic_scene_forests_msgs::msg::SceneGraph::ConstSharedPtr m_latest_scene_graph_msg;
  std::shared_ptr<fzi::semantic_scene_forests::SceneGraph> m_latest_scene_graph;
  std::unordered_map<int64_t, std::string> m_object_class_names;
  std::unordered_map<int64_t, std::vector<sensor_msgs::msg::Image>> m_visuals_cache;

  // Last graph applied to the view; differs from m_latest_scene_graph when the user has
  // not yet pressed the refresh button.
  std::shared_ptr<const fzi::semantic_scene_forests::SceneGraph> m_displayed_scene_graph;
  std::uint64_t m_displayed_message_count{0};
  std::uint64_t m_latest_message_count{0};

  int64_t m_selected_object_id{-1};
  QString m_selected_object_class;

  // Most recent inspected ("expanded") node set from a query — used to re-apply the
  // filter when the user toggles "Show only inspected nodes" back on. UI-thread only.
  std::unordered_set<int64_t> m_last_inspected;
  // Whether the in-flight query is a FindObject (carries an answer node to focus). UI-thread only.
  bool m_active_query_is_find{false};

  void onSceneGraphMsg(semantic_scene_forests_msgs::msg::SceneGraph::ConstSharedPtr msg);
  void onManagedObjectsMsg(
    semantic_scene_forests_object_msgs::msg::ManagedObjectArray::ConstSharedPtr msg);
  void requestVisuals(int64_t object_id);
  void syncDisplayedGraphToLatest();
};

} // namespace semantic_scene_forests_visualization_rqt
