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

#include "semantic_scene_forests_visualization_rqt/scene_graph_plugin.hpp"

#include <mutex>
#include <utility>

#include <QHBoxLayout>
#include <QMetaObject>
#include <QSplitter>
#include <QString>
#include <QWidget>
#include <Qt>

// qt_gui_cpp renamed its public headers from .h to .hpp after Jazzy.
// Support both so the plugin builds across distros.
#if __has_include(<qt_gui_cpp/plugin_context.hpp>)
#  include <qt_gui_cpp/plugin_context.hpp>
#  include <qt_gui_cpp/settings.hpp>
#else
#  include <qt_gui_cpp/plugin_context.h>
#  include <qt_gui_cpp/settings.h>
#endif

#include <rclcpp/qos.hpp>

#include "semantic_scene_forests_visualization_rqt/graph_view.hpp"
#include "semantic_scene_forests_visualization_rqt/image_panel.hpp"
#include "semantic_scene_forests_visualization_rqt/side_panel.hpp"
#include <semantic_scene_forests/scene_graph.hpp>

namespace semantic_scene_forests_visualization_rqt {

namespace {

constexpr const char* SCENE_GRAPH_TOPIC          = "scene_graph";
constexpr const char* MANAGED_OBJECTS_TOPIC      = "managed_objects";
constexpr const char* UPDATE_SCENE_GRAPH_ACTION  = "update_scene_graph";
constexpr const char* PROMPT_LLM_ACTION          = "prompt_llm";
constexpr const char* FIND_OBJECT_ACTION         = "find_object";
constexpr const char* GET_OBJECT_VISUALS_SERVICE = "get_object_visuals";
constexpr int LEFT_PANEL_WIDTH                   = 260;
constexpr int RIGHT_PANEL_WIDTH                  = 380;

} // namespace

SceneGraphPlugin::SceneGraphPlugin()
  : rqt_gui_cpp::Plugin{}
{
  setObjectName("SceneGraphPlugin");
}

SceneGraphPlugin::~SceneGraphPlugin() = default;

void SceneGraphPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  m_widget     = new QWidget;
  auto* layout = new QHBoxLayout{m_widget};
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  auto* splitter = new QSplitter{Qt::Horizontal, m_widget};
  m_side_panel   = new SidePanel;
  m_graph_view   = new GraphView;
  m_image_panel  = new ImagePanel;
  splitter->addWidget(m_side_panel);
  splitter->addWidget(m_graph_view);
  splitter->addWidget(m_image_panel);
  splitter->setStretchFactor(0, 0);
  splitter->setStretchFactor(1, 1);
  splitter->setStretchFactor(2, 0);
  splitter->setSizes({LEFT_PANEL_WIDTH, 600, RIGHT_PANEL_WIDTH});
  layout->addWidget(splitter);

  m_widget->setWindowTitle("Scene Graph View");
  if (context.serialNumber() > 1)
  {
    m_widget->setWindowTitle(m_widget->windowTitle() +
                             QString{" (%1)"}.arg(context.serialNumber()));
  }
  context.addWidget(m_widget);

  // Side panel → plugin / view.
  connect(m_side_panel, &SidePanel::viewModeChanged, this, &SceneGraphPlugin::onViewModeChanged);
  connect(m_side_panel, &SidePanel::requestPhase1, this, [this]() {
    sendActionGoal(UpdateSceneGraph::Goal::PHASE_1);
  });
  connect(m_side_panel, &SidePanel::requestPhase2, this, [this]() {
    sendActionGoal(UpdateSceneGraph::Goal::PHASE_2);
  });
  connect(m_side_panel, &SidePanel::queryRequested, this, &SceneGraphPlugin::onQueryRequested);
  connect(m_side_panel, &SidePanel::queryCancelRequested, this, &SceneGraphPlugin::onQueryCancel);
  connect(m_side_panel,
          &SidePanel::showInspectedToggled,
          this,
          &SceneGraphPlugin::onShowInspectedToggled);

  // Graph view → plugin.
  connect(m_graph_view, &GraphView::nodeSelected, this, &SceneGraphPlugin::onNodeSelected);
  connect(m_graph_view, &GraphView::selectionCleared, this, &SceneGraphPlugin::onSelectionCleared);
  connect(m_graph_view, &GraphView::refreshRequested, this, &SceneGraphPlugin::onRefreshRequested);

  // Cross-thread bridges from ROS callbacks → UI thread.
  connect(this,
          &SceneGraphPlugin::sceneGraphReceived,
          this,
          &SceneGraphPlugin::onSceneGraphReceived,
          Qt::QueuedConnection);
  connect(this,
          &SceneGraphPlugin::managedObjectsReceived,
          this,
          &SceneGraphPlugin::onManagedObjectsReceived,
          Qt::QueuedConnection);
  connect(this,
          &SceneGraphPlugin::visualsReceived,
          this,
          &SceneGraphPlugin::onVisualsReceived,
          Qt::QueuedConnection);
  connect(this,
          &SceneGraphPlugin::actionFeedback,
          this,
          &SceneGraphPlugin::onActionFeedback,
          Qt::QueuedConnection);
  connect(this,
          &SceneGraphPlugin::actionFinished,
          this,
          &SceneGraphPlugin::onActionFinished,
          Qt::QueuedConnection);
  connect(this,
          &SceneGraphPlugin::queryInspected,
          this,
          &SceneGraphPlugin::onQueryInspected,
          Qt::QueuedConnection);
  connect(this,
          &SceneGraphPlugin::queryFinished,
          this,
          &SceneGraphPlugin::onQueryFinished,
          Qt::QueuedConnection);

  // ROS plumbing on the node provided by RQT.
  const auto qos    = rclcpp::QoS{1}.reliable().transient_local();
  m_scene_graph_sub = node_->create_subscription<semantic_scene_forests_msgs::msg::SceneGraph>(
    SCENE_GRAPH_TOPIC,
    qos,
    [this](semantic_scene_forests_msgs::msg::SceneGraph::ConstSharedPtr msg) {
      onSceneGraphMsg(msg);
    });
  m_managed_objects_sub =
    node_->create_subscription<semantic_scene_forests_object_msgs::msg::ManagedObjectArray>(
      MANAGED_OBJECTS_TOPIC,
      qos,
      [this](semantic_scene_forests_object_msgs::msg::ManagedObjectArray::ConstSharedPtr msg) {
        onManagedObjectsMsg(msg);
      });
  m_action_client =
    rclcpp_action::create_client<UpdateSceneGraph>(node_, UPDATE_SCENE_GRAPH_ACTION);
  m_prompt_client = rclcpp_action::create_client<PromptLLM>(node_, PROMPT_LLM_ACTION);
  m_find_client   = rclcpp_action::create_client<FindObject>(node_, FIND_OBJECT_ACTION);
  m_visuals_client =
    node_->create_client<semantic_scene_forests_object_msgs::srv::GetObjectVisuals>(
      GET_OBJECT_VISUALS_SERVICE);

  m_side_panel->setViewMode(m_graph_view->viewMode());
}

void SceneGraphPlugin::shutdownPlugin()
{
  m_scene_graph_sub.reset();
  m_managed_objects_sub.reset();
  m_action_client.reset();
  m_prompt_client.reset();
  m_find_client.reset();
  m_prompt_goal.reset();
  m_find_goal.reset();
  m_visuals_client.reset();
}

void SceneGraphPlugin::saveSettings(qt_gui_cpp::Settings& /*plugin_settings*/,
                                    qt_gui_cpp::Settings& instance_settings) const
{
  instance_settings.setValue("view_mode",
                             static_cast<int>(m_graph_view != nullptr ? m_graph_view->viewMode()
                                                                      : ViewMode::FORCE_DIRECTED));
}

void SceneGraphPlugin::restoreSettings(const qt_gui_cpp::Settings& /*plugin_settings*/,
                                       const qt_gui_cpp::Settings& instance_settings)
{
  if (instance_settings.contains("view_mode"))
  {
    const int raw = instance_settings.value("view_mode").toInt();
    const ViewMode mode =
      (raw == static_cast<int>(ViewMode::LAYERED)) ? ViewMode::LAYERED : ViewMode::FORCE_DIRECTED;
    m_graph_view->setViewMode(mode);
    m_side_panel->setViewMode(mode);
  }
}

void SceneGraphPlugin::onSceneGraphMsg(
  semantic_scene_forests_msgs::msg::SceneGraph::ConstSharedPtr msg)
{
  auto parsed = fzi::semantic_scene_forests::SceneGraph::fromMsg(*msg);
  if (!parsed.has_value())
  {
    return;
  }
  std::shared_ptr<fzi::semantic_scene_forests::SceneGraph> graph = std::move(parsed.value());
  {
    std::lock_guard<std::mutex> lock{m_mutex};
    m_latest_scene_graph_msg = msg;
    m_latest_scene_graph     = graph;
    ++m_latest_message_count;
  }
  emit sceneGraphReceived();
}

void SceneGraphPlugin::onManagedObjectsMsg(
  semantic_scene_forests_object_msgs::msg::ManagedObjectArray::ConstSharedPtr msg)
{
  std::unordered_map<int64_t, std::string> names;
  names.reserve(msg->objects.size());
  for (const auto& obj : msg->objects)
  {
    names.emplace(obj.id, obj.class_name);
  }
  {
    std::lock_guard<std::mutex> lock{m_mutex};
    m_object_class_names = std::move(names);
  }
  emit managedObjectsReceived();
}

void SceneGraphPlugin::onSceneGraphReceived()
{
  std::shared_ptr<const fzi::semantic_scene_forests::SceneGraph> graph;
  std::unordered_map<int64_t, std::string> names;
  std::uint64_t latest_count = 0;
  std::size_t node_count     = 0;
  std::size_t edge_count     = 0;
  {
    std::lock_guard<std::mutex> lock{m_mutex};
    graph        = m_latest_scene_graph;
    names        = m_object_class_names;
    latest_count = m_latest_message_count;
    if (graph != nullptr)
    {
      node_count = graph->getNodes().size();
      edge_count = graph->getEdges().size();
    }
  }

  // Live updates: counts and label names always refresh so the user can see new data arriving.
  m_side_panel->setCounts(node_count, edge_count);
  m_graph_view->setObjectClassNames(std::move(names));

  if (m_displayed_scene_graph == nullptr)
  {
    // First message — populate the view immediately so the user isn't staring at a blank canvas.
    m_displayed_scene_graph   = graph;
    m_displayed_message_count = latest_count;
    m_graph_view->setGraph(graph);
    m_side_panel->setMessageStatus(tr("Scene graph loaded."));
    return;
  }

  if (latest_count > m_displayed_message_count)
  {
    m_graph_view->setHasPendingUpdate(true);
    m_side_panel->setMessageStatus(tr("New scene graph available — press ↻ to refresh."));
  }
}

void SceneGraphPlugin::onManagedObjectsReceived()
{
  std::unordered_map<int64_t, std::string> names;
  {
    std::lock_guard<std::mutex> lock{m_mutex};
    names = m_object_class_names;
  }
  m_graph_view->setObjectClassNames(std::move(names));
}

void SceneGraphPlugin::onVisualsReceived(qint64 object_id)
{
  std::vector<sensor_msgs::msg::Image> images;
  {
    std::lock_guard<std::mutex> lock{m_mutex};
    if (auto it = m_visuals_cache.find(object_id); it != m_visuals_cache.end())
    {
      images = it->second;
    }
  }
  if (object_id != m_selected_object_id)
  {
    return;
  }
  m_image_panel->showImages(object_id, m_selected_object_class, images);
}

void SceneGraphPlugin::onNodeSelected(qint64 node_id, bool is_object)
{
  if (!is_object)
  {
    m_selected_object_id = -1;
    m_image_panel->showPlaceholder(tr("Concept selected — no images."));
    return;
  }
  m_selected_object_id = node_id;
  QString class_name;
  {
    std::lock_guard<std::mutex> lock{m_mutex};
    if (auto it = m_object_class_names.find(node_id); it != m_object_class_names.end())
    {
      class_name = QString::fromStdString(it->second);
    }
  }
  m_selected_object_class = class_name;
  m_image_panel->showLoading(node_id, class_name);
  requestVisuals(node_id);
}

void SceneGraphPlugin::onSelectionCleared()
{
  m_selected_object_id = -1;
  m_selected_object_class.clear();
  m_image_panel->clear();
}

void SceneGraphPlugin::onViewModeChanged(ViewMode mode)
{
  m_graph_view->setViewMode(mode);
}

void SceneGraphPlugin::syncDisplayedGraphToLatest()
{
  std::shared_ptr<const fzi::semantic_scene_forests::SceneGraph> graph;
  std::uint64_t latest_count = 0;
  {
    std::lock_guard<std::mutex> lock{m_mutex};
    graph        = m_latest_scene_graph;
    latest_count = m_latest_message_count;
  }
  if (graph == nullptr || graph == m_displayed_scene_graph)
  {
    return;
  }
  m_displayed_scene_graph   = graph;
  m_displayed_message_count = latest_count;
  m_graph_view->setGraph(graph);
}

void SceneGraphPlugin::onRefreshRequested()
{
  syncDisplayedGraphToLatest();
  m_side_panel->setMessageStatus(tr("Scene graph refreshed."));
}

void SceneGraphPlugin::requestVisuals(int64_t object_id)
{
  if (m_visuals_client == nullptr || !m_visuals_client->service_is_ready())
  {
    return;
  }
  auto request =
    std::make_shared<semantic_scene_forests_object_msgs::srv::GetObjectVisuals::Request>();
  request->object_ids.push_back(object_id);
  m_visuals_client->async_send_request(
    request,
    [this, object_id](
      rclcpp::Client<semantic_scene_forests_object_msgs::srv::GetObjectVisuals>::SharedFuture
        future) {
      auto response = future.get();
      std::vector<sensor_msgs::msg::Image> images;
      for (const auto& visual : response->visuals)
      {
        for (const auto& img : visual.image_crops)
        {
          images.push_back(img);
        }
      }
      {
        std::lock_guard<std::mutex> lock{m_mutex};
        m_visuals_cache[object_id] = std::move(images);
      }
      emit visualsReceived(static_cast<qint64>(object_id));
    });
}

void SceneGraphPlugin::sendActionGoal(std::uint8_t phase)
{
  if (m_action_client == nullptr || !m_action_client->action_server_is_ready())
  {
    m_side_panel->clearActionProgress(tr("Action server not available."));
    return;
  }
  UpdateSceneGraph::Goal goal;
  goal.phase = phase;
  m_side_panel->setActionProgress(0, 0, tr("Sending goal…"));

  rclcpp_action::Client<UpdateSceneGraph>::SendGoalOptions options;
  options.feedback_callback = [this](
                                GoalHandle::SharedPtr /*handle*/,
                                const std::shared_ptr<const UpdateSceneGraph::Feedback> feedback) {
    emit actionFeedback(
      feedback->current_object, feedback->total_objects, QString::fromStdString(feedback->status));
  };
  options.result_callback = [this](const GoalHandle::WrappedResult& result) {
    const bool ok = (result.code == rclcpp_action::ResultCode::SUCCEEDED) &&
                    result.result != nullptr && result.result->success;
    QString message =
      (result.result != nullptr) ? QString::fromStdString(result.result->message) : QString{};
    emit actionFinished(ok, message);
  };
  m_action_client->async_send_goal(goal, options);
}

void SceneGraphPlugin::onActionFeedback(unsigned current, unsigned total, QString status)
{
  m_side_panel->setActionProgress(current, total, status);
}

void SceneGraphPlugin::onActionFinished(bool success, QString message)
{
  const QString prefix = success ? tr("Done") : tr("Failed");
  m_side_panel->clearActionProgress(prefix + (message.isEmpty() ? QString{} : ": " + message));
}

namespace {

// Maps a non-SUCCEEDED result code to a human-readable label for the status line.
QString resultCodeLabel(rclcpp_action::ResultCode code)
{
  switch (code)
  {
    case rclcpp_action::ResultCode::CANCELED:
      return SceneGraphPlugin::tr("Cancelled.");
    case rclcpp_action::ResultCode::ABORTED:
      return SceneGraphPlugin::tr("Aborted by server.");
    default:
      return SceneGraphPlugin::tr("Failed.");
  }
}

template <typename Feedback> QVector<qint64> inspectedIds(const Feedback& feedback)
{
  QVector<qint64> ids;
  ids.reserve(static_cast<int>(feedback->expanded_node_ids.size()));
  for (const auto id : feedback->expanded_node_ids)
  {
    ids.push_back(static_cast<qint64>(id));
  }
  return ids;
}

} // namespace

void SceneGraphPlugin::onQueryRequested(int kind, QString text)
{
  // The query agent reasons over the latest scene graph snapshot. Sync the view to it
  // now so the inspected node ids (and the edges between them) resolve against the same
  // data instead of a stale, pre-refresh snapshot.
  syncDisplayedGraphToLatest();

  const std::string query = text.toStdString();
  m_active_query_is_find  = (kind == 1);

  if (kind == 1) // FindObject
  {
    if (m_find_client == nullptr || !m_find_client->action_server_is_ready())
    {
      m_side_panel->setQueryStatus(tr("FindObject action server not available."));
      return;
    }
    FindObject::Goal goal;
    goal.object_description = query;

    rclcpp_action::Client<FindObject>::SendGoalOptions options;
    options.goal_response_callback =
      [this](rclcpp_action::ClientGoalHandle<FindObject>::SharedPtr handle) {
        std::lock_guard<std::mutex> lock{m_mutex};
        m_find_goal = handle;
        if (handle == nullptr)
        {
          emit queryFinished(false, tr("Goal rejected by server."), -1);
        }
      };
    options.feedback_callback =
      [this](rclcpp_action::ClientGoalHandle<FindObject>::SharedPtr /*handle*/,
             const std::shared_ptr<const FindObject::Feedback> feedback) {
        emit queryInspected(inspectedIds(feedback), QString::fromStdString(feedback->status));
      };
    options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<FindObject>::WrappedResult& result) {
        {
          std::lock_guard<std::mutex> lock{m_mutex};
          m_find_goal.reset();
        }
        const bool ok = (result.code == rclcpp_action::ResultCode::SUCCEEDED) &&
                        result.result != nullptr && result.result->success;
        QString message =
          (result.code != rclcpp_action::ResultCode::SUCCEEDED)
            ? resultCodeLabel(result.code)
            : (result.result != nullptr ? QString::fromStdString(result.result->message)
                                        : QString{});
        const qint64 object_id =
          (ok && result.result != nullptr) ? static_cast<qint64>(result.result->object_id) : -1;
        emit queryFinished(ok, message, object_id);
      };
    m_find_client->async_send_goal(goal, options);
  }
  else // PromptLLM
  {
    if (m_prompt_client == nullptr || !m_prompt_client->action_server_is_ready())
    {
      m_side_panel->setQueryStatus(tr("PromptLLM action server not available."));
      return;
    }
    PromptLLM::Goal goal;
    goal.query = query;

    rclcpp_action::Client<PromptLLM>::SendGoalOptions options;
    options.goal_response_callback =
      [this](rclcpp_action::ClientGoalHandle<PromptLLM>::SharedPtr handle) {
        std::lock_guard<std::mutex> lock{m_mutex};
        m_prompt_goal = handle;
        if (handle == nullptr)
        {
          emit queryFinished(false, tr("Goal rejected by server."), -1);
        }
      };
    options.feedback_callback =
      [this](rclcpp_action::ClientGoalHandle<PromptLLM>::SharedPtr /*handle*/,
             const std::shared_ptr<const PromptLLM::Feedback> feedback) {
        emit queryInspected(inspectedIds(feedback), QString::fromStdString(feedback->status));
      };
    options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<PromptLLM>::WrappedResult& result) {
        {
          std::lock_guard<std::mutex> lock{m_mutex};
          m_prompt_goal.reset();
        }
        const bool ok = (result.code == rclcpp_action::ResultCode::SUCCEEDED) &&
                        result.result != nullptr && result.result->success;
        QString message;
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
        {
          message = resultCodeLabel(result.code);
        }
        else if (result.result != nullptr)
        {
          // Prefer the LLM's free-text response; fall back to the status message.
          message = result.result->response.empty()
                      ? QString::fromStdString(result.result->message)
                      : QString::fromStdString(result.result->response);
        }
        emit queryFinished(ok, message, -1);
      };
    m_prompt_client->async_send_goal(goal, options);
  }

  m_side_panel->setQueryRunning(true);
  m_side_panel->setQueryStatus(tr("Sending query…"));
}

void SceneGraphPlugin::onQueryCancel()
{
  std::lock_guard<std::mutex> lock{m_mutex};
  if (m_prompt_goal != nullptr && m_prompt_client != nullptr)
  {
    m_prompt_client->async_cancel_goal(m_prompt_goal);
  }
  if (m_find_goal != nullptr && m_find_client != nullptr)
  {
    m_find_client->async_cancel_goal(m_find_goal);
  }
  m_side_panel->setQueryStatus(tr("Cancelling…"));
}

void SceneGraphPlugin::onQueryInspected(QVector<qint64> ids, QString status)
{
  m_last_inspected.clear();
  m_last_inspected.reserve(static_cast<std::size_t>(ids.size()));
  for (const qint64 id : ids)
  {
    m_last_inspected.insert(static_cast<int64_t>(id));
  }
  m_graph_view->setInspectedFilter(m_last_inspected);
  if (!status.isEmpty())
  {
    m_side_panel->setQueryStatus(status);
  }
}

void SceneGraphPlugin::onQueryFinished(bool success, QString message, qint64 object_id)
{
  m_side_panel->setQueryRunning(false);
  const QString prefix = success ? tr("Done") : tr("Failed");
  m_side_panel->setQueryStatus(message.isEmpty() ? prefix : (prefix + ": " + message));

  // A successful FindObject carries an answer node. It's often identified from a neighbour
  // listing without ever being expanded, so it isn't part of expanded_node_ids — add it
  // explicitly so it (and its edge to the inspected concepts) is shown.
  const bool has_answer_node = success && m_active_query_is_find;
  if (has_answer_node)
  {
    m_last_inspected.insert(static_cast<int64_t>(object_id));
  }

  if (m_last_inspected.empty())
  {
    // Nothing was inspected — drop the filter and disable the toggle.
    m_graph_view->clearInspectedFilter();
    m_side_panel->setInspectedFilterAvailable(false);
  }
  else
  {
    // Keep showing the final inspected set; let the user toggle it off manually.
    m_graph_view->setInspectedFilter(m_last_inspected);
    m_side_panel->setInspectedFilterAvailable(true);
  }

  if (has_answer_node)
  {
    m_graph_view->focusNode(static_cast<int64_t>(object_id));
  }
}

void SceneGraphPlugin::onShowInspectedToggled(bool enabled)
{
  if (enabled)
  {
    m_graph_view->setInspectedFilter(m_last_inspected);
  }
  else
  {
    m_graph_view->clearInspectedFilter();
  }
}

} // namespace semantic_scene_forests_visualization_rqt
