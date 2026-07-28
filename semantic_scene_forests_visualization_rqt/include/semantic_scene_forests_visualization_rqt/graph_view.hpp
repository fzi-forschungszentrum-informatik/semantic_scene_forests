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
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <QGraphicsScene>
#include <QGraphicsView>
#include <QObject>
#include <QPointF>
#include <QString>
#include <QTimer>

class QToolButton;

namespace fzi::semantic_scene_forests {
class SceneGraph;
} // namespace fzi::semantic_scene_forests

namespace semantic_scene_forests_visualization_rqt {

class NodeItem;
class EdgeItem;

enum class ViewMode : std::uint8_t
{
  FORCE_DIRECTED,
  LAYERED,
};

struct TopLevelConcept
{
  int64_t id;
  QString label;
};

class GraphView : public QGraphicsView
{
  Q_OBJECT

public:
  explicit GraphView(QWidget* parent = nullptr);
  ~GraphView() override;

  void setGraph(std::shared_ptr<const fzi::semantic_scene_forests::SceneGraph> graph);
  void setObjectClassNames(std::unordered_map<int64_t, std::string> names);

  void setViewMode(ViewMode mode);
  [[nodiscard]] ViewMode viewMode() const { return m_view_mode; }

  void setHiddenRoots(std::unordered_set<int64_t> hidden);
  [[nodiscard]] const std::unordered_set<int64_t>& hiddenRoots() const { return m_hidden_roots; }

  // Restricts the scene to the given node ids ("inspected"/expanded nodes streamed by a
  // semantic-query action). While active it takes precedence over the hidden-roots filter.
  void setInspectedFilter(std::unordered_set<int64_t> ids);
  void clearInspectedFilter();
  [[nodiscard]] bool inspectedFilterActive() const { return m_inspected_filter.has_value(); }

  // Selects and centers the node with the given id, if present in the current scene.
  void focusNode(int64_t id);

  [[nodiscard]] std::vector<TopLevelConcept> topLevelConcepts() const;

  [[nodiscard]] std::size_t visibleNodeCount() const { return m_nodes.size(); }
  [[nodiscard]] std::size_t visibleEdgeCount() const { return m_edges.size(); }

  void rebuildScene();
  void setHasPendingUpdate(bool pending);
  [[nodiscard]] bool hasPendingUpdate() const { return m_pending_update; }

signals:
  void nodeSelected(qint64 node_id, bool is_object);
  void selectionCleared();
  void topLevelConceptsChanged();
  void refreshRequested();

protected:
  void wheelEvent(QWheelEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;

private slots:
  void onSelectionChanged();
  void stepForceLayout();

  // NOLINTNEXTLINE(readability-redundant-access-specifiers) — Qt slots need their own section.
private:
  std::shared_ptr<const fzi::semantic_scene_forests::SceneGraph> m_graph;
  std::unordered_map<int64_t, std::string> m_object_class_names;
  std::unordered_set<int64_t> m_hidden_roots;
  std::optional<std::unordered_set<int64_t>> m_inspected_filter;
  ViewMode m_view_mode{ViewMode::FORCE_DIRECTED};

  QGraphicsScene m_scene;
  std::unordered_map<int64_t, NodeItem*> m_nodes;
  std::vector<EdgeItem*> m_edges;

  QTimer m_layout_timer;
  int m_layout_steps_remaining{0};

  bool m_panning{false};
  QPoint m_pan_start;

  QToolButton* m_refresh_button{nullptr};
  bool m_pending_update{false};

  void positionRefreshButton();
  void updateRefreshButtonStyle();

  void clearScene();
  void computeForceDirectedInitial();
  void computeLayeredPositions();
  void applyFilter();
  void updateHighlighting();
  void fitContent();
  void updateSceneRectFromItems();
  [[nodiscard]] std::unordered_set<int64_t> reachableFromVisibleRoots() const;
  [[nodiscard]] std::string objectLabelFor(int64_t id, const std::string& fallback) const;
};

} // namespace semantic_scene_forests_visualization_rqt
