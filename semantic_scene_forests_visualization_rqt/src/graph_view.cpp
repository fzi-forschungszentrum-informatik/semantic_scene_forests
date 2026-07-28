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

#include "semantic_scene_forests_visualization_rqt/graph_view.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <queue>
#include <random>
#include <utility>
#include <variant>

#include <QMouseEvent>
#include <QResizeEvent>
#include <QScrollBar>
#include <QSignalBlocker>
#include <QToolButton>
#include <QWheelEvent>

#include "semantic_scene_forests_visualization_rqt/color_palette.hpp"
#include "semantic_scene_forests_visualization_rqt/edge_item.hpp"
#include "semantic_scene_forests_visualization_rqt/node_item.hpp"
#include <semantic_scene_forests/graph_nodes.hpp>
#include <semantic_scene_forests/scene_graph.hpp>

namespace semantic_scene_forests_visualization_rqt {

namespace {

constexpr qreal NODE_SPACING      = 180.0;
constexpr qreal LAYER_MIN_SPACING = 300.0;
// Wide bounds: fitInView can leave the initial transform far below 1.0 for big scenes,
// so the bounds must accommodate that or the wheel zoom-in gets clamped before it can run.
constexpr qreal MIN_SCALE   = 0.001;
constexpr qreal MAX_SCALE   = 100.0;
constexpr qreal ZOOM_FACTOR = 1.25;

// Hard upper bound on FR iterations; early-stop on small max displacement still triggers earlier.
constexpr int FORCE_STEPS               = 40;
constexpr int FORCE_TIMER_INTERVAL_MS   = 33;
constexpr qreal FORCE_TIMESTEP          = 0.04;
constexpr qreal FORCE_DAMPING           = 0.88;
constexpr qreal FORCE_REPULSION         = 12000.0;
constexpr qreal FORCE_ATTRACTION        = 0.06;
constexpr qreal FORCE_GLOBAL_ATTRACTION = 0.02;
constexpr qreal FORCE_MAX_VELOCITY      = 60.0;

bool isConcept(const fzi::semantic_scene_forests::SceneGraphNode& node)
{
  return std::holds_alternative<fzi::semantic_scene_forests::ConceptNode>(node);
}

int64_t nodeId(const fzi::semantic_scene_forests::SceneGraphNode& node)
{
  return std::visit([](const auto& n) -> int64_t { return n.id; }, node);
}

std::string nodeName(const fzi::semantic_scene_forests::SceneGraphNode& node)
{
  return std::visit(
    [](const auto& n) -> std::string {
      using T = std::decay_t<decltype(n)>;
      if constexpr (std::is_same_v<T, fzi::semantic_scene_forests::ConceptNode>)
      {
        return n.concept_name;
      }
      else
      {
        return n.class_name;
      }
    },
    node);
}

} // namespace

GraphView::GraphView(QWidget* parent)
  : QGraphicsView{parent}
{
  setScene(&m_scene);
  m_scene.setBackgroundBrush(QBrush{BACKGROUND_COLOR});
  setRenderHint(QPainter::Antialiasing, true);
  setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  setResizeAnchor(QGraphicsView::AnchorViewCenter);
  setDragMode(QGraphicsView::RubberBandDrag);
  // Hidden scrollbars: in some Qt builds an active scrollbar intercepts the wheel before
  // QGraphicsView::wheelEvent runs, which broke zoom in tall layered layouts. Navigation
  // is done via wheel-zoom and middle-button drag-pan (handled below).
  setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

  m_layout_timer.setInterval(FORCE_TIMER_INTERVAL_MS);
  connect(&m_layout_timer, &QTimer::timeout, this, &GraphView::stepForceLayout);
  connect(&m_scene, &QGraphicsScene::selectionChanged, this, &GraphView::onSelectionChanged);

  m_refresh_button = new QToolButton{this};
  m_refresh_button->setText("↻");
  m_refresh_button->setToolTip(tr("Refresh scene graph view"));
  m_refresh_button->setAutoRaise(false);
  m_refresh_button->setFixedSize(32, 32);
  m_refresh_button->raise();
  updateRefreshButtonStyle();
  connect(m_refresh_button, &QToolButton::clicked, this, &GraphView::refreshRequested);
  positionRefreshButton();
}

void GraphView::positionRefreshButton()
{
  if (m_refresh_button == nullptr)
  {
    return;
  }
  const int margin = 8;
  m_refresh_button->move(viewport()->width() - m_refresh_button->width() - margin, margin);
}

void GraphView::updateRefreshButtonStyle()
{
  if (m_refresh_button == nullptr)
  {
    return;
  }
  // Highlight when new data is waiting. Matches the Rust app's orange accent.
  const QString style = m_pending_update
                          ? "QToolButton { background: #ff9a3c; color: #1e1e24; border: 1px solid"
                            " #ff7a1c; border-radius: 4px; font-size: 18px; font-weight: bold; }"
                            "QToolButton:hover { background: #ffaf5c; }"
                          : "QToolButton { background: #3a3a44; color: #dddddd; border: 1px solid"
                            " #555560; border-radius: 4px; font-size: 18px; }"
                            "QToolButton:hover { background: #4a4a56; }";
  m_refresh_button->setStyleSheet(style);
}

void GraphView::setHasPendingUpdate(bool pending)
{
  if (m_pending_update == pending)
  {
    return;
  }
  m_pending_update = pending;
  updateRefreshButtonStyle();
}

GraphView::~GraphView() = default;

void GraphView::setGraph(std::shared_ptr<const fzi::semantic_scene_forests::SceneGraph> graph)
{
  m_graph = std::move(graph);
  rebuildScene();
}

void GraphView::setObjectClassNames(std::unordered_map<int64_t, std::string> names)
{
  m_object_class_names = std::move(names);
  for (auto& [id, item] : m_nodes)
  {
    if (item->kind() == NodeKind::OBJECT)
    {
      item->setLabel(objectLabelFor(id, item->label()));
    }
  }
}

void GraphView::setViewMode(ViewMode mode)
{
  if (m_view_mode == mode)
  {
    return;
  }
  m_view_mode = mode;
  rebuildScene();
}

void GraphView::setHiddenRoots(std::unordered_set<int64_t> hidden)
{
  m_hidden_roots = std::move(hidden);
  rebuildScene();
}

void GraphView::setInspectedFilter(std::unordered_set<int64_t> ids)
{
  m_inspected_filter = std::move(ids);
  rebuildScene();
}

void GraphView::clearInspectedFilter()
{
  if (!m_inspected_filter.has_value())
  {
    return;
  }
  m_inspected_filter.reset();
  rebuildScene();
}

void GraphView::focusNode(int64_t id)
{
  auto it = m_nodes.find(id);
  if (it == m_nodes.end())
  {
    return;
  }
  m_scene.clearSelection();
  it->second->setSelected(true);
  centerOn(it->second);
}

std::vector<TopLevelConcept> GraphView::topLevelConcepts() const
{
  std::vector<TopLevelConcept> result;
  if (m_graph == nullptr)
  {
    return result;
  }
  for (const auto& cn : m_graph->getConceptNodes())
  {
    const auto outgoing = m_graph->outgoingEdges(cn.id);
    const bool is_sink  = !outgoing.has_value() || outgoing.value().empty();
    if (is_sink)
    {
      result.push_back({cn.id, QString::fromStdString(cn.concept_name)});
    }
  }
  std::sort(result.begin(), result.end(), [](const TopLevelConcept& a, const TopLevelConcept& b) {
    return a.id < b.id;
  });
  return result;
}

std::string GraphView::objectLabelFor(int64_t id, const std::string& fallback) const
{
  if (auto it = m_object_class_names.find(id); it != m_object_class_names.end())
  {
    return it->second;
  }
  return fallback.empty() ? "object" : fallback;
}

std::unordered_set<int64_t> GraphView::reachableFromVisibleRoots() const
{
  std::unordered_set<int64_t> included;
  if (m_graph == nullptr)
  {
    return included;
  }
  const auto top_level = topLevelConcepts();
  if (top_level.empty())
  {
    // No top-level concepts identified — include everything.
    for (const auto& node : m_graph->getNodes())
    {
      included.insert(nodeId(node));
    }
    return included;
  }

  std::queue<int64_t> queue;
  for (const auto& tlc : top_level)
  {
    if (m_hidden_roots.contains(tlc.id))
    {
      continue;
    }
    if (included.insert(tlc.id).second)
    {
      queue.push(tlc.id);
    }
  }
  while (!queue.empty())
  {
    const int64_t current = queue.front();
    queue.pop();
    auto incoming = m_graph->incomingEdges(current);
    if (!incoming.has_value())
    {
      continue;
    }
    for (const auto& [src_id, edge] : incoming.value())
    {
      if (included.insert(src_id).second)
      {
        queue.push(src_id);
      }
    }
  }
  return included;
}

void GraphView::clearScene()
{
  m_layout_timer.stop();
  m_layout_steps_remaining = 0;
  // Drop our lookup tables BEFORE clearing the scene: m_scene.clear() destroys the
  // items, which fires selectionChanged. Our slot would otherwise touch dangling
  // pointers in m_nodes/m_edges. Block signals as a belt-and-braces measure.
  m_nodes.clear();
  m_edges.clear();
  const QSignalBlocker blocker{&m_scene};
  m_scene.clear();
}

void GraphView::rebuildScene()
{
  clearScene();
  setHasPendingUpdate(false);
  if (m_graph == nullptr)
  {
    emit topLevelConceptsChanged();
    return;
  }

  const auto included =
    m_inspected_filter.has_value() ? *m_inspected_filter : reachableFromVisibleRoots();

  for (const auto& node : m_graph->getNodes())
  {
    const int64_t id = nodeId(node);
    if (!included.contains(id))
    {
      continue;
    }
    const bool is_concept = isConcept(node);
    const NodeKind kind   = is_concept ? NodeKind::CONCEPT : NodeKind::OBJECT;
    std::string label     = nodeName(node);
    if (!is_concept)
    {
      label = objectLabelFor(id, label);
    }
    auto* item = new NodeItem{id, kind, label};
    m_scene.addItem(item);
    m_nodes.emplace(id, item);
  }

  for (const auto& [src_id, tgt_id, edge] : m_graph->getEdges())
  {
    auto src_it = m_nodes.find(src_id);
    auto tgt_it = m_nodes.find(tgt_id);
    if (src_it == m_nodes.end() || tgt_it == m_nodes.end())
    {
      continue;
    }
    auto* edge_item = new EdgeItem{src_it->second, tgt_it->second, edge.label};
    m_scene.addItem(edge_item);
    src_it->second->registerEdge(edge_item);
    tgt_it->second->registerEdge(edge_item);
    m_edges.push_back(edge_item);
  }

  if (m_view_mode == ViewMode::LAYERED)
  {
    computeLayeredPositions();
  }
  else
  {
    computeForceDirectedInitial();
    m_layout_steps_remaining = FORCE_STEPS;
    m_layout_timer.start();
  }

  for (auto* edge : m_edges)
  {
    edge->updateGeometry();
  }

  updateSceneRectFromItems();
  fitContent();

  emit topLevelConceptsChanged();
}

void GraphView::updateSceneRectFromItems()
{
  if (m_nodes.empty())
  {
    setSceneRect(-200, -200, 400, 400);
    return;
  }
  QRectF bounds = m_scene.itemsBoundingRect();
  // Pad so nodes near the edge aren't flush against the viewport border.
  const qreal pad = 200.0;
  bounds.adjust(-pad, -pad, pad, pad);
  setSceneRect(bounds);
}

void GraphView::fitContent()
{
  if (m_nodes.empty())
  {
    return;
  }
  fitInView(sceneRect(), Qt::KeepAspectRatio);
}

void GraphView::computeForceDirectedInitial()
{
  if (m_nodes.empty())
  {
    return;
  }
  const qreal radius = NODE_SPACING * std::sqrt(static_cast<qreal>(m_nodes.size()));
  std::mt19937 rng{42};
  std::uniform_real_distribution<qreal> angle_dist{0.0, 2.0 * std::numbers::pi};
  for (auto& [id, item] : m_nodes)
  {
    const qreal angle = angle_dist(rng);
    item->setPos(radius * std::cos(angle), radius * std::sin(angle));
  }
}

void GraphView::computeLayeredPositions()
{
  if (m_graph == nullptr || m_nodes.empty())
  {
    return;
  }

  std::unordered_set<int64_t> concept_ids;
  std::unordered_map<int64_t, std::vector<int64_t>> concept_outgoing;
  for (const auto& [id, item] : m_nodes)
  {
    if (item->kind() == NodeKind::CONCEPT)
    {
      concept_ids.insert(id);
    }
  }
  for (const auto& [src_id, tgt_id, edge] : m_graph->getEdges())
  {
    if (!m_nodes.contains(src_id) || !m_nodes.contains(tgt_id))
    {
      continue;
    }
    if (concept_ids.contains(src_id))
    {
      concept_outgoing[src_id].push_back(tgt_id);
    }
  }

  std::unordered_map<int64_t, std::uint32_t> layers;
  layers.reserve(concept_ids.size());
  for (int64_t id : concept_ids)
  {
    layers[id] = 0;
  }
  bool changed = true;
  while (changed)
  {
    changed = false;
    for (int64_t id : concept_ids)
    {
      auto succ_it = concept_outgoing.find(id);
      if (succ_it == concept_outgoing.end() || succ_it->second.empty())
      {
        continue;
      }
      std::uint32_t max_succ = 0;
      for (int64_t s : succ_it->second)
      {
        if (auto layer_it = layers.find(s); layer_it != layers.end())
        {
          max_succ = std::max(max_succ, layer_it->second);
        }
      }
      const std::uint32_t new_layer = max_succ + 1;
      if (layers[id] < new_layer)
      {
        layers[id] = new_layer;
        changed    = true;
      }
    }
  }

  std::uint32_t max_concept_layer = 0;
  for (const auto& [id, l] : layers)
  {
    max_concept_layer = std::max(max_concept_layer, l);
  }
  const std::uint32_t object_layer = max_concept_layer + 1;

  std::unordered_map<std::uint32_t, std::vector<int64_t>> layer_groups;
  for (const auto& [id, item] : m_nodes)
  {
    const std::uint32_t layer = (item->kind() == NodeKind::CONCEPT) ? layers[id] : object_layer;
    layer_groups[layer].push_back(id);
  }
  for (auto& [layer, group] : layer_groups)
  {
    std::sort(group.begin(), group.end());
  }

  std::size_t max_layer_count = 1;
  for (const auto& [layer, group] : layer_groups)
  {
    max_layer_count = std::max(max_layer_count, group.size());
  }
  const qreal layer_spacing =
    std::max(static_cast<qreal>(max_layer_count) * NODE_SPACING / 4.0, LAYER_MIN_SPACING);

  for (const auto& [layer, group] : layer_groups)
  {
    const qreal y     = static_cast<qreal>(layer) * layer_spacing;
    const qreal count = static_cast<qreal>(group.size());
    for (std::size_t i = 0; i < group.size(); ++i)
    {
      const qreal x = (static_cast<qreal>(i) - ((count - 1.0) / 2.0)) * NODE_SPACING;
      m_nodes[group[i]]->setPos(x, y);
    }
  }
}

void GraphView::stepForceLayout()
{
  if (m_nodes.empty() || m_layout_steps_remaining <= 0)
  {
    m_layout_timer.stop();
    return;
  }
  --m_layout_steps_remaining;

  std::unordered_map<int64_t, QPointF> forces;
  forces.reserve(m_nodes.size());
  for (const auto& [id, item] : m_nodes)
  {
    forces[id] = QPointF{0.0, 0.0};
  }

  // All-pairs repulsion.
  std::vector<std::pair<int64_t, NodeItem*>> nodes_vec{m_nodes.begin(), m_nodes.end()};
  for (std::size_t i = 0; i < nodes_vec.size(); ++i)
  {
    for (std::size_t j = i + 1; j < nodes_vec.size(); ++j)
    {
      const QPointF delta   = nodes_vec[i].second->pos() - nodes_vec[j].second->pos();
      qreal dist_sq         = (delta.x() * delta.x()) + (delta.y() * delta.y());
      dist_sq               = std::max(dist_sq, 1.0);
      const qreal force_mag = FORCE_REPULSION / dist_sq;
      const qreal dist      = std::sqrt(dist_sq);
      const QPointF unit    = delta / dist;
      forces[nodes_vec[i].first] += unit * force_mag;
      forces[nodes_vec[j].first] -= unit * force_mag;
    }
  }

  // Edge attraction.
  for (const auto* edge : m_edges)
  {
    if (edge->source() == nullptr || edge->target() == nullptr)
    {
      continue;
    }
    const QPointF delta = edge->target()->pos() - edge->source()->pos();
    const QPointF pull  = delta * FORCE_ATTRACTION;
    forces[edge->source()->id()] += pull;
    forces[edge->target()->id()] -= pull;
  }

  // Apply forces with simple Euler integration + damping; clamp velocity for stability.
  qreal max_disp = 0.0;
  for (auto& [id, item] : m_nodes)
  {
    QPointF velocity = forces[id] * FORCE_TIMESTEP;
    velocity -= item->pos() * FORCE_GLOBAL_ATTRACTION;
    velocity *= FORCE_DAMPING;
    const qreal vmag = std::hypot(velocity.x(), velocity.y());
    if (vmag > FORCE_MAX_VELOCITY)
    {
      velocity *= FORCE_MAX_VELOCITY / vmag;
    }
    item->setPos(item->pos() + velocity);
    max_disp = std::max(max_disp, vmag);
  }

  for (auto* edge : m_edges)
  {
    edge->updateGeometry();
  }

  if (max_disp < 1e-2)
  {
    m_layout_timer.stop();
    m_layout_steps_remaining = 0;
  }
  updateSceneRectFromItems();
}

void GraphView::onSelectionChanged()
{
  const auto selected = m_scene.selectedItems();
  if (selected.isEmpty())
  {
    updateHighlighting();
    emit selectionCleared();
    return;
  }
  auto* node = qgraphicsitem_cast<NodeItem*>(selected.first());
  if (node == nullptr)
  {
    return;
  }
  updateHighlighting();
  emit nodeSelected(static_cast<qint64>(node->id()), node->kind() == NodeKind::OBJECT);
}

void GraphView::updateHighlighting()
{
  // If a concept node is selected, highlight every node reachable via incoming edges
  // (i.e. its sub-DAG of children). Matches the Rust HIGHLIGHT_COLOR behavior.
  std::unordered_set<int64_t> highlighted;
  if (m_graph != nullptr)
  {
    for (auto* item : m_scene.selectedItems())
    {
      auto* node = qgraphicsitem_cast<NodeItem*>(item);
      if (node == nullptr || node->kind() != NodeKind::CONCEPT)
      {
        continue;
      }
      std::queue<int64_t> queue;
      queue.push(node->id());
      highlighted.insert(node->id());
      while (!queue.empty())
      {
        const int64_t current = queue.front();
        queue.pop();
        auto incoming = m_graph->incomingEdges(current);
        if (!incoming.has_value())
        {
          continue;
        }
        for (const auto& [src_id, edge] : incoming.value())
        {
          if (highlighted.insert(src_id).second)
          {
            queue.push(src_id);
          }
        }
      }
    }
  }
  for (auto& [id, item] : m_nodes)
  {
    item->setHighlighted(highlighted.contains(id));
  }
}

void GraphView::wheelEvent(QWheelEvent* event)
{
  const int delta_y =
    event->angleDelta().y() != 0 ? event->angleDelta().y() : event->pixelDelta().y();
  if (delta_y == 0)
  {
    event->ignore();
    return;
  }
  const qreal current = transform().m11();
  const qreal factor  = (delta_y > 0) ? ZOOM_FACTOR : (1.0 / ZOOM_FACTOR);
  const qreal next    = current * factor;
  if (next < MIN_SCALE || next > MAX_SCALE)
  {
    event->accept();
    return;
  }
  scale(factor, factor);
  event->accept();
}

void GraphView::mousePressEvent(QMouseEvent* event)
{
  if (event->button() == Qt::MiddleButton)
  {
    m_panning   = true;
    m_pan_start = event->pos();
    setCursor(Qt::ClosedHandCursor);
    event->accept();
    return;
  }
  QGraphicsView::mousePressEvent(event);
}

void GraphView::mouseMoveEvent(QMouseEvent* event)
{
  if (m_panning)
  {
    const QPoint delta = event->pos() - m_pan_start;
    m_pan_start        = event->pos();
    horizontalScrollBar()->setValue(horizontalScrollBar()->value() - delta.x());
    verticalScrollBar()->setValue(verticalScrollBar()->value() - delta.y());
    event->accept();
    return;
  }
  QGraphicsView::mouseMoveEvent(event);
}

void GraphView::resizeEvent(QResizeEvent* event)
{
  QGraphicsView::resizeEvent(event);
  positionRefreshButton();
}

void GraphView::mouseReleaseEvent(QMouseEvent* event)
{
  if (m_panning && event->button() == Qt::MiddleButton)
  {
    m_panning = false;
    unsetCursor();
    event->accept();
    return;
  }
  QGraphicsView::mouseReleaseEvent(event);
}

} // namespace semantic_scene_forests_visualization_rqt
