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

#include "semantic_scene_forests_visualization_rqt/node_item.hpp"

#include <QBrush>
#include <QFont>
#include <QFontMetricsF>
#include <QPainter>
#include <QPen>
#include <QPointF>
#include <QString>
#include <QStyleOptionGraphicsItem>

#include "semantic_scene_forests_visualization_rqt/color_palette.hpp"
#include "semantic_scene_forests_visualization_rqt/edge_item.hpp"

namespace semantic_scene_forests_visualization_rqt {

namespace {

constexpr qreal LABEL_GAP   = 4.0;
constexpr int LABEL_FONT_PT = 9;

QFont labelFont()
{
  QFont font{"Monospace", LABEL_FONT_PT};
  font.setStyleHint(QFont::TypeWriter);
  return font;
}

} // namespace

NodeItem::NodeItem(int64_t id, NodeKind kind, const std::string& label)
  : m_id{id}
  , m_kind{kind}
  , m_label{label}
{
  setFlag(QGraphicsItem::ItemIsMovable, true);
  setFlag(QGraphicsItem::ItemIsSelectable, true);
  setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
  setZValue(1.0);
  rebuildStaticLabel();
}

void NodeItem::setLabel(const std::string& label)
{
  if (m_label == label)
  {
    return;
  }
  m_label = label;
  rebuildStaticLabel();
  prepareGeometryChange();
  update();
}

void NodeItem::setHighlighted(bool highlighted)
{
  if (m_highlighted == highlighted)
  {
    return;
  }
  m_highlighted = highlighted;
  update();
}

void NodeItem::rebuildStaticLabel()
{
  const QString text = QString{"[%1] %2"}.arg(m_id).arg(QString::fromStdString(m_label));
  m_static_label.setText(text);
  m_static_label.setTextFormat(Qt::PlainText);
  m_static_label.prepare(QTransform{}, labelFont());
}

QRectF NodeItem::boundingRect() const
{
  const auto label_size = m_static_label.size();
  const qreal r         = radius();
  const qreal half_w    = std::max<qreal>(r, label_size.width() / 2.0) + 2.0;
  const qreal top       = -r - LABEL_GAP - label_size.height() - 2.0;
  const qreal bottom    = r + 2.0;
  return QRectF{QPointF{-half_w, top}, QPointF{half_w, bottom}};
}

QColor NodeItem::fillColor() const
{
  if (isSelected())
  {
    return SELECTED_NODE_COLOR;
  }
  if (m_highlighted)
  {
    return HIGHLIGHT_COLOR;
  }
  if (m_kind == NodeKind::CONCEPT)
  {
    return CONCEPT_NODE_COLOR;
  }
  return OBJECT_NODE_COLOR;
}

void NodeItem::paint(QPainter* painter,
                     const QStyleOptionGraphicsItem* /*option*/,
                     QWidget* /*widget*/)
{
  painter->setRenderHint(QPainter::Antialiasing, true);

  const qreal r     = radius();
  const QColor fill = fillColor();
  QPen border{fill.darker(160)};
  border.setWidthF(1.0);
  painter->setPen(border);
  painter->setBrush(QBrush{fill});
  painter->drawEllipse(QPointF{0.0, 0.0}, r, r);

  painter->setFont(labelFont());
  painter->setPen(QPen{QColor{220, 220, 220}});
  const auto label_size = m_static_label.size();
  const QPointF label_pos{-label_size.width() / 2.0, -r - LABEL_GAP - label_size.height()};
  painter->drawStaticText(label_pos, m_static_label);
}

QVariant NodeItem::itemChange(GraphicsItemChange change, const QVariant& value)
{
  if (change == ItemPositionHasChanged)
  {
    for (auto* edge : m_edges)
    {
      edge->updateGeometry();
    }
  }
  return QGraphicsItem::itemChange(change, value);
}

} // namespace semantic_scene_forests_visualization_rqt
