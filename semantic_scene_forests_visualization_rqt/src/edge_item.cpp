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

#include "semantic_scene_forests_visualization_rqt/edge_item.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include <QBrush>
#include <QFont>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QPolygonF>
#include <QString>
#include <QStyleOptionGraphicsItem>

#include "semantic_scene_forests_visualization_rqt/color_palette.hpp"
#include "semantic_scene_forests_visualization_rqt/node_item.hpp"

namespace semantic_scene_forests_visualization_rqt {

namespace {

constexpr qreal ARROW_SIZE      = 8.0;
constexpr qreal STROKE_WIDTH    = 1.25;
constexpr qreal NODE_MARGIN_PAD = 1.0;
constexpr int LABEL_FONT_PT     = 8;

QFont edgeLabelFont()
{
  QFont font{"Monospace", LABEL_FONT_PT};
  font.setStyleHint(QFont::TypeWriter);
  return font;
}

} // namespace

EdgeItem::EdgeItem(NodeItem* source, NodeItem* target, const std::string& label)
  : m_source{source}
  , m_target{target}
  , m_label{label}
  , m_color{labelColor(label)}
{
  setZValue(0.0);
  m_static_label.setText(QString::fromStdString(label));
  m_static_label.setTextFormat(Qt::PlainText);
  m_static_label.prepare(QTransform{}, edgeLabelFont());
  updateGeometry();
}

void EdgeItem::updateGeometry()
{
  prepareGeometryChange();
  if (m_source == nullptr || m_target == nullptr)
  {
    m_p1 = QPointF{0.0, 0.0};
    m_p2 = QPointF{0.0, 0.0};
    return;
  }
  const QPointF src   = m_source->pos();
  const QPointF dst   = m_target->pos();
  const QPointF delta = dst - src;
  const qreal length  = std::hypot(delta.x(), delta.y());
  if (length < 1e-6)
  {
    m_p1 = src;
    m_p2 = dst;
    return;
  }
  const QPointF dir = delta / length;
  m_p1              = src + (dir * (m_source->radius() + NODE_MARGIN_PAD));
  m_p2              = dst - (dir * (m_target->radius() + NODE_MARGIN_PAD));
  update();
}

QRectF EdgeItem::boundingRect() const
{
  const qreal min_x = std::min(m_p1.x(), m_p2.x());
  const qreal min_y = std::min(m_p1.y(), m_p2.y());
  const qreal max_x = std::max(m_p1.x(), m_p2.x());
  const qreal max_y = std::max(m_p1.y(), m_p2.y());
  const qreal pad   = ARROW_SIZE + 4.0 + m_static_label.size().height();
  return QRectF{QPointF{min_x - pad, min_y - pad}, QPointF{max_x + pad, max_y + pad}};
}

QPainterPath EdgeItem::shape() const
{
  QPainterPath path;
  path.moveTo(m_p1);
  path.lineTo(m_p2);
  QPainterPathStroker stroker;
  stroker.setWidth(STROKE_WIDTH + 4.0);
  return stroker.createStroke(path);
}

void EdgeItem::paint(QPainter* painter,
                     const QStyleOptionGraphicsItem* /*option*/,
                     QWidget* /*widget*/)
{
  if (m_source == nullptr || m_target == nullptr)
  {
    return;
  }
  painter->setRenderHint(QPainter::Antialiasing, true);

  QPen pen{m_color};
  pen.setWidthF(STROKE_WIDTH);
  painter->setPen(pen);
  painter->drawLine(m_p1, m_p2);

  // Arrowhead at m_p2 pointing along (m_p2 - m_p1).
  const QPointF delta = m_p2 - m_p1;
  const qreal length  = std::hypot(delta.x(), delta.y());
  if (length < 1e-6)
  {
    return;
  }
  const qreal angle      = std::atan2(delta.y(), delta.x());
  const qreal wing_angle = std::numbers::pi / 7.0;
  const QPointF tip      = m_p2;
  const QPointF left{tip.x() - (ARROW_SIZE * std::cos(angle - wing_angle)),
                     tip.y() - (ARROW_SIZE * std::sin(angle - wing_angle))};
  const QPointF right{tip.x() - (ARROW_SIZE * std::cos(angle + wing_angle)),
                      tip.y() - (ARROW_SIZE * std::sin(angle + wing_angle))};
  QPolygonF head;
  head << tip << left << right;
  painter->setBrush(QBrush{m_color});
  painter->drawPolygon(head);

  // Label at midpoint, offset slightly perpendicular to the line.
  painter->setFont(edgeLabelFont());
  painter->setPen(QPen{m_color.lighter(140)});
  const QPointF mid = (m_p1 + m_p2) / 2.0;
  const QPointF perp{-delta.y() / length, delta.x() / length};
  const auto label_size = m_static_label.size();
  const QPointF label_pos =
    mid + (perp * 6.0) - QPointF{label_size.width() / 2.0, label_size.height() / 2.0};
  painter->drawStaticText(label_pos, m_static_label);
}

} // namespace semantic_scene_forests_visualization_rqt
