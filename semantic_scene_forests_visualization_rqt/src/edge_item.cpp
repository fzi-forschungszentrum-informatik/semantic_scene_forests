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

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): fixed parameter order
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
  const QPointF SRC   = m_source->pos();
  const QPointF DST   = m_target->pos();
  const QPointF DELTA = DST - SRC;
  const qreal LENGTH  = std::hypot(DELTA.x(), DELTA.y());
  if (LENGTH < 1e-6)
  {
    m_p1 = SRC;
    m_p2 = DST;
    return;
  }
  const QPointF DIR = DELTA / LENGTH;
  m_p1              = SRC + (DIR * (m_source->radius() + NODE_MARGIN_PAD));
  m_p2              = DST - (DIR * (m_target->radius() + NODE_MARGIN_PAD));
  update();
}

QRectF EdgeItem::boundingRect() const
{
  const qreal MIN_X = std::min(m_p1.x(), m_p2.x());
  const qreal MIN_Y = std::min(m_p1.y(), m_p2.y());
  const qreal MAX_X = std::max(m_p1.x(), m_p2.x());
  const qreal MAX_Y = std::max(m_p1.y(), m_p2.y());
  const qreal PAD   = ARROW_SIZE + 4.0 + m_static_label.size().height();
  return QRectF{QPointF{MIN_X - PAD, MIN_Y - PAD}, QPointF{MAX_X + PAD, MAX_Y + PAD}};
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
  const QPointF DELTA = m_p2 - m_p1;
  const qreal LENGTH  = std::hypot(DELTA.x(), DELTA.y());
  if (LENGTH < 1e-6)
  {
    return;
  }
  const qreal ANGLE      = std::atan2(DELTA.y(), DELTA.x());
  const qreal WING_ANGLE = std::numbers::pi / 7.0;
  const QPointF TIP      = m_p2;
  const QPointF LEFT{TIP.x() - (ARROW_SIZE * std::cos(ANGLE - WING_ANGLE)),
                     TIP.y() - (ARROW_SIZE * std::sin(ANGLE - WING_ANGLE))};
  const QPointF RIGHT{TIP.x() - (ARROW_SIZE * std::cos(ANGLE + WING_ANGLE)),
                      TIP.y() - (ARROW_SIZE * std::sin(ANGLE + WING_ANGLE))};
  QPolygonF head;
  head << TIP << LEFT << RIGHT;
  painter->setBrush(QBrush{m_color});
  painter->drawPolygon(head);

  // Label at midpoint, offset slightly perpendicular to the line.
  painter->setFont(edgeLabelFont());
  painter->setPen(QPen{m_color.lighter(140)});
  const QPointF MID = (m_p1 + m_p2) / 2.0;
  const QPointF PERP{-DELTA.y() / LENGTH, DELTA.x() / LENGTH};
  const auto LABEL_SIZE = m_static_label.size();
  const QPointF LABEL_POS =
    MID + (PERP * 6.0) - QPointF{LABEL_SIZE.width() / 2.0, LABEL_SIZE.height() / 2.0};
  painter->drawStaticText(LABEL_POS, m_static_label);
}

} // namespace semantic_scene_forests_visualization_rqt
