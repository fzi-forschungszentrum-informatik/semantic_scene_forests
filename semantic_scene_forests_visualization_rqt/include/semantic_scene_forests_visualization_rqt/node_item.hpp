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
#include <string>
#include <vector>

#include <QColor>
#include <QGraphicsItem>
#include <QRectF>
#include <QStaticText>

namespace semantic_scene_forests_visualization_rqt {

class EdgeItem;

enum class NodeKind : std::uint8_t
{
  CONCEPT,
  OBJECT,
};

class NodeItem : public QGraphicsItem
{
public:
  static constexpr qreal CONCEPT_RADIUS = 8.0;
  static constexpr qreal OBJECT_RADIUS  = 16.0;

  NodeItem(int64_t id, NodeKind kind, const std::string& label);

  [[nodiscard]] qreal radius() const
  {
    return m_kind == NodeKind::OBJECT ? OBJECT_RADIUS : CONCEPT_RADIUS;
  }

  [[nodiscard]] int64_t id() const { return m_id; }
  [[nodiscard]] NodeKind kind() const { return m_kind; }
  [[nodiscard]] const std::string& label() const { return m_label; }

  void setLabel(const std::string& label);
  void setHighlighted(bool highlighted);
  [[nodiscard]] bool highlighted() const { return m_highlighted; }

  void registerEdge(EdgeItem* edge) { m_edges.push_back(edge); }
  [[nodiscard]] const std::vector<EdgeItem*>& edges() const { return m_edges; }

  [[nodiscard]] QRectF boundingRect() const override;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

protected:
  QVariant itemChange(GraphicsItemChange change, const QVariant& value) override;

private:
  int64_t m_id;
  NodeKind m_kind;
  std::string m_label;
  QStaticText m_static_label;
  bool m_highlighted{false};
  std::vector<EdgeItem*> m_edges;

  void rebuildStaticLabel();
  [[nodiscard]] QColor fillColor() const;
};

} // namespace semantic_scene_forests_visualization_rqt
