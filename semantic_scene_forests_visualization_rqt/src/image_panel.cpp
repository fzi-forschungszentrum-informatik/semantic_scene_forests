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

#include "semantic_scene_forests_visualization_rqt/image_panel.hpp"

#include <QFrame>
#include <QLabel>
#include <QPixmap>
#include <QScrollArea>
#include <QSizePolicy>
#include <QVBoxLayout>
#include <Qt>

#include "semantic_scene_forests_visualization_rqt/image_utils.hpp"

namespace semantic_scene_forests_visualization_rqt {

namespace {

constexpr int IMAGE_MAX_WIDTH = 360;

} // namespace

ImagePanel::ImagePanel(QWidget* parent)
  : QWidget{parent}
{
  auto* outer = new QVBoxLayout{this};
  outer->setContentsMargins(4, 4, 4, 4);
  outer->setSpacing(4);

  m_header = new QLabel{tr("No selection")};
  m_header->setStyleSheet("font-weight: bold;");
  outer->addWidget(m_header);

  m_scroll = new QScrollArea{this};
  m_scroll->setWidgetResizable(true);
  m_scroll->setFrameShape(QFrame::NoFrame);
  outer->addWidget(m_scroll, 1);

  m_content        = new QWidget{m_scroll};
  m_content_layout = new QVBoxLayout{m_content};
  m_content_layout->setContentsMargins(0, 0, 0, 0);
  m_content_layout->setSpacing(8);
  m_content_layout->addStretch(1);
  m_scroll->setWidget(m_content);

  showPlaceholder(tr("Select an object node to view images."));
}

void ImagePanel::replaceContent()
{
  // Remove children of m_content_layout.
  while (auto* item = m_content_layout->takeAt(0))
  {
    if (auto* widget = item->widget())
    {
      widget->deleteLater();
    }
    delete item;
  }
}

void ImagePanel::clear()
{
  showPlaceholder(tr("Select an object node to view images."));
}

void ImagePanel::showPlaceholder(const QString& text)
{
  m_header->setText(tr("Images"));
  replaceContent();
  auto* msg = new QLabel{text, m_content};
  msg->setAlignment(Qt::AlignCenter);
  msg->setWordWrap(true);
  msg->setStyleSheet("color: #aaaaaa;");
  m_content_layout->addWidget(msg);
  m_content_layout->addStretch(1);
}

void ImagePanel::showLoading(int64_t object_id, const QString& class_name)
{
  m_header->setText(tr("Object [%1] %2").arg(object_id).arg(class_name));
  replaceContent();
  auto* msg = new QLabel{tr("Loading images…"), m_content};
  msg->setAlignment(Qt::AlignCenter);
  m_content_layout->addWidget(msg);
  m_content_layout->addStretch(1);
}

void ImagePanel::showImages(int64_t object_id,
                            const QString& class_name,
                            const std::vector<sensor_msgs::msg::Image>& images)
{
  m_header->setText(tr("Object [%1] %2").arg(object_id).arg(class_name));
  replaceContent();
  if (images.empty())
  {
    auto* msg = new QLabel{tr("No images available."), m_content};
    msg->setAlignment(Qt::AlignCenter);
    msg->setStyleSheet("color: #aaaaaa;");
    m_content_layout->addWidget(msg);
    m_content_layout->addStretch(1);
    return;
  }
  for (std::size_t i = 0; i < images.size(); ++i)
  {
    QPixmap pixmap = imageMsgToPixmap(images[i]);
    if (pixmap.width() > IMAGE_MAX_WIDTH)
    {
      pixmap = pixmap.scaledToWidth(IMAGE_MAX_WIDTH, Qt::SmoothTransformation);
    }
    auto* caption = new QLabel{
      tr("Image %1 (%2)").arg(i + 1).arg(QString::fromStdString(images[i].encoding)), m_content};
    caption->setStyleSheet("color: #cccccc;");
    m_content_layout->addWidget(caption);

    auto* picture = new QLabel{m_content};
    picture->setPixmap(pixmap);
    picture->setAlignment(Qt::AlignCenter);
    m_content_layout->addWidget(picture);
  }
  m_content_layout->addStretch(1);
}

} // namespace semantic_scene_forests_visualization_rqt
