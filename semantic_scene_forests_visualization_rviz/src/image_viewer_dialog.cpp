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

#include "semantic_scene_forests_visualization_rviz/image_viewer_dialog.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include <QGridLayout>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QPixmap>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QtGlobal>

namespace semantic_scene_forests_visualization_rviz {

ImageViewerDialog::ImageViewerDialog(int64_t object_id,
                                     const std::string& class_name,
                                     const std::vector<sensor_msgs::msg::Image>& images,
                                     QWidget* parent)
  : QDialog(parent)
{
  setWindowTitle(QString("Object %1: %2").arg(object_id).arg(QString::fromStdString(class_name)));
  setMinimumSize(400, 300);
  resize(600, 500);

  auto* outer = new QVBoxLayout(this);
  outer->setSpacing(8);
  outer->setContentsMargins(8, 8, 8, 8);

  auto* title = new QLabel(QString("<b>Object %1</b> — %2 — %3 image(s)")
                             .arg(object_id)
                             .arg(QString::fromStdString(class_name))
                             .arg(static_cast<int>(images.size())));
  outer->addWidget(title);

  auto* scroll_area = new QScrollArea();
  scroll_area->setWidgetResizable(true);
  outer->addWidget(scroll_area, 1);

  auto* grid_widget = new QWidget();
  auto* grid        = new QGridLayout(grid_widget);
  grid->setSpacing(8);
  scroll_area->setWidget(grid_widget);

  static constexpr int K_COLUMNS    = 3;
  static constexpr int K_THUMB_SIZE = 200;

  for (int i = 0; i < static_cast<int>(images.size()); ++i)
  {
    auto* cell        = new QWidget();
    auto* cell_layout = new QVBoxLayout(cell);
    cell_layout->setSpacing(2);
    cell_layout->setContentsMargins(0, 0, 0, 0);

    auto* img_label = new QLabel();
    img_label->setAlignment(Qt::AlignCenter);
    img_label->setMinimumSize(K_THUMB_SIZE, K_THUMB_SIZE);
    img_label->setMaximumSize(K_THUMB_SIZE, K_THUMB_SIZE);

    const QPixmap PX = imageToPixmap(images[static_cast<size_t>(i)]);
    img_label->setPixmap(
      PX.scaled(K_THUMB_SIZE, K_THUMB_SIZE, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    cell_layout->addWidget(img_label);

    auto* info_label =
      new QLabel(QString("Image %1\n%2×%3\n%4")
                   .arg(i + 1)
                   .arg(images[static_cast<size_t>(i)].width)
                   .arg(images[static_cast<size_t>(i)].height)
                   .arg(QString::fromStdString(images[static_cast<size_t>(i)].encoding)));
    info_label->setAlignment(Qt::AlignCenter);
    info_label->setWordWrap(true);
    cell_layout->addWidget(info_label);

    grid->addWidget(cell, i / K_COLUMNS, i % K_COLUMNS);
  }

  auto* btn_row = new QHBoxLayout();
  btn_row->addStretch();
  auto* close_btn = new QPushButton("Close");
  connect(close_btn, &QPushButton::clicked, this, &QDialog::accept);
  btn_row->addWidget(close_btn);
  outer->addLayout(btn_row);
}

QPixmap ImageViewerDialog::imageToPixmap(const sensor_msgs::msg::Image& img)
{
  if (img.data.empty() || img.width == 0 || img.height == 0)
  {
    QPixmap placeholder(200, 200);
    placeholder.fill(Qt::darkGray);
    return placeholder;
  }

  const auto W    = static_cast<int>(img.width);
  const auto H    = static_cast<int>(img.height);
  const auto STEP = static_cast<int>(img.step);

  if (img.encoding == "rgb8")
  {
    QImage qi(img.data.data(), W, H, STEP, QImage::Format_RGB888);
    return QPixmap::fromImage(qi.copy());
  }

  if (img.encoding == "bgr8")
  {
#if (QT_VERSION >= QT_VERSION_CHECK(5, 14, 0))
    QImage qi(img.data.data(), W, H, STEP, QImage::Format_BGR888);
    return QPixmap::fromImage(qi.copy());
#else
    // Manual R↔B swap for Qt < 5.14
    QImage qi(w, h, QImage::Format_RGB888);
    for (int row = 0; row < h; ++row)
    {
      const uint8_t* src = img.data.data() + row * step;
      uint8_t* dst       = qi.scanLine(row);
      for (int col = 0; col < w; ++col)
      {
        dst[col * 3 + 0] = src[col * 3 + 2];
        dst[col * 3 + 1] = src[col * 3 + 1];
        dst[col * 3 + 2] = src[col * 3 + 0];
      }
    }
    return QPixmap::fromImage(qi);
#endif
  }

  if (img.encoding == "mono8")
  {
    QImage qi(img.data.data(), W, H, STEP, QImage::Format_Grayscale8);
    return QPixmap::fromImage(qi.copy());
  }

  if (img.encoding == "32FC1")
  {
    float min_val = std::numeric_limits<float>::max();
    float max_val = std::numeric_limits<float>::lowest();

    for (int row = 0; row < H; ++row)
    {
      const auto* row_ptr = reinterpret_cast<const float*>(img.data.data() + row * STEP);
      for (int col = 0; col < W; ++col)
      {
        const float V = row_ptr[col];
        if (std::isfinite(V))
        {
          min_val = std::min(min_val, V);
          max_val = std::max(max_val, V);
        }
      }
    }

    const float RANGE = (max_val > min_val) ? (max_val - min_val) : 1.0F;
    QImage qi(W, H, QImage::Format_Grayscale8);
    for (int row = 0; row < H; ++row)
    {
      const auto* row_ptr = reinterpret_cast<const float*>(img.data.data() + row * STEP);
      uint8_t* dst        = qi.scanLine(row);
      for (int col = 0; col < W; ++col)
      {
        const float V = row_ptr[col];
        dst[col]      = std::isfinite(V) ? static_cast<uint8_t>((V - min_val) / RANGE * 255.0F) : 0;
      }
    }
    return QPixmap::fromImage(qi);
  }

  // Unsupported encoding — grey placeholder with label
  QPixmap placeholder(200, 200);
  placeholder.fill(Qt::darkGray);
  return placeholder;
}

} // namespace semantic_scene_forests_visualization_rviz
