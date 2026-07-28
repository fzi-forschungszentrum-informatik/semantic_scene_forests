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

#include "semantic_scene_forests_visualization_rqt/image_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include <QImage>
#include <Qt>

namespace semantic_scene_forests_visualization_rqt {

namespace {

constexpr int PLACEHOLDER_SIZE = 200;

QPixmap makePlaceholder()
{
  QPixmap placeholder{PLACEHOLDER_SIZE, PLACEHOLDER_SIZE};
  placeholder.fill(Qt::darkGray);
  return placeholder;
}

QPixmap rgba8ToPixmap(const sensor_msgs::msg::Image& img, int w, int h, int step)
{
  QImage qi{img.data.data(), w, h, step, QImage::Format_RGBA8888};
  return QPixmap::fromImage(qi.copy());
}

QPixmap bgra8ToPixmap(const sensor_msgs::msg::Image& img, int w, int h, int step)
{
  // ARGB32 on little-endian has byte order B,G,R,A which matches bgra8.
  QImage qi{img.data.data(), w, h, step, QImage::Format_ARGB32};
  return QPixmap::fromImage(qi.copy());
}

QPixmap mono16ToPixmap(const sensor_msgs::msg::Image& img, int w, int h, int step)
{
  std::uint16_t min_val = std::numeric_limits<std::uint16_t>::max();
  std::uint16_t max_val = 0;
  for (int row = 0; row < h; ++row)
  {
    const auto* row_ptr = reinterpret_cast<const std::uint16_t*>(img.data.data() + (row * step));
    for (int col = 0; col < w; ++col)
    {
      min_val = std::min(min_val, row_ptr[col]);
      max_val = std::max(max_val, row_ptr[col]);
    }
  }
  const float range = (max_val > min_val) ? static_cast<float>(max_val - min_val) : 1.0F;
  QImage qi{w, h, QImage::Format_Grayscale8};
  for (int row = 0; row < h; ++row)
  {
    const auto* row_ptr = reinterpret_cast<const std::uint16_t*>(img.data.data() + (row * step));
    auto* dst           = qi.scanLine(row);
    for (int col = 0; col < w; ++col)
    {
      dst[col] = static_cast<std::uint8_t>((row_ptr[col] - min_val) / range * 255.0F);
    }
  }
  return QPixmap::fromImage(qi);
}

} // namespace

QPixmap imageMsgToPixmap(const sensor_msgs::msg::Image& img)
{
  if (img.data.empty() || img.width == 0 || img.height == 0)
  {
    return makePlaceholder();
  }

  const auto w    = static_cast<int>(img.width);
  const auto h    = static_cast<int>(img.height);
  const auto step = static_cast<int>(img.step);

  if (img.encoding == "rgb8")
  {
    QImage qi{img.data.data(), w, h, step, QImage::Format_RGB888};
    return QPixmap::fromImage(qi.copy());
  }

  if (img.encoding == "bgr8")
  {
    QImage qi{img.data.data(), w, h, step, QImage::Format_BGR888};
    return QPixmap::fromImage(qi.copy());
  }

  if (img.encoding == "rgba8")
  {
    return rgba8ToPixmap(img, w, h, step);
  }

  if (img.encoding == "bgra8")
  {
    return bgra8ToPixmap(img, w, h, step);
  }

  if (img.encoding == "mono8")
  {
    QImage qi{img.data.data(), w, h, step, QImage::Format_Grayscale8};
    return QPixmap::fromImage(qi.copy());
  }

  if (img.encoding == "mono16")
  {
    return mono16ToPixmap(img, w, h, step);
  }

  return makePlaceholder();
}

} // namespace semantic_scene_forests_visualization_rqt
