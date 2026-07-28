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

#ifndef SEMANTIC_SCENE_FORESTS_VISUALIZATION_RVIZ__IMAGE_VIEWER_DIALOG_HPP_
#define SEMANTIC_SCENE_FORESTS_VISUALIZATION_RVIZ__IMAGE_VIEWER_DIALOG_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <QDialog>

#include "sensor_msgs/msg/image.hpp"

namespace semantic_scene_forests_visualization_rviz {

class ImageViewerDialog : public QDialog
{
  Q_OBJECT

public:
  explicit ImageViewerDialog(int64_t object_id,
                             const std::string& class_name,
                             const std::vector<sensor_msgs::msg::Image>& images,
                             QWidget* parent = nullptr);

private:
  static QPixmap imageToPixmap(const sensor_msgs::msg::Image& img);
};

} // namespace semantic_scene_forests_visualization_rviz

#endif // SEMANTIC_SCENE_FORESTS_VISUALIZATION_RVIZ__IMAGE_VIEWER_DIALOG_HPP_
