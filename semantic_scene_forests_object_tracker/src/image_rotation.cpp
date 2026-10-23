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

#include "image_rotation.hpp"

#include <opencv2/core.hpp>
#include <stdexcept>

namespace semantic_scene_forests_object_tracker::detail {

bool isValidRotation(int degrees_cw)
{
  return degrees_cw == 0 || degrees_cw == 90 || degrees_cw == 180 || degrees_cw == 270;
}

cv::Mat rotateImage(const cv::Mat& image, int degrees_cw)
{
  cv::Mat rotated;
  switch (degrees_cw)
  {
    case 0:
      return image;
    case 90:
      cv::rotate(image, rotated, cv::ROTATE_90_CLOCKWISE);
      return rotated;
    case 180:
      cv::rotate(image, rotated, cv::ROTATE_180);
      return rotated;
    case 270:
      cv::rotate(image, rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
      return rotated;
    default:
      throw std::invalid_argument("image rotation must be 0, 90, 180 or 270 degrees");
  }
}

vision_msgs::msg::BoundingBox2D
rotateBox(const vision_msgs::msg::BoundingBox2D& box, int degrees_cw, int width, int height)
{
  // Continuous pixel coordinates: 90 deg CW maps (x, y) to (height - y, x).
  auto rotated    = box;
  const double X  = box.center.position.x;
  const double Y  = box.center.position.y;
  auto& center    = rotated.center.position;
  const bool SWAP = degrees_cw == 90 || degrees_cw == 270;
  switch (degrees_cw)
  {
    case 0:
      break;
    case 90:
      center.x = height - Y;
      center.y = X;
      break;
    case 180:
      center.x = width - X;
      center.y = height - Y;
      break;
    case 270:
      center.x = Y;
      center.y = width - X;
      break;
    default:
      throw std::invalid_argument("image rotation must be 0, 90, 180 or 270 degrees");
  }
  if (SWAP)
  {
    rotated.size_x = box.size_y;
    rotated.size_y = box.size_x;
  }
  return rotated;
}

} // namespace semantic_scene_forests_object_tracker::detail
