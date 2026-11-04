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

#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include "image_rotation.hpp"

namespace semantic_scene_forests_object_tracker::detail {
namespace {

TEST(ImageRotation, BoxCoversTheSamePixelAsTheRotatedImage)
{
  // 4 x 2 image with a unique value per pixel; the box covers pixel (x=3, y=0).
  cv::Mat image(2, 4, CV_8UC1);
  for (int i = 0; i < 8; ++i)
  {
    image.data[i] = static_cast<uint8_t>(i + 1);
  }
  vision_msgs::msg::BoundingBox2D box;
  box.center.position.x = 3.5;
  box.center.position.y = 0.5;
  box.size_x            = 1.0;
  box.size_y            = 1.0;
  for (const int ROTATION : {0, 90, 180, 270})
  {
    const cv::Mat ROTATED = rotateImage(image, ROTATION);
    const auto BOX        = rotateBox(box, ROTATION, image.cols, image.rows);
    const int X           = static_cast<int>(BOX.center.position.x);
    const int Y           = static_cast<int>(BOX.center.position.y);
    EXPECT_EQ(ROTATED.at<uint8_t>(Y, X), image.at<uint8_t>(0, 3)) << ROTATION;
  }
}

TEST(ImageRotation, QuarterTurnsSwapBoxSize)
{
  vision_msgs::msg::BoundingBox2D box;
  box.size_x = 8.0;
  box.size_y = 4.0;
  EXPECT_DOUBLE_EQ(rotateBox(box, 90, 32, 16).size_x, 4.0);
  EXPECT_DOUBLE_EQ(rotateBox(box, 180, 32, 16).size_x, 8.0);
  EXPECT_FALSE(isValidRotation(45));
}

} // namespace
} // namespace semantic_scene_forests_object_tracker::detail
