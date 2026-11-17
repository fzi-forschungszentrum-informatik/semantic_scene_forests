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

#include <memory>
#include <optional>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>

namespace semantic_scene_forests_object_tracker {
namespace detail {

// Transforms a bounding box from its source frame into the target frame of
// `transform`: world center = R_tf * center + translation, world extents from
// the composed rotation R_tf * R_box, and an identity orientation because the
// resulting extents are world-axis-aligned. Returns nullopt when the geometry
// is non-finite, degenerate (nonpositive sizes) or either quaternion has a
// zero/nonfinite norm.
std::optional<vision_msgs::msg::BoundingBox3D>
transformBoundingBoxToFrame(const vision_msgs::msg::BoundingBox3D& bbox,
                            const geometry_msgs::msg::TransformStamped& transform);

// JPEG-encodes a BGR crop (cv_bridge format string, so cv_bridge decodes it again). Null if
// OpenCV cannot encode it.
std::shared_ptr<const sensor_msgs::msg::CompressedImage>
encodeCrop(const cv::Mat& bgr, const std_msgs::msg::Header& header, int jpeg_quality);

// Same for a raw image message (add_object, add_observations); null if it cannot be converted.
std::shared_ptr<const sensor_msgs::msg::CompressedImage>
encodeCrop(const sensor_msgs::msg::Image& image, int jpeg_quality);

} // namespace detail
} // namespace semantic_scene_forests_object_tracker
