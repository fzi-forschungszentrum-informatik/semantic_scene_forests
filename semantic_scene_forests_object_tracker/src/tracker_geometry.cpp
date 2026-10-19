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

#include "tracker_geometry.hpp"

#include <cmath>

#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>

namespace semantic_scene_forests_object_tracker::detail {

std::optional<vision_msgs::msg::BoundingBox3D>
transformBoundingBoxToFrame(const vision_msgs::msg::BoundingBox3D& bbox,
                            const geometry_msgs::msg::TransformStamped& transform)
{
  const auto& rotation        = transform.transform.rotation;
  const auto& translation     = transform.transform.translation;
  const auto& center          = bbox.center.position;
  const auto& size            = bbox.size;
  const auto& box_orientation = bbox.center.orientation;

  bool inputs_finite  = std::isfinite(center.x) && std::isfinite(center.y) &&
                        std::isfinite(center.z) && std::isfinite(size.x) && std::isfinite(size.y) &&
                        std::isfinite(size.z) && std::isfinite(translation.x) &&
                        std::isfinite(translation.y) && std::isfinite(translation.z) &&
                        std::isfinite(rotation.x) && std::isfinite(rotation.y) &&
                        std::isfinite(rotation.z) && std::isfinite(rotation.w) &&
                        std::isfinite(box_orientation.x) && std::isfinite(box_orientation.y) &&
                        std::isfinite(box_orientation.z) && std::isfinite(box_orientation.w);
  bool sizes_positive = size.x > 0.0 && size.y > 0.0 && size.z > 0.0;

  if (!inputs_finite || !sizes_positive)
  {
    return std::nullopt;
  }

  double rotation_norm = std::sqrt((rotation.x * rotation.x) + (rotation.y * rotation.y) +
                                   (rotation.z * rotation.z) + (rotation.w * rotation.w));
  double box_orientation_norm =
    std::sqrt((box_orientation.x * box_orientation.x) + (box_orientation.y * box_orientation.y) +
              (box_orientation.z * box_orientation.z) + (box_orientation.w * box_orientation.w));
  if (!std::isfinite(rotation_norm) || rotation_norm <= 0.0 ||
      !std::isfinite(box_orientation_norm) || box_orientation_norm <= 0.0)
  {
    return std::nullopt;
  }

  // Normalize valid quaternions rather than replace them; w=0 half turns stay valid.
  tf2::Quaternion tf_rotation(rotation.x / rotation_norm,
                              rotation.y / rotation_norm,
                              rotation.z / rotation_norm,
                              rotation.w / rotation_norm);
  tf2::Quaternion normalized_box_orientation(box_orientation.x / box_orientation_norm,
                                             box_orientation.y / box_orientation_norm,
                                             box_orientation.z / box_orientation_norm,
                                             box_orientation.w / box_orientation_norm);

  tf2::Matrix3x3 r_tf(tf_rotation);
  tf2::Matrix3x3 r_box(normalized_box_orientation);
  // Box orientation applies before the camera rotation: R_tf * R_box.
  tf2::Matrix3x3 r_composed = r_tf * r_box;

  // World-axis-aligned extents: per-axis sum of |composed-rotation rows| * size.
  tf2::Vector3 box_size(size.x, size.y, size.z);
  tf2::Vector3 world_extents(0.0, 0.0, 0.0);
  for (int row = 0; row < 3; ++row)
  {
    const tf2::Vector3& composed_row = r_composed.getRow(row);
    world_extents[row]               = (std::abs(composed_row.x()) * box_size.x()) +
                                       (std::abs(composed_row.y()) * box_size.y()) +
                                       (std::abs(composed_row.z()) * box_size.z());
  }

  tf2::Vector3 box_center(center.x, center.y, center.z);
  tf2::Vector3 world_center =
    r_tf * box_center + tf2::Vector3(translation.x, translation.y, translation.z);

  if (!std::isfinite(world_center.x()) || !std::isfinite(world_center.y()) ||
      !std::isfinite(world_center.z()) || !std::isfinite(world_extents.x()) ||
      !std::isfinite(world_extents.y()) || !std::isfinite(world_extents.z()))
  {
    return std::nullopt;
  }

  vision_msgs::msg::BoundingBox3D world_bbox;
  world_bbox.center.position.x = world_center.x();
  world_bbox.center.position.y = world_center.y();
  world_bbox.center.position.z = world_center.z();
  // The published extents are world-axis-aligned, so the orientation is identity.
  world_bbox.center.orientation.x = 0.0;
  world_bbox.center.orientation.y = 0.0;
  world_bbox.center.orientation.z = 0.0;
  world_bbox.center.orientation.w = 1.0;
  world_bbox.size.x               = world_extents.x();
  world_bbox.size.y               = world_extents.y();
  world_bbox.size.z               = world_extents.z();
  return world_bbox;
}

} // namespace semantic_scene_forests_object_tracker::detail
