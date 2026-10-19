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

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>

#include "tracker_geometry.hpp"

namespace {

using semantic_scene_forests_object_tracker::detail::transformBoundingBoxToFrame;

constexpr double K_TOLERANCE  = 1e-9;
constexpr double K_HALF_SQRT2 = 0.7071067811865476; // sqrt(2)/2, a 90 deg half angle

// A 2x1x1 box under a 90 deg yaw must come out 1x2x1: the extents rotate with
// the transform, the translation moves the center, and the world-axis-aligned
// result carries identity orientation.
TEST(TransformBoundingBoxToFrame, Yaw90SwapsExtentsAndMovesCenter)
{
  vision_msgs::msg::BoundingBox3D box; // identity orientation by default (w=1)
  box.size.x = 2.0;
  box.size.y = 1.0;
  box.size.z = 1.0;

  geometry_msgs::msg::TransformStamped transform;
  transform.transform.translation.x = 1.0;
  transform.transform.translation.y = 2.0;
  transform.transform.translation.z = 3.0;
  transform.transform.rotation.z    = K_HALF_SQRT2;
  transform.transform.rotation.w    = K_HALF_SQRT2;

  auto world_box = transformBoundingBoxToFrame(box, transform);
  if (!world_box)
  {
    FAIL() << "expected a world-frame box";
  }
  auto out = world_box.value();
  EXPECT_NEAR(out.size.x, 1.0, K_TOLERANCE);
  EXPECT_NEAR(out.size.y, 2.0, K_TOLERANCE);
  EXPECT_NEAR(out.size.z, 1.0, K_TOLERANCE);
  EXPECT_NEAR(out.center.position.x, 1.0, K_TOLERANCE);
  EXPECT_NEAR(out.center.position.y, 2.0, K_TOLERANCE);
  EXPECT_NEAR(out.center.position.z, 3.0, K_TOLERANCE);
  EXPECT_NEAR(out.center.orientation.x, 0.0, K_TOLERANCE);
  EXPECT_NEAR(out.center.orientation.y, 0.0, K_TOLERANCE);
  EXPECT_NEAR(out.center.orientation.z, 0.0, K_TOLERANCE);
  EXPECT_NEAR(out.center.orientation.w, 1.0, K_TOLERANCE);
}

// Box yaw90 composed with camera roll90 must yield extents (1,3,2); the
// reversed rotation product would give (3,2,1).
TEST(TransformBoundingBoxToFrame, BoxAndCameraRotationsCompose)
{
  vision_msgs::msg::BoundingBox3D box;
  box.center.position.x    = 1.0;
  box.center.position.y    = 1.0;
  box.center.position.z    = 1.0;
  box.size.x               = 2.0;
  box.size.y               = 1.0;
  box.size.z               = 3.0;
  box.center.orientation.z = K_HALF_SQRT2;
  box.center.orientation.w = K_HALF_SQRT2;

  geometry_msgs::msg::TransformStamped transform;
  transform.transform.translation.x = 1.0;
  transform.transform.translation.y = 2.0;
  transform.transform.translation.z = 3.0;
  transform.transform.rotation.x    = K_HALF_SQRT2;
  transform.transform.rotation.w    = K_HALF_SQRT2;

  auto world_box = transformBoundingBoxToFrame(box, transform);
  if (!world_box)
  {
    FAIL() << "expected a world-frame box";
  }
  auto out = world_box.value();
  EXPECT_NEAR(out.size.x, 1.0, K_TOLERANCE);
  EXPECT_NEAR(out.size.y, 3.0, K_TOLERANCE);
  EXPECT_NEAR(out.size.z, 2.0, K_TOLERANCE);
  EXPECT_NEAR(out.center.position.x, 2.0, K_TOLERANCE);
  EXPECT_NEAR(out.center.position.y, 1.0, K_TOLERANCE);
  EXPECT_NEAR(out.center.position.z, 4.0, K_TOLERANCE);
}

// Zero size and a zero-norm quaternion are rejected, not silently repaired.
TEST(TransformBoundingBoxToFrame, RejectsZeroSizeAndZeroQuaternion)
{
  geometry_msgs::msg::TransformStamped transform; // identity rotation, zero translation

  vision_msgs::msg::BoundingBox3D zero_size; // all dimensions default to 0
  EXPECT_FALSE(transformBoundingBoxToFrame(zero_size, transform).has_value());

  vision_msgs::msg::BoundingBox3D box;
  box.size.x               = 2.0;
  box.size.y               = 1.0;
  box.size.z               = 1.0;
  box.center.orientation.w = 0.0; // zero-norm orientation
  EXPECT_FALSE(transformBoundingBoxToFrame(box, transform).has_value());
}

} // namespace
