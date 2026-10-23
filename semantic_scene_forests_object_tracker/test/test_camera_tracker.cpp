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

#include <initializer_list>
#include <string>

#include <sensor_msgs/image_encodings.hpp>

#include "camera_tracker.hpp"

namespace semantic_scene_forests_object_tracker {
namespace {

struct Det
{
  double x;
  std::string label = "box";
  double size       = 0.4;
};

class CameraTrackerTest : public testing::Test
{
protected:
  CameraTrackerTest()
  {
    params.world_frame            = "map";
    params.confirmation_threshold = 3;
    params.crop_padding           = 0;
  }

  CameraTracker::FrameResult frame(CameraTracker& tracker, int sec, std::initializer_list<Det> dets)
  {
    vision_msgs::msg::Detection2DArray two;
    vision_msgs::msg::Detection3DArray three;
    three.header.frame_id  = "map";
    three.header.stamp.sec = sec;
    two.header             = three.header;
    int id                 = 0;
    for (const auto& det : dets)
    {
      vision_msgs::msg::Detection2D a;
      vision_msgs::msg::Detection3D b;
      a.id = b.id = std::to_string(++id);
      vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
      hypothesis.hypothesis.class_id = det.label;
      hypothesis.hypothesis.score    = 0.99;
      a.results.push_back(hypothesis);
      b.results.push_back(hypothesis);
      a.bbox.center.position.x    = 8;
      a.bbox.center.position.y    = 4;
      a.bbox.size_x               = 8;
      a.bbox.size_y               = 4;
      b.bbox.center.position.x    = det.x;
      b.bbox.center.position.z    = 1;
      b.bbox.center.orientation.w = 1;
      b.bbox.size.x = b.bbox.size.y = b.bbox.size.z = det.size;
      two.detections.push_back(a);
      three.detections.push_back(b);
    }
    sensor_msgs::msg::Image image;
    image.header   = three.header;
    image.width    = 32;
    image.height   = 16;
    image.encoding = sensor_msgs::image_encodings::BGR8;
    image.step     = image.width * 3;
    image.data.assign(image.height * image.step, 127);
    const auto IDENTITY = []() -> std::optional<geometry_msgs::msg::TransformStamped> {
      geometry_msgs::msg::TransformStamped transform;
      transform.transform.rotation.w = 1.0;
      return transform;
    };
    return tracker.process(two, three, image, IDENTITY, params, {});
  }

  object_tracker::Params params;
  rclcpp::Logger logger = rclcpp::get_logger("test");
};

TEST_F(CameraTrackerTest, FlickeringLabelKeepsOneTrack)
{
  CameraTracker tracker("front", 0, logger);
  frame(tracker, 1, {{0, "box"}});
  frame(tracker, 2, {{0, "chair"}});
  frame(tracker, 3, {{0, "box"}});
  ASSERT_EQ(tracker.tracklets().size(), 1u);
  EXPECT_EQ(tracker.tracklets()[0].class_name, "box");
}

TEST_F(CameraTrackerTest, SameFrameDuplicatesBecomeOneTrack)
{
  CameraTracker tracker("front", 0, logger);
  for (int sec = 1; sec <= 3; ++sec)
  {
    frame(tracker, sec, {{0, "box"}, {0.05, "crate"}});
  }
  EXPECT_EQ(tracker.trackCount(), 1u);
  EXPECT_EQ(tracker.tracklets().size(), 1u);
}

TEST_F(CameraTrackerTest, IncompatibleSizeStartsNewTrack)
{
  CameraTracker tracker("front", 0, logger);
  for (int sec = 1; sec <= 3; ++sec)
  {
    frame(tracker, sec, {{0}});
  }
  for (int sec = 4; sec <= 6; ++sec)
  {
    frame(tracker, sec, {{0, "box", 1.0}});
  }
  EXPECT_EQ(tracker.tracklets().size(), 2u);
}

TEST_F(CameraTrackerTest, TrackEndsAfterLocalTimeoutAndOldFramesAreRejected)
{
  CameraTracker tracker("front", 0, logger);
  for (int sec = 1; sec <= 3; ++sec)
  {
    frame(tracker, sec, {{0}});
  }
  ASSERT_EQ(tracker.tracklets().size(), 1u);
  EXPECT_EQ(frame(tracker, 3, {{0}}), CameraTracker::FrameResult::OUT_OF_ORDER);
  frame(tracker, 9, {}); // 6 s > local_track_timeout (5 s)
  EXPECT_TRUE(tracker.tracklets().empty());
}

TEST_F(CameraTrackerTest, CropsAreRotatedUpright)
{
  CameraTracker tracker("front", 90, logger);
  for (int sec = 1; sec <= 3; ++sec)
  {
    frame(tracker, sec, {{0}});
  }
  const auto TRACKLETS = tracker.tracklets();
  ASSERT_EQ(TRACKLETS.size(), 1u);
  ASSERT_FALSE(TRACKLETS[0].crops->empty());
  // An 8 x 4 box in the unrotated image is 4 x 8 after a quarter turn.
  EXPECT_EQ(TRACKLETS[0].crops->back().width, 4u);
  EXPECT_EQ(TRACKLETS[0].crops->back().height, 8u);
}

TEST(CameraTracker, RejectsInvalidRotation)
{
  EXPECT_THROW(CameraTracker("front", 45, rclcpp::get_logger("test")), std::invalid_argument);
}

} // namespace
} // namespace semantic_scene_forests_object_tracker
