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

#include <limits>
#include <stdexcept>

#include "semantic_scene_forests_object_tracker/tracked_object.hpp"
#include "tracker_geometry.hpp"

namespace semantic_scene_forests_object_tracker {
namespace {

rclcpp::Time observationTime(int32_t seconds, uint32_t nanoseconds = 0)
{
  return rclcpp::Time(seconds, nanoseconds, RCL_ROS_TIME);
}

DetectionAdapter detection(double x = 0.0, double size = 0.2, const std::string& label = "cup")
{
  DetectionAdapter result;
  result.class_name                  = label;
  result.class_id                    = 7;
  result.score                       = 0.8;
  result.frame_id                    = "world";
  result.bbox3d.center.position.x    = x;
  result.bbox3d.center.orientation.w = 1.0;
  result.bbox3d.size.x               = size;
  result.bbox3d.size.y               = size;
  result.bbox3d.size.z               = size;
  return result;
}

TEST(TrackedObject, CenterNoiseDoesNotInflateDimensions)
{
  TrackedObject object(detection(), observationTime(10), 1, 0.1, 0.1, 2, 2);
  for (int i = 0; i < 100; ++i)
  {
    object.predict(0.1);
    object.update(detection(i % 2 == 0 ? 0.15 : -0.15),
                  observationTime(10) + rclcpp::Duration::from_nanoseconds((i + 1) * 100000000LL));
  }
  EXPECT_TRUE(object.bbox3d_size.isApprox(Eigen::Vector3d::Constant(0.2), 1e-9));
}

TEST(TrackedObject, DimensionsRecoverAfterOversizedMeasurement)
{
  TrackedObject object(detection(), observationTime(10), 1, 0.1, 0.1, 2, 2);
  object.update(detection(0.0, 2.0), observationTime(11));
  for (int i = 0; i < 30; ++i)
  {
    object.update(detection(), observationTime(12 + i));
  }
  for (int axis = 0; axis < 3; ++axis)
  {
    EXPECT_NEAR(object.bbox3d_size[axis], 0.2, 0.01);
  }
}

TEST(TrackedObject, PredictionAndSerializationPreserveObservationTime)
{
  TrackedObject object(detection(), observationTime(1792713600, 123456789), 1, 0.1, 0.1, 2, 2);
  object.update(detection(0.1), observationTime(1792713600, 123456790));
  object.predict(20.0);
  EXPECT_EQ(object.last_seen, observationTime(1792713600, 123456790));
  const auto message = object.toDetection3D();
  EXPECT_EQ(message.header.stamp.sec, 1792713600);
  EXPECT_EQ(message.header.stamp.nanosec, 123456790u);
  EXPECT_EQ(message.header.frame_id, "world");
}

TEST(TrackedObject, MergePreservesNewestCoherentStateAndSurvivorHistory)
{
  TrackedObject survivor(detection(-1.0), observationTime(10), 1, 0.1, 0.1, 2, 2);
  survivor.update(detection(-0.8), observationTime(11));
  TrackedObject newer(detection(1.0, 0.4), observationTime(12), 2, 0.1, 0.1, 2, 2);
  auto latest                        = detection(1.2, 0.6);
  latest.score                       = 0.95;
  latest.bbox3d.center.orientation.z = 0.6;
  latest.bbox3d.center.orientation.w = 0.8;
  latest.bbox2d.size_x               = 42.0;
  newer.predict(2.0);
  newer.update(latest, observationTime(13, 500000000));
  sensor_msgs::msg::Image crop;
  crop.width = 1;
  survivor.addCrop(crop);
  crop.width = 2;
  newer.addCrop(crop);
  crop.width = 3;
  newer.addCrop(crop);

  survivor.mergeFrom(newer);
  EXPECT_EQ(survivor.track_id, 1u);
  EXPECT_EQ(survivor.last_seen, observationTime(13, 500000000));
  EXPECT_TRUE(survivor.getPosition().isApprox(newer.getPosition()));
  EXPECT_TRUE(survivor.getPositionCovariance().isApprox(newer.getPositionCovariance()));
  EXPECT_TRUE(survivor.bbox3d_size.isApprox(newer.bbox3d_size));
  EXPECT_DOUBLE_EQ(survivor.score, 0.95);
  EXPECT_DOUBLE_EQ(survivor.last_detection.bbox2d.size_x, 42.0);
  EXPECT_DOUBLE_EQ(survivor.toDetection3D().bbox.center.orientation.z, 0.6);
  EXPECT_EQ(survivor.toDetection3D().header.stamp.sec, 13);
  EXPECT_TRUE(survivor.confirmed);
  ASSERT_EQ(survivor.getCrops().size(), 2u);
  EXPECT_EQ(survivor.getCrops()[0].width, 2u);
  EXPECT_EQ(survivor.getCrops()[1].width, 3u);

  // An older merge must not overwrite the state just adopted.
  TrackedObject older(detection(-2.0), observationTime(9), 3, 0.1, 0.1, 2, 2);
  survivor.mergeFrom(older);
  EXPECT_EQ(survivor.last_seen, observationTime(13, 500000000));
  EXPECT_TRUE(survivor.getPosition().isApprox(newer.getPosition()));
  // Combined cup history must still outvote a new class observation.
  auto other_class       = detection();
  other_class.class_name = "bottle";
  other_class.class_id   = 8;
  for (int i = 0; i < 3; ++i)
  {
    survivor.update(other_class, observationTime(14 + i));
  }
  EXPECT_EQ(survivor.class_name, "cup");
  EXPECT_EQ(survivor.class_id, 7);
}

TEST(TrackedObject, FirstObservationConfirmsAtThresholdOne)
{
  TrackedObject object(detection(), observationTime(10), 1, 0.1, 0.1, 1, 2);
  EXPECT_TRUE(object.confirmed);
  EXPECT_EQ(object.observation_count, 1);
}

TEST(TrackedObject, ConfiguredWeightControlsDimensionFiltering)
{
  TrackedObject object(detection(), observationTime(10), 1, 0.1, 0.1, 2, 2, 0.5);
  object.update(detection(0.15, 0.6), observationTime(11));
  EXPECT_TRUE(object.bbox3d_size.isApprox(Eigen::Vector3d::Constant(0.4)));
  TrackedObject unfiltered(detection(), observationTime(10), 2, 0.1, 0.1, 2, 2, 1.0);
  unfiltered.update(detection(0.15, 0.6), observationTime(11));
  EXPECT_TRUE(unfiltered.bbox3d_size.isApprox(Eigen::Vector3d::Constant(0.6)));
}

TEST(TrackedObject, RejectsInvalidDimensionFilterWeight)
{
  for (double alpha : {0.0, -0.2, 1.1, std::numeric_limits<double>::quiet_NaN()})
  {
    EXPECT_THROW((TrackedObject(detection(), observationTime(10), 1, 0.1, 0.1, 2, 2, alpha)),
                 std::invalid_argument);
  }
}

TEST(TrackedObject, DecidedLabelWinsUntilANewLabelAppears)
{
  TrackedObject object(detection(), observationTime(10), 1, 0.1, 0.1, 2, 2);
  object.update(detection(), observationTime(11));
  object.update(detection(0.0, 0.2, "mug"), observationTime(12));
  EXPECT_FALSE(object.setDecidedLabel("bowl"));
  ASSERT_TRUE(object.setDecidedLabel("mug"));
  object.update(detection(), observationTime(13));
  EXPECT_EQ(object.class_name, "mug");
  object.update(detection(0.0, 0.2, "vase"), observationTime(14));
  EXPECT_EQ(object.class_name, "cup");
}

TEST(TrackedObject, OnlyPartialOverlapsWithComparableSizesAreAmbiguous)
{
  std::vector<std::unique_ptr<TrackedObject>> tracks;
  const auto add = [&tracks](double x, double size) {
    tracks.push_back(std::make_unique<TrackedObject>(
      detection(x, size), observationTime(10), tracks.size() + 1, 0.1, 0.1, 1, 2));
  };
  add(0.0, 0.2);  // 1
  add(0.01, 0.2); // 2: clear overlap with 1 (merged deterministically)
  add(1.0, 0.2);  // 3
  add(1.15, 0.2); // 4: partial overlap with 3
  add(1.0, 0.6);  // 5: overlaps 3 and 4 but too large
  const auto all = [](const TrackedObject&) { return true; };

  const auto pair = detail::findAmbiguousPair(tracks, all, 0.1, 0.5, 2.0, 0.01, {});
  ASSERT_TRUE(pair);
  EXPECT_EQ(tracks[pair->first]->track_id, 3u);
  EXPECT_EQ(tracks[pair->second]->track_id, 4u);
  EXPECT_FALSE(detail::findAmbiguousPair(tracks, all, 0.1, 0.5, 2.0, 0.01, {{3, 4}}));
}

TEST(TrackedObject, PartialViewSliversOfOneObjectAreCompatibleAndOverlap)
{
  // Measured on the Spot bag: one face seen as a 9 cm and as a 1 cm thick box.
  const Eigen::Vector3d A(0.09, 0.95, 0.38);
  const Eigen::Vector3d B(0.01, 0.95, 0.40);
  EXPECT_TRUE(TrackedObject::sizesCompatible(A, B, 2.0));
  EXPECT_GT(TrackedObject::coveredFraction({1.87, 0.37, -0.26}, A, {1.88, 0.27, -0.27}, B, 0.1),
            0.5);
  // Three 4-6 cm boxes 1 cm apart from one frame: padding makes them one object.
  EXPECT_GT(TrackedObject::coveredFraction(
              {1.53, 1.06, 0.52}, {0.05, 0.03, 0.01}, {1.54, 1.10, 0.52}, {0.04, 0.02, 0.02}, 0.1),
            0.5);
}

} // namespace
} // namespace semantic_scene_forests_object_tracker
