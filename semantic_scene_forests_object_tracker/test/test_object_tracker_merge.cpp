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

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <semantic_scene_forests_object_msgs/msg/object_candidate.hpp>
#include <semantic_scene_forests_object_tracker/object_tracker_component.hpp>

namespace {

using semantic_scene_forests_object_msgs::msg::ObjectCandidate;
using semantic_scene_forests_object_tracker::ObjectTrackerComponent;

constexpr int K_IMAGE_WIDTH  = 240;
constexpr int K_IMAGE_HEIGHT = 180;
constexpr int K_BOX2D_EXTENT = 20; // px
constexpr double K_BOX3D_EXTENT =
  90.0; // m; tracks 10 m apart overlap with IoU 0.8, 20 m apart with IoU ~0.64

struct TrackSpec
{
  std::string id;
  double x3d;
  double u;
  double v;
  cv::Vec3b color;
};

const cv::Vec3b K_RED{0, 0, 255};
const cv::Vec3b K_GREEN{0, 255, 0};
const cv::Vec3b K_BLUE{255, 0, 0};

builtin_interfaces::msg::Time toStamp(double sec)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec     = static_cast<std::int32_t>(sec);
  stamp.nanosec = static_cast<std::uint32_t>(std::llround((sec - std::floor(sec)) * 1e9));
  return stamp;
}

class ObjectTrackerMergeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    // The executor must be constructed after rclcpp::init(): its guard condition
    // requires an initialized context.
    m_executor  = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    m_tracker   = std::make_shared<ObjectTrackerComponent>(rclcpp::NodeOptions());
    m_test_node = rclcpp::Node::make_shared("object_tracker_merge_test_node");

    const auto PUB_QOS = rclcpp::QoS(10).reliable();
    m_det2d_pub =
      m_test_node->create_publisher<vision_msgs::msg::Detection2DArray>("detections_2d", PUB_QOS);
    m_det3d_pub =
      m_test_node->create_publisher<vision_msgs::msg::Detection3DArray>("detections_3d", PUB_QOS);
    m_image_pub = m_test_node->create_publisher<sensor_msgs::msg::Image>("camera/image", PUB_QOS);
    m_cand_sub  = m_test_node->create_subscription<ObjectCandidate>(
      "object_candidates",
      rclcpp::QoS(50).reliable(),
      // ConstSharedPtr-by-ref keeps performance-unnecessary-value-param happy and
      // matches an rclcpp-supported callback signature.
      [this](const ObjectCandidate::ConstSharedPtr& msg) { m_candidates.push_back(*msg); });

    m_executor->add_node(m_tracker);
    m_executor->add_node(m_test_node);

    awaitMiddlewareMatch();
  }

  void TearDown() override
  {
    m_executor->remove_node(m_test_node);
    m_executor->remove_node(m_tracker);
    m_test_node.reset();
    m_tracker.reset();
    m_candidates.clear();
  }

  // Publishes one synchronized frame (equal stamps on all three messages) and
  // waits until the tracker has published `expected_candidates` candidates for
  // exactly this stamp. Waiting on the candidate output of each frame keeps the
  // test free of arbitrary sleeps.
  void publishFrameAndWait(double stamp_sec,
                           const std::vector<TrackSpec>& tracks,
                           size_t expected_candidates)
  {
    const auto STAMP = toStamp(stamp_sec);
    publishFrame(STAMP, tracks);

    if (expected_candidates == 0)
    {
      m_executor->spin_some();
      return;
    }

    const auto DEADLINE = std::chrono::steady_clock::now() + K_TIMEOUT;
    while (std::chrono::steady_clock::now() < DEADLINE)
    {
      m_executor->spin_some();
      if (candidatesWithStamp(STAMP).size() >= expected_candidates)
      {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  std::vector<ObjectCandidate> candidatesWithStamp(const builtin_interfaces::msg::Time& stamp)
  {
    std::vector<ObjectCandidate> matches;
    for (const auto& candidate : m_candidates)
    {
      if (candidate.header.stamp == stamp)
      {
        matches.push_back(candidate);
      }
    }
    return matches;
  }

  static size_t countCropsWithColor(const ObjectCandidate& candidate, const cv::Vec3b& color)
  {
    size_t count = 0;
    for (const auto& crop : candidate.image_crops)
    {
      if (cropCenterColor(crop) == color)
      {
        ++count;
      }
    }
    return count;
  }

  static cv::Vec3b cropCenterColor(const sensor_msgs::msg::Image& crop_msg)
  {
    const auto CV_IMG  = cv_bridge::toCvCopy(crop_msg, "bgr8");
    const cv::Mat& img = CV_IMG->image;
    return img.at<cv::Vec3b>(img.rows / 2, img.cols / 2);
  }

private:
  void awaitMiddlewareMatch()
  {
    const auto DEADLINE = std::chrono::steady_clock::now() + K_TIMEOUT;
    while (std::chrono::steady_clock::now() < DEADLINE)
    {
      m_executor->spin_some();
      const bool MATCHED =
        m_det2d_pub->get_subscription_count() > 0 && m_det3d_pub->get_subscription_count() > 0 &&
        m_image_pub->get_subscription_count() > 0 && m_cand_sub->get_publisher_count() > 0;
      if (MATCHED)
      {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    FAIL() << "middleware matching timeout: pubs/subs did not match within deadline";
  }

  void publishFrame(const builtin_interfaces::msg::Time& stamp,
                    const std::vector<TrackSpec>& tracks)
  {
    vision_msgs::msg::Detection2DArray det2d_msg;
    det2d_msg.header.stamp    = stamp;
    det2d_msg.header.frame_id = "map";
    vision_msgs::msg::Detection3DArray det3d_msg;
    det3d_msg.header.stamp    = stamp;
    det3d_msg.header.frame_id = "map";

    for (const auto& track : tracks)
    {
      det2d_msg.detections.push_back(makeDet2D(track, stamp));
      det3d_msg.detections.push_back(makeDet3D(track, stamp));
    }

    sensor_msgs::msg::Image image = makeImage(stamp);

    m_det2d_pub->publish(det2d_msg);
    m_det3d_pub->publish(det3d_msg);
    m_image_pub->publish(image);
  }

  static vision_msgs::msg::Detection2D makeDet2D(const TrackSpec& spec,
                                                 const builtin_interfaces::msg::Time& stamp)
  {
    vision_msgs::msg::Detection2D det;
    det.header.stamp           = stamp;
    det.id                     = spec.id;
    det.bbox.center.position.x = spec.u;
    det.bbox.center.position.y = spec.v;
    det.bbox.size_x            = K_BOX2D_EXTENT;
    det.bbox.size_y            = K_BOX2D_EXTENT;
    det.results.push_back(makeHypothesis());
    return det;
  }

  static vision_msgs::msg::Detection3D makeDet3D(const TrackSpec& spec,
                                                 const builtin_interfaces::msg::Time& stamp)
  {
    vision_msgs::msg::Detection3D det;
    det.header.stamp           = stamp;
    det.id                     = spec.id;
    det.bbox.center.position.x = spec.x3d;
    det.bbox.center.position.y = 0.0;
    det.bbox.center.position.z = 0.0;
    det.bbox.size.x            = K_BOX3D_EXTENT;
    det.bbox.size.y            = K_BOX3D_EXTENT;
    det.bbox.size.z            = K_BOX3D_EXTENT;
    det.results.push_back(makeHypothesis());
    return det;
  }

  static vision_msgs::msg::ObjectHypothesisWithPose makeHypothesis()
  {
    vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
    hypothesis.hypothesis.class_id = "target";
    hypothesis.hypothesis.score    = 0.9;
    return hypothesis;
  }

  static sensor_msgs::msg::Image makeImage(const builtin_interfaces::msg::Time& stamp)
  {
    cv::Mat image(K_IMAGE_HEIGHT, K_IMAGE_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
    image(cv::Rect(0, 0, K_IMAGE_WIDTH, 60)).setTo(K_RED);
    image(cv::Rect(0, 60, K_IMAGE_WIDTH, 60)).setTo(K_GREEN);
    image(cv::Rect(0, 120, K_IMAGE_WIDTH, 60)).setTo(K_BLUE);
    auto msg          = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    msg->header.stamp = stamp;
    return *msg;
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;
  std::shared_ptr<ObjectTrackerComponent> m_tracker;
  rclcpp::Node::SharedPtr m_test_node;

  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr m_det2d_pub;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr m_det3d_pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_pub;
  rclcpp::Subscription<ObjectCandidate>::SharedPtr m_cand_sub;

  std::vector<ObjectCandidate> m_candidates;

  static constexpr auto K_TIMEOUT = std::chrono::seconds(15);
};

// The first (lowest track id) track survives the three-way merge because all
// tracks tie on observation count and the merge loop prefers the earlier track.
TEST_F(ObjectTrackerMergeTest, FirstTrackSurvivesMerge)
{
  const TrackSpec A{.id = "A", .x3d = 0.0, .u = 120.0, .v = 30.0, .color = K_RED};
  const TrackSpec B{.id = "B", .x3d = 10.0, .u = 120.0, .v = 90.0, .color = K_GREEN};
  const TrackSpec C{.id = "C", .x3d = 20.0, .u = 120.0, .v = 150.0, .color = K_BLUE};

  // Three frames confirm all tracks (confirmation_threshold = 3). In the third
  // frame the already overlapping tracks merge into a single survivor.
  publishFrameAndWait(100.0, {A, B, C}, 0);
  publishFrameAndWait(100.1, {A, B, C}, 0);
  publishFrameAndWait(100.2, {A, B, C}, 1);

  const auto FINAL_CANDIDATES = candidatesWithStamp(toStamp(100.2));
  ASSERT_EQ(FINAL_CANDIDATES.size(), 1U);
  EXPECT_EQ(FINAL_CANDIDATES[0].id, 1);
  ASSERT_EQ(FINAL_CANDIDATES[0].image_crops.size(), 3U);
  EXPECT_EQ(countCropsWithColor(FINAL_CANDIDATES[0], K_RED), 1U);
  EXPECT_EQ(countCropsWithColor(FINAL_CANDIDATES[0], K_GREEN), 1U);
  EXPECT_EQ(countCropsWithColor(FINAL_CANDIDATES[0], K_BLUE), 1U);
}

// The middle track is observed in every frame and therefore reaches the highest
// observation count; it must absorb the first and last tracks' crops.
TEST_F(ObjectTrackerMergeTest, MiddleTrackSurvivesMerge)
{
  const TrackSpec A{.id = "A", .x3d = 0.0, .u = 120.0, .v = 30.0, .color = K_RED};
  const TrackSpec B{.id = "B", .x3d = 10.0, .u = 120.0, .v = 90.0, .color = K_GREEN};
  const TrackSpec C{.id = "C", .x3d = 20.0, .u = 120.0, .v = 150.0, .color = K_BLUE};

  publishFrameAndWait(100.0, {A, B, C}, 0);
  publishFrameAndWait(100.1, {A, B, C}, 0);
  publishFrameAndWait(100.2, {B}, 1); // B confirms first
  publishFrameAndWait(100.3, {B}, 1);
  publishFrameAndWait(100.4, {A, B, C}, 1); // merge frame

  const auto FINAL_CANDIDATES = candidatesWithStamp(toStamp(100.4));
  ASSERT_EQ(FINAL_CANDIDATES.size(), 1U);
  EXPECT_EQ(FINAL_CANDIDATES[0].id, 2);
  ASSERT_EQ(FINAL_CANDIDATES[0].image_crops.size(), 5U); // bounded by max_crops_per_object
  EXPECT_EQ(countCropsWithColor(FINAL_CANDIDATES[0], K_GREEN), 3U);
  EXPECT_EQ(countCropsWithColor(FINAL_CANDIDATES[0], K_RED), 1U);
  EXPECT_EQ(countCropsWithColor(FINAL_CANDIDATES[0], K_BLUE), 1U);
}

// The last track is observed in every frame, reaches the highest observation
// count, and must survive while retaining the absorbed tracks' crops.
TEST_F(ObjectTrackerMergeTest, LastTrackSurvivesMerge)
{
  const TrackSpec A{.id = "A", .x3d = 0.0, .u = 120.0, .v = 30.0, .color = K_RED};
  const TrackSpec B{.id = "B", .x3d = 10.0, .u = 120.0, .v = 90.0, .color = K_GREEN};
  const TrackSpec C{.id = "C", .x3d = 20.0, .u = 120.0, .v = 150.0, .color = K_BLUE};

  publishFrameAndWait(100.0, {A, B, C}, 0);
  publishFrameAndWait(100.1, {A, B, C}, 0);
  publishFrameAndWait(100.2, {C}, 1); // C confirms first
  publishFrameAndWait(100.3, {C}, 1);
  publishFrameAndWait(100.4, {A, B, C}, 1); // merge frame

  const auto FINAL_CANDIDATES = candidatesWithStamp(toStamp(100.4));
  ASSERT_EQ(FINAL_CANDIDATES.size(), 1U);
  EXPECT_EQ(FINAL_CANDIDATES[0].id, 3);
  ASSERT_EQ(FINAL_CANDIDATES[0].image_crops.size(), 5U);
  EXPECT_EQ(countCropsWithColor(FINAL_CANDIDATES[0], K_BLUE), 3U);
  EXPECT_EQ(countCropsWithColor(FINAL_CANDIDATES[0], K_RED), 1U);
  EXPECT_EQ(countCropsWithColor(FINAL_CANDIDATES[0], K_GREEN), 1U);
}

// A single track observed in eight frames attaches one crop per frame from the
// third frame on; the stored deque must never exceed max_crops_per_object.
TEST_F(ObjectTrackerMergeTest, CropsBoundedByMaxCropsPerObject)
{
  const TrackSpec A{.id = "A", .x3d = 0.0, .u = 120.0, .v = 30.0, .color = K_RED};

  for (int i = 0; i < 8; ++i)
  {
    const double STAMP_SEC = 100.0 + (0.1 * i);
    const size_t EXPECTED  = (i >= 2) ? 1U : 0U; // confirmed from the third frame on
    publishFrameAndWait(STAMP_SEC, {A}, EXPECTED);
  }

  const auto FINAL_CANDIDATES = candidatesWithStamp(toStamp(100.7));
  ASSERT_EQ(FINAL_CANDIDATES.size(), 1U);
  EXPECT_EQ(FINAL_CANDIDATES[0].id, 1);
  ASSERT_EQ(FINAL_CANDIDATES[0].image_crops.size(), 5U);
  for (const auto& crop : FINAL_CANDIDATES[0].image_crops)
  {
    EXPECT_EQ(crop.width, 40U);
    EXPECT_EQ(crop.height, 40U);
  }
}

// A 20 px box with 10 px padding next to the image corner must produce a crop
// clipped to the image: 20 px box + 10 px padding on the in-bounds side.
TEST_F(ObjectTrackerMergeTest, CropPaddingClampsAtImageBoundary)
{
  const TrackSpec A{.id = "A", .x3d = 0.0, .u = 10.0, .v = 10.0, .color = K_RED};

  publishFrameAndWait(100.0, {A}, 0);
  publishFrameAndWait(100.1, {A}, 0);
  publishFrameAndWait(100.2, {A}, 1);

  const auto FINAL_CANDIDATES = candidatesWithStamp(toStamp(100.2));
  ASSERT_EQ(FINAL_CANDIDATES.size(), 1U);
  ASSERT_EQ(FINAL_CANDIDATES[0].image_crops.size(), 1U);

  const auto& crop = FINAL_CANDIDATES[0].image_crops[0];
  EXPECT_EQ(crop.width, 30U);
  EXPECT_EQ(crop.height, 30U);

  const auto CENTER = cropCenterColor(crop);
  EXPECT_EQ(CENTER[0], K_RED[0]);
  EXPECT_EQ(CENTER[1], K_RED[1]);
  EXPECT_EQ(CENTER[2], K_RED[2]);
}

} // namespace
