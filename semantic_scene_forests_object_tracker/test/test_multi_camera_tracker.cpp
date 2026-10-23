
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

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include "semantic_scene_forests_object_manager/object_manager_node.hpp"
#include "semantic_scene_forests_object_msgs/msg/object_candidate.hpp"
#include "semantic_scene_forests_object_msgs/srv/get_object_visuals.hpp"
#include "semantic_scene_forests_object_msgs/srv/get_objects.hpp"
#include "semantic_scene_forests_object_tracker/object_tracker_component.hpp"
#include "semantic_scene_forests_object_tracker/tracked_object.hpp"

namespace {

using namespace std::chrono_literals;

using semantic_scene_forests_object_tracker::DetectionAdapter;
using semantic_scene_forests_object_tracker::ObjectTrackerComponent;
using semantic_scene_forests_object_tracker::TrackedObject;
using ObjectCandidate  = semantic_scene_forests_object_msgs::msg::ObjectCandidate;
using GetObjectVisuals = semantic_scene_forests_object_msgs::srv::GetObjectVisuals;
using GetObjects       = semantic_scene_forests_object_msgs::srv::GetObjects;

rclcpp::NodeOptions twoCameraOptions()
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("world_frame", "map");
  options.append_parameter_override("camera_names", std::vector<std::string>{"front", "rear"});
  options.append_parameter_override("cameras.front.detections_2d_topic", "/front/detections_2d");
  options.append_parameter_override("cameras.front.detections_3d_topic", "/front/detections_3d");
  options.append_parameter_override("cameras.front.image_topic", "/front/image");
  options.append_parameter_override("cameras.rear.detections_2d_topic", "/rear/detections_2d");
  options.append_parameter_override("cameras.rear.detections_3d_topic", "/rear/detections_3d");
  options.append_parameter_override("cameras.rear.image_topic", "/rear/image");
  return options;
}

rclcpp::NodeOptions commonTrackerOptions(int confirmation_threshold      = 3,
                                         double gate_threshold_strict    = 3.0,
                                         double gate_threshold_confirmed = 10.0,
                                         double merge_iou_threshold      = 0.6)
{
  auto options = twoCameraOptions();
  options.append_parameter_override("confirmation_threshold", confirmation_threshold);
  options.append_parameter_override("gate_threshold_strict", gate_threshold_strict);
  options.append_parameter_override("gate_threshold_confirmed", gate_threshold_confirmed);
  options.append_parameter_override("merge_iou_threshold", merge_iou_threshold);
  options.append_parameter_override("crop_padding", 0);
  options.append_parameter_override("max_crops_per_object", 20);
  return options;
}

rclcpp::NodeOptions defaultTrackerOptions()
{
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args",
                     "-r",
                     "detections_2d:=/remapped/detections_2d",
                     "-r",
                     "detections_3d:=/remapped/detections_3d",
                     "-r",
                     "camera/image:=/remapped/image"});
  options.append_parameter_override("world_frame", "map");
  options.append_parameter_override("confirmation_threshold", 3);
  options.append_parameter_override("crop_padding", 0);
  options.append_parameter_override("max_crops_per_object", 20);
  return options;
}

sensor_msgs::msg::Image
makeImage(const rclcpp::Time& stamp, const std::string& frame_id, uint8_t tag)
{
  sensor_msgs::msg::Image image;
  image.header.stamp    = stamp;
  image.header.frame_id = frame_id;
  image.height          = 32;
  image.width           = 32;
  image.encoding        = sensor_msgs::image_encodings::BGR8;
  image.step            = image.width * 3;
  image.data.assign(image.height * image.step, tag);
  return image;
}

vision_msgs::msg::Detection2DArray
makeDetections2D(const rclcpp::Time& stamp, const std::string& frame_id, const std::string& id)
{
  vision_msgs::msg::Detection2DArray array;
  array.header.stamp    = stamp;
  array.header.frame_id = frame_id;

  vision_msgs::msg::Detection2D detection;
  detection.id                     = id;
  detection.bbox.center.position.x = 16.0;
  detection.bbox.center.position.y = 16.0;
  detection.bbox.size_x            = 8.0;
  detection.bbox.size_y            = 8.0;
  vision_msgs::msg::ObjectHypothesisWithPose result;
  result.hypothesis.class_id = "cup";
  result.hypothesis.score    = 0.99;
  detection.results.push_back(result);
  array.detections.push_back(std::move(detection));
  return array;
}

vision_msgs::msg::Detection3DArray makeDetections3D(const rclcpp::Time& stamp,
                                                    const std::string& frame_id,
                                                    const std::string& id,
                                                    double x,
                                                    double size)
{
  vision_msgs::msg::Detection3DArray array;
  array.header.stamp    = stamp;
  array.header.frame_id = frame_id;

  vision_msgs::msg::Detection3D detection;
  detection.id                        = id;
  detection.bbox.center.position.x    = x;
  detection.bbox.center.orientation.w = 1.0;
  detection.bbox.size.x               = size;
  detection.bbox.size.y               = size;
  detection.bbox.size.z               = size;
  vision_msgs::msg::ObjectHypothesisWithPose result;
  result.hypothesis.class_id = "cup";
  result.hypothesis.score    = 0.99;
  detection.results.push_back(result);
  array.detections.push_back(std::move(detection));
  return array;
}

struct CameraPublisher
{
  std::string frame_id;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_2d;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detections_3d;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image;

  void publish(const rclcpp::Time& stamp,
               const std::string& local_id,
               double local_x,
               uint8_t tag,
               bool publish_2d                   = true,
               bool publish_3d                   = true,
               bool publish_image                = true,
               const std::string& frame_override = "",
               double bbox_size                  = 1.0) const
  {
    const std::string FRAME = frame_override.empty() ? frame_id : frame_override;
    if (publish_2d)
    {
      detections_2d->publish(makeDetections2D(stamp, FRAME, local_id));
    }
    if (publish_3d)
    {
      detections_3d->publish(makeDetections3D(stamp, FRAME, local_id, local_x, bbox_size));
    }
    if (publish_image)
    {
      image->publish(makeImage(stamp, FRAME, tag));
    }
  }
};

geometry_msgs::msg::TransformStamped makeTransform(const std::string& child_frame, double x)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id         = "map";
  transform.child_frame_id          = child_frame;
  transform.transform.translation.x = x;
  transform.transform.rotation.w    = 1.0;
  return transform;
}

class TrackerHarness
{
public:
  explicit TrackerHarness(bool two_cameras                = true,
                          bool with_manager               = false,
                          int confirmation_threshold      = 3,
                          double gate_threshold_strict    = 3.0,
                          double gate_threshold_confirmed = 10.0,
                          double merge_iou_threshold      = 0.6)
  {
    m_source_node = std::make_shared<rclcpp::Node>("multi_camera_test_source");
    m_tracker     = std::make_shared<ObjectTrackerComponent>(
      two_cameras ? commonTrackerOptions(confirmation_threshold,
                                         gate_threshold_strict,
                                         gate_threshold_confirmed,
                                         merge_iou_threshold)
                  : defaultTrackerOptions());

    const auto SENSOR_QOS = rclcpp::SensorDataQoS();
    if (two_cameras)
    {
      m_front = makeCamera(
        "front_frame", "/front/detections_2d", "/front/detections_3d", "/front/image", SENSOR_QOS);
      m_rear = makeCamera(
        "rear_frame", "/rear/detections_2d", "/rear/detections_3d", "/rear/image", SENSOR_QOS);
    }
    else
    {
      m_front = makeCamera("camera_frame",
                           "/remapped/detections_2d",
                           "/remapped/detections_3d",
                           "/remapped/image",
                           SENSOR_QOS);
    }

    m_candidates_sub = m_source_node->create_subscription<ObjectCandidate>(
      "object_candidates",
      rclcpp::QoS(1000).reliable().transient_local(),
      [this](ObjectCandidate::ConstSharedPtr candidate) {
        std::scoped_lock lock(m_candidates_mutex);
        m_candidates.push_back(*candidate);
      });

    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(m_source_node->get_clock());
    m_tf_listener =
      std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer, m_source_node, false);
    m_tf_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(m_source_node);
    m_tf_broadcaster->sendTransform(makeTransform(m_front->frame_id, 1.0));
    if (two_cameras)
    {
      m_tf_broadcaster->sendTransform(makeTransform(m_rear->frame_id, -1.0));
    }

    if (with_manager)
    {
      rclcpp::NodeOptions manager_options;
      manager_options.append_parameter_override("frame_id", "map");
      m_manager =
        std::make_shared<semantic_scene_forests_object_manager::ObjectManagerNode>(manager_options);
      m_get_objects = m_source_node->create_client<GetObjects>("get_objects");
      m_get_visuals = m_source_node->create_client<GetObjectVisuals>("get_object_visuals");
    }

    m_executor.add_node(m_source_node);
    m_executor.add_node(m_tracker);
    if (m_manager)
    {
      m_executor.add_node(m_manager);
    }
    m_spinner = std::thread([this]() { m_executor.spin(); });
  }

  ~TrackerHarness()
  {
    m_executor.cancel();
    if (m_spinner.joinable())
    {
      m_spinner.join();
    }
  }

  template <typename Predicate>
  static bool waitUntil(Predicate&& predicate, std::chrono::milliseconds timeout)
  {
    const auto DEADLINE = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < DEADLINE)
    {
      if (predicate())
      {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return predicate();
  }

  bool ready(bool manager = false) const
  {
    return waitUntil(
      [this, manager]() {
        const bool FRONT_READY = m_front->detections_2d->get_subscription_count() > 0 &&
                                 m_front->detections_3d->get_subscription_count() > 0 &&
                                 m_front->image->get_subscription_count() > 0;
        const bool REAR_READY  = !m_rear || (m_rear->detections_2d->get_subscription_count() > 0 &&
                                             m_rear->detections_3d->get_subscription_count() > 0 &&
                                             m_rear->image->get_subscription_count() > 0);
        const bool TF_READY    = m_tf_buffer->canTransform(
          "map", m_front->frame_id, tf2::TimePointZero, tf2::durationFromSec(0.0));
        const bool REAR_TF_READY =
          !m_rear || m_tf_buffer->canTransform(
                       "map", m_rear->frame_id, tf2::TimePointZero, tf2::durationFromSec(0.0));
        const bool SERVICES_READY =
          !manager || (m_get_objects->service_is_ready() && m_get_visuals->service_is_ready());
        return FRONT_READY && REAR_READY && TF_READY && REAR_TF_READY && SERVICES_READY;
      },
      5s);
  }

  CameraPublisher& front() { return *m_front; }
  CameraPublisher& rear() { return *m_rear; }

  bool setConfirmationThreshold(int64_t threshold)
  {
    return m_tracker->set_parameter(rclcpp::Parameter("confirmation_threshold", threshold))
      .successful;
  }

  std::vector<ObjectCandidate> candidates() const
  {
    std::scoped_lock lock(m_candidates_mutex);
    return m_candidates;
  }

  bool waitForCandidates(size_t count, std::chrono::milliseconds timeout = 2s) const
  {
    return waitUntil([this, count]() { return candidates().size() >= count; }, timeout);
  }

  bool noCandidatesFor(std::chrono::milliseconds duration) const
  {
    const auto DEADLINE = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < DEADLINE)
    {
      if (!candidates().empty())
      {
        return false;
      }
      std::this_thread::sleep_for(5ms);
    }
    return candidates().empty();
  }

  bool noCropTagFor(uint8_t tag, std::chrono::milliseconds duration) const
  {
    const auto DEADLINE = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < DEADLINE)
    {
      for (const auto& candidate : candidates())
      {
        for (const auto& crop : candidate.image_crops)
        {
          if (!crop.data.empty() && crop.data.front() == tag)
          {
            return false;
          }
        }
      }
      std::this_thread::sleep_for(5ms);
    }
    return true;
  }

  std::optional<GetObjects::Response> getObjects() const
  {
    auto future = m_get_objects->async_send_request(std::make_shared<GetObjects::Request>());
    if (!waitUntil([&future]() { return future.wait_for(0s) == std::future_status::ready; }, 2s))
    {
      return std::nullopt;
    }
    return *future.get();
  }

  std::optional<GetObjectVisuals::Response> getVisuals(const std::vector<int64_t>& ids) const
  {
    auto request        = std::make_shared<GetObjectVisuals::Request>();
    request->object_ids = ids;
    auto future         = m_get_visuals->async_send_request(request);
    if (!waitUntil([&future]() { return future.wait_for(0s) == std::future_status::ready; }, 2s))
    {
      return std::nullopt;
    }
    return *future.get();
  }

private:
  CameraPublisher makeCamera(const std::string& frame_id,
                             const std::string& detections_2d_topic,
                             const std::string& detections_3d_topic,
                             const std::string& image_topic,
                             const rclcpp::QoS& qos) const
  {
    CameraPublisher camera;
    camera.frame_id = frame_id;
    camera.detections_2d =
      m_source_node->create_publisher<vision_msgs::msg::Detection2DArray>(detections_2d_topic, qos);
    camera.detections_3d =
      m_source_node->create_publisher<vision_msgs::msg::Detection3DArray>(detections_3d_topic, qos);
    camera.image = m_source_node->create_publisher<sensor_msgs::msg::Image>(image_topic, qos);
    return camera;
  }

  rclcpp::Node::SharedPtr m_source_node;
  rclcpp::Node::SharedPtr m_tracker;
  rclcpp::Node::SharedPtr m_manager;
  std::optional<CameraPublisher> m_front;
  std::optional<CameraPublisher> m_rear;
  rclcpp::Subscription<ObjectCandidate>::SharedPtr m_candidates_sub;
  rclcpp::Client<GetObjects>::SharedPtr m_get_objects;
  rclcpp::Client<GetObjectVisuals>::SharedPtr m_get_visuals;
  std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> m_tf_listener;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> m_tf_broadcaster;
  mutable std::mutex m_candidates_mutex;
  std::vector<ObjectCandidate> m_candidates;
  rclcpp::executors::MultiThreadedExecutor m_executor;
  std::thread m_spinner;
};

bool containsCrop(const ObjectCandidate& candidate, uint8_t tag, const std::string& frame_id)
{
  return std::ranges::any_of(candidate.image_crops, [tag, &frame_id](const auto& crop) {
    return crop.header.frame_id == frame_id && !crop.data.empty() && crop.data.front() == tag;
  });
}

std::set<int64_t> candidateIds(const std::vector<ObjectCandidate>& candidates)
{
  std::set<int64_t> ids;
  for (const auto& candidate : candidates)
  {
    ids.insert(candidate.id);
  }
  return ids;
}

void publishBatches(CameraPublisher& camera,
                    double start_time,
                    int count,
                    const std::string& local_id,
                    double local_x,
                    uint8_t tag,
                    double bbox_size = 1.0)
{
  for (int index = 0; index < count; ++index)
  {
    camera.publish(rclcpp::Time(static_cast<int64_t>((start_time + index * 0.01) * 1e9)),
                   local_id,
                   local_x,
                   tag,
                   true,
                   true,
                   true,
                   "",
                   bbox_size);
  }
}

TEST(MultiCameraTrackerParameters, AcceptsNamedIndependentInputs)
{
  EXPECT_NO_THROW(std::make_shared<ObjectTrackerComponent>(twoCameraOptions()));
}

TEST(MultiCameraTrackerParameters, RejectsEmptyCameraName)
{
  auto options = twoCameraOptions();
  options.append_parameter_override("camera_names", std::vector<std::string>{"front", ""});
  EXPECT_THROW(std::make_shared<ObjectTrackerComponent>(options), std::exception);
}

TEST(MultiCameraTrackerParameters, RejectsDuplicateResolvedTopics)
{
  auto options = twoCameraOptions();
  options.append_parameter_override("cameras.rear.detections_2d_topic", "/front/detections_2d");
  EXPECT_THROW(std::make_shared<ObjectTrackerComponent>(options), std::exception);
}

TEST(MultiCameraTracker, CameraInputsDoNotCrossPair)
{
  TrackerHarness harness;
  ASSERT_TRUE(harness.ready());

  for (int index = 0; index < 3; ++index)
  {
    const auto STAMP = rclcpp::Time(static_cast<int64_t>((1.0 + index * 0.01) * 1e9));
    harness.front().publish(STAMP, "1", 4.0, 10, true, false, false);
    harness.rear().publish(STAMP, "1", 6.0, 20, false, true, true);
  }
  EXPECT_TRUE(harness.noCandidatesFor(300ms));

  publishBatches(harness.front(), 2.0, 3, "1", 4.0, 11);
  ASSERT_TRUE(harness.waitUntil(
    [&harness]() {
      const auto CANDIDATES = harness.candidates();
      return !CANDIDATES.empty() && std::ranges::any_of(CANDIDATES, [](const auto& candidate) {
        return containsCrop(candidate, 11, "front_frame");
      });
    },
    2s));
  for (const auto& candidate : harness.candidates())
  {
    EXPECT_FALSE(containsCrop(candidate, 20, "rear_frame"));
  }
}

TEST(MultiCameraTracker, OverlappingCamerasShareGlobalTrackAndCropHistory)
{
  TrackerHarness harness;
  ASSERT_TRUE(harness.ready());

  for (int index = 0; index < 3; ++index)
  {
    const auto STAMP = rclcpp::Time(static_cast<int64_t>((3.0 + index * 0.01) * 1e9));
    harness.front().publish(STAMP, "7", 4.0, 30);
    harness.rear().publish(STAMP, "42", 6.0, 40);
  }
  ASSERT_TRUE(harness.waitUntil(
    [&harness]() {
      const auto CANDIDATES = harness.candidates();
      return candidateIds(CANDIDATES).size() == 1U &&
             std::ranges::any_of(CANDIDATES, [](const auto& candidate) {
               return containsCrop(candidate, 30, "front_frame") &&
                      containsCrop(candidate, 40, "rear_frame");
             });
    },
    2s));
  const auto CANDIDATES = harness.candidates();
  ASSERT_EQ(candidateIds(CANDIDATES).size(), 1U);
  EXPECT_TRUE(std::ranges::any_of(
    CANDIDATES, [](const auto& candidate) { return containsCrop(candidate, 30, "front_frame"); }));
  EXPECT_TRUE(std::ranges::any_of(
    CANDIDATES, [](const auto& candidate) { return containsCrop(candidate, 40, "rear_frame"); }));
}

TEST(MultiCameraTracker, ReusedLocalIdsRemainSeparateForDisjointObjects)
{
  TrackerHarness harness;
  ASSERT_TRUE(harness.ready());
  for (int index = 0; index < 3; ++index)
  {
    const auto STAMP = rclcpp::Time(static_cast<int64_t>((4.0 + index * 0.01) * 1e9));
    harness.front().publish(STAMP, "1", 1.0, 50);
    harness.rear().publish(STAMP, "1", 9.0, 60);
  }
  ASSERT_TRUE(harness.waitUntil(
    [&harness]() { return candidateIds(harness.candidates()).size() == 2U; }, 2s));
  EXPECT_EQ(candidateIds(harness.candidates()).size(), 2U);
}

TEST(MultiCameraTracker, SequentialCameraHandoverKeepsGlobalId)
{
  TrackerHarness harness;
  ASSERT_TRUE(harness.ready());
  publishBatches(harness.front(), 5.0, 3, "7", 4.0, 70);
  ASSERT_TRUE(harness.waitForCandidates(1));
  const auto BEFORE       = harness.candidates();
  const int64_t GLOBAL_ID = BEFORE.back().id;
  publishBatches(harness.rear(), 6.0, 3, "99", 6.0, 80);
  ASSERT_TRUE(harness.waitUntil(
    [&harness, GLOBAL_ID]() {
      return std::ranges::any_of(harness.candidates(), [GLOBAL_ID](const auto& candidate) {
        return candidate.id == GLOBAL_ID && containsCrop(candidate, 80, "rear_frame");
      });
    },
    2s));
  const auto AFTER = harness.candidates();
  EXPECT_EQ(candidateIds(AFTER).size(), 1U);
  EXPECT_TRUE(std::ranges::any_of(AFTER, [GLOBAL_ID](const auto& candidate) {
    return candidate.id == GLOBAL_ID && containsCrop(candidate, 80, "rear_frame");
  }));
}

TEST(MultiCameraTracker, RecycledLocalIdAtDifferentPositionCreatesNewGlobalId)
{
  TrackerHarness harness;
  ASSERT_TRUE(harness.ready());
  publishBatches(harness.front(), 7.0, 3, "1", 1.0, 90);
  ASSERT_TRUE(harness.waitForCandidates(1));
  publishBatches(harness.front(), 8.0, 3, "1", 19.0, 100);
  ASSERT_TRUE(harness.waitUntil(
    [&harness]() { return candidateIds(harness.candidates()).size() >= 2U; }, 2s));
  EXPECT_GE(candidateIds(harness.candidates()).size(), 2U);
}

TEST(MultiCameraTracker, DelayedAndDuplicateSourceFramesDoNotReverseTrackTime)
{
  TrackerHarness harness;
  ASSERT_TRUE(harness.ready());
  publishBatches(harness.front(), 10.0, 3, "1", 4.0, 110);
  ASSERT_TRUE(harness.waitForCandidates(1));
  const auto BEFORE       = harness.candidates();
  const auto BEFORE_STAMP = rclcpp::Time(BEFORE.back().header.stamp).nanoseconds();
  publishBatches(harness.rear(), 9.9, 1, "2", 6.0, 120);
  ASSERT_TRUE(harness.waitForCandidates(BEFORE.size() + 1));
  const auto AFTER_DELAYED = harness.candidates();
  EXPECT_GE(rclcpp::Time(AFTER_DELAYED.back().header.stamp).nanoseconds(), BEFORE_STAMP);
  EXPECT_TRUE(containsCrop(AFTER_DELAYED.back(), 120, "rear_frame"));
  harness.rear().publish(rclcpp::Time(static_cast<int64_t>(9.9e9)), "2", 6.0, 121);
  EXPECT_TRUE(harness.noCropTagFor(121, 300ms));
  harness.front().publish(rclcpp::Time(static_cast<int64_t>(10.1e9)), "1", 4.0, 130);
  ASSERT_TRUE(harness.waitForCandidates(BEFORE.size() + 2));
  const auto ALL = harness.candidates();
  for (size_t index = 1; index < ALL.size(); ++index)
  {
    EXPECT_GE(rclcpp::Time(ALL[index].header.stamp).nanoseconds(),
              rclcpp::Time(ALL[index - 1].header.stamp).nanoseconds());
  }
}

TEST(MultiCameraTracker, ActiveCameraContinuesWhenOtherCameraHasNoTf)
{
  TrackerHarness harness;
  ASSERT_TRUE(harness.ready());
  harness.rear().publish(
    rclcpp::Time(static_cast<int64_t>(11e9)), "1", 6.0, 140, true, true, true, "missing_frame");
  publishBatches(harness.front(), 12.0, 3, "1", 4.0, 150);
  ASSERT_TRUE(harness.waitForCandidates(1));
  EXPECT_TRUE(std::ranges::any_of(harness.candidates(), [](const auto& candidate) {
    return containsCrop(candidate, 150, "front_frame");
  }));
}

TEST(MultiCameraTracker, DefaultCameraPreservesTopicRemaps)
{
  TrackerHarness harness(false);
  ASSERT_TRUE(harness.ready(false));
  publishBatches(harness.front(), 13.0, 3, "1", 4.0, 160);
  ASSERT_TRUE(harness.waitForCandidates(1));
  EXPECT_EQ(candidateIds(harness.candidates()).size(), 1U);
  EXPECT_TRUE(containsCrop(harness.candidates().back(), 160, "camera_frame"));
}

TEST(MultiCameraTracker, LongDelayedSourceCanConfirmNewTrack)
{
  TrackerHarness harness;
  ASSERT_TRUE(harness.ready());

  publishBatches(harness.front(), 100.0, 3, "1", 4.0, 165);
  ASSERT_TRUE(harness.waitForCandidates(1));

  // This separate object's observations arrive far behind the active camera's frames.
  publishBatches(harness.rear(), 1.0, 3, "77", 21.0, 166);
  ASSERT_TRUE(harness.waitUntil(
    [&harness]() { return candidateIds(harness.candidates()).size() >= 2U; }, 2s));
}

TEST(MultiCameraTracker, PublishedTrackSurvivesMergeWithNewerTrack)
{
  TrackerHarness harness(true, false, 5, 3.0, 0.1);
  ASSERT_TRUE(harness.ready());

  publishBatches(harness.front(), 20.0, 5, "1", 4.0, 170, 10.0);
  ASSERT_TRUE(harness.waitForCandidates(1));
  const int64_t PUBLISHED_ID = harness.candidates().back().id;
  ASSERT_TRUE(harness.setConfirmationThreshold(6));

  // The new track confirms with more observations than the published survivor.
  const auto BEFORE_NEW_TRACK = harness.candidates().size();
  publishBatches(harness.rear(), 21.0, 5, "2", 8.0, 180, 10.0);
  ASSERT_TRUE(harness.waitForCandidates(BEFORE_NEW_TRACK + 5));
  harness.rear().publish(
    rclcpp::Time(static_cast<int64_t>(21.05e9)), "2", 8.0, 181, true, true, true, "", 10.0);
  ASSERT_TRUE(harness.waitUntil(
    [&harness, PUBLISHED_ID]() {
      const auto CANDIDATES = harness.candidates();
      return candidateIds(CANDIDATES).size() == 1U &&
             std::ranges::any_of(CANDIDATES, [PUBLISHED_ID](const auto& candidate) {
               return candidate.id == PUBLISHED_ID && containsCrop(candidate, 181, "rear_frame");
             });
    },
    2s));
}

TEST(MultiCameraTracker, TwoPublishedTracksRemainAfterOverlap)
{
  TrackerHarness harness(true, false, 3, 0.1, 0.1);
  ASSERT_TRUE(harness.ready());

  publishBatches(harness.front(), 30.0, 3, "1", 4.0, 190, 1.0);
  publishBatches(harness.rear(), 30.1, 3, "2", 8.0, 191, 1.0);
  ASSERT_TRUE(harness.waitUntil(
    [&harness]() { return candidateIds(harness.candidates()).size() == 2U; }, 2s));
  const auto INITIAL = candidateIds(harness.candidates());
  ASSERT_EQ(INITIAL.size(), 2U);

  publishBatches(harness.front(), 31.0, 3, "1", 4.0, 192, 20.0);
  publishBatches(harness.rear(), 31.1, 3, "2", 8.0, 193, 20.0);
  const auto FINAL_STAMP = static_cast<int64_t>(32e9);
  harness.front().publish(rclcpp::Time(FINAL_STAMP), "1", 4.0, 194, true, true, true, "", 20.0);
  harness.rear().publish(rclcpp::Time(FINAL_STAMP), "2", 8.0, 195, true, true, true, "", 20.0);

  ASSERT_TRUE(harness.waitUntil(
    [&harness, &INITIAL, FINAL_STAMP]() {
      const auto CANDIDATES = harness.candidates();
      return std::ranges::all_of(INITIAL, [&CANDIDATES, FINAL_STAMP](int64_t id) {
        return std::ranges::any_of(CANDIDATES, [id, FINAL_STAMP](const auto& candidate) {
          return candidate.id == id &&
                 rclcpp::Time(candidate.header.stamp).nanoseconds() >= FINAL_STAMP;
        });
      });
    },
    2s));
}

TEST(TrackedObject, ObservationTimeNeverMovesBackwards)
{
  DetectionAdapter detection;
  detection.class_name                  = "cup";
  detection.class_id                    = 1;
  detection.score                       = 0.99;
  detection.frame_id                    = "map";
  detection.bbox3d.center.orientation.w = 1.0;
  detection.bbox3d.size.x               = 1.0;
  detection.bbox3d.size.y               = 1.0;
  detection.bbox3d.size.z               = 1.0;
  TrackedObject object(detection, 10.0, 1, 0.4, 0.05, 3, 20);
  object.update(detection, 9.0);
  EXPECT_DOUBLE_EQ(object.last_seen, 10.0);
  TrackedObject newer(detection, 20.0, 2, 0.4, 0.05, 3, 20);
  object.mergeFrom(newer);
  EXPECT_DOUBLE_EQ(object.last_seen, 20.0);
}

TEST(MultiCameraTracker, ManagerKeepsSharedGlobalIdsAndVisuals)
{
  TrackerHarness harness(true, true);
  ASSERT_TRUE(harness.ready(true));
  publishBatches(harness.front(), 14.0, 3, "1", 4.0, 170);
  ASSERT_TRUE(harness.waitForCandidates(1));
  ASSERT_TRUE(TrackerHarness::waitUntil(
    [&harness]() {
      const auto OBJECTS = harness.getObjects();
      return OBJECTS && OBJECTS->objects.size() == 1U;
    },
    2s));
  const auto FIRST_OBJECTS = harness.getObjects();
  ASSERT_TRUE(FIRST_OBJECTS);
  const int64_t GLOBAL_ID = FIRST_OBJECTS->objects.front().id;
  publishBatches(harness.rear(), 15.0, 3, "42", 6.0, 180);
  ASSERT_TRUE(harness.waitForCandidates(2));
  publishBatches(harness.front(), 16.0, 3, "1", 19.0, 190);
  ASSERT_TRUE(harness.waitForCandidates(3));
  ASSERT_TRUE(TrackerHarness::waitUntil(
    [&harness]() {
      const auto OBJECTS = harness.getObjects();
      return OBJECTS && OBJECTS->objects.size() == 2U;
    },
    2s));
  const auto OBJECTS = harness.getObjects();
  ASSERT_TRUE(OBJECTS);
  std::vector<int64_t> ids;
  for (const auto& object : OBJECTS->objects)
  {
    ids.push_back(object.id);
  }
  EXPECT_NE(std::ranges::find(ids, GLOBAL_ID), ids.end());
  const auto VISUALS = harness.getVisuals(ids);
  ASSERT_TRUE(VISUALS);
  ASSERT_EQ(VISUALS->visuals.size(), 2U);
  for (const auto& visuals : VISUALS->visuals)
  {
    ASSERT_FALSE(visuals.image_crops.empty());
    if (visuals.object_id == GLOBAL_ID)
    {
      EXPECT_TRUE(std::ranges::any_of(visuals.image_crops, [](const auto& crop) {
        return !crop.data.empty() && crop.data.front() == 180;
      }));
    }
  }
}

} // namespace

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int RESULT = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return RESULT;
}
