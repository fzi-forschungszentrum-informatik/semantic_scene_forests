
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
#include <cstddef>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
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

#include "semantic_scene_forests/scene_graph.hpp"
#include "semantic_scene_forests_object_msgs/msg/object_candidate.hpp"
#include "semantic_scene_forests_object_msgs/srv/add_object.hpp"
#include "semantic_scene_forests_object_msgs/srv/add_observations.hpp"
#include "semantic_scene_forests_object_tracker/tracked_object.hpp"
#include "semantic_scene_forests_object_tracker/tracker_worker.hpp"

namespace {

using namespace std::chrono_literals;

using fzi::semantic_scene_forests::ObjectNode;
using fzi::semantic_scene_forests::SceneGraph;
using semantic_scene_forests_object_tracker::DetectionAdapter;
using semantic_scene_forests_object_tracker::TrackedObject;
using semantic_scene_forests_object_tracker::TrackerWorker;
using ObjectCandidate = semantic_scene_forests_object_msgs::msg::ObjectCandidate;
using AddObservations = semantic_scene_forests_object_msgs::srv::AddObservations;
using AddObject       = semantic_scene_forests_object_msgs::srv::AddObject;

//! The object node with this id; throws (failing the test) if the graph has none.
ObjectNode objectNode(const SceneGraph& graph, int64_t id)
{
  const auto NODE = graph.findNode(id);
  if (!NODE || !std::holds_alternative<ObjectNode>(*NODE))
  {
    throw std::runtime_error("object node " + std::to_string(id) + " missing");
  }
  return std::get<ObjectNode>(*NODE);
}

rclcpp::NodeOptions twoCameraOptions()
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("tracker.world_frame", "map");
  options.append_parameter_override("tracker.snapshot_rate_hz", 100.0);
  options.append_parameter_override("tracker.camera_names",
                                    std::vector<std::string>{"front", "rear"});
  options.append_parameter_override("tracker.cameras.front.detections_2d_topic",
                                    "/front/detections_2d");
  options.append_parameter_override("tracker.cameras.front.detections_3d_topic",
                                    "/front/detections_3d");
  options.append_parameter_override("tracker.cameras.front.image_topic", "/front/image");
  options.append_parameter_override("tracker.cameras.rear.detections_2d_topic",
                                    "/rear/detections_2d");
  options.append_parameter_override("tracker.cameras.rear.detections_3d_topic",
                                    "/rear/detections_3d");
  options.append_parameter_override("tracker.cameras.rear.image_topic", "/rear/image");
  return options;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): fixed parameter order
rclcpp::NodeOptions commonTrackerOptions(int confirmation_threshold      = 3,
                                         double gate_threshold_strict    = 3.0,
                                         double gate_threshold_confirmed = 10.0,
                                         double merge_overlap_threshold  = 0.6,
                                         double crop_interval            = 0.0)
{
  auto options = twoCameraOptions();
  options.append_parameter_override("tracker.crop_replace_min_interval_seconds", crop_interval);
  options.append_parameter_override("tracker.confirmation_threshold", confirmation_threshold);
  options.append_parameter_override("tracker.gate_threshold_strict", gate_threshold_strict);
  options.append_parameter_override("tracker.gate_threshold_confirmed", gate_threshold_confirmed);
  options.append_parameter_override("tracker.merge_overlap_threshold", merge_overlap_threshold);
  // These scenarios grow boxes abruptly; size gating is covered in test_camera_tracker.
  options.append_parameter_override("tracker.association_max_size_ratio", 100.0);
  options.append_parameter_override("tracker.crop_padding", 0);
  options.append_parameter_override("tracker.max_crops_per_object", 20);
  return options;
}

rclcpp::NodeOptions defaultTrackerOptions()
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("tracker.crop_replace_min_interval_seconds", 0.0);
  options.arguments({"--ros-args",
                     "-r",
                     "detections_2d:=/remapped/detections_2d",
                     "-r",
                     "detections_3d:=/remapped/detections_3d",
                     "-r",
                     "camera/image:=/remapped/image"});
  options.append_parameter_override("tracker.world_frame", "map");
  options.append_parameter_override("tracker.snapshot_rate_hz", 100.0);
  options.append_parameter_override("tracker.confirmation_threshold", 3);
  options.append_parameter_override("tracker.crop_padding", 0);
  options.append_parameter_override("tracker.max_crops_per_object", 20);
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
  image.data.assign(static_cast<size_t>(image.height) * image.step, tag);
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

vision_msgs::msg::Detection3DArray
makeDetections3D(const rclcpp::Time& stamp,
                 const std::string& frame_id,
                 const std::string& id,
                 // NOLINTNEXTLINE(bugprone-easily-swappable-parameters): fixed parameter order
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
               // NOLINTNEXTLINE(bugprone-easily-swappable-parameters): fixed parameter order
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
                          int confirmation_threshold      = 3,
                          double gate_threshold_strict    = 3.0,
                          double gate_threshold_confirmed = 10.0,
                          double merge_overlap_threshold  = 0.6,
                          double crop_interval            = 0.0)
  {
    m_source_node = std::make_shared<rclcpp::Node>("multi_camera_test_source");
    m_host =
      std::make_shared<rclcpp::Node>("scene_graph_manager",
                                     two_cameras ? commonTrackerOptions(confirmation_threshold,
                                                                        gate_threshold_strict,
                                                                        gate_threshold_confirmed,
                                                                        merge_overlap_threshold,
                                                                        crop_interval)
                                                 : defaultTrackerOptions());
    m_diagnostics.emplace(m_host);
    m_worker.initialize(*m_host, m_graph, "tracker", *m_diagnostics);
    m_worker.start();

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

    // Each graph change appends the objects to the history, like a snapshot did.
    m_poll_timer = m_source_node->create_wall_timer(10ms, [this]() { pollGraph(); });

    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(m_source_node->get_clock());
    m_tf_listener =
      std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer, m_source_node, false);
    m_tf_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(m_source_node);
    m_tf_broadcaster->sendTransform(makeTransform(m_front->frame_id, 1.0));
    if (two_cameras)
    {
      m_tf_broadcaster->sendTransform(makeTransform(m_rear->frame_id, -1.0));
    }

    m_add_observations = m_source_node->create_client<AddObservations>("add_observations");
    m_add_object       = m_source_node->create_client<AddObject>("add_object");

    m_executor.add_node(m_source_node);
    m_executor.add_node(m_host);
    m_spinner = std::thread([this]() { m_executor.spin(); });
  }

  ~TrackerHarness()
  {
    m_executor.cancel();
    if (m_spinner.joinable())
    {
      m_spinner.join();
    }
    m_worker.stop();
  }

  template <typename TPredicate>
  static bool waitUntil(TPredicate&& predicate, std::chrono::milliseconds timeout)
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

  bool ready() const
  {
    return waitUntil(
      [this]() {
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
        return FRONT_READY && REAR_READY && TF_READY && REAR_TF_READY;
      },
      5s);
  }

  CameraPublisher& front()
  {
    if (!m_front)
    {
      throw std::runtime_error("front camera not configured");
    }
    return *m_front;
  }
  CameraPublisher& rear()
  {
    if (!m_rear)
    {
      throw std::runtime_error("rear camera not configured in this harness");
    }
    return *m_rear;
  }

  bool setConfirmationThreshold(int64_t threshold)
  {
    return m_host->set_parameter(rclcpp::Parameter("tracker.confirmation_threshold", threshold))
      .successful;
  }

  std::vector<ObjectCandidate> candidates() const
  {
    std::scoped_lock lock(m_candidates_mutex);
    return m_candidates;
  }

  std::vector<ObjectCandidate> latestCandidates() const
  {
    std::scoped_lock lock(m_candidates_mutex);
    return m_latest_candidates;
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

  SceneGraph& graph() { return *m_graph; }

  bool setParameter(const std::string& name, double value)
  {
    return m_host->set_parameter(rclcpp::Parameter(name, value)).successful;
  }

  std::optional<AddObject::Response>
  addObject(const std::string& class_name,
            const std::string& frame_id,
            // NOLINTNEXTLINE(bugprone-easily-swappable-parameters): fixed parameter order
            double x,
            double size,
            int stamp_sec = 0) const
  {
    if (!m_add_object->wait_for_service(2s))
    {
      return std::nullopt;
    }
    auto request                       = std::make_shared<AddObject::Request>();
    request->header.frame_id           = frame_id;
    request->header.stamp.sec          = stamp_sec;
    request->class_name                = class_name;
    request->bbox.center.position.x    = x;
    request->bbox.center.orientation.w = 1.0;
    request->bbox.size.x = request->bbox.size.y = request->bbox.size.z = size;
    request->image_crops.push_back(makeImage(rclcpp::Time(1, 0), "manual", 99));
    auto future = m_add_object->async_send_request(request);
    if (!waitUntil([&future]() { return future.wait_for(0s) == std::future_status::ready; }, 2s))
    {
      return std::nullopt;
    }
    return *future.get();
  }

  // The graph's object nodes as candidates; crops decoded back to images.
  std::vector<ObjectCandidate> graphObjects() const
  {
    std::vector<ObjectCandidate> result;
    for (const auto& node : m_graph->getObjectNodes())
    {
      ObjectCandidate candidate;
      candidate.id                        = node.id;
      candidate.class_name                = node.class_name;
      candidate.detection.bbox            = node.bbox;
      candidate.detection.header.frame_id = node.frame_id;
      candidate.last_seen                 = node.last_seen;
      for (const auto& crop : node.crops)
      {
        auto image   = *cv_bridge::toCvCopy(*crop, "bgr8")->toImageMsg();
        image.header = crop->header;
        candidate.image_crops.push_back(std::move(image));
      }
      result.push_back(std::move(candidate));
    }
    return result;
  }

  std::optional<AddObservations::Response>
  addObservations(std::vector<ObjectCandidate> observations) const
  {
    if (!m_add_observations->wait_for_service(2s))
    {
      return std::nullopt;
    }
    auto request          = std::make_shared<AddObservations::Request>();
    request->observations = std::move(observations);
    auto future           = m_add_observations->async_send_request(request);
    if (!waitUntil([&future]() { return future.wait_for(0s) == std::future_status::ready; }, 2s))
    {
      return std::nullopt;
    }
    return *future.get();
  }

private:
  void pollGraph()
  {
    auto objects = graphObjects();
    // Change signature: ids, observation times and crop counts.
    std::vector<int64_t> signature;
    for (const auto& object : objects)
    {
      signature.push_back(object.id);
      signature.push_back(rclcpp::Time(object.last_seen).nanoseconds());
      signature.push_back(static_cast<int64_t>(object.image_crops.size()));
    }
    if (signature == m_seen_signature)
    {
      return;
    }
    m_seen_signature = std::move(signature);
    std::scoped_lock lock(m_candidates_mutex);
    m_candidates.insert(m_candidates.end(), objects.begin(), objects.end());
    m_latest_candidates = std::move(objects);
  }

  // NOLINTNEXTLINE(bugprone-easily-swappable-parameters): fixed parameter order
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

  std::shared_ptr<SceneGraph> m_graph{std::make_shared<SceneGraph>()};
  rclcpp::Node::SharedPtr m_source_node;
  rclcpp::Node::SharedPtr m_host;
  TrackerWorker m_worker; // Declared after the host node: destroyed before it.
  std::optional<diagnostic_updater::Updater> m_diagnostics; // Destroyed before the worker.
  std::optional<CameraPublisher> m_front;
  std::optional<CameraPublisher> m_rear;
  rclcpp::TimerBase::SharedPtr m_poll_timer;
  std::vector<int64_t> m_seen_signature; // only touched by m_poll_timer
  rclcpp::Client<AddObservations>::SharedPtr m_add_observations;
  rclcpp::Client<AddObject>::SharedPtr m_add_object;
  std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> m_tf_listener;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> m_tf_broadcaster;
  mutable std::mutex m_candidates_mutex;
  std::vector<ObjectCandidate> m_candidates;
  std::vector<ObjectCandidate> m_latest_candidates;
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
                    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters): fixed parameter order
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

void initializeWorker(const rclcpp::NodeOptions& options)
{
  auto graph = std::make_shared<SceneGraph>();
  auto host  = std::make_shared<rclcpp::Node>("scene_graph_manager", options);
  diagnostic_updater::Updater diagnostics(host);
  TrackerWorker worker;
  worker.initialize(*host, graph, "tracker", diagnostics);
}

TEST(MultiCameraTrackerParameters, AcceptsNamedIndependentInputs)
{
  EXPECT_NO_THROW(initializeWorker(twoCameraOptions()));
}

TEST(MultiCameraTrackerParameters, RejectsEmptyCameraName)
{
  auto options = twoCameraOptions();
  options.append_parameter_override("tracker.camera_names", std::vector<std::string>{"front", ""});
  EXPECT_THROW(initializeWorker(options), std::exception);
}

TEST(MultiCameraTrackerParameters, RejectsDuplicateResolvedTopics)
{
  auto options = twoCameraOptions();
  options.append_parameter_override("tracker.cameras.rear.detections_2d_topic",
                                    "/front/detections_2d");
  EXPECT_THROW(initializeWorker(options), std::exception);
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

TEST(MultiCameraTracker, ExactSyncDropsFramesWithMismatchedStamps)
{
  TrackerHarness harness;
  ASSERT_TRUE(harness.ready());
  for (int index = 0; index < 3; ++index)
  {
    const auto STAMP = rclcpp::Time(static_cast<int64_t>((3.0 + index * 0.01) * 1e9));
    harness.front().publish(STAMP, "7", 4.0, 30, true, true, false);
    harness.front().publish(STAMP + rclcpp::Duration::from_nanoseconds(1000000),
                            "7",
                            4.0,
                            30,
                            false,
                            false,
                            true); // Image 1 ms later: not the same frame.
  }
  EXPECT_TRUE(harness.noCandidatesFor(300ms));
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
  const auto BEFORE_STAMP = rclcpp::Time(BEFORE.back().last_seen).nanoseconds();
  // The rear camera confirms the same object from older frames: it joins the global track
  // without moving its observation time backwards.
  publishBatches(harness.rear(), 9.9, 3, "2", 6.0, 120);
  // Its older crops are not stored (the graph only takes newer ones), so give the cycles time.
  std::this_thread::sleep_for(300ms);
  const auto AFTER_DELAYED = harness.candidates();
  EXPECT_EQ(candidateIds(AFTER_DELAYED).size(), 1U);
  EXPECT_GE(rclcpp::Time(AFTER_DELAYED.back().last_seen).nanoseconds(), BEFORE_STAMP);
  // A duplicate of the rear camera's last frame is rejected.
  harness.rear().publish(rclcpp::Time(static_cast<int64_t>(9.92e9)), "2", 6.0, 121);
  EXPECT_TRUE(harness.noCropTagFor(121, 300ms));
  harness.front().publish(rclcpp::Time(static_cast<int64_t>(10.1e9)), "1", 4.0, 130);
  ASSERT_TRUE(harness.waitForCandidates(BEFORE.size() + 1));
  const auto ALL = harness.candidates();
  for (size_t index = 1; index < ALL.size(); ++index)
  {
    EXPECT_GE(rclcpp::Time(ALL[index].last_seen).nanoseconds(),
              rclcpp::Time(ALL[index - 1].last_seen).nanoseconds());
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
  ASSERT_TRUE(harness.ready());
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

  const auto CONFIRMED = harness.latestCandidates();
  const auto DELAYED   = std::ranges::find_if(
    CONFIRMED, [](const auto& candidate) { return containsCrop(candidate, 166, "rear_frame"); });
  ASSERT_NE(DELAYED, CONFIRMED.end());
  const auto DELAYED_ID = DELAYED->id;
  publishBatches(harness.rear(), 2.0, 3, "77", 21.0, 167);
  ASSERT_TRUE(harness.waitUntil(
    [&harness, DELAYED_ID]() {
      return std::ranges::any_of(harness.latestCandidates(), [DELAYED_ID](const auto& candidate) {
        return candidate.id == DELAYED_ID && containsCrop(candidate, 167, "rear_frame");
      });
    },
    2s));
  EXPECT_EQ(candidateIds(harness.candidates()).size(), 2U);
}

TEST(MultiCameraTracker, PublishedTrackSurvivesMergeWithNewerTrack)
{
  TrackerHarness harness(true, 5, 3.0, 0.1);
  ASSERT_TRUE(harness.ready());

  publishBatches(harness.front(), 20.0, 5, "1", 4.0, 170, 10.0);
  ASSERT_TRUE(harness.waitForCandidates(1));
  const int64_t PUBLISHED_ID = harness.candidates().back().id;
  ASSERT_TRUE(harness.setConfirmationThreshold(6));

  // The new track confirms with more observations than the published survivor.
  publishBatches(harness.rear(), 21.0, 5, "2", 8.0, 180, 10.0);
  // The rear track is not confirmed yet, so nothing changes in the graph to wait for.
  std::this_thread::sleep_for(100ms);
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

TEST(MultiCameraTracker, SnapshotRetiresPublishedTrackAfterMerge)
{
  TrackerHarness harness(true, 3, 0.1, 0.1);
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
    [&harness, &INITIAL]() {
      const auto CANDIDATES = harness.latestCandidates();
      return CANDIDATES.size() == 1U && INITIAL.contains(CANDIDATES.front().id) &&
             rclcpp::Time(CANDIDATES.front().last_seen).nanoseconds() >= FINAL_STAMP;
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
  const rclcpp::Time INITIAL_TIME(1700000000, 123456789, RCL_ROS_TIME);
  const rclcpp::Time DELAYED_TIME(1699999999, 987654321, RCL_ROS_TIME);
  const rclcpp::Time NEWER_TIME(1700000001, 234567891, RCL_ROS_TIME);
  TrackedObject object(detection, INITIAL_TIME, 1, 0.4, 0.05, 3, 20);
  object.update(detection, DELAYED_TIME);
  EXPECT_EQ(object.last_seen, INITIAL_TIME);
  EXPECT_EQ(rclcpp::Time(object.toDetection3D().header.stamp), INITIAL_TIME);
  TrackedObject newer(detection, NEWER_TIME, 2, 0.4, 0.05, 3, 20);
  object.mergeFrom(newer);
  EXPECT_EQ(object.last_seen, NEWER_TIME);
  EXPECT_EQ(rclcpp::Time(object.toDetection3D().header.stamp), NEWER_TIME);
}

ObjectCandidate
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): fixed parameter order
observation(const std::string& class_name, const std::string& frame_id, double x, double size)
{
  ObjectCandidate observation;
  observation.class_name                          = class_name;
  observation.detection.header.frame_id           = frame_id;
  observation.detection.bbox.center.position.x    = x;
  observation.detection.bbox.center.orientation.w = 1.0;
  observation.detection.bbox.size.x               = size;
  observation.detection.bbox.size.y               = size;
  observation.detection.bbox.size.z               = size;
  observation.image_crops.push_back(makeImage(rclcpp::Time(1, 0), "manual", 99));
  return observation;
}

TEST(MultiCameraTracker, ManualObservationsJoinTracksOrStartPersistentOnes)
{
  TrackerHarness harness;
  ASSERT_TRUE(harness.ready());
  publishBatches(harness.front(), 3.0, 3, "1", 4.0, 30); // A cup at map x = 5.
  ASSERT_TRUE(harness.waitForCandidates(1));
  const int64_t CAMERA_ID = harness.candidates().front().id;

  auto negative_stamp                       = observation("cup", "map", 40.0, 0.5);
  negative_stamp.detection.header.stamp.sec = -1;
  const auto RESPONSE                       = harness.addObservations({
    observation("cup", "front_frame", 4.1, 0.9), // the camera's cup, seen by someone else
    observation("cup", "map", 20.0, 0.5),        // something no camera saw
    observation("", "map", 0.0, 0.5),
    observation("cup", "nowhere", 0.0, 0.5),
    observation("cup", "map", 30.0, 0.0), // no size
    negative_stamp,
  });
  ASSERT_TRUE(RESPONSE);
  // NOLINTBEGIN(bugprone-unchecked-optional-access): guarded by ASSERT_TRUE, which the check does
  // not model
  EXPECT_FALSE(RESPONSE->success);
  ASSERT_EQ(RESPONSE->object_ids.size(), 6U);
  EXPECT_EQ(RESPONSE->object_ids[0], CAMERA_ID);
  EXPECT_FALSE(RESPONSE->created[0]);
  const int64_t MANUAL_ID = RESPONSE->object_ids[1];
  EXPECT_NE(MANUAL_ID, 0);
  EXPECT_NE(MANUAL_ID, CAMERA_ID);
  EXPECT_TRUE(RESPONSE->created[1]);
  ASSERT_EQ(RESPONSE->class_names.size(), 6U);
  EXPECT_EQ(RESPONSE->class_names[1], "cup");
  EXPECT_EQ(RESPONSE->class_names[2], "");
  EXPECT_EQ(RESPONSE->object_ids[2], 0);
  EXPECT_EQ(RESPONSE->object_ids[3], 0);
  EXPECT_EQ(RESPONSE->object_ids[4], 0);
  EXPECT_EQ(RESPONSE->object_ids[5], 0);
  EXPECT_NE(RESPONSE->message.find("empty class_name"), std::string::npos);
  EXPECT_NE(RESPONSE->message.find("negative detection timestamp"), std::string::npos);
  // NOLINTEND(bugprone-unchecked-optional-access)

  // The manual track outlives its one-cycle tracklet and carries the observation's crop.
  std::this_thread::sleep_for(100ms); // ~10 global cycles at 100 Hz
  const auto LATEST = harness.latestCandidates();
  const auto MANUAL = std::ranges::find(LATEST, MANUAL_ID, &ObjectCandidate::id);
  ASSERT_NE(MANUAL, LATEST.end());
  EXPECT_DOUBLE_EQ(MANUAL->detection.bbox.center.position.x, 20.0);
  EXPECT_TRUE(containsCrop(*MANUAL, 99, "manual"));
  EXPECT_NE(std::ranges::find(LATEST, CAMERA_ID, &ObjectCandidate::id), LATEST.end());
}

TEST(MultiCameraTracker, GraphKeepsGlobalIdsAndCrops)
{
  TrackerHarness harness;
  ASSERT_TRUE(harness.ready());
  const auto OBJECTS = [&harness]() { return harness.graph().getObjectNodes(); };
  publishBatches(harness.front(), 14.0, 3, "1", 4.0, 170);
  ASSERT_TRUE(TrackerHarness::waitUntil([&]() { return OBJECTS().size() == 1U; }, 2s));
  const int64_t GLOBAL_ID = OBJECTS().front().id;
  publishBatches(harness.rear(), 15.0, 3, "42", 6.0, 180);
  publishBatches(harness.front(), 16.0, 3, "1", 19.0, 190);
  ASSERT_TRUE(TrackerHarness::waitUntil([&]() { return OBJECTS().size() == 2U; }, 2s));
  for (const auto& object : OBJECTS())
  {
    EXPECT_FALSE(object.crops.empty());
    EXPECT_EQ(object.source, ObjectNode::SOURCE_CAMERA);
    EXPECT_EQ(object.frame_id, "map");
  }
  EXPECT_TRUE(harness.graph().hasNode(GLOBAL_ID));
}

// D3 writeToGraph: create, move, class vote ignored.
TEST(MultiCameraTracker, ConfirmedTrackCreatesNodeMovesItAndNeverTouchesItsClass)
{
  TrackerHarness harness;
  ASSERT_TRUE(harness.ready());
  publishBatches(harness.front(), 40.0, 3, "1", 4.0, 10);
  ASSERT_TRUE(
    TrackerHarness::waitUntil([&]() { return harness.graph().getObjectNodes().size() == 1U; }, 2s));
  const auto CREATED = harness.graph().getObjectNodes().front();
  EXPECT_EQ(CREATED.class_name, "cup");
  EXPECT_DOUBLE_EQ(CREATED.bbox.center.position.x, 5.0); // front camera at map x = 1
  // Another component renames it; the tracker keeps observing "cup" but must not rename it back.
  ASSERT_TRUE(harness.graph().setObjectClass(CREATED.id, "mug"));
  publishBatches(harness.front(), 41.0, 3, "1", 4.2, 11);
  ASSERT_TRUE(TrackerHarness::waitUntil(
    [&]() { return harness.graph().getObjectNodes().front().bbox.center.position.x > 5.0; }, 2s));
  const auto MOVED = harness.graph().getObjectNodes().front();
  EXPECT_EQ(MOVED.id, CREATED.id);
  EXPECT_EQ(MOVED.class_name, "mug");
  EXPECT_GT(rclcpp::Time(MOVED.last_seen), rclcpp::Time(CREATED.last_seen));
}

// D3: a newer crop is only offered; the graph takes it after crop_replace_min_interval_seconds.
TEST(MultiCameraTracker, CropsFollowTheReplaceInterval)
{
  TrackerHarness harness(true, 3, 3.0, 10.0, 0.6, 10.0);
  ASSERT_TRUE(harness.ready());
  publishBatches(harness.front(), 50.0, 3, "1", 4.0, 10);
  ASSERT_TRUE(
    TrackerHarness::waitUntil([&]() { return harness.graph().getObjectNodes().size() == 1U; }, 2s));
  publishBatches(harness.front(), 51.0, 3, "1", 4.0, 11); // 1 s later: too soon
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(harness.graph().getObjectNodes().front().crops.size(), 1U);
  publishBatches(harness.front(), 61.0, 3, "1", 4.0, 12); // 11 s later: taken
  ASSERT_TRUE(TrackerHarness::waitUntil(
    [&]() { return harness.graph().getObjectNodes().front().crops.size() == 2U; }, 2s));
}

// D3: node deleted while its track lives: nothing more is written, the node is not recreated.
TEST(MultiCameraTracker, DeletedNodeIsNotRecreatedWhileItsTrackLives)
{
  TrackerHarness harness;
  ASSERT_TRUE(harness.ready());
  publishBatches(harness.front(), 70.0, 3, "1", 4.0, 10);
  ASSERT_TRUE(
    TrackerHarness::waitUntil([&]() { return harness.graph().getObjectNodes().size() == 1U; }, 2s));
  ASSERT_TRUE(harness.graph().removeNode(harness.graph().getObjectNodes().front().id));
  publishBatches(harness.front(), 71.0, 3, "1", 4.0, 11);
  publishBatches(harness.front(), 72.0, 3, "1", 4.0, 12);
  std::this_thread::sleep_for(300ms);
  EXPECT_TRUE(harness.graph().getObjectNodes().empty());
}

// An explicit sighting (add_observations) brings a deleted object back under the same id.
TEST(MultiCameraTracker, ExplicitSightingRevivesDeletedObject)
{
  TrackerHarness harness;
  ASSERT_TRUE(harness.ready());
  publishBatches(harness.front(), 80.0, 3, "1", 4.0, 10);
  ASSERT_TRUE(
    TrackerHarness::waitUntil([&]() { return harness.graph().getObjectNodes().size() == 1U; }, 2s));
  const int64_t ID = harness.graph().getObjectNodes().front().id;
  ASSERT_TRUE(harness.graph().removeNode(ID));
  publishBatches(harness.front(), 81.0, 3, "1", 4.0, 11); // camera sightings: stays hidden
  std::this_thread::sleep_for(200ms);
  ASSERT_TRUE(harness.graph().getObjectNodes().empty());
  const auto RESPONSE = harness.addObservations({observation("cup", "map", 5.0, 1.0)});
  ASSERT_TRUE(RESPONSE && RESPONSE->success) << (RESPONSE ? RESPONSE->message : "no response");
  // NOLINTBEGIN(bugprone-unchecked-optional-access): guarded by ASSERT_TRUE, which the check does
  // not model
  EXPECT_EQ(RESPONSE->object_ids.front(), ID);
  // NOLINTEND(bugprone-unchecked-optional-access)
  ASSERT_TRUE(TrackerHarness::waitUntil([&]() { return harness.graph().hasNode(ID); }, 2s));
}

// D3: retire. A stale track whose pose a differently sized track now occupies is replaced.
TEST(MultiCameraTracker, RetiredTrackLeavesTheGraph)
{
  TrackerHarness harness;
  ASSERT_TRUE(harness.setParameter("tracker.association_max_size_ratio", 2.0));
  ASSERT_TRUE(harness.ready());
  publishBatches(harness.front(), 10.0, 3, "1", 4.0, 10, 1.0);
  ASSERT_TRUE(
    TrackerHarness::waitUntil([&]() { return harness.graph().getObjectNodes().size() == 1U; }, 2s));
  const int64_t OLD_ID = harness.graph().getObjectNodes().front().id;
  publishBatches(harness.rear(), 100.0, 3, "2", 6.0, 20, 10.0); // 90 s later > stale_timeout
  ASSERT_TRUE(TrackerHarness::waitUntil(
    [&]() {
      const auto NODES = harness.graph().getObjectNodes();
      return NODES.size() == 1U && NODES.front().id != OLD_ID;
    },
    2s));
}

// D3: merge. The younger identity's node goes, the older one survives.
TEST(MultiCameraTracker, MergedTrackLeavesTheGraph)
{
  TrackerHarness harness(true, 3, 0.1, 0.1);
  ASSERT_TRUE(harness.ready());
  publishBatches(harness.front(), 30.0, 3, "1", 4.0, 190, 1.0);
  publishBatches(harness.rear(), 30.1, 3, "2", 8.0, 191, 1.0);
  ASSERT_TRUE(
    TrackerHarness::waitUntil([&]() { return harness.graph().getObjectNodes().size() == 2U; }, 2s));
  publishBatches(harness.front(), 31.0, 3, "1", 4.0, 192, 20.0);
  publishBatches(harness.rear(), 31.1, 3, "2", 8.0, 193, 20.0);
  ASSERT_TRUE(
    TrackerHarness::waitUntil([&]() { return harness.graph().getObjectNodes().size() == 1U; }, 2s));
}

// D3/D4: add_object starts a confirmed track (no confirmation count) that is then a normal track.
TEST(MultiCameraTracker, AddObjectTrackIsUpdatedMergedAndRetiredLikeAnyOther)
{
  TrackerHarness harness;
  ASSERT_TRUE(harness.setParameter("tracker.association_max_size_ratio", 2.0));
  ASSERT_TRUE(harness.ready());

  const auto BAD = harness.addObject("", "map", 0.0, 0.5);
  ASSERT_TRUE(BAD);
  // NOLINTBEGIN(bugprone-unchecked-optional-access): guarded by ASSERT_TRUE, which the check does
  // not model
  EXPECT_FALSE(BAD->success);

  const auto ADDED = harness.addObject("cup", "map", 5.0, 1.0, 10);
  ASSERT_TRUE(ADDED);
  ASSERT_TRUE(ADDED->success) << ADDED->message;
  const int64_t ID = ADDED->object_id;
  ASSERT_TRUE(TrackerHarness::waitUntil([&]() { return harness.graph().hasNode(ID); }, 2s));
  // NOLINTEND(bugprone-unchecked-optional-access)
  const auto NODE = objectNode(harness.graph(), ID);
  EXPECT_EQ(NODE.source, ObjectNode::SOURCE_MANUAL);
  EXPECT_EQ(NODE.class_name, "cup");
  ASSERT_EQ(NODE.crops.size(), 1U); // JPEG-encoded on ingress
  EXPECT_EQ(NODE.crops.front()->format, "bgr8; jpeg compressed bgr8");

  // Updated: a camera sighting of the same cup (map x = 5) joins it and moves its last_seen.
  publishBatches(harness.front(), 12.0, 3, "1", 4.0, 30, 1.0);
  ASSERT_TRUE(TrackerHarness::waitUntil(
    [&]() {
      const auto NODES = harness.graph().getObjectNodes();
      return NODES.size() == 1U && NODES.front().id == ID && NODES.front().last_seen.sec >= 12;
    },
    2s));
  EXPECT_EQ(objectNode(harness.graph(), ID).source, ObjectNode::SOURCE_MANUAL);

  // Retired: a differently sized track much later takes over its pose.
  publishBatches(harness.rear(), 200.0, 3, "2", 6.0, 40, 10.0);
  ASSERT_TRUE(TrackerHarness::waitUntil(
    [&]() {
      const auto NODES = harness.graph().getObjectNodes();
      return NODES.size() == 1U && NODES.front().id != ID;
    },
    2s));
}

// D3/D4: an add_object track overlapping a camera track merges into it (older id survives; the
// merged track takes the newer observation time, here the manual one).
TEST(MultiCameraTracker, AddObjectTrackMergesIntoOverlappingCameraTrack)
{
  TrackerHarness harness;
  ASSERT_TRUE(harness.ready());
  publishBatches(harness.rear(), 10.0, 3, "9", 6.0, 50, 1.0); // map x = 5
  ASSERT_TRUE(
    TrackerHarness::waitUntil([&]() { return harness.graph().getObjectNodes().size() == 1U; }, 2s));
  const int64_t CAMERA_ID = harness.graph().getObjectNodes().front().id;
  const auto ADDED        = harness.addObject("cup", "map", 5.0, 1.0, 20);
  ASSERT_TRUE(ADDED && ADDED->success);
  // NOLINTBEGIN(bugprone-unchecked-optional-access): guarded by ASSERT_TRUE, which the check does
  // not model
  EXPECT_NE(ADDED->object_id, CAMERA_ID);
  // NOLINTEND(bugprone-unchecked-optional-access)
  ASSERT_TRUE(TrackerHarness::waitUntil(
    [&]() {
      const auto NODES = harness.graph().getObjectNodes();
      return NODES.size() == 1U && NODES.front().id == CAMERA_ID &&
             NODES.front().last_seen.sec == 20;
    },
    2s));
  // NOLINTBEGIN(bugprone-unchecked-optional-access): guarded by ASSERT_TRUE, which the check does
  // not model
  EXPECT_FALSE(harness.graph().hasNode(ADDED->object_id));
  // NOLINTEND(bugprone-unchecked-optional-access)
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
