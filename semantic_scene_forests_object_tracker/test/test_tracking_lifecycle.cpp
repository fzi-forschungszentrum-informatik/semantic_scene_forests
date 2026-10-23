// Copyright (c) 2026 FZI Forschungszentrum Informatik
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <thread>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <semantic_scene_forests_object_msgs/msg/object_candidate_array.hpp>
#include <semantic_scene_forests_object_tracker/object_tracker_component.hpp>

using namespace std::chrono_literals;
using semantic_scene_forests_object_tracker::ObjectTrackerComponent;
using Snapshot = semantic_scene_forests_object_msgs::msg::ObjectCandidateArray;

namespace {

struct Observation
{
  double x;
  std::string label = "box";
  double size       = 0.4;
};

class TrackingLifecycle : public testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void start(std::vector<rclcpp::Parameter> overrides = {})
  {
    std::vector<rclcpp::Parameter> params{{"confirmation_threshold", 2},
                                          {"measurement_noise_std", 0.01},
                                          {"gate_threshold_confirmed", 1000.0},
                                          {"gate_threshold_strict", 1000.0},
                                          {"max_crops_per_object", 2}};
    params.insert(params.end(), overrides.begin(), overrides.end());
    tracker =
      std::make_shared<ObjectTrackerComponent>(rclcpp::NodeOptions().parameter_overrides(params));
    io    = std::make_shared<rclcpp::Node>("tracking_lifecycle_io");
    det2  = io->create_publisher<vision_msgs::msg::Detection2DArray>("detections_2d",
                                                                    rclcpp::SensorDataQoS());
    det3  = io->create_publisher<vision_msgs::msg::Detection3DArray>("detections_3d",
                                                                    rclcpp::SensorDataQoS());
    image = io->create_publisher<sensor_msgs::msg::Image>("camera/image", rclcpp::SensorDataQoS());
    snapshots = io->create_subscription<Snapshot>("object_candidate_snapshots",
                                                  rclcpp::QoS(1).reliable().transient_local(),
                                                  [this](Snapshot::ConstSharedPtr msg) {
                                                    latest = *msg;
                                                    ++received;
                                                  });
    executor.add_node(tracker);
    executor.add_node(io);
    ASSERT_TRUE(wait([this] {
      return det2->get_subscription_count() && det3->get_subscription_count() &&
             image->get_subscription_count();
    }));
    spin(50ms);
  }

  bool wait(const std::function<bool()>& predicate)
  {
    const auto deadline = std::chrono::steady_clock::now() + 1500ms;
    while (!predicate() && std::chrono::steady_clock::now() < deadline)
    {
      executor.spin_some();
      std::this_thread::sleep_for(2ms);
    }
    return predicate();
  }

  void spin(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline)
    {
      executor.spin_some();
      std::this_thread::sleep_for(2ms);
    }
  }

  void publish(int seconds,
               std::initializer_list<Observation> observations,
               bool valid_image        = true,
               bool include_image      = true,
               uint32_t two_offset_ns  = 0,
               uint32_t observation_ns = 0)
  {
    vision_msgs::msg::Detection2DArray two;
    vision_msgs::msg::Detection3DArray three;
    three.header.frame_id      = "map";
    three.header.stamp.sec     = seconds;
    three.header.stamp.nanosec = observation_ns;
    two.header                 = three.header;
    two.header.stamp.nanosec += two_offset_ns;
    size_t index = 0;
    for (const auto& observation : observations)
    {
      vision_msgs::msg::Detection2D a;
      vision_msgs::msg::Detection3D b;
      a.header = two.header;
      b.header = three.header;
      a.id = b.id = std::to_string(++index);
      vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
      hypothesis.hypothesis.class_id = observation.label;
      hypothesis.hypothesis.score    = 0.99;
      a.results.push_back(hypothesis);
      b.results.push_back(hypothesis);
      a.bbox.center.position.x = a.bbox.center.position.y = 16;
      a.bbox.size_x = a.bbox.size_y = 8;
      b.bbox.center.position.x      = observation.x;
      b.bbox.center.position.z      = 1;
      b.bbox.center.orientation.w   = 1;
      b.bbox.size.x = b.bbox.size.y = b.bbox.size.z = observation.size;
      two.detections.push_back(a);
      three.detections.push_back(b);
    }
    sensor_msgs::msg::Image img;
    img.header = three.header;
    img.height = img.width = 32;
    img.encoding           = valid_image ? "bgr8" : "not_an_encoding";
    img.step               = 96;
    img.data.resize(32 * 96, 127);
    det2->publish(two);
    det3->publish(three);
    if (include_image)
      image->publish(img);
  }

  void
  batch(int seconds, std::initializer_list<Observation> observations, uint32_t observation_ns = 0)
  {
    const auto before = received;
    publish(seconds, observations, true, true, 0, observation_ns);
    ASSERT_TRUE(wait([this, before] { return received > before; })) << "batch " << seconds;
  }

  void confirm(std::initializer_list<Observation> observations)
  {
    batch(10, observations);
    batch(11, observations);
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<ObjectTrackerComponent> tracker;
  rclcpp::Node::SharedPtr io;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr det2;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr det3;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image;
  rclcpp::Subscription<Snapshot>::SharedPtr snapshots;
  Snapshot latest;
  size_t received = 0;
};

TEST_F(TrackingLifecycle, InitialEmptySnapshotIsLatched)
{
  start();
  ASSERT_TRUE(wait([this] { return received > 0; }));
  EXPECT_TRUE(latest.objects.empty());
  EXPECT_EQ(latest.header.frame_id, "map");
}

TEST_F(TrackingLifecycle, RecentReacquisitionAndEmptyBatchesPreserveObservationTime)
{
  start();
  bool remembered_diagnostic = false;
  auto diagnostics           = io->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10, [&remembered_diagnostic](const diagnostic_msgs::msg::DiagnosticArray& msg) {
      for (const auto& status : msg.status)
      {
        bool recent_zero    = false;
        bool remembered_one = false;
        for (const auto& value : status.values)
        {
          recent_zero |= value.key == "Recent tracks" && value.value == "0";
          remembered_one |= value.key == "Remembered tracks" && value.value == "1";
        }
        remembered_diagnostic |= recent_zero && remembered_one;
      }
    });
  confirm({{0}});
  ASSERT_EQ(latest.objects.size(), 1u);
  const auto id = latest.objects[0].id;
  batch(16, {{0.1}}); // Exactly the inclusive five-second boundary.
  ASSERT_EQ(latest.objects.size(), 1u);
  EXPECT_EQ(latest.objects[0].id, id);
  EXPECT_EQ(latest.objects[0].last_seen.sec, 16);
  EXPECT_EQ(latest.objects[0].last_seen.nanosec, 0u);
  batch(22, {});
  batch(30, {});
  ASSERT_EQ(latest.objects.size(), 1u);
  EXPECT_EQ(latest.objects[0].id, id);
  EXPECT_EQ(latest.objects[0].last_seen.sec, 16);
  EXPECT_EQ(latest.objects[0].detection.header.stamp.sec, 16);
  EXPECT_EQ(latest.objects[0].detection.header.stamp.nanosec, 0u);
  EXPECT_LE(latest.objects[0].image_crops.size(), 2u);
  EXPECT_TRUE(wait([&remembered_diagnostic] { return remembered_diagnostic; }));
}

TEST_F(TrackingLifecycle, PhysicalGateRejectsDistantRecentDetection)
{
  start();
  confirm({{0}});
  batch(12, {{3}});
  batch(13, {{3}});
  ASSERT_EQ(latest.objects.size(), 2u);
  EXPECT_EQ(latest.objects[0].last_seen.sec, 11);
  EXPECT_EQ(latest.objects[1].last_seen.sec, 13);
}

TEST_F(TrackingLifecycle, StalenessPrecedesMatchingAndOnlyConfirmedReplacementRetiresOldId)
{
  start();
  confirm({{0}});
  ASSERT_EQ(latest.objects.size(), 1u);
  const auto old_id = latest.objects[0].id;
  batch(17, {{0.05}});
  ASSERT_EQ(latest.objects.size(), 1u);
  EXPECT_EQ(latest.objects[0].id, old_id);
  EXPECT_EQ(latest.objects[0].last_seen.sec, 11);
  batch(18, {{0.05}});
  ASSERT_EQ(latest.objects.size(), 1u);
  EXPECT_NE(latest.objects[0].id, old_id);
  EXPECT_EQ(latest.objects[0].last_seen.sec, 18);
}

TEST_F(TrackingLifecycle, DifferentClassMayReplaceRememberedObject)
{
  start();
  confirm({{0}});
  batch(17, {{0, "chair"}});
  batch(18, {{0, "chair"}});
  ASSERT_EQ(latest.objects.size(), 1u);
  EXPECT_EQ(latest.objects[0].class_name, "chair");
  EXPECT_EQ(latest.objects[0].id, 2);
}

TEST_F(TrackingLifecycle, AmbiguousRememberedPartnersAreKeptAndReconsidered)
{
  start();
  confirm({{0, "old_a"}, {0.3, "old_b"}});
  batch(17, {{0.15, "new"}});
  batch(18, {{0.15, "new"}});
  ASSERT_EQ(latest.objects.size(), 3u);
  batch(19, {{-0.02, "new"}});
  ASSERT_EQ(latest.objects.size(), 2u);
  EXPECT_EQ(latest.objects[0].class_name, "old_b");
  EXPECT_EQ(latest.objects[1].class_name, "new");
}

TEST_F(TrackingLifecycle, AmbiguityIncludesRecentCandidatesNotObservedInCurrentBatch)
{
  start();
  confirm({{0, "old"}});
  batch(17, {{-0.1, "new_a"}, {0.1, "new_b"}});
  batch(18, {{-0.1, "new_a"}, {0.1, "new_b"}});
  ASSERT_EQ(latest.objects.size(), 3u);
  batch(19, {{-0.1, "new_a"}});
  EXPECT_EQ(latest.objects.size(), 3u);
  batch(25, {}); // All remembered: an older identity must never retire a newer one.
  EXPECT_EQ(latest.objects.size(), 3u);
}

TEST_F(TrackingLifecycle, ReplacementRequiresComparableDimensionsNotOverlapAlone)
{
  start();
  confirm({{0}});
  batch(17, {{0, "large", 1.2}});
  batch(18, {{0, "large", 1.2}});
  EXPECT_EQ(latest.objects.size(), 2u);
}

TEST_F(TrackingLifecycle, RememberedOverlapCannotBypassReplacementDistance)
{
  start();
  confirm({{0, "box", 2.0}});
  batch(17, {{0.3, "box", 2.0}});
  batch(18, {{0.3, "box", 2.0}});
  EXPECT_EQ(latest.objects.size(), 2u);
}

TEST_F(TrackingLifecycle, RecentDifferentClassObjectIsNotReplaced)
{
  start();
  confirm({{0}});
  batch(12, {{0, "chair"}});
  batch(13, {{0, "chair"}});
  ASSERT_EQ(latest.objects.size(), 2u);
  EXPECT_EQ(latest.objects[0].last_seen.sec, 11);
}

TEST_F(TrackingLifecycle, TentativeLifetimeUsesUnconfirmedTimeoutNotReacquisitionTimeout)
{
  start();
  batch(10, {{0}});
  batch(16, {{0}});
  ASSERT_EQ(latest.objects.size(), 1u);
  EXPECT_EQ(latest.objects[0].id, 1);
}

TEST_F(TrackingLifecycle, RememberedTracksDoNotConsumeActiveCapacity)
{
  start({{"max_trackable_objects", 1}});
  confirm({{0}});
  batch(17, {{3}});
  batch(18, {{3}});
  EXPECT_EQ(latest.objects.size(), 2u);
}

TEST_F(TrackingLifecycle, ExpiredTentativeTracksArePrunedBeforeAdmission)
{
  start({{"max_trackable_objects", 1}, {"unconfirmed_track_timeout", 2.0}});
  batch(10, {{0}});
  batch(13, {{3}});
  batch(14, {{3}});
  ASSERT_EQ(latest.objects.size(), 1u);
  EXPECT_EQ(latest.objects[0].id, 2);
}

TEST_F(TrackingLifecycle, FinalSnapshotRemovesPreviouslyPublishedMergedTrack)
{
  start();
  confirm({{0}, {0.8}});
  ASSERT_EQ(latest.objects.size(), 2u);
  batch(12, {{0.25}, {0.55}});
  batch(13, {{0.38}, {0.42}});
  ASSERT_EQ(latest.objects.size(), 1u);
  EXPECT_EQ(latest.objects[0].id, 1);
  EXPECT_EQ(latest.objects[0].last_seen.sec, 13);
  EXPECT_LE(latest.objects[0].image_crops.size(), 2u);
}

TEST_F(TrackingLifecycle, DuplicateOutOfOrderAndMissingStreamDoNotChangeObservations)
{
  start();
  confirm({{0}});
  ASSERT_EQ(latest.objects.size(), 1u);
  const auto before = received;
  publish(11, {{0.4}});
  spin(100ms);
  publish(9, {{0.4}});
  spin(100ms);
  EXPECT_EQ(received, before);
  EXPECT_EQ(latest.objects[0].last_seen.sec, 11);
  publish(12, {{0.4}}, true, false);
  spin(100ms);
  EXPECT_EQ(received, before);
}

TEST_F(TrackingLifecycle, FailedImageConversionDoesNotConsumeObservationBatch)
{
  start();
  confirm({{0}});
  ASSERT_EQ(latest.objects.size(), 1u);
  const auto before = received;
  publish(17, {{0.1}}, false);
  spin(100ms);
  EXPECT_EQ(received, before);
  batch(17, {{0.1}});
  ASSERT_EQ(latest.objects.size(), 1u);
  EXPECT_EQ(latest.objects[0].last_seen.sec, 11);
  batch(18, {{0.1}});
  ASSERT_EQ(latest.objects.size(), 1u);
  EXPECT_EQ(latest.objects[0].id, 2);
}

TEST_F(TrackingLifecycle, ConfiguredDimensionAlphaReachesTrackFilter)
{
  start({{"bbox_size_alpha", 0.5}});
  batch(10, {{0, "box", 0.4}});
  batch(11, {{0, "box", 0.8}});
  ASSERT_EQ(latest.objects.size(), 1u);
  EXPECT_NEAR(latest.objects[0].detection.bbox.size.x, 0.6, 1e-9);
}

TEST_F(TrackingLifecycle, ApproximatelySynchronizedBatchUsesThreeDimensionalStamp)
{
  start();
  confirm({{0}});
  const auto before = received;
  publish(12, {{0.1}}, true, true, 10000000);
  spin(20ms);
  // ApproximateTime may need a later sample to prove the optimal non-exact match.
  publish(13, {});
  ASSERT_TRUE(wait([this, before] { return received > before; }));
  ASSERT_EQ(latest.objects.size(), 1u);
  EXPECT_EQ(latest.objects[0].last_seen.sec, 12);
  EXPECT_EQ(latest.objects[0].last_seen.nanosec, 0u);
  EXPECT_EQ(latest.objects[0].detection.header.stamp.nanosec, 0u);
}

TEST_F(TrackingLifecycle, EstablishedNeighborDoesNotGainReplacementAuthorityThroughAge)
{
  start();
  confirm({{0, "a"}, {0.1, "b"}});
  for (int seconds = 12; seconds <= 17; ++seconds)
  {
    batch(seconds, {{0, "a"}});
  }
  ASSERT_EQ(latest.objects.size(), 2u);
  EXPECT_EQ(latest.objects[0].id, 1);
  EXPECT_EQ(latest.objects[1].id, 2);
  EXPECT_EQ(latest.objects[1].last_seen.sec, 11);
}

TEST_F(TrackingLifecycle, SuccessfulReplacementDoesNotGrantPermanentRetirementAuthority)
{
  start();
  confirm({{0, "old"}});
  batch(17, {{0.1, "a"}});
  batch(18, {{0.1, "a"}});
  ASSERT_EQ(latest.objects.size(), 1u);
  batch(19, {{0.2, "b"}});
  batch(20, {{0.2, "b"}});
  for (int seconds = 21; seconds <= 26; ++seconds)
  {
    batch(seconds, {{0.1, "a"}});
  }
  ASSERT_EQ(latest.objects.size(), 2u);
  EXPECT_EQ(latest.objects[0].class_name, "a");
  EXPECT_EQ(latest.objects[1].class_name, "b");
}

TEST_F(TrackingLifecycle, EpochObservationStampIsPublishedExactly)
{
  start({{"confirmation_threshold", 1}});
  batch(1792713600, {{0}}, 123456789);
  ASSERT_EQ(latest.objects.size(), 1u);
  EXPECT_EQ(latest.objects[0].last_seen.sec, 1792713600);
  EXPECT_EQ(latest.objects[0].last_seen.nanosec, 123456789u);
  EXPECT_EQ(latest.objects[0].detection.header.stamp, latest.objects[0].last_seen);
}

TEST_F(TrackingLifecycle, OneNanosecondNewerEpochBatchIsAccepted)
{
  start({{"confirmation_threshold", 1}});
  batch(1792713600, {{0}}, 123456789);
  batch(1792713600, {{0.1}}, 123456790);
  ASSERT_EQ(latest.objects.size(), 1u);
  EXPECT_EQ(latest.objects[0].last_seen.nanosec, 123456790u);
  EXPECT_GT(latest.objects[0].detection.bbox.center.position.x, 0.05);
}

TEST_F(TrackingLifecycle, EpochReacquisitionBoundaryIsExactToOneNanosecond)
{
  start({{"confirmation_threshold", 1}});
  batch(1792713600, {{0}}, 123456789);
  batch(1792713605, {{0}}, 123456789);
  ASSERT_EQ(latest.objects.size(), 1u);
  EXPECT_EQ(latest.objects[0].id, 1);
  batch(1792713610, {{0}}, 123456790); // Five seconds plus one nanosecond.
  ASSERT_EQ(latest.objects.size(), 1u);
  EXPECT_EQ(latest.objects[0].id, 2);
  EXPECT_EQ(latest.objects[0].last_seen.nanosec, 123456790u);
}

TEST_F(TrackingLifecycle, RejectsCommonInvalidLifecycleParameters)
{
  EXPECT_THROW(std::make_shared<ObjectTrackerComponent>(rclcpp::NodeOptions().parameter_overrides(
                 {rclcpp::Parameter("bbox_size_alpha", 0.0)})),
               std::exception);
  EXPECT_THROW(std::make_shared<ObjectTrackerComponent>(rclcpp::NodeOptions().parameter_overrides(
                 {rclcpp::Parameter("reacquisition_timeout", 0.0)})),
               std::exception);
}

} // namespace
