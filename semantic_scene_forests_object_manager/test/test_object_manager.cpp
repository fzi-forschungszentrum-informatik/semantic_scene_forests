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
#include <condition_variable>
#include <cstdint>
#include <future>
#include <latch>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "semantic_scene_forests_object_manager/object_manager.hpp"
#include "semantic_scene_forests_object_manager/object_manager_node.hpp"
#include "semantic_scene_forests_object_msgs/msg/object_candidate.hpp"
#include "semantic_scene_forests_object_msgs/msg/object_candidate_array.hpp"
#include "semantic_scene_forests_object_msgs/srv/edit_object.hpp"
#include "semantic_scene_forests_object_msgs/srv/unblock_object.hpp"

namespace {

using namespace std::chrono_literals;

using semantic_scene_forests_object_manager::ObjectManager;
using semantic_scene_forests_object_manager::ObjectManagerNode;
using semantic_scene_forests_object_msgs::msg::ManagedObjectArray;
using semantic_scene_forests_object_msgs::msg::ObjectCandidate;
using semantic_scene_forests_object_msgs::msg::ObjectCandidateArray;
using semantic_scene_forests_object_msgs::srv::EditObject;
using semantic_scene_forests_object_msgs::srv::UnblockObject;

constexpr int64_t OBJECT_ID = 7;

sensor_msgs::msg::Image makeCrop(const std::string& frame_id, uint8_t tag)
{
  sensor_msgs::msg::Image crop;
  crop.header.frame_id = frame_id;
  crop.height          = 1;
  crop.width           = 1;
  crop.encoding        = "mono8";
  crop.step            = 1;
  crop.data            = {tag};
  return crop;
}

ObjectCandidate makeCandidate(int64_t id, const std::string& class_name, size_t crop_count)
{
  ObjectCandidate candidate;
  candidate.id                                  = id;
  candidate.class_name                          = class_name;
  candidate.header.frame_id                     = "world";
  candidate.header.stamp                        = rclcpp::Time(100, 0);
  candidate.last_seen                           = rclcpp::Time(90, 123456789);
  candidate.detection.header.frame_id           = "world";
  candidate.detection.header.stamp              = candidate.last_seen;
  candidate.detection.bbox.center.position.x    = 1.5;
  candidate.detection.bbox.center.orientation.w = 1.0;
  candidate.detection.bbox.size.x               = 0.4;
  candidate.detection.bbox.size.y               = 0.5;
  candidate.detection.bbox.size.z               = 0.6;
  vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
  hypothesis.hypothesis.class_id = class_name;
  hypothesis.hypothesis.score    = 0.8;
  candidate.detection.results.push_back(hypothesis);
  candidate.image_crops.reserve(crop_count);
  for (size_t i = 0; i < crop_count; ++i)
  {
    candidate.image_crops.push_back(makeCrop("cam_" + std::to_string(i), static_cast<uint8_t>(i)));
  }
  return candidate;
}

ObjectCandidateArray makeSnapshot(std::vector<ObjectCandidate> objects)
{
  ObjectCandidateArray snapshot;
  snapshot.header.frame_id = "world";
  snapshot.header.stamp    = rclcpp::Time(100, 0);
  snapshot.objects         = std::move(objects);
  return snapshot;
}

EditObject::Request makeEditRequest(int64_t id, std::vector<uint32_t> remove_indices)
{
  EditObject::Request request;
  request.object_id           = id;
  request.update_class_name   = true;
  request.class_name          = "edited_class";
  request.update_pose         = true;
  request.pose.position.x     = 9.0;
  request.pose.orientation.w  = 8.0;
  request.remove_crop_indices = std::move(remove_indices);
  return request;
}

class ObjectManagerTest : public ::testing::Test
{
protected:
  ObjectManager m_manager;
  rclcpp::Time m_now{100, 0};
};

TEST_F(ObjectManagerTest, BlockedObjectIsSuppressedUntilSubsequentSnapshot)
{
  m_manager.processObjectSnapshot(makeSnapshot({makeCandidate(OBJECT_ID, "cup", 3)}), m_now);
  ASSERT_FALSE(m_manager.getManagedObjects("world").empty());
  ASSERT_TRUE(m_manager.editObject(makeEditRequest(OBJECT_ID, {0}), m_now));

  ASSERT_TRUE(m_manager.blockObject(OBJECT_ID));

  // Blocking erases the payload from every accessor and counts the block.
  EXPECT_TRUE(m_manager.getManagedObjects("world").empty());
  EXPECT_TRUE(m_manager.buildMessage("world", m_now).objects.empty());
  EXPECT_TRUE(m_manager.getObjectVisuals({OBJECT_ID}).empty());
  EXPECT_EQ(m_manager.getBlockedObjectCount(), 1U);

  // A block survives absence as well as a later snapshot containing the ID.
  m_manager.processObjectSnapshot(makeSnapshot({}), m_now);
  m_manager.processObjectSnapshot(makeSnapshot({makeCandidate(OBJECT_ID, "cup", 3)}), m_now);
  EXPECT_TRUE(m_manager.getManagedObjects("world").empty());
  EXPECT_EQ(m_manager.getBlockedObjectCount(), 1U);
  EXPECT_FALSE(m_manager.editObject(makeEditRequest(OBJECT_ID, {0}), m_now));

  // Unblocking lifts the suppression but does not resurrect the old payload.
  ASSERT_TRUE(m_manager.unblockObject(OBJECT_ID));
  EXPECT_EQ(m_manager.getBlockedObjectCount(), 0U);
  EXPECT_TRUE(m_manager.getManagedObjects("world").empty());

  // A subsequent snapshot is accepted again, without the erased overrides.
  m_manager.processObjectSnapshot(makeSnapshot({makeCandidate(OBJECT_ID, "cup", 1)}), m_now);
  const auto OBJECTS = m_manager.getManagedObjects("world");
  ASSERT_EQ(OBJECTS.size(), 1U);
  EXPECT_EQ(OBJECTS[0].id, OBJECT_ID);
  EXPECT_EQ(OBJECTS[0].class_name, "cup");
  EXPECT_EQ(OBJECTS[0].source, 0);
  EXPECT_DOUBLE_EQ(OBJECTS[0].detection.bbox.center.position.x, 1.5);
  EXPECT_EQ(m_manager.getObjectVisuals({OBJECT_ID})[0].image_crops.size(), 1U);
}

TEST_F(ObjectManagerTest, EditObjectValidatesCropIndices)
{
  m_manager.processObjectSnapshot(makeSnapshot({makeCandidate(OBJECT_ID, "chair", 3)}), m_now);

  // Duplicate indices collapse: {1, 1} removes a single crop and keeps the rest.
  EXPECT_TRUE(m_manager.editObject(makeEditRequest(OBJECT_ID, {1, 1}), m_now));
  const auto VISUALS = m_manager.getObjectVisuals({OBJECT_ID});
  ASSERT_EQ(VISUALS.size(), 1U);
  ASSERT_EQ(VISUALS[0].image_crops.size(), 2U);
  EXPECT_EQ(VISUALS[0].image_crops[0].header.frame_id, "cam_0");
  EXPECT_EQ(VISUALS[0].image_crops[1].header.frame_id, "cam_2");

  // An out-of-range index rejects the entire edit: the object state afterwards
  // is completely unchanged (ROS messages compare by value).
  const auto BEFORE_OBJECTS    = m_manager.getManagedObjects("world");
  const auto BEFORE_VISUALS    = m_manager.getObjectVisuals({OBJECT_ID});
  auto invalid_edit            = makeEditRequest(OBJECT_ID, {0, 99});
  invalid_edit.class_name      = "rejected_class";
  invalid_edit.pose.position.x = -1.0;
  EXPECT_FALSE(m_manager.editObject(invalid_edit, m_now + rclcpp::Duration::from_seconds(1.0)));
  EXPECT_EQ(m_manager.getManagedObjects("world"), BEFORE_OBJECTS);
  EXPECT_EQ(m_manager.getObjectVisuals({OBJECT_ID}), BEFORE_VISUALS);
}

TEST_F(ObjectManagerTest, BlockAndSnapshotOverlapStaysConsistent)
{
  ObjectManager manager;
  manager.processObjectSnapshot(makeSnapshot({makeCandidate(OBJECT_ID, "cup", 3)}), m_now);

  // A real block/snapshot race under the manager mutex: whichever op wins,
  // the object must end up suppressed and absent.
  std::latch start(2);
  std::thread blocker([&manager, &start]() {
    start.arrive_and_wait();
    manager.blockObject(OBJECT_ID);
  });
  std::thread late_snapshot([&manager, &start, this]() {
    start.arrive_and_wait();
    manager.processObjectSnapshot(makeSnapshot({makeCandidate(OBJECT_ID, "cup", 3)}), m_now);
  });
  blocker.join();
  late_snapshot.join();

  EXPECT_TRUE(manager.getManagedObjects("world").empty());
  manager.processObjectSnapshot(makeSnapshot({makeCandidate(OBJECT_ID, "cup", 3)}), m_now);
  EXPECT_TRUE(manager.getManagedObjects("world").empty());
}

TEST_F(ObjectManagerTest, SnapshotMembershipIsAuthoritative)
{
  const auto A = makeCandidate(OBJECT_ID, "cup", 1);
  const auto B = makeCandidate(8, "chair", 2);
  m_manager.processObjectSnapshot(makeSnapshot({A, B}), m_now);
  ASSERT_EQ(m_manager.getManagedObjects("world").size(), 2U);

  m_manager.processObjectSnapshot(makeSnapshot({A}), m_now);
  const auto OBJECTS = m_manager.buildMessage("world", m_now).objects;
  ASSERT_EQ(OBJECTS.size(), 1U);
  EXPECT_EQ(OBJECTS[0].id, OBJECT_ID);
  EXPECT_TRUE(m_manager.getObjectVisuals({8}).empty());

  m_manager.processObjectSnapshot(makeSnapshot({}), m_now);
  EXPECT_TRUE(m_manager.getManagedObjects("world").empty());
  EXPECT_TRUE(m_manager.getObjectVisuals({}).empty());
}

TEST_F(ObjectManagerTest, RepublicationDoesNotAdvanceObservationOrModificationTime)
{
  auto snapshot = makeSnapshot({makeCandidate(OBJECT_ID, "cup", 1)});
  m_manager.processObjectSnapshot(snapshot, m_now);
  const auto FIRST = m_manager.getManagedObjects("world")[0];
  EXPECT_EQ(FIRST.last_seen.sec, 90);
  EXPECT_EQ(FIRST.last_seen.nanosec, 123456789U);
  EXPECT_EQ(FIRST.detection.header.stamp, FIRST.last_seen);
  EXPECT_EQ(FIRST.detection.header.frame_id, "world");
  EXPECT_EQ(FIRST.last_updated, static_cast<builtin_interfaces::msg::Time>(m_now));

  const auto LATER                 = m_now + rclcpp::Duration::from_seconds(10);
  snapshot.header.stamp            = LATER;
  snapshot.objects[0].header.stamp = LATER;
  m_manager.processObjectSnapshot(snapshot, LATER);
  const auto REPUBLISHED = m_manager.buildMessage("world", LATER);
  ASSERT_EQ(REPUBLISHED.objects.size(), 1U);
  EXPECT_EQ(REPUBLISHED.objects[0], FIRST);
  EXPECT_EQ(REPUBLISHED.header.stamp, static_cast<builtin_interfaces::msg::Time>(LATER));

  snapshot.objects[0].last_seen              = rclcpp::Time(105, 987654321);
  snapshot.objects[0].detection.header.stamp = snapshot.objects[0].last_seen;
  m_manager.processObjectSnapshot(snapshot, LATER);
  const auto OBSERVED = m_manager.getManagedObjects("world")[0];
  EXPECT_EQ(OBSERVED.last_seen, snapshot.objects[0].last_seen);
  EXPECT_EQ(OBSERVED.last_updated, static_cast<builtin_interfaces::msg::Time>(LATER));
}

TEST_F(ObjectManagerTest, ClassOverrideKeepsHypothesesConsistentWhilePoseAndCropsUpdate)
{
  auto candidate = makeCandidate(OBJECT_ID, "cup", 1);
  m_manager.processObjectSnapshot(makeSnapshot({candidate}), m_now);
  EditObject::Request edit;
  edit.object_id         = OBJECT_ID;
  edit.update_class_name = true;
  edit.class_name        = "curated_cup";
  const auto EDIT_TIME   = m_now + rclcpp::Duration::from_seconds(1);
  ASSERT_TRUE(m_manager.editObject(edit, EDIT_TIME));
  auto objects = m_manager.getManagedObjects("world");
  ASSERT_EQ(objects.size(), 1U);
  EXPECT_EQ(objects[0].detection.results[0].hypothesis.class_id, "curated_cup");
  EXPECT_EQ(objects[0].last_seen, candidate.last_seen);
  EXPECT_EQ(objects[0].last_updated, static_cast<builtin_interfaces::msg::Time>(EDIT_TIME));

  candidate.class_name                               = "tracker_class";
  candidate.detection.results[0].hypothesis.class_id = "tracker_class";
  candidate.detection.results[0].hypothesis.score    = 0.9;
  candidate.detection.bbox.center.position.x         = 2.5;
  candidate.image_crops.push_back(makeCrop("new_cam", 9));
  const auto UPDATE_TIME = m_now + rclcpp::Duration::from_seconds(2);
  m_manager.processObjectSnapshot(makeSnapshot({candidate}), UPDATE_TIME);
  objects = m_manager.getManagedObjects("world");
  EXPECT_EQ(objects[0].class_name, "curated_cup");
  ASSERT_EQ(objects[0].detection.results.size(), 1U);
  EXPECT_EQ(objects[0].detection.results[0].hypothesis.class_id, "curated_cup");
  EXPECT_DOUBLE_EQ(objects[0].detection.results[0].hypothesis.score, 0.9);
  EXPECT_DOUBLE_EQ(objects[0].detection.bbox.center.position.x, 2.5);
  EXPECT_EQ(m_manager.getObjectVisuals({OBJECT_ID})[0].image_crops.size(), 2U);
  EXPECT_EQ(objects[0].source, 1);
  EXPECT_EQ(objects[0].last_updated, static_cast<builtin_interfaces::msg::Time>(UPDATE_TIME));
}

TEST_F(ObjectManagerTest, PoseOverrideRetainsExplicitPoseWhileDimensionsAndObservationUpdate)
{
  auto candidate = makeCandidate(OBJECT_ID, "cup", 1);
  m_manager.processObjectSnapshot(makeSnapshot({candidate}), m_now);
  EditObject::Request edit;
  edit.object_id          = OBJECT_ID;
  edit.update_pose        = true;
  edit.pose.position.x    = 9.0;
  edit.pose.orientation.z = 0.6;
  edit.pose.orientation.w = 0.8;
  ASSERT_TRUE(m_manager.editObject(edit, m_now + rclcpp::Duration::from_seconds(1)));
  EXPECT_EQ(m_manager.getManagedObjects("world")[0].last_seen, candidate.last_seen);

  candidate.class_name                               = "bowl";
  candidate.detection.results[0].hypothesis.class_id = "bowl";
  candidate.detection.bbox.center.position.x         = 2.0;
  candidate.detection.bbox.size.x                    = 0.7;
  candidate.last_seen                                = rclcpp::Time(102, 234567891);
  candidate.detection.header.stamp                   = candidate.last_seen;
  candidate.image_crops.push_back(makeCrop("new_cam", 9));
  const auto UPDATE_TIME = m_now + rclcpp::Duration::from_seconds(3);
  m_manager.processObjectSnapshot(makeSnapshot({candidate}), UPDATE_TIME);
  const auto OBJECTS = m_manager.getManagedObjects("world");
  ASSERT_EQ(OBJECTS.size(), 1U);
  EXPECT_EQ(OBJECTS[0].detection.bbox.center, edit.pose);
  EXPECT_DOUBLE_EQ(OBJECTS[0].detection.bbox.size.x, 0.7);
  EXPECT_EQ(OBJECTS[0].class_name, "bowl");
  EXPECT_EQ(OBJECTS[0].detection.results[0].hypothesis.class_id, "bowl");
  EXPECT_EQ(OBJECTS[0].last_seen, candidate.last_seen);
  EXPECT_EQ(OBJECTS[0].detection.header, candidate.detection.header);
  EXPECT_EQ(OBJECTS[0].source, 1);
  EXPECT_EQ(OBJECTS[0].last_updated, static_cast<builtin_interfaces::msg::Time>(UPDATE_TIME));
  EXPECT_EQ(m_manager.getObjectVisuals({OBJECT_ID})[0].image_crops.size(), 2U);
}

TEST_F(ObjectManagerTest, CropCurationSurvivesSnapshotsIncludingRemovalOfAllCrops)
{
  auto candidate = makeCandidate(OBJECT_ID, "cup", 3);
  m_manager.processObjectSnapshot(makeSnapshot({candidate}), m_now);
  EditObject::Request edit;
  edit.object_id           = OBJECT_ID;
  edit.remove_crop_indices = {1};
  ASSERT_TRUE(m_manager.editObject(edit, m_now));
  const auto CURATED                         = m_manager.getObjectVisuals({OBJECT_ID});
  candidate.image_crops                      = {makeCrop("replacement", 9)};
  candidate.detection.bbox.center.position.x = 3;
  m_manager.processObjectSnapshot(makeSnapshot({candidate}), m_now);
  EXPECT_EQ(m_manager.getObjectVisuals({OBJECT_ID}), CURATED);
  auto objects = m_manager.getManagedObjects("world");
  EXPECT_DOUBLE_EQ(objects[0].detection.bbox.center.position.x, 3);
  EXPECT_EQ(objects[0].source, 1);

  edit.remove_crop_indices = {0, 1};
  const auto EDIT_TIME     = m_now + rclcpp::Duration::from_seconds(1);
  ASSERT_TRUE(m_manager.editObject(edit, EDIT_TIME));
  m_manager.processObjectSnapshot(makeSnapshot({candidate}),
                                  m_now + rclcpp::Duration::from_seconds(2));
  ASSERT_EQ(m_manager.getObjectVisuals({OBJECT_ID}).size(), 1U);
  EXPECT_TRUE(m_manager.getObjectVisuals({OBJECT_ID})[0].image_crops.empty());
  objects = m_manager.getManagedObjects("world");
  EXPECT_EQ(objects[0].last_updated, static_cast<builtin_interfaces::msg::Time>(EDIT_TIME));
  EXPECT_EQ(objects[0].last_seen, candidate.last_seen);
  EXPECT_EQ(objects[0].source, 1);
}

TEST_F(ObjectManagerTest, RetiringAnIdDiscardsOverridesWithoutTransferringToReplacement)
{
  m_manager.processObjectSnapshot(makeSnapshot({makeCandidate(OBJECT_ID, "cup", 3)}), m_now);
  ASSERT_TRUE(m_manager.editObject(makeEditRequest(OBJECT_ID, {0}), m_now));
  const auto REPLACEMENT = makeCandidate(8, "bowl", 1);
  m_manager.processObjectSnapshot(makeSnapshot({REPLACEMENT}), m_now);
  const auto OBJECTS = m_manager.getManagedObjects("world");
  ASSERT_EQ(OBJECTS.size(), 1U);
  EXPECT_EQ(OBJECTS[0].id, 8);
  EXPECT_EQ(OBJECTS[0].class_name, "bowl");
  EXPECT_EQ(OBJECTS[0].source, 0);
  EXPECT_TRUE(m_manager.getObjectVisuals({OBJECT_ID}).empty());

  m_manager.processObjectSnapshot(makeSnapshot({makeCandidate(OBJECT_ID, "cup", 3)}), m_now);
  EXPECT_EQ(m_manager.getManagedObjects("world")[0].class_name, "cup");
  EXPECT_EQ(m_manager.getManagedObjects("world")[0].source, 0);
  EXPECT_EQ(m_manager.getObjectVisuals({OBJECT_ID})[0].image_crops.size(), 3U);
}

TEST_F(ObjectManagerTest, RejectedEditDoesNotInstallOverrides)
{
  auto candidate = makeCandidate(OBJECT_ID, "cup", 1);
  m_manager.processObjectSnapshot(makeSnapshot({candidate}), m_now);
  const auto BEFORE = m_manager.getManagedObjects("world");
  EXPECT_FALSE(m_manager.editObject(makeEditRequest(OBJECT_ID, {0, 99}), m_now));
  EXPECT_EQ(m_manager.getManagedObjects("world"), BEFORE);

  candidate.class_name                               = "bowl";
  candidate.detection.results[0].hypothesis.class_id = "bowl";
  candidate.detection.bbox.center.position.x         = 3;
  candidate.image_crops                              = {makeCrop("replacement", 9)};
  m_manager.processObjectSnapshot(makeSnapshot({candidate}), m_now);
  const auto OBJECTS = m_manager.getManagedObjects("world");
  EXPECT_EQ(OBJECTS[0].class_name, "bowl");
  EXPECT_DOUBLE_EQ(OBJECTS[0].detection.bbox.center.position.x, 3);
  EXPECT_EQ(OBJECTS[0].source, 0);
  EXPECT_EQ(m_manager.getObjectVisuals({OBJECT_ID})[0].image_crops, candidate.image_crops);
}

TEST_F(ObjectManagerTest, EmptyEditDoesNotCurateCropsOrChangeModificationTime)
{
  auto candidate = makeCandidate(OBJECT_ID, "cup", 0);
  m_manager.processObjectSnapshot(makeSnapshot({candidate}), m_now);
  EditObject::Request edit;
  edit.object_id = OBJECT_ID;
  ASSERT_TRUE(m_manager.editObject(edit, m_now + rclcpp::Duration::from_seconds(1)));
  EXPECT_EQ(m_manager.getManagedObjects("world")[0].last_updated,
            static_cast<builtin_interfaces::msg::Time>(m_now));
  EXPECT_EQ(m_manager.getManagedObjects("world")[0].source, 0);
  candidate.image_crops.push_back(makeCrop("new_cam", 9));
  m_manager.processObjectSnapshot(makeSnapshot({candidate}), m_now);
  EXPECT_EQ(m_manager.getObjectVisuals({OBJECT_ID})[0].image_crops.size(), 1U);
}

class ObjectManagerNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    m_node        = std::make_shared<ObjectManagerNode>(rclcpp::NodeOptions());
    m_client_node = std::make_shared<rclcpp::Node>("ssf_object_manager_test_client");
    m_executor.add_node(m_node);
    m_executor.add_node(m_client_node);
    m_spinner        = std::thread([this]() { m_executor.spin(); });
    m_unblock_client = m_client_node->create_client<UnblockObject>("unblock_object");
  }

  void TearDown() override
  {
    m_executor.cancel();
    if (m_spinner.joinable())
    {
      m_spinner.join();
    }
  }

  rclcpp::Node::SharedPtr m_node;
  rclcpp::Node::SharedPtr m_client_node;
  rclcpp::executors::SingleThreadedExecutor m_executor;
  std::thread m_spinner;
  rclcpp::Client<UnblockObject>::SharedPtr m_unblock_client;
};

TEST_F(ObjectManagerNodeTest, UnknownUnblockServiceRespondsFalse)
{
  // A real service call over the node's unblock_object service: an unknown id
  // must report failure instead of success.
  ASSERT_TRUE(m_unblock_client->wait_for_service(5s));

  auto request       = std::make_shared<UnblockObject::Request>();
  request->object_id = 999;

  auto future = m_unblock_client->async_send_request(request);
  ASSERT_EQ(future.wait_for(5s), std::future_status::ready);
  EXPECT_FALSE(future.get()->success);
}

class ObjectManagerSnapshotNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    m_client_node = std::make_shared<rclcpp::Node>("ssf_snapshot_test_client", "/snapshot_test");
    m_publisher   = m_client_node->create_publisher<ObjectCandidateArray>(
      "object_candidate_snapshots", rclcpp::QoS(1).reliable().transient_local());
    m_subscription = m_client_node->create_subscription<ManagedObjectArray>(
      "managed_objects",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const ManagedObjectArray& message) {
        const std::scoped_lock LOCK{m_mutex};
        m_messages.push_back(message);
        m_received.notify_all();
      });
    m_executor.add_node(m_client_node);
    m_spinner = std::thread([this]() { m_executor.spin(); });
  }

  void TearDown() override
  {
    m_executor.cancel();
    if (m_spinner.joinable())
    {
      m_spinner.join();
    }
  }

  rclcpp::Node::SharedPtr m_client_node;
  rclcpp::Node::SharedPtr m_node;
  rclcpp::Publisher<ObjectCandidateArray>::SharedPtr m_publisher;
  rclcpp::Subscription<ManagedObjectArray>::SharedPtr m_subscription;
  rclcpp::executors::SingleThreadedExecutor m_executor;
  std::thread m_spinner;
  std::mutex m_mutex;
  std::condition_variable m_received;
  std::vector<ManagedObjectArray> m_messages;
};

TEST_F(ObjectManagerSnapshotNodeTest,
       LateJoiningManagerReconcilesRetainedSnapshotAndPublishesRemovalPromptly)
{
  // Publish once before the real manager exists; no post-discovery republish.
  m_publisher->publish(makeSnapshot({makeCandidate(OBJECT_ID, "cup", 1)}));
  auto options = rclcpp::NodeOptions().arguments({"--ros-args", "-r", "__ns:=/snapshot_test"});
  // A 100-second timer cannot account for either result within the deadline.
  options.parameter_overrides({rclcpp::Parameter("publish_frequency_hz", 0.01)});
  m_node = std::make_shared<ObjectManagerNode>(options);
  m_executor.add_node(m_node);

  std::unique_lock lock(m_mutex);
  ASSERT_TRUE(m_received.wait_for(
    lock, 5s, [this]() { return !m_messages.empty() && m_messages.back().objects.size() == 1U; }));
  const auto OBJECT = m_messages.back().objects[0];
  EXPECT_EQ(OBJECT.id, OBJECT_ID);
  EXPECT_EQ(OBJECT.last_seen.sec, 90);
  EXPECT_EQ(OBJECT.last_seen.nanosec, 123456789U);
  EXPECT_EQ(OBJECT.detection.header.stamp, OBJECT.last_seen);
  m_messages.clear();
  lock.unlock();

  m_publisher->publish(makeSnapshot({}));
  lock.lock();
  ASSERT_TRUE(m_received.wait_for(
    lock, 5s, [this]() { return !m_messages.empty() && m_messages.back().objects.empty(); }));
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
