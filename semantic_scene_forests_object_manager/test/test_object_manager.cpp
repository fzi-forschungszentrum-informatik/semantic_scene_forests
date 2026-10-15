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
//      documentation and other materials provided with the distribution.
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
#include <latch>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "semantic_scene_forests_object_manager/object_manager.hpp"
#include "semantic_scene_forests_object_manager/object_manager_node.hpp"
#include "semantic_scene_forests_object_msgs/msg/managed_object.hpp"
#include "semantic_scene_forests_object_msgs/msg/managed_object_array.hpp"
#include "semantic_scene_forests_object_msgs/msg/object_candidate.hpp"
#include "semantic_scene_forests_object_msgs/msg/object_visuals.hpp"
#include "semantic_scene_forests_object_msgs/srv/block_object.hpp"
#include "semantic_scene_forests_object_msgs/srv/edit_object.hpp"
#include "semantic_scene_forests_object_msgs/srv/get_objects.hpp"
#include "semantic_scene_forests_object_msgs/srv/unblock_object.hpp"

namespace {

using namespace std::chrono_literals;

using semantic_scene_forests_object_manager::ObjectManager;
using semantic_scene_forests_object_manager::ObjectManagerNode;
using semantic_scene_forests_object_msgs::msg::ManagedObject;
using semantic_scene_forests_object_msgs::msg::ManagedObjectArray;
using semantic_scene_forests_object_msgs::msg::ObjectCandidate;
using semantic_scene_forests_object_msgs::msg::ObjectVisuals;
using semantic_scene_forests_object_msgs::srv::BlockObject;
using semantic_scene_forests_object_msgs::srv::EditObject;
using semantic_scene_forests_object_msgs::srv::GetObjects;
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
  candidate.detection.bbox.center.position.x    = 1.5;
  candidate.detection.bbox.center.orientation.w = 1.0;
  candidate.image_crops.reserve(crop_count);
  for (size_t i = 0; i < crop_count; ++i)
  {
    candidate.image_crops.push_back(makeCrop("cam_" + std::to_string(i), static_cast<uint8_t>(i)));
  }
  return candidate;
}

EditObject::Request makeEditRequest(int64_t id, std::vector<uint32_t> remove_indices)
{
  EditObject::Request request;
  request.object_id           = id;
  request.update_class_name   = true;
  request.update_pose         = true;
  request.remove_crop_indices = std::move(remove_indices);
  request.class_name          = "edited_class";
  request.pose.position.x     = 9.0;
  request.pose.orientation.w  = 8.0;
  return request;
}

class ObjectManagerTest : public ::testing::Test
{
protected:
  ObjectManager m_manager;
  rclcpp::Time m_now{100, 0};

  static ObjectCandidate candidate(int64_t id = OBJECT_ID, size_t crop_count = 3)
  {
    return makeCandidate(id, "cup", crop_count);
  }
};

TEST_F(ObjectManagerTest, BlockRemovesObjectFromAllAccessors)
{
  ASSERT_TRUE(m_manager.processObjectCandidate(candidate(), m_now));
  ASSERT_FALSE(m_manager.getManagedObjects("world").empty());

  ASSERT_TRUE(m_manager.blockObject(OBJECT_ID));

  EXPECT_TRUE(m_manager.getManagedObjects("world").empty());
  EXPECT_TRUE(m_manager.buildMessage("world", m_now).objects.empty());
  EXPECT_TRUE(m_manager.getObjectVisuals({}).empty());
  EXPECT_TRUE(m_manager.getObjectVisuals({OBJECT_ID}).empty());
}

TEST_F(ObjectManagerTest, LateCandidateForBlockedObjectIsDenied)
{
  ASSERT_TRUE(m_manager.processObjectCandidate(candidate(), m_now));
  ASSERT_TRUE(m_manager.blockObject(OBJECT_ID));

  EXPECT_FALSE(m_manager.processObjectCandidate(candidate(), m_now));
  EXPECT_TRUE(m_manager.getManagedObjects("world").empty());
}

TEST_F(ObjectManagerTest, UnblockLeavesNoPayloadUntilNewCandidate)
{
  ASSERT_TRUE(m_manager.processObjectCandidate(makeCandidate(OBJECT_ID, "chair", 2), m_now));
  ASSERT_TRUE(m_manager.blockObject(OBJECT_ID));
  ASSERT_TRUE(m_manager.unblockObject(OBJECT_ID));

  EXPECT_TRUE(m_manager.getManagedObjects("world").empty());
  EXPECT_TRUE(m_manager.getObjectVisuals({OBJECT_ID}).empty());

  ASSERT_TRUE(m_manager.processObjectCandidate(makeCandidate(OBJECT_ID, "cup", 1), m_now));
  const auto OBJECTS = m_manager.getManagedObjects("world");
  ASSERT_EQ(OBJECTS.size(), 1U);
  EXPECT_EQ(OBJECTS[0].id, OBJECT_ID);
  EXPECT_EQ(OBJECTS[0].class_name, "cup");
}

TEST_F(ObjectManagerTest, UnknownUnblockReturnsFalse)
{
  EXPECT_FALSE(m_manager.unblockObject(999));
}

TEST_F(ObjectManagerTest, EditBlockedObjectFails)
{
  ASSERT_TRUE(m_manager.processObjectCandidate(candidate(), m_now));
  ASSERT_TRUE(m_manager.blockObject(OBJECT_ID));

  EXPECT_FALSE(m_manager.editObject(makeEditRequest(OBJECT_ID, {0}), m_now));
}

TEST_F(ObjectManagerTest, RepeatedBlockIsIdempotent)
{
  ASSERT_TRUE(m_manager.processObjectCandidate(candidate(), m_now));
  ASSERT_TRUE(m_manager.blockObject(OBJECT_ID));

  EXPECT_TRUE(m_manager.blockObject(OBJECT_ID));
  EXPECT_TRUE(m_manager.getManagedObjects("world").empty());
}

TEST_F(ObjectManagerTest, DuplicateCropIndicesRemoveSingleCrop)
{
  ASSERT_TRUE(m_manager.processObjectCandidate(candidate(), m_now));

  EXPECT_TRUE(m_manager.editObject(makeEditRequest(OBJECT_ID, {1, 1}), m_now));

  const auto VISUALS = m_manager.getObjectVisuals({OBJECT_ID});
  ASSERT_EQ(VISUALS.size(), 1U);
  EXPECT_EQ(VISUALS[0].object_id, OBJECT_ID);
  ASSERT_EQ(VISUALS[0].image_crops.size(), 2U);
  EXPECT_EQ(VISUALS[0].image_crops[0].header.frame_id, "cam_0");
  EXPECT_EQ(VISUALS[0].image_crops[1].header.frame_id, "cam_2");
}

TEST_F(ObjectManagerTest, InvalidCropIndexRejectsEntireEdit)
{
  ASSERT_TRUE(m_manager.processObjectCandidate(makeCandidate(OBJECT_ID, "chair", 3), m_now));
  const auto BEFORE_OBJECTS = m_manager.getManagedObjects("world");
  const auto BEFORE_VISUALS = m_manager.getObjectVisuals({OBJECT_ID});
  ASSERT_EQ(BEFORE_OBJECTS.size(), 1U);
  ASSERT_EQ(BEFORE_VISUALS.size(), 1U);
  ASSERT_EQ(BEFORE_VISUALS[0].image_crops.size(), 3U);

  EXPECT_FALSE(m_manager.editObject(makeEditRequest(OBJECT_ID, {0, 99}), m_now));

  const auto AFTER_OBJECTS = m_manager.getManagedObjects("world");
  const auto AFTER_VISUALS = m_manager.getObjectVisuals({OBJECT_ID});
  ASSERT_EQ(AFTER_OBJECTS.size(), 1U);
  ASSERT_EQ(AFTER_VISUALS.size(), 1U);
  EXPECT_EQ(AFTER_OBJECTS[0].id, OBJECT_ID);
  EXPECT_EQ(AFTER_OBJECTS[0].class_name, "chair");
  EXPECT_DOUBLE_EQ(AFTER_OBJECTS[0].detection.bbox.center.position.x, 1.5);
  EXPECT_DOUBLE_EQ(AFTER_OBJECTS[0].detection.bbox.center.orientation.w, 1.0);
  EXPECT_EQ(AFTER_OBJECTS[0].source, 0U);
  EXPECT_EQ(AFTER_OBJECTS[0].last_updated, BEFORE_OBJECTS[0].last_updated);
  ASSERT_EQ(AFTER_VISUALS[0].image_crops.size(), 3U);
  EXPECT_EQ(AFTER_VISUALS[0].image_crops[0].header.frame_id, "cam_0");
  EXPECT_EQ(AFTER_VISUALS[0].image_crops[2].header.frame_id, "cam_2");
}

TEST_F(ObjectManagerTest, OtherObjectsUnaffectedByBlock)
{
  ASSERT_TRUE(m_manager.processObjectCandidate(makeCandidate(1, "cup", 1), m_now));
  ASSERT_TRUE(m_manager.processObjectCandidate(makeCandidate(2, "mug", 2), m_now));
  ASSERT_TRUE(m_manager.blockObject(1));

  const auto OBJECTS = m_manager.getManagedObjects("world");
  ASSERT_EQ(OBJECTS.size(), 1U);
  EXPECT_EQ(OBJECTS[0].id, 2);
  EXPECT_EQ(OBJECTS[0].class_name, "mug");

  EXPECT_TRUE(m_manager.editObject(makeEditRequest(2, {0}), m_now));
  const auto VISUALS = m_manager.getObjectVisuals({2});
  ASSERT_EQ(VISUALS.size(), 1U);
  EXPECT_EQ(VISUALS[0].image_crops.size(), 1U);
}

TEST_F(ObjectManagerTest, BlockedCountDiagnostics)
{
  EXPECT_EQ(m_manager.getBlockedObjectCount(), 0U);
  ASSERT_TRUE(m_manager.processObjectCandidate(makeCandidate(1, "cup", 1), m_now));
  EXPECT_EQ(m_manager.getBlockedObjectCount(), 0U);
  ASSERT_TRUE(m_manager.blockObject(1));
  EXPECT_EQ(m_manager.getBlockedObjectCount(), 1U);
  ASSERT_TRUE(m_manager.processObjectCandidate(makeCandidate(2, "mug", 1), m_now));
  ASSERT_TRUE(m_manager.blockObject(2));
  EXPECT_EQ(m_manager.getBlockedObjectCount(), 2U);
  EXPECT_EQ(m_manager.getManagedObjects("world").size(), 0U);
  ASSERT_TRUE(m_manager.unblockObject(1));
  EXPECT_EQ(m_manager.getBlockedObjectCount(), 1U);
  EXPECT_TRUE(m_manager.unblockObject(2));
  EXPECT_EQ(m_manager.getBlockedObjectCount(), 0U);
}

TEST_F(ObjectManagerTest, SimultaneousBlockAndCandidateRace)
{
  for (int iteration = 0; iteration < 25; ++iteration)
  {
    ObjectManager manager;
    ASSERT_TRUE(manager.processObjectCandidate(makeCandidate(OBJECT_ID, "cup", 3), m_now));

    std::latch start(2);
    std::thread blocker([&manager, &start]() {
      start.arrive_and_wait();
      manager.blockObject(OBJECT_ID);
    });
    std::thread late_candidate([&manager, &start, this]() {
      start.arrive_and_wait();
      manager.processObjectCandidate(makeCandidate(OBJECT_ID, "cup", 3), m_now);
    });
    blocker.join();
    late_candidate.join();

    EXPECT_TRUE(manager.getManagedObjects("world").empty());
    EXPECT_FALSE(manager.processObjectCandidate(makeCandidate(OBJECT_ID, "cup", 3), m_now));
  }
}

class ObjectManagerNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    m_node        = std::make_shared<ObjectManagerNode>(rclcpp::NodeOptions());
    m_client_node = std::make_shared<rclcpp::Node>("ssf_task2_test_client");

    m_executor.add_node(m_node);
    m_executor.add_node(m_client_node);
    m_spinner = std::thread([this]() { m_executor.spin(); });

    m_unblock_client     = m_client_node->create_client<UnblockObject>("unblock_object");
    m_block_client       = m_client_node->create_client<BlockObject>("block_object");
    m_get_objects_client = m_client_node->create_client<GetObjects>("get_objects");
    m_edit_client        = m_client_node->create_client<EditObject>("edit_object");
    m_candidate_pub      = m_client_node->create_publisher<ObjectCandidate>("object_candidates",
                                                                       rclcpp::SensorDataQoS());

    ASSERT_TRUE(m_unblock_client->wait_for_service(5s));
    ASSERT_TRUE(m_block_client->wait_for_service(5s));
    ASSERT_TRUE(m_get_objects_client->wait_for_service(5s));
    ASSERT_TRUE(m_edit_client->wait_for_service(5s));
  }

  void TearDown() override
  {
    m_executor.cancel();
    if (m_spinner.joinable())
    {
      m_spinner.join();
    }
    m_executor.remove_node(m_node);
    m_executor.remove_node(m_client_node);
    m_node.reset();
    m_client_node.reset();
  }

  void publishCandidate(int64_t id, size_t crop_count)
  {
    auto message = std::make_unique<ObjectCandidate>(makeCandidate(id, "cup", crop_count));
    m_candidate_pub->publish(std::move(message));
  }

  template <typename TService>
  typename TService::Response::SharedPtr
  callService(const std::shared_ptr<rclcpp::Client<TService>>& client,
              const std::shared_ptr<typename TService::Request>& request)
  {
    auto future_result = client->async_send_request(request);
    if (future_result.wait_for(5s) != std::future_status::ready)
    {
      return nullptr;
    }
    return future_result.get();
  }

  bool hasObject(int64_t id)
  {
    auto request  = std::make_shared<GetObjects::Request>();
    auto response = callService(m_get_objects_client, request);
    if (response == nullptr)
    {
      return false;
    }
    return std::any_of(response->objects.begin(),
                       response->objects.end(),
                       [id](const ManagedObject& object) { return object.id == id; });
  }

  // Polls until the async candidate topic callback has been applied.
  bool waitUntilObject(int64_t id)
  {
    for (size_t attempt = 0; attempt < 200; ++attempt)
    {
      if (hasObject(id))
      {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  rclcpp::Node::SharedPtr m_node;
  rclcpp::Node::SharedPtr m_client_node;
  rclcpp::executors::SingleThreadedExecutor m_executor;
  std::thread m_spinner;

  rclcpp::Client<UnblockObject>::SharedPtr m_unblock_client;
  rclcpp::Client<BlockObject>::SharedPtr m_block_client;
  rclcpp::Client<GetObjects>::SharedPtr m_get_objects_client;
  rclcpp::Client<EditObject>::SharedPtr m_edit_client;
  rclcpp::Publisher<ObjectCandidate>::SharedPtr m_candidate_pub;
};

TEST_F(ObjectManagerNodeTest, UnblockServiceResponseAndLifecycle)
{
  // Unknown unblock must fail.
  auto unblock_unknown_request        = std::make_shared<UnblockObject::Request>();
  unblock_unknown_request->object_id  = 999;
  const auto UNBLOCK_UNKNOWN_RESPONSE = callService(m_unblock_client, unblock_unknown_request);
  ASSERT_NE(UNBLOCK_UNKNOWN_RESPONSE, nullptr);
  EXPECT_FALSE(UNBLOCK_UNKNOWN_RESPONSE->success);

  // Editing an unknown object fails with an accurate message.
  auto edit_unknown_request = std::make_shared<EditObject::Request>(makeEditRequest(999, {0}));
  const auto EDIT_UNKNOWN_RESPONSE = callService(m_edit_client, edit_unknown_request);
  ASSERT_NE(EDIT_UNKNOWN_RESPONSE, nullptr);
  EXPECT_FALSE(EDIT_UNKNOWN_RESPONSE->success);
  EXPECT_NE(EDIT_UNKNOWN_RESPONSE->message.find("ot found"), std::string::npos)
    << "message was: " << EDIT_UNKNOWN_RESPONSE->message;

  // Candidate makes the object active.
  publishCandidate(OBJECT_ID, 3);
  ASSERT_TRUE(waitUntilObject(OBJECT_ID));

  // Blocking removes it from the service view.
  auto block_request        = std::make_shared<BlockObject::Request>();
  block_request->object_id  = OBJECT_ID;
  const auto BLOCK_RESPONSE = callService(m_block_client, block_request);
  ASSERT_NE(BLOCK_RESPONSE, nullptr);
  EXPECT_TRUE(BLOCK_RESPONSE->success);
  EXPECT_FALSE(hasObject(OBJECT_ID));

  // Editing a blocked object fails; node reports unknown/blocked accurately.
  auto edit_blocked_request =
    std::make_shared<EditObject::Request>(makeEditRequest(OBJECT_ID, {0}));
  const auto EDIT_BLOCKED_RESPONSE = callService(m_edit_client, edit_blocked_request);
  ASSERT_NE(EDIT_BLOCKED_RESPONSE, nullptr);
  EXPECT_FALSE(EDIT_BLOCKED_RESPONSE->success);
  EXPECT_NE(EDIT_BLOCKED_RESPONSE->message.find("ot found"), std::string::npos)
    << "message was: " << EDIT_BLOCKED_RESPONSE->message;

  // Unblock succeeds for a blocked ID.
  auto unblock_request        = std::make_shared<UnblockObject::Request>();
  unblock_request->object_id  = OBJECT_ID;
  const auto UNBLOCK_RESPONSE = callService(m_unblock_client, unblock_request);
  ASSERT_NE(UNBLOCK_RESPONSE, nullptr);
  EXPECT_TRUE(UNBLOCK_RESPONSE->success);

  // A second unblock of the now unblocked ID reports failure.
  const auto UNBLOCK_AGAIN_RESPONSE = callService(m_unblock_client, unblock_request);
  ASSERT_NE(UNBLOCK_AGAIN_RESPONSE, nullptr);
  EXPECT_FALSE(UNBLOCK_AGAIN_RESPONSE->success);

  // A new candidate restores the object.
  publishCandidate(OBJECT_ID, 1);
  ASSERT_TRUE(waitUntilObject(OBJECT_ID));

  // Editing with an invalid crop index is rejected with an accurate message.
  auto edit_invalid_request =
    std::make_shared<EditObject::Request>(makeEditRequest(OBJECT_ID, {99}));
  const auto EDIT_INVALID_RESPONSE = callService(m_edit_client, edit_invalid_request);
  ASSERT_NE(EDIT_INVALID_RESPONSE, nullptr);
  EXPECT_FALSE(EDIT_INVALID_RESPONSE->success);
  EXPECT_NE(EDIT_INVALID_RESPONSE->message.find("ndex"), std::string::npos)
    << "message was: " << EDIT_INVALID_RESPONSE->message;
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
