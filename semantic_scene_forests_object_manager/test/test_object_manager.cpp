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
#include "semantic_scene_forests_object_msgs/msg/object_candidate.hpp"
#include "semantic_scene_forests_object_msgs/srv/edit_object.hpp"
#include "semantic_scene_forests_object_msgs/srv/unblock_object.hpp"

namespace {

using namespace std::chrono_literals;

using semantic_scene_forests_object_manager::ObjectManager;
using semantic_scene_forests_object_manager::ObjectManagerNode;
using semantic_scene_forests_object_msgs::msg::ObjectCandidate;
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

TEST_F(ObjectManagerTest, BlockedObjectIsSuppressedUntilFreshCandidate)
{
  ASSERT_TRUE(m_manager.processObjectCandidate(makeCandidate(OBJECT_ID, "cup", 3), m_now));
  ASSERT_FALSE(m_manager.getManagedObjects("world").empty());

  ASSERT_TRUE(m_manager.blockObject(OBJECT_ID));

  // Blocking erases the payload from every accessor and counts the block.
  EXPECT_TRUE(m_manager.getManagedObjects("world").empty());
  EXPECT_TRUE(m_manager.buildMessage("world", m_now).objects.empty());
  EXPECT_TRUE(m_manager.getObjectVisuals({OBJECT_ID}).empty());
  EXPECT_EQ(m_manager.getBlockedObjectCount(), 1U);

  // A late candidate for the blocked id is denied, and so is an edit.
  EXPECT_FALSE(m_manager.processObjectCandidate(makeCandidate(OBJECT_ID, "cup", 3), m_now));
  EXPECT_FALSE(m_manager.editObject(makeEditRequest(OBJECT_ID, {0}), m_now));

  // Unblocking lifts the suppression but does not resurrect the old payload.
  ASSERT_TRUE(m_manager.unblockObject(OBJECT_ID));
  EXPECT_EQ(m_manager.getBlockedObjectCount(), 0U);
  EXPECT_TRUE(m_manager.getManagedObjects("world").empty());

  // A fresh candidate is accepted again.
  ASSERT_TRUE(m_manager.processObjectCandidate(makeCandidate(OBJECT_ID, "cup", 1), m_now));
  const auto OBJECTS = m_manager.getManagedObjects("world");
  ASSERT_EQ(OBJECTS.size(), 1U);
  EXPECT_EQ(OBJECTS[0].id, OBJECT_ID);
  EXPECT_EQ(OBJECTS[0].class_name, "cup");
}

TEST_F(ObjectManagerTest, EditObjectValidatesCropIndices)
{
  ASSERT_TRUE(m_manager.processObjectCandidate(makeCandidate(OBJECT_ID, "chair", 3), m_now));

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

TEST_F(ObjectManagerTest, BlockAndCandidateOverlapStaysConsistent)
{
  ObjectManager manager;
  ASSERT_TRUE(manager.processObjectCandidate(makeCandidate(OBJECT_ID, "cup", 3), m_now));

  // A real block/candidate race under the manager mutex: whichever op wins,
  // the object must end up suppressed and absent.
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

} // namespace

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int RESULT = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return RESULT;
}
