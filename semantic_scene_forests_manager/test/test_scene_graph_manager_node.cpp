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
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <semantic_scene_forests_msgs/msg/scene_graph.hpp>
#include <semantic_scene_forests_msgs/srv/add_scene_context.hpp>
#include <semantic_scene_forests_object_msgs/srv/block_object.hpp>
#include <semantic_scene_forests_object_msgs/srv/get_object_visuals.hpp>
#include <semantic_scene_forests_object_msgs/srv/get_objects.hpp>

#include "scene_graph_manager_node.hpp"

namespace {

using namespace std::chrono_literals;
using semantic_scene_forests_manager::SceneGraphManagerNode;
using semantic_scene_forests_manager::TrackUpdate;
using semantic_scene_forests_manager::Tx;
using SceneGraphMsg    = semantic_scene_forests_msgs::msg::SceneGraph;
using AddSceneContext  = semantic_scene_forests_msgs::srv::AddSceneContext;
using BlockObject      = semantic_scene_forests_object_msgs::srv::BlockObject;
using GetObjects       = semantic_scene_forests_object_msgs::srv::GetObjects;
using GetObjectVisuals = semantic_scene_forests_object_msgs::srv::GetObjectVisuals;

class SceneGraphManagerNodeTest : public ::testing::Test
{
protected:
  void TearDown() override
  {
    // The spin thread is declared last, so its destructor joins before the nodes go away.
    m_executor->cancel();
  }

  void start(const std::vector<rclcpp::Parameter>& parameters = {})
  {
    m_node = std::make_shared<SceneGraphManagerNode>(
      rclcpp::NodeOptions().parameter_overrides(parameters));
    m_client   = std::make_shared<rclcpp::Node>("manager_test_client");
    m_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    m_executor->add_node(m_node);
    m_executor->add_node(m_client);
    m_spin_thread = std::jthread([this] { m_executor->spin(); });
  }

  int64_t addObject(const std::string& class_name, size_t crop_count = 0)
  {
    return m_node->database().write([&](Tx& tx) {
      TrackUpdate update;
      update.id         = tx.allocateObjectId();
      update.class_name = class_name;
      update.last_seen  = rclcpp::Time(42, 7);
      update.crops      = std::vector<sensor_msgs::msg::Image>(crop_count);
      for (auto& crop : *update.crops)
      {
        crop.encoding = "mono8";
        crop.width = crop.height = crop.step = 1;
        crop.data                            = {7};
      }
      tx.upsertTrack(update, rclcpp::Time(50, 0));
      return update.id;
    });
  }

  template <class TSrv>
  typename TSrv::Response::SharedPtr call(const std::string& name,
                                          const typename TSrv::Request& request)
  {
    auto client = m_client->create_client<TSrv>(name);
    EXPECT_TRUE(client->wait_for_service(5s));
    auto future = client->async_send_request(std::make_shared<typename TSrv::Request>(request));
    EXPECT_EQ(future.wait_for(5s), std::future_status::ready);
    return future.get();
  }

  //! Waits until a received scene_graph satisfies pred.
  bool waitForGraph(const std::function<bool(const SceneGraphMsg&)>& pred)
  {
    // Shared so a callback still running after we return does not touch a dead frame.
    struct State
    {
      std::mutex mutex;
      std::condition_variable cv;
      bool matched{false};
    };
    auto state = std::make_shared<State>();
    auto sub =
      m_client->create_subscription<SceneGraphMsg>("scene_graph",
                                                   rclcpp::QoS(1).reliable().transient_local(),
                                                   [state, pred](const SceneGraphMsg& msg) {
                                                     const std::scoped_lock LOCK{state->mutex};
                                                     state->matched = state->matched || pred(msg);
                                                     state->cv.notify_all();
                                                   });
    std::unique_lock lock{state->mutex};
    return state->cv.wait_for(lock, 5s, [&] { return state->matched; });
  }

  bool conceptExists(int64_t id)
  {
    return m_node->database().read(
      [id](const Tx& tx) { return tx.visibleGraph().findNode(id).has_value(); });
  }

  std::shared_ptr<SceneGraphManagerNode> m_node;
  std::shared_ptr<rclcpp::Node> m_client;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> m_executor;
  std::jthread m_spin_thread;
};

TEST_F(SceneGraphManagerNodeTest, GetObjectsReturnsVisibleObjects)
{
  start();
  const int64_t CUP   = addObject("cup");
  const int64_t PLATE = addObject("plate");
  m_node->database().write([&](Tx& tx) { tx.block(PLATE); });

  const auto RESPONSE = call<GetObjects>("get_objects", GetObjects::Request{});

  ASSERT_EQ(RESPONSE->objects.size(), 1U);
  EXPECT_EQ(RESPONSE->objects[0].id, CUP);
  EXPECT_EQ(RESPONSE->objects[0].class_name, "cup");
  EXPECT_FALSE(RESPONSE->objects[0].is_blocked);
  EXPECT_EQ(RESPONSE->objects[0].header.frame_id, "map");
}

TEST_F(SceneGraphManagerNodeTest, BlockServiceHidesObjectFromSceneGraph)
{
  start();
  const int64_t CUP = addObject("cup");
  ASSERT_TRUE(waitForGraph([CUP](const SceneGraphMsg& msg) {
    return msg.header.frame_id == "map" && msg.object_nodes.size() == 1 &&
           msg.object_nodes[0].id == CUP && msg.object_nodes[0].last_seen.sec == 42;
  }));

  BlockObject::Request request;
  request.object_id   = CUP;
  const auto RESPONSE = call<BlockObject>("block_object", request);
  EXPECT_TRUE(RESPONSE->success);

  EXPECT_TRUE(waitForGraph([](const SceneGraphMsg& msg) { return msg.object_nodes.empty(); }));
}

TEST_F(SceneGraphManagerNodeTest, GetObjectVisualsReturnsCrops)
{
  start();
  const int64_t CUP = addObject("cup", 2);
  GetObjectVisuals::Request request;
  request.object_ids = {CUP, CUP + 100};

  const auto RESPONSE = call<GetObjectVisuals>("get_object_visuals", request);

  ASSERT_EQ(RESPONSE->visuals.size(), 1U);
  EXPECT_EQ(RESPONSE->visuals[0].object_id, CUP);
  EXPECT_EQ(RESPONSE->visuals[0].image_crops.size(), 2U);
}

TEST_F(SceneGraphManagerNodeTest, AddSceneContextCreatesConcept)
{
  start();
  const int64_t CUP = addObject("cup");
  AddSceneContext::Request request;
  request.node_id = CUP;
  request.context = "on the kitchen table";

  const auto RESPONSE = call<AddSceneContext>("add_scene_context", request);

  ASSERT_TRUE(RESPONSE->success);
  EXPECT_TRUE(RESPONSE->created);
  EXPECT_TRUE(conceptExists(RESPONSE->context_node_id));

  request.node_id = CUP + 100;
  EXPECT_FALSE(call<AddSceneContext>("add_scene_context", request)->success);
}

TEST_F(SceneGraphManagerNodeTest, PruningWorkerRemovesEdgelessConcept)
{
  start({rclcpp::Parameter("workers", std::vector<std::string>{"pruning"}),
         rclcpp::Parameter("worker_types.pruning", "semantic_scene_forests_manager::PruningWorker"),
         rclcpp::Parameter("pruning.interval_seconds", 0.05)});
  const int64_t CONCEPT =
    m_node->database().write([](Tx& tx) { return tx.findOrCreateConcept("red"); });

  const auto DEADLINE = std::chrono::steady_clock::now() + 5s;
  while (conceptExists(CONCEPT) && std::chrono::steady_clock::now() < DEADLINE)
  {
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_FALSE(conceptExists(CONCEPT));
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
