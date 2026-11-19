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
#include <future>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <semantic_scene_forests/scene_graph.hpp>
#include <semantic_scene_forests_msgs/action/run_worker.hpp>

#include "object_attribution_worker.hpp"

namespace {

using namespace std::chrono_literals;
using semantic_scene_forests_generation::ObjectAttributionWorker;
using RunWorker = semantic_scene_forests_msgs::action::RunWorker;

TEST(ObjectAttributionWorkerTest, LoadsWithNoTasksAndServesAction)
{
  auto node =
    std::make_shared<rclcpp::Node>("object_attribution_test",
                                   rclcpp::NodeOptions().parameter_overrides({
                                     {"object_attribution.server_url", "http://127.0.0.1:9"},
                                     {"object_attribution.tasks", std::vector<std::string>{}},
                                   }));
  auto graph = std::make_shared<fzi::semantic_scene_forests::SceneGraph>();
  diagnostic_updater::Updater diagnostics(node);
  ObjectAttributionWorker worker;
  worker.initialize(*node, graph, "object_attribution", diagnostics);
  worker.start();

  auto client = rclcpp_action::create_client<RunWorker>(node, "attribute_objects");
  EXPECT_TRUE(client->wait_for_action_server(5s));

  auto stopped = std::async(std::launch::async, [&worker] { worker.stop(); });
  EXPECT_EQ(stopped.wait_for(2s), std::future_status::ready);
}

TEST(ObjectAttributionWorkerTest, TechnicalFailureAbortsAndLeavesInputsPending)
{
  auto node =
    std::make_shared<rclcpp::Node>("attribution_failure_test",
                                   rclcpp::NodeOptions().parameter_overrides({
                                     {"object_attribution.server_url", "http://127.0.0.1:9"},
                                     {"object_attribution.tasks", std::vector<std::string>{}},
                                     {"object_attribution.poll_interval_seconds", 60.0},
                                   }));
  auto graph = std::make_shared<fzi::semantic_scene_forests::SceneGraph>();
  ASSERT_TRUE(graph->addObject({.id = graph->allocateId(), .class_name = "cup"}));
  ASSERT_TRUE(graph->addObject({.id = graph->allocateId(), .class_name = "bottle"}));
  diagnostic_updater::Updater diagnostics(node);
  ObjectAttributionWorker worker;
  worker.initialize(*node, graph, "object_attribution", diagnostics);
  worker.start();

  auto client = rclcpp_action::create_client<RunWorker>(node, "attribute_objects");
  ASSERT_TRUE(client->wait_for_action_server(5s));
  auto accepted = client->async_send_goal(RunWorker::Goal{});
  ASSERT_EQ(rclcpp::spin_until_future_complete(node, accepted, 5s),
            rclcpp::FutureReturnCode::SUCCESS);
  const auto& goal = accepted.get();
  ASSERT_NE(goal, nullptr);
  auto finished = client->async_get_result(goal);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node, finished, 5s),
            rclcpp::FutureReturnCode::SUCCESS);
  const auto& result = finished.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_FALSE(result.result->success);
  EXPECT_NE(result.result->message.find("setup request failed"), std::string::npos);
  EXPECT_EQ(graph->unprocessedBy("object_attribution").size(), 2U);
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
