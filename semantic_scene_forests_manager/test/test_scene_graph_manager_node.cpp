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
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <semantic_scene_forests_msgs/srv/add_concept.hpp>
#include <semantic_scene_forests_msgs/srv/add_edge.hpp>
#include <semantic_scene_forests_msgs/srv/get_concept.hpp>
#include <semantic_scene_forests_object_msgs/srv/delete_object.hpp>
#include <semantic_scene_forests_object_msgs/srv/edit_object.hpp>
#include <semantic_scene_forests_object_msgs/srv/get_object_visuals.hpp>
#include <semantic_scene_forests_object_msgs/srv/get_objects.hpp>

#include "scene_graph_manager_node.hpp"

namespace {

using namespace std::chrono_literals;
using fzi::semantic_scene_forests::ObjectNode;
using fzi::semantic_scene_forests::SceneGraph;
using semantic_scene_forests_manager::SceneGraphManagerNode;
using AddConcept       = semantic_scene_forests_msgs::srv::AddConcept;
using GetConcept       = semantic_scene_forests_msgs::srv::GetConcept;
using AddEdge          = semantic_scene_forests_msgs::srv::AddEdge;
using DeleteObject     = semantic_scene_forests_object_msgs::srv::DeleteObject;
using EditObject       = semantic_scene_forests_object_msgs::srv::EditObject;
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

  //! Adds an object with `crop_count` JPEG crops of a 4x4 grey image.
  int64_t addObject(const std::string& class_name, size_t crop_count = 0)
  {
    ObjectNode node;
    node.id            = m_node->graph()->allocateId();
    node.class_name    = class_name;
    node.frame_id      = "map";
    node.last_seen.sec = 42;
    const cv_bridge::CvImage IMAGE{
      std_msgs::msg::Header{}, "bgr8", cv::Mat(4, 4, CV_8UC3, cv::Scalar(90, 90, 90))};
    for (size_t i = 0; i < crop_count; ++i)
    {
      node.crops.push_back(std::make_shared<sensor_msgs::msg::CompressedImage>(
        *IMAGE.toCompressedImageMsg(cv_bridge::JPG)));
    }
    const int64_t ID = node.id;
    EXPECT_TRUE(m_node->graph()->addObject(std::move(node)).has_value());
    return ID;
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

  bool addConcept(int64_t source,
                  const std::string& relation,
                  const std::string& label,
                  AddConcept::Response& out)
  {
    AddConcept::Request request;
    request.source_id = source;
    request.relation  = relation;
    request.label     = label;
    out               = *call<AddConcept>("add_concept", request);
    return out.success;
  }

  // NOLINTNEXTLINE(bugprone-easily-swappable-parameters): source/target ids, mirrors the service
  // request
  bool addEdge(int64_t source, int64_t target, const std::string& label)
  {
    AddEdge::Request request;
    request.source_id = source;
    request.target_id = target;
    request.label     = label;
    return call<AddEdge>("add_edge", request)->success;
  }

  std::shared_ptr<SceneGraphManagerNode> m_node;
  std::shared_ptr<rclcpp::Node> m_client;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> m_executor;
  std::jthread m_spin_thread;
};

TEST_F(SceneGraphManagerNodeTest, GetObjectsReturnsObjectNodes)
{
  start();
  const int64_t CUP = addObject("cup");

  const auto RESPONSE = call<GetObjects>("get_objects", GetObjects::Request{});

  ASSERT_EQ(RESPONSE->objects.size(), 1U);
  EXPECT_EQ(RESPONSE->objects[0].id, CUP);
  EXPECT_EQ(RESPONSE->objects[0].class_name, "cup");
  EXPECT_EQ(RESPONSE->objects[0].header.frame_id, "map");
  EXPECT_EQ(RESPONSE->objects[0].last_seen.sec, 42);
}

TEST_F(SceneGraphManagerNodeTest, DeleteObjectRemovesObjectAndItsEdges)
{
  start();
  const int64_t CUP = addObject("cup");
  AddConcept::Response concept_response;
  ASSERT_TRUE(addConcept(CUP, "has_property", "red", concept_response));

  DeleteObject::Request request;
  request.object_id = CUP;
  EXPECT_TRUE(call<DeleteObject>("delete_object", request)->success);
  EXPECT_FALSE(m_node->graph()->hasNode(CUP));
  EXPECT_FALSE(call<DeleteObject>("delete_object", request)->success);

  // A concept id is not an object: refused, concept stays.
  request.object_id = concept_response.concept_id;
  EXPECT_FALSE(call<DeleteObject>("delete_object", request)->success);
  EXPECT_TRUE(m_node->graph()->hasNode(concept_response.concept_id));
}

TEST_F(SceneGraphManagerNodeTest, EditObjectChangesClass)
{
  start();
  const int64_t CUP = addObject("cup");
  EditObject::Request request;
  request.object_id  = CUP;
  request.class_name = "mug";

  EXPECT_TRUE(call<EditObject>("edit_object", request)->success);
  const auto NODE = m_node->graph()->findNode(CUP);
  if (!NODE)
  {
    FAIL() << "object node missing";
  }
  EXPECT_EQ(std::get<ObjectNode>(*NODE).class_name, "mug");

  request.object_id = CUP + 100;
  EXPECT_FALSE(call<EditObject>("edit_object", request)->success);
}

TEST_F(SceneGraphManagerNodeTest, AddConceptNewExistingInvalidAndMissingSource)
{
  start();
  const int64_t CUP   = addObject("cup");
  const int64_t PLATE = addObject("plate");
  AddConcept::Response response;

  ASSERT_TRUE(addConcept(CUP, "on", "kitchen table", response));
  const int64_t CONCEPT = response.concept_id;
  EXPECT_TRUE(m_node->graph()->hasEdge(CUP, CONCEPT, "on"));

  // An existing label is an error: add_concept only creates; link with add_edge instead.
  EXPECT_FALSE(addConcept(PLATE, "on", "kitchen table", response));
  EXPECT_FALSE(m_node->graph()->hasEdge(PLATE, CONCEPT, "on"));
  GetConcept::Request lookup;
  lookup.label     = " kitchen table ";
  const auto FOUND = call<GetConcept>("get_concept", lookup);
  ASSERT_TRUE(FOUND->found);
  EXPECT_EQ(FOUND->concept_id, CONCEPT);
  lookup.label = "garage";
  EXPECT_FALSE(call<GetConcept>("get_concept", lookup)->found);
  EXPECT_TRUE(addEdge(PLATE, CONCEPT, "on"));

  const size_t CONCEPTS = m_node->graph()->getConceptNodes().size();
  EXPECT_FALSE(addConcept(CUP, "on", "   ", response));
  EXPECT_FALSE(addConcept(CUP + 100, "on", "floor", response));
  // Neither failure leaves an orphan concept behind.
  EXPECT_EQ(m_node->graph()->getConceptNodes().size(), CONCEPTS);
}

TEST_F(SceneGraphManagerNodeTest, AddEdgeValidCycleAndIntoObject)
{
  start();
  const int64_t CUP = addObject("cup");
  AddConcept::Response a;
  AddConcept::Response b;
  ASSERT_TRUE(addConcept(CUP, "is_type_of", "mug", a));
  ASSERT_TRUE(addConcept(a.concept_id, "is_type_of", "container", b));
  const int64_t CONCEPT = a.concept_id;

  EXPECT_TRUE(addEdge(CUP, b.concept_id, "related_to"));
  EXPECT_FALSE(addEdge(b.concept_id, CONCEPT, "is_type_of")); // closes a cycle
  EXPECT_FALSE(addEdge(CONCEPT, CUP, "describes"));           // into an object
}

TEST_F(SceneGraphManagerNodeTest, GetObjectVisualsDecodesJpegCrops)
{
  start();
  const int64_t CUP = addObject("cup", 2);
  GetObjectVisuals::Request request;
  request.object_ids = {CUP, CUP + 100};

  const auto RESPONSE = call<GetObjectVisuals>("get_object_visuals", request);

  ASSERT_EQ(RESPONSE->visuals.size(), 1U);
  EXPECT_EQ(RESPONSE->visuals[0].object_id, CUP);
  ASSERT_EQ(RESPONSE->visuals[0].image_crops.size(), 2U);
  const auto& crop = RESPONSE->visuals[0].image_crops[0];
  EXPECT_EQ(crop.encoding, "bgr8");
  EXPECT_EQ(crop.width, 4U);
  EXPECT_EQ(crop.height, 4U);
  EXPECT_NEAR(crop.data[0], 90, 3); // JPEG is lossy
}

TEST_F(SceneGraphManagerNodeTest, PruningWorkerRemovesEdgelessConcept)
{
  start({rclcpp::Parameter("workers", std::vector<std::string>{"pruning"}),
         rclcpp::Parameter("worker_types.pruning", "semantic_scene_forests_manager::PruningWorker"),
         rclcpp::Parameter("pruning.interval_seconds", 0.05)});
  const auto CONCEPT = m_node->graph()->findOrAddConcept("red");
  ASSERT_TRUE(CONCEPT.has_value());

  const auto DEADLINE = std::chrono::steady_clock::now() + 5s;
  while (m_node->graph()->hasNode(CONCEPT->id) && std::chrono::steady_clock::now() < DEADLINE)
  {
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_FALSE(m_node->graph()->hasNode(CONCEPT->id));
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
