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
#include <semantic_scene_forests/scene_graph.hpp>

using namespace fzi::semantic_scene_forests;

TEST(SceneGraphTest, AddObjectNode)
{
  SceneGraph graph;
  auto detection = std::make_shared<vision_msgs::msg::Detection3D>();
  std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals{};

  auto result = graph.addObjectNode("cup", detection, visuals);
  ASSERT_TRUE(result.has_value());
  EXPECT_GT(result.value(), 0);
  EXPECT_TRUE(graph.hasNode(result.value()));

  auto node = graph.findNode(result.value());
  ASSERT_TRUE(node.has_value());
  EXPECT_TRUE(std::holds_alternative<ObjectNode>(node.value()));
}

TEST(SceneGraphTest, AddObjectNodeWithId)
{
  SceneGraph graph;
  auto detection = std::make_shared<vision_msgs::msg::Detection3D>();
  std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals{};

  auto result = graph.addObjectNodeWithId(10, "cup", detection, visuals);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), 10);
  EXPECT_TRUE(graph.hasNode(10));
}

TEST(SceneGraphTest, UpdateObjectNodeExistingIdIsNoOp)
{
  SceneGraph graph;
  auto detection             = std::make_shared<vision_msgs::msg::Detection3D>();
  detection->header.frame_id = "camera";
  std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals{};
  visuals[0] = std::make_shared<sensor_msgs::msg::Image>();

  auto obj = graph.addObjectNodeWithId(10, "cup", detection, visuals).value();

  auto msg = std::make_unique<semantic_scene_forests_msgs::msg::SceneGraph>();
  semantic_scene_forests_msgs::msg::ObjectNode object_msg;
  object_msg.id = obj;
  msg->object_nodes.push_back(object_msg);

  auto result = graph.updateSceneGraphFromMsg(*msg);
  ASSERT_TRUE(result.has_value());

  auto node = graph.findNode(obj);
  ASSERT_TRUE(node.has_value());
  ASSERT_TRUE(std::holds_alternative<ObjectNode>(node.value()));

  const auto& object_node = std::get<ObjectNode>(node.value());
  EXPECT_EQ(object_node.class_name, "cup");
  ASSERT_NE(object_node.detection, nullptr);
  EXPECT_EQ(object_node.detection->header.frame_id, "camera");
  EXPECT_NE(object_node.visuals[0], nullptr);
}

TEST(SceneGraphTest, AddConceptNode)
{
  SceneGraph graph;

  auto result = graph.addConceptNode("on");
  ASSERT_TRUE(result.has_value());
  EXPECT_LT(result.value(), 0);
  EXPECT_TRUE(graph.hasNode(result.value()));

  auto node = graph.findNode(result.value());
  ASSERT_TRUE(node.has_value());
  EXPECT_TRUE(std::holds_alternative<ConceptNode>(node.value()));
  const auto& concept_node = std::get<ConceptNode>(node.value());
  EXPECT_EQ(concept_node.concept_name, "on");
}

TEST(SceneGraphTest, AddConceptNodeWithId)
{
  SceneGraph graph;

  auto result = graph.addConceptNodeWithId(-20, "above");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), -20);
  EXPECT_TRUE(graph.hasNode(-20));
}

TEST(SceneGraphTest, AddMultipleNodesSeparateIdSpaces)
{
  SceneGraph graph;
  auto detection = std::make_shared<vision_msgs::msg::Detection3D>();
  std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals{};

  auto obj0     = graph.addObjectNode("a", detection, visuals).value();
  auto concept0 = graph.addConceptNode("b").value();
  auto obj1     = graph.addObjectNode("c", detection, visuals).value();
  auto concept1 = graph.addConceptNode("d").value();

  EXPECT_EQ(obj0, 1);
  EXPECT_EQ(obj1, 2);
  EXPECT_EQ(concept0, -1);
  EXPECT_EQ(concept1, -2);

  auto nodes = graph.getNodes();
  EXPECT_EQ(nodes.size(), 4U);
}

TEST(SceneGraphTest, RemoveNode)
{
  SceneGraph graph;
  auto detection = std::make_shared<vision_msgs::msg::Detection3D>();
  std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals{};

  auto id = graph.addObjectNode("cup", detection, visuals).value();

  auto removed = graph.removeNode(id);
  ASSERT_TRUE(removed.has_value());
  EXPECT_FALSE(graph.hasNode(id));
  EXPECT_TRUE(std::holds_alternative<ObjectNode>(removed.value()));
}

TEST(SceneGraphTest, RemoveNodeNonExistent)
{
  SceneGraph graph;

  auto result = graph.removeNode(42);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().kind, SceneGraphErrorKind::NODE_DOES_NOT_EXIST);
}

TEST(SceneGraphTest, AddEdge)
{
  SceneGraph graph;
  auto detection = std::make_shared<vision_msgs::msg::Detection3D>();
  std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals{};

  auto obj_id     = graph.addObjectNode("cup", detection, visuals).value();
  auto concept_id = graph.addConceptNode("on").value();

  auto edge = graph.addEdge(obj_id, concept_id, "relation");
  ASSERT_TRUE(edge.has_value());
  EXPECT_EQ(edge.value().label, "relation");

  auto edges = graph.findEdges(obj_id, concept_id);
  ASSERT_TRUE(edges.has_value());
  EXPECT_EQ(edges.value().size(), 1U);
  EXPECT_EQ(edges.value()[0].label, "relation");
}

TEST(SceneGraphTest, AddEdgeInvalidTarget)
{
  SceneGraph graph;
  auto detection = std::make_shared<vision_msgs::msg::Detection3D>();
  std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals{};

  auto obj1 = graph.addObjectNode("cup", detection, visuals).value();
  auto obj2 = graph.addObjectNode("plate", detection, visuals).value();

  auto result = graph.addEdge(obj1, obj2, "on");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().kind, SceneGraphErrorKind::INVALID_EDGE);
}

TEST(SceneGraphTest, AddEdgeCycleDetection)
{
  SceneGraph graph;

  auto a = graph.addConceptNode("A").value();
  auto b = graph.addConceptNode("B").value();
  auto c = graph.addConceptNode("C").value();

  graph.addEdge(a, b, "ab").value();
  graph.addEdge(b, c, "bc").value();

  auto result = graph.addEdge(c, a, "ca");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().kind, SceneGraphErrorKind::CYCLE);
}

TEST(SceneGraphTest, RemoveEdge)
{
  SceneGraph graph;
  auto detection = std::make_shared<vision_msgs::msg::Detection3D>();
  std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals{};

  auto obj_id     = graph.addObjectNode("cup", detection, visuals).value();
  auto concept_id = graph.addConceptNode("on").value();
  graph.addEdge(obj_id, concept_id, "rel").value();

  auto removed = graph.removeEdge(obj_id, concept_id, "rel");
  ASSERT_TRUE(removed.has_value());

  auto edges = graph.findEdges(obj_id, concept_id);
  ASSERT_TRUE(edges.has_value());
  EXPECT_TRUE(edges.value().empty());
}

TEST(SceneGraphTest, RemoveIsolatedNodes)
{
  SceneGraph graph;
  auto detection = std::make_shared<vision_msgs::msg::Detection3D>();
  std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals{};

  auto obj        = graph.addObjectNode("cup", detection, visuals).value();
  auto concept_id = graph.addConceptNode("on").value();
  auto isolated   = graph.addConceptNode("lonely").value();

  graph.addEdge(obj, concept_id, "rel").value();

  auto removed = graph.removeIsolatedNodes();
  ASSERT_TRUE(removed.has_value());
  EXPECT_EQ(removed.value().size(), 1U);

  EXPECT_TRUE(graph.hasNode(obj));
  EXPECT_TRUE(graph.hasNode(concept_id));
  EXPECT_FALSE(graph.hasNode(isolated));
}

TEST(SceneGraphTest, OutgoingEdges)
{
  SceneGraph graph;
  auto detection = std::make_shared<vision_msgs::msg::Detection3D>();
  std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals{};

  auto obj      = graph.addObjectNode("cup", detection, visuals).value();
  auto concept1 = graph.addConceptNode("on").value();
  auto concept2 = graph.addConceptNode("above").value();

  graph.addEdge(obj, concept1, "rel1").value();
  graph.addEdge(obj, concept2, "rel2").value();

  auto outgoing = graph.outgoingEdges(obj);
  ASSERT_TRUE(outgoing.has_value());
  EXPECT_EQ(outgoing.value().size(), 2U);
}

TEST(SceneGraphTest, IncomingEdges)
{
  SceneGraph graph;
  auto detection = std::make_shared<vision_msgs::msg::Detection3D>();
  std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals{};

  auto obj1       = graph.addObjectNode("cup", detection, visuals).value();
  auto obj2       = graph.addObjectNode("plate", detection, visuals).value();
  auto concept_id = graph.addConceptNode("on").value();

  graph.addEdge(obj1, concept_id, "rel1").value();
  graph.addEdge(obj2, concept_id, "rel2").value();

  auto incoming = graph.incomingEdges(concept_id);
  ASSERT_TRUE(incoming.has_value());
  EXPECT_EQ(incoming.value().size(), 2U);
}

TEST(SceneGraphTest, FindConceptNodesByName)
{
  SceneGraph graph;

  graph.addConceptNode("on").value();
  graph.addConceptNode("above").value();
  graph.addConceptNode("on").value();

  auto found = graph.findConceptNodesByName("on");
  EXPECT_EQ(found.size(), 2U);
}

TEST(SceneGraphTest, FindEdgesByLabel)
{
  SceneGraph graph;
  auto detection = std::make_shared<vision_msgs::msg::Detection3D>();
  std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals{};

  auto obj1       = graph.addObjectNode("cup", detection, visuals).value();
  auto obj2       = graph.addObjectNode("plate", detection, visuals).value();
  auto concept_id = graph.addConceptNode("on").value();

  graph.addEdge(obj1, concept_id, "same").value();
  graph.addEdge(obj2, concept_id, "same").value();
  graph.addEdge(obj1, concept_id, "different").value();

  auto found = graph.findEdgesByLabel("same");
  EXPECT_EQ(found.size(), 2U);
}

TEST(SceneGraphTest, GetObjectNodes)
{
  SceneGraph graph;
  auto detection = std::make_shared<vision_msgs::msg::Detection3D>();
  std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals{};

  graph.addObjectNode("cup", detection, visuals).value();
  graph.addObjectNode("plate", detection, visuals).value();
  graph.addConceptNode("on").value();

  auto objects = graph.getObjectNodes();
  EXPECT_EQ(objects.size(), 2U);
}

TEST(SceneGraphTest, GetConceptNodes)
{
  SceneGraph graph;
  auto detection = std::make_shared<vision_msgs::msg::Detection3D>();
  std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals{};

  graph.addObjectNode("cup", detection, visuals).value();
  graph.addConceptNode("on").value();
  graph.addConceptNode("above").value();

  auto concepts = graph.getConceptNodes();
  EXPECT_EQ(concepts.size(), 2U);
}

TEST(SceneGraphTest, GetEdges)
{
  SceneGraph graph;
  auto detection = std::make_shared<vision_msgs::msg::Detection3D>();
  std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals{};

  auto obj        = graph.addObjectNode("cup", detection, visuals).value();
  auto concept_id = graph.addConceptNode("on").value();
  graph.addEdge(obj, concept_id, "rel").value();

  auto edges = graph.getEdges();
  EXPECT_EQ(edges.size(), 1U);
  EXPECT_EQ(std::get<0>(edges[0]), obj);
  EXPECT_EQ(std::get<1>(edges[0]), concept_id);
}

TEST(SceneGraphTest, ToMsg)
{
  SceneGraph graph;
  auto detection = std::make_shared<vision_msgs::msg::Detection3D>();
  std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals{};

  auto obj        = graph.addObjectNode("cup", detection, visuals).value();
  auto concept_id = graph.addConceptNode("on").value();
  graph.addEdge(obj, concept_id, "relation").value();

  auto msg = graph.toMsg();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->object_nodes.size(), 1U);
  EXPECT_EQ(msg->concept_nodes.size(), 1U);
  EXPECT_EQ(msg->edges.size(), 1U);
  EXPECT_EQ(msg->edges[0].source_id, obj);
  EXPECT_EQ(msg->edges[0].target_id, concept_id);
  EXPECT_EQ(msg->edges[0].label, "relation");
}

TEST(SceneGraphTest, FromMsg)
{
  auto msg = std::make_unique<semantic_scene_forests_msgs::msg::SceneGraph>();

  semantic_scene_forests_msgs::msg::ObjectNode obj_node;
  obj_node.id = 10;
  msg->object_nodes.push_back(obj_node);

  semantic_scene_forests_msgs::msg::ConceptNode concept_node;
  concept_node.id         = -20;
  concept_node.class_name = "on";
  msg->concept_nodes.push_back(concept_node);

  semantic_scene_forests_msgs::msg::Edge edge;
  edge.source_id = 10;
  edge.target_id = -20;
  edge.label     = "relation";
  msg->edges.push_back(edge);

  SceneGraph graph;
  auto result = SceneGraph::fromMsg(*msg);
  ASSERT_TRUE(result.has_value());

  auto& loaded_graph = result.value();
  EXPECT_TRUE(loaded_graph->hasNode(10));
  EXPECT_TRUE(loaded_graph->hasNode(-20));

  auto edges = loaded_graph->findEdges(10, -20);
  ASSERT_TRUE(edges.has_value());
  EXPECT_EQ(edges.value().size(), 1U);
}

TEST(SceneGraphTest, UpdateSceneGraphFromMsg)
{
  SceneGraph graph;
  auto detection = std::make_shared<vision_msgs::msg::Detection3D>();
  std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals{};

  auto obj1 = graph.addObjectNode("cup", detection, visuals).value();
  auto obj2 = graph.addObjectNode("plate", detection, visuals).value();

  auto msg = std::make_unique<semantic_scene_forests_msgs::msg::SceneGraph>();

  semantic_scene_forests_msgs::msg::ObjectNode obj_node;
  obj_node.id = obj1;
  msg->object_nodes.push_back(obj_node);

  semantic_scene_forests_msgs::msg::ConceptNode concept_node;
  concept_node.id         = -20;
  concept_node.class_name = "on";
  msg->concept_nodes.push_back(concept_node);

  semantic_scene_forests_msgs::msg::Edge edge;
  edge.source_id = obj1;
  edge.target_id = -20;
  edge.label     = "relation";
  msg->edges.push_back(edge);

  auto result = graph.updateSceneGraphFromMsg(*msg);
  ASSERT_TRUE(result.has_value());

  EXPECT_TRUE(graph.hasNode(obj1));
  EXPECT_FALSE(graph.hasNode(obj2));
  EXPECT_TRUE(graph.hasNode(-20));

  auto edges = graph.findEdges(obj1, -20);
  ASSERT_TRUE(edges.has_value());
  EXPECT_EQ(edges.value().size(), 1U);
}

TEST(SceneGraphTest, HasEdge)
{
  SceneGraph graph;
  auto detection = std::make_shared<vision_msgs::msg::Detection3D>();
  std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals{};

  auto obj        = graph.addObjectNode("cup", detection, visuals).value();
  auto concept_id = graph.addConceptNode("on").value();
  graph.addEdge(obj, concept_id, "relation").value();

  EXPECT_TRUE(graph.hasEdge(obj, concept_id, "relation"));
  EXPECT_FALSE(graph.hasEdge(obj, concept_id, "other"));
}

TEST(SceneGraphTest, AddObjectNodeWithIdUpdatesNextId)
{
  SceneGraph graph;
  auto detection = std::make_shared<vision_msgs::msg::Detection3D>();
  std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals{};

  graph.addObjectNodeWithId(30, "cup", detection, visuals).value();

  auto next_obj = graph.addObjectNode("plate", detection, visuals).value();
  EXPECT_EQ(next_obj, 31);
}

TEST(SceneGraphTest, AddConceptNodeWithIdUpdatesNextId)
{
  SceneGraph graph;

  graph.addConceptNodeWithId(-50, "custom").value();

  auto next_concept = graph.addConceptNode("auto").value();
  EXPECT_EQ(next_concept, -51);
}

TEST(SceneGraphTest, ClonePreservesMetadataAndIsIndependent)
{
  SceneGraph graph;
  auto detection = std::make_shared<vision_msgs::msg::Detection3D>();
  detection->id  = "original";
  std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals{};
  visuals[0]                  = std::make_shared<sensor_msgs::msg::Image>();
  visuals[0]->header.frame_id = "camera";

  const auto object_id  = graph.addObjectNode("cup", detection, visuals).value();
  const auto concept_id = graph.addConceptNode("red").value();
  ASSERT_TRUE(graph.addEdge(object_id, concept_id, "has_color").has_value());

  auto clone_result = graph.clone();
  ASSERT_TRUE(clone_result.has_value());
  auto& clone = *clone_result.value();

  const auto cloned_object = clone.findNode(object_id);
  ASSERT_TRUE(cloned_object.has_value());
  const auto& object = std::get<ObjectNode>(*cloned_object);
  EXPECT_EQ(object.class_name, "cup");
  ASSERT_NE(object.detection, nullptr);
  EXPECT_EQ(object.detection->id, "original");
  EXPECT_NE(object.detection, detection);
  ASSERT_NE(object.visuals[0], nullptr);
  EXPECT_EQ(object.visuals[0]->header.frame_id, "camera");
  EXPECT_NE(object.visuals[0], visuals[0]);

  ASSERT_TRUE(clone.addConceptNode("blue").has_value());
  EXPECT_EQ(graph.getConceptNodes().size(), 1U);
  EXPECT_EQ(clone.getConceptNodes().size(), 2U);
}

TEST(SceneGraphTest, SwapKeepsIdMappingsValid)
{
  SceneGraph first;
  SceneGraph second;
  const auto first_id  = first.addConceptNodeWithId(-10, "first").value();
  const auto second_id = second.addConceptNodeWithId(-20, "second").value();

  first.swap(second);

  EXPECT_FALSE(first.hasNode(first_id));
  EXPECT_TRUE(first.hasNode(second_id));
  EXPECT_TRUE(second.hasNode(first_id));
  ASSERT_TRUE(first.addConceptNode("third").has_value());
  ASSERT_TRUE(second.addConceptNode("fourth").has_value());
}
