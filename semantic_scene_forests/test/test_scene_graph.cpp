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
#include <stdexcept>

using namespace fzi::semantic_scene_forests;
using std::chrono::seconds;

namespace {

int64_t addObject(SceneGraph& graph, const std::string& class_name)
{
  const auto ID = graph.allocateId();
  EXPECT_TRUE(graph.addObject({.id = ID, .class_name = class_name}).has_value());
  return ID;
}

builtin_interfaces::msg::Time stamp(int32_t sec)
{
  builtin_interfaces::msg::Time time;
  time.sec = sec;
  return time;
}

CropPtr crop(int32_t sec)
{
  auto image          = std::make_shared<sensor_msgs::msg::CompressedImage>();
  image->header.stamp = stamp(sec);
  return image;
}

template <typename T> T nodeAs(const SceneGraph& graph, const int64_t id)
{
  const auto NODE = graph.findNode(id);
  if (!NODE)
  {
    throw std::runtime_error("missing node");
  }
  return std::get<T>(*NODE);
}

std::vector<int32_t> cropSeconds(const SceneGraph& graph, int64_t id)
{
  std::vector<int32_t> result;
  const auto OBJECT = nodeAs<ObjectNode>(graph, id);
  result.reserve(OBJECT.crops.size());
  for (const auto& image : OBJECT.crops)
  {
    result.push_back(image->header.stamp.sec);
  }
  return result;
}

} // namespace

TEST(SceneGraphTest, AddObject)
{
  SceneGraph graph;
  const auto ID = graph.allocateId();
  EXPECT_GT(ID, 0);

  ObjectNode object{.id = ID, .class_name = "cup", .frame_id = "map", .crops = {crop(1)}};
  object.bbox.center.position.x = 1.5;
  ASSERT_TRUE(graph.addObject(object).has_value());

  const auto STORED = nodeAs<ObjectNode>(graph, ID);
  EXPECT_EQ(STORED.class_name, "cup");
  EXPECT_EQ(STORED.frame_id, "map");
  EXPECT_DOUBLE_EQ(STORED.bbox.center.position.x, 1.5);
  EXPECT_EQ(STORED.crops.size(), 1U);
}

TEST(SceneGraphTest, AddObjectDuplicateIdFailsAndExplicitIdAdvancesCounter)
{
  SceneGraph graph;
  ASSERT_TRUE(graph.addObject({.id = 10, .class_name = "cup"}).has_value());

  const auto DUPLICATE = graph.addObject({.id = 10, .class_name = "plate"});
  ASSERT_FALSE(DUPLICATE.has_value());
  EXPECT_EQ(DUPLICATE.error().kind, SceneGraphErrorKind::NODE_ALREADY_EXISTS);
  EXPECT_EQ(graph.allocateId(), 11);
}

TEST(SceneGraphTest, AddConceptNode)
{
  SceneGraph graph;

  auto result = graph.addConceptNode("on");
  ASSERT_TRUE(result.has_value());
  EXPECT_GT(result.value(), 0);

  EXPECT_EQ(nodeAs<ConceptNode>(graph, result.value()).concept_name, "on");
}

TEST(SceneGraphTest, AddConceptNodeWithIdAdvancesSharedCounter)
{
  SceneGraph graph;

  ASSERT_EQ(graph.addConceptNodeWithId(42, "red").value(), 42);
  EXPECT_TRUE(graph.hasNode(42));
  EXPECT_EQ(graph.addConceptNode("auto").value(), 43);
}

TEST(SceneGraphTest, ObjectsAndConceptsShareOneIdCounter)
{
  SceneGraph graph;

  EXPECT_EQ(addObject(graph, "a"), 1);
  EXPECT_EQ(graph.addConceptNode("b").value(), 2);
  EXPECT_EQ(addObject(graph, "c"), 3);
  EXPECT_EQ(graph.findOrAddConcept("d").value().id, 4);
  EXPECT_EQ(graph.getNodes().size(), 4U);
}

TEST(SceneGraphTest, RemoveNode)
{
  SceneGraph graph;
  auto id = addObject(graph, "cup");

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
  auto obj_id     = addObject(graph, "cup");
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
  auto obj1 = addObject(graph, "cup");
  auto obj2 = addObject(graph, "plate");

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
  auto obj_id     = addObject(graph, "cup");
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
  auto obj        = addObject(graph, "cup");
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

TEST(SceneGraphTest, PruneIsolatedConceptsKeepsObjects)
{
  SceneGraph graph;
  const auto LONE_OBJECT  = addObject(graph, "cup");
  const auto LONE_CONCEPT = graph.addConceptNode("lonely").value();

  EXPECT_EQ(graph.pruneIsolatedConcepts(), std::vector<int64_t>{LONE_CONCEPT});
  EXPECT_TRUE(graph.hasNode(LONE_OBJECT));
}

TEST(SceneGraphTest, OutgoingEdges)
{
  SceneGraph graph;
  auto obj      = addObject(graph, "cup");
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
  auto obj1       = addObject(graph, "cup");
  auto obj2       = addObject(graph, "plate");
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
  auto obj1       = addObject(graph, "cup");
  auto obj2       = addObject(graph, "plate");
  auto concept_id = graph.addConceptNode("on").value();

  graph.addEdge(obj1, concept_id, "same").value();
  graph.addEdge(obj2, concept_id, "same").value();
  graph.addEdge(obj1, concept_id, "different").value();

  auto found = graph.findEdgesByLabel("same");
  EXPECT_EQ(found.size(), 2U);
}

TEST(SceneGraphTest, GetObjectAndConceptNodes)
{
  SceneGraph graph;
  addObject(graph, "cup");
  addObject(graph, "plate");
  graph.addConceptNode("on").value();

  EXPECT_EQ(graph.getObjectNodes().size(), 2U);
  EXPECT_EQ(graph.getConceptNodes().size(), 1U);
}

TEST(SceneGraphTest, GetEdges)
{
  SceneGraph graph;
  auto obj        = addObject(graph, "cup");
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
  auto obj        = addObject(graph, "cup");
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
  concept_node.id         = 20;
  concept_node.class_name = "on";
  msg->concept_nodes.push_back(concept_node);

  semantic_scene_forests_msgs::msg::Edge edge;
  edge.source_id = 10;
  edge.target_id = 20;
  edge.label     = "relation";
  msg->edges.push_back(edge);

  auto result = SceneGraph::fromMsg(*msg);
  ASSERT_TRUE(result.has_value());

  auto& loaded_graph = result.value();
  EXPECT_TRUE(loaded_graph->hasNode(10));
  EXPECT_TRUE(loaded_graph->hasNode(20));
  EXPECT_TRUE(loaded_graph->hasEdge(10, 20, "relation"));
  EXPECT_EQ(loaded_graph->allocateId(), 21);
}

TEST(SceneGraphTest, MessageRoundTripKeepsObjectFields)
{
  SceneGraph graph;
  ObjectNode object{.id           = 7,
                    .class_name   = "cup",
                    .frame_id     = "map",
                    .source       = ObjectNode::SOURCE_MANUAL,
                    .last_seen    = stamp(12),
                    .last_updated = stamp(11)};
  object.bbox.center.position.x = 1.5;
  object.bbox.size.z            = 0.25;
  ASSERT_TRUE(graph.addObject(object).has_value());
  const auto RED = graph.addConceptEdge(7, "has_color", "red").value().id;

  const auto MSG = graph.toMsg();
  ASSERT_EQ(MSG->object_nodes.size(), 1U);
  EXPECT_EQ(MSG->object_nodes[0].header.frame_id, "map");
  EXPECT_EQ(MSG->object_nodes[0].header.stamp.sec, 12);

  auto restored = SceneGraph::fromMsg(*MSG);
  ASSERT_TRUE(restored.has_value());
  const auto RESTORED_OBJECT = nodeAs<ObjectNode>(*restored.value(), 7);
  EXPECT_EQ(RESTORED_OBJECT.class_name, "cup");
  EXPECT_EQ(RESTORED_OBJECT.bbox, object.bbox);
  EXPECT_EQ(RESTORED_OBJECT.frame_id, "map");
  EXPECT_EQ(RESTORED_OBJECT.source, ObjectNode::SOURCE_MANUAL);
  EXPECT_EQ(RESTORED_OBJECT.last_seen, stamp(12));
  EXPECT_EQ(RESTORED_OBJECT.last_updated, stamp(11));
  EXPECT_TRUE(restored.value()->hasEdge(7, RED, "has_color"));
}

TEST(SceneGraphTest, UpdateSceneGraphFromMsg)
{
  SceneGraph graph;
  auto obj1 = addObject(graph, "cup");
  auto obj2 = addObject(graph, "plate");

  auto msg = std::make_unique<semantic_scene_forests_msgs::msg::SceneGraph>();

  semantic_scene_forests_msgs::msg::ObjectNode obj_node;
  obj_node.id = obj1;
  msg->object_nodes.push_back(obj_node);

  semantic_scene_forests_msgs::msg::ConceptNode concept_node;
  concept_node.id         = 20;
  concept_node.class_name = "on";
  msg->concept_nodes.push_back(concept_node);

  semantic_scene_forests_msgs::msg::Edge edge;
  edge.source_id = obj1;
  edge.target_id = 20;
  edge.label     = "relation";
  msg->edges.push_back(edge);

  auto result = graph.updateSceneGraphFromMsg(*msg);
  ASSERT_TRUE(result.has_value());

  EXPECT_TRUE(graph.hasNode(obj1));
  EXPECT_FALSE(graph.hasNode(obj2));
  EXPECT_TRUE(graph.hasNode(20));
  EXPECT_TRUE(graph.hasEdge(obj1, 20, "relation"));
}

TEST(SceneGraphTest, HasEdge)
{
  SceneGraph graph;
  auto obj        = addObject(graph, "cup");
  auto concept_id = graph.addConceptNode("on").value();
  graph.addEdge(obj, concept_id, "relation").value();

  EXPECT_TRUE(graph.hasEdge(obj, concept_id, "relation"));
  EXPECT_FALSE(graph.hasEdge(obj, concept_id, "other"));
}

TEST(SceneGraphTest, ClonePreservesNodeDataAndIsIndependent)
{
  SceneGraph graph;
  const auto CROP = crop(3);
  ASSERT_TRUE(graph.addObject({.id = 1, .class_name = "cup", .frame_id = "map", .crops = {CROP}})
                .has_value());
  ASSERT_TRUE(graph.markProcessed(1, "worker").has_value());
  const auto CONCEPT_ID = graph.addConceptEdge(1, "has_color", "red").value().id;

  auto clone_result = graph.clone();
  ASSERT_TRUE(clone_result.has_value());
  auto& clone = *clone_result.value();

  const auto OBJECT = nodeAs<ObjectNode>(clone, 1);
  EXPECT_EQ(OBJECT.class_name, "cup");
  EXPECT_EQ(OBJECT.frame_id, "map");
  ASSERT_EQ(OBJECT.crops.size(), 1U);
  EXPECT_EQ(OBJECT.crops[0], CROP); // crops are immutable and shared
  EXPECT_TRUE(OBJECT.workers.at("worker").processed);
  EXPECT_TRUE(clone.hasEdge(1, CONCEPT_ID, "has_color"));

  ASSERT_TRUE(clone.addConceptNode("blue").has_value());
  EXPECT_EQ(graph.getConceptNodes().size(), 1U);
  EXPECT_EQ(clone.getConceptNodes().size(), 2U);
}

TEST(SceneGraphTest, SwapKeepsIdMappingsValid)
{
  SceneGraph first;
  SceneGraph second;
  const auto FIRST_ID  = first.addConceptNodeWithId(10, "first").value();
  const auto SECOND_ID = second.addConceptNodeWithId(20, "second").value();

  first.swap(second);

  EXPECT_FALSE(first.hasNode(FIRST_ID));
  EXPECT_TRUE(first.hasNode(SECOND_ID));
  EXPECT_TRUE(second.hasNode(FIRST_ID));
  ASSERT_TRUE(first.addConceptNode("third").has_value());
  ASSERT_TRUE(second.addConceptNode("fourth").has_value());
}

// ---------------------------------------------------------------------------
// Atomic operations used by the tracker, the manager and the workers.
// ---------------------------------------------------------------------------

TEST(SceneGraphTest, UpdateObjectPoseMovesLastUpdatedOnlyOnChangeAndReportsUnknownId)
{
  SceneGraph graph;
  ASSERT_TRUE(
    graph.addObject({.id = 1, .last_seen = stamp(1), .last_updated = stamp(1)}).has_value());

  vision_msgs::msg::BoundingBox3D moved;
  moved.center.position.x = 2.0;
  ASSERT_TRUE(graph.updateObjectPose(1, moved, stamp(2)).has_value());
  ASSERT_TRUE(graph.updateObjectPose(1, moved, stamp(3)).has_value());

  const auto OBJECT = nodeAs<ObjectNode>(graph, 1);
  EXPECT_EQ(OBJECT.bbox, moved);
  EXPECT_EQ(OBJECT.last_seen, stamp(3));
  EXPECT_EQ(OBJECT.last_updated, stamp(2));

  ASSERT_TRUE(graph.removeNode(1).has_value());
  const auto UNKNOWN = graph.updateObjectPose(1, moved, stamp(4));
  ASSERT_FALSE(UNKNOWN.has_value());
  EXPECT_EQ(UNKNOWN.error().kind, SceneGraphErrorKind::NODE_DOES_NOT_EXIST);
}

TEST(SceneGraphTest, OfferCropAppendsThenReplacesOldestAfterInterval)
{
  SceneGraph graph;
  graph.setCropPolicy(3, seconds(10));
  ASSERT_TRUE(graph.addObject({.id = 1}).has_value());

  EXPECT_TRUE(graph.offerCrop(1, crop(0)).value());
  EXPECT_FALSE(graph.offerCrop(1, crop(5)).value()); // too soon after the newest
  EXPECT_TRUE(graph.offerCrop(1, crop(10)).value());
  EXPECT_TRUE(graph.offerCrop(1, crop(20)).value());
  // Full: 25 is 25 s after the oldest (0) but only 5 s after the newest (20).
  EXPECT_FALSE(graph.offerCrop(1, crop(25)).value());
  EXPECT_EQ(cropSeconds(graph, 1), (std::vector<int32_t>{0, 10, 20}));
  // Full: 30 is at least 10 s after both the oldest and the newest, so it replaces the oldest.
  EXPECT_TRUE(graph.offerCrop(1, crop(30)).value());
  EXPECT_EQ(cropSeconds(graph, 1), (std::vector<int32_t>{10, 20, 30}));

  const auto UNKNOWN = graph.offerCrop(99, crop(40));
  ASSERT_FALSE(UNKNOWN.has_value());
  EXPECT_EQ(UNKNOWN.error().kind, SceneGraphErrorKind::NODE_DOES_NOT_EXIST);
}

TEST(SceneGraphTest, OfferCropDefaultPolicyIsTenSeconds)
{
  SceneGraph graph;
  ASSERT_TRUE(graph.addObject({.id = 1, .crops = {crop(0)}}).has_value());
  EXPECT_FALSE(graph.offerCrop(1, crop(9)).value());
  EXPECT_TRUE(graph.offerCrop(1, crop(10)).value());
}

TEST(SceneGraphTest, SetObjectClass)
{
  SceneGraph graph;
  const auto ID = addObject(graph, "cup");
  ASSERT_TRUE(graph.setObjectClass(ID, "mug").has_value());
  EXPECT_EQ(nodeAs<ObjectNode>(graph, ID).class_name, "mug");
  EXPECT_FALSE(graph.setObjectClass(99, "mug").has_value());
}

TEST(SceneGraphTest, FindOrAddConceptLabelRules)
{
  SceneGraph graph;

  const auto CREATED = graph.findOrAddConcept(" \tred\t ");
  ASSERT_TRUE(CREATED.has_value());
  EXPECT_TRUE(CREATED->created);
  EXPECT_EQ(nodeAs<ConceptNode>(graph, CREATED->id).concept_name, "red");

  const auto FOUND = graph.findOrAddConcept("red");
  ASSERT_TRUE(FOUND.has_value());
  EXPECT_FALSE(FOUND->created);
  EXPECT_EQ(FOUND->id, CREATED->id);

  EXPECT_TRUE(graph.findOrAddConcept(std::string(128, 'a')).has_value());
  for (const std::string& bad : {std::string(" \t "), std::string(129, 'a'), std::string("a\nb")})
  {
    const auto RESULT = graph.findOrAddConcept(bad);
    ASSERT_FALSE(RESULT.has_value()) << bad;
    EXPECT_EQ(RESULT.error().kind, SceneGraphErrorKind::INVALID_LABEL);
  }
  EXPECT_EQ(graph.getConceptNodes().size(), 2U);
}

TEST(SceneGraphTest, AddConceptEdgeRemovesOnlyTheConceptItCreated)
{
  SceneGraph graph;
  const auto CUP = addObject(graph, "cup");

  // Unknown source: the concept created for the edge is removed again.
  const auto UNKNOWN = graph.addConceptEdge(999, "has_color", "red");
  ASSERT_FALSE(UNKNOWN.has_value());
  EXPECT_EQ(UNKNOWN.error().kind, SceneGraphErrorKind::NODE_DOES_NOT_EXIST);
  EXPECT_TRUE(graph.getConceptNodes().empty());

  const auto RED = graph.addConceptEdge(CUP, "has_color", "red");
  ASSERT_TRUE(RED.has_value());
  EXPECT_TRUE(RED->created);

  // An existing label is an error and changes nothing; findConcept retrieves it.
  const auto DUPLICATE = graph.addConceptEdge(CUP, "has_color", " red ");
  ASSERT_FALSE(DUPLICATE.has_value());
  EXPECT_EQ(DUPLICATE.error().kind, SceneGraphErrorKind::NODE_ALREADY_EXISTS);
  EXPECT_EQ(graph.findConcept(" red "), RED->id);
  EXPECT_FALSE(graph.findConcept("blue").has_value());
  EXPECT_FALSE(graph.findConcept("  ").has_value());
  EXPECT_EQ(graph.getConceptNodes().size(), 1U);

  EXPECT_TRUE(graph.hasNode(RED->id));
  EXPECT_EQ(graph.getEdges().size(), 1U);
}

TEST(SceneGraphTest, OneBadUnitDoesNotAffectTheOthers)
{
  SceneGraph graph;
  const auto CUP = addObject(graph, "cup");

  EXPECT_TRUE(graph.addConceptEdge(CUP, "has_color", "red").has_value());
  EXPECT_FALSE(graph.addConceptEdge(CUP, "has_material", "  ").has_value());
  EXPECT_FALSE(graph.addConceptEdge(999, "has_shape", "round").has_value());
  EXPECT_TRUE(graph.addConceptEdge(CUP, "has_material", "ceramic").has_value());

  EXPECT_EQ(graph.getConceptNodes().size(), 2U);
  EXPECT_EQ(graph.findEdgesByLabel("has_color").size(), 1U);
  EXPECT_EQ(graph.findEdgesByLabel("has_material").size(), 1U);
}

TEST(SceneGraphTest, MarkProcessedAndUnprocessedBy)
{
  SceneGraph graph;
  const auto CUP = addObject(graph, "cup");
  const auto RED = graph.addConceptNode("red").value();

  ASSERT_TRUE(graph.markProcessed(CUP, "attribution").has_value());
  EXPECT_EQ(graph.unprocessedBy("attribution"), std::vector<int64_t>{RED});
  EXPECT_EQ(graph.unprocessedBy("concepts"), (std::vector<int64_t>{CUP, RED}));
  EXPECT_FALSE(graph.markProcessed(999, "attribution").has_value());
}

// ---------------------------------------------------------------------------
// update_from_msg regressions: exact edge reconciliation and reversal, kind
// collision rejection, in-place partial failure semantics, and payload
// preservation across updates.
// ---------------------------------------------------------------------------

TEST(SceneGraphTest, UpdateFromMsgRemovesEdgesAbsentFromMessage)
{
  SceneGraph graph;
  const auto OBJECT_ID  = addObject(graph, "cup");
  const auto CONCEPT_ID = graph.addConceptNode("on").value();
  ASSERT_TRUE(graph.addEdge(OBJECT_ID, CONCEPT_ID, "rel").has_value());

  // Snapshot with the edge dropped: the update must remove it from the graph.
  auto msg = graph.toMsg();
  msg->edges.clear();
  ASSERT_TRUE(graph.updateSceneGraphFromMsg(*msg).has_value());

  EXPECT_TRUE(graph.hasNode(OBJECT_ID));
  EXPECT_TRUE(graph.hasNode(CONCEPT_ID));
  EXPECT_TRUE(graph.getEdges().empty());
}

TEST(SceneGraphTest, UpdateFromMsgReplacesReversedEdgeWithoutFalseCycle)
{
  SceneGraph graph;
  const auto FIRST_ID  = graph.addConceptNode("first").value();
  const auto SECOND_ID = graph.addConceptNode("second").value();
  ASSERT_TRUE(graph.addEdge(FIRST_ID, SECOND_ID, "rel").has_value());

  // Snapshot with the edge reversed: replacing an edge by its own opposite
  // direction must not trip cycle detection on the old edge.
  auto msg = graph.toMsg();
  for (auto& edge : msg->edges)
  {
    if (edge.source_id == FIRST_ID && edge.target_id == SECOND_ID)
    {
      edge.source_id = SECOND_ID;
      edge.target_id = FIRST_ID;
    }
  }

  ASSERT_TRUE(graph.updateSceneGraphFromMsg(*msg).has_value());

  EXPECT_FALSE(graph.hasEdge(FIRST_ID, SECOND_ID, "rel"));
  EXPECT_TRUE(graph.hasEdge(SECOND_ID, FIRST_ID, "rel"));
  ASSERT_EQ(graph.getEdges().size(), 1U);
}

TEST(SceneGraphTest, UpdateFromMsgRejectsNodeIdKindCollisions)
{
  SceneGraph graph;
  const auto OBJECT_ID = addObject(graph, "cup");

  // A concept reusing an existing object id collides and is rejected; the
  // object keeps its payload and the graph is otherwise untouched.
  auto msg = std::make_unique<semantic_scene_forests_msgs::msg::SceneGraph>();
  semantic_scene_forests_msgs::msg::ConceptNode concept_msg;
  concept_msg.id         = OBJECT_ID;
  concept_msg.class_name = "imposter";
  msg->concept_nodes.push_back(concept_msg);

  const auto RESULT = graph.updateSceneGraphFromMsg(*msg);
  ASSERT_FALSE(RESULT.has_value());
  EXPECT_EQ(RESULT.error().kind, SceneGraphErrorKind::NODE_ALREADY_EXISTS);
  EXPECT_EQ(nodeAs<ObjectNode>(graph, OBJECT_ID).class_name, "cup");
  EXPECT_TRUE(graph.getConceptNodes().empty());

  // The mirror case: an object message reusing an existing concept id.
  SceneGraph other;
  const auto OTHER_CONCEPT_ID = other.addConceptNode("on").value();
  auto object_collision       = std::make_unique<semantic_scene_forests_msgs::msg::SceneGraph>();
  semantic_scene_forests_msgs::msg::ObjectNode object_msg;
  object_msg.id = OTHER_CONCEPT_ID;
  object_collision->object_nodes.push_back(object_msg);

  const auto COLLISION = other.updateSceneGraphFromMsg(*object_collision);
  ASSERT_FALSE(COLLISION.has_value());
  EXPECT_EQ(COLLISION.error().kind, SceneGraphErrorKind::NODE_ALREADY_EXISTS);
  EXPECT_TRUE(other.getObjectNodes().empty());
  EXPECT_TRUE(other.hasNode(OTHER_CONCEPT_ID));
}

TEST(SceneGraphTest, UpdateFromMsgAppliesValidChangesBeforeFailingEdge)
{
  SceneGraph graph;
  const auto OBJECT_ID  = addObject(graph, "cup");
  const auto CONCEPT_ID = graph.addConceptNode("on").value();
  ASSERT_TRUE(graph.addEdge(OBJECT_ID, CONCEPT_ID, "rel").has_value());

  // Snapshot plus a new concept with a valid edge, followed by an invalid edge
  // (its target is an object, which can never receive edges).
  auto msg = graph.toMsg();
  semantic_scene_forests_msgs::msg::ConceptNode new_concept;
  new_concept.id         = 30;
  new_concept.class_name = "new";
  msg->concept_nodes.push_back(new_concept);
  semantic_scene_forests_msgs::msg::Edge good_edge;
  good_edge.source_id = OBJECT_ID;
  good_edge.target_id = 30;
  good_edge.label     = "rel";
  msg->edges.push_back(good_edge);
  semantic_scene_forests_msgs::msg::Edge bad_edge;
  bad_edge.source_id = 30;
  bad_edge.target_id = OBJECT_ID;
  bad_edge.label     = "invalid";
  msg->edges.push_back(bad_edge);

  // The invalid edge fails the update with an error, but every change the
  // message applied beforehand stays: the graph is edited in place, and only
  // the offending operation has no effect.
  const auto RESULT = graph.updateSceneGraphFromMsg(*msg);
  ASSERT_FALSE(RESULT.has_value());
  EXPECT_EQ(RESULT.error().kind, SceneGraphErrorKind::INVALID_EDGE);

  EXPECT_TRUE(graph.hasNode(30));
  EXPECT_TRUE(graph.hasEdge(OBJECT_ID, 30, "rel"));
  EXPECT_TRUE(graph.hasEdge(OBJECT_ID, CONCEPT_ID, "rel"));
  EXPECT_FALSE(graph.hasEdge(30, OBJECT_ID, "invalid"));

  // The graph remains fully usable after the partial failure.
  auto followup = graph.toMsg();
  followup->edges.clear();
  ASSERT_TRUE(graph.updateSceneGraphFromMsg(*followup).has_value());
  EXPECT_TRUE(graph.getEdges().empty());
  EXPECT_TRUE(graph.hasNode(OBJECT_ID));
}

TEST(SceneGraphTest, UpdateFromMsgUpdatesObjectButKeepsCropsAndWorkerState)
{
  SceneGraph graph;
  const auto CROP = crop(1);
  ASSERT_TRUE(graph.addObject({.id = 10, .class_name = "cup", .crops = {CROP}}).has_value());
  ASSERT_TRUE(graph.markProcessed(10, "worker").has_value());

  auto msg = std::make_unique<semantic_scene_forests_msgs::msg::SceneGraph>();
  semantic_scene_forests_msgs::msg::ObjectNode object_msg;
  object_msg.id              = 10;
  object_msg.class_name      = "mug";
  object_msg.header.frame_id = "map";
  msg->object_nodes.push_back(object_msg);

  ASSERT_TRUE(graph.updateSceneGraphFromMsg(*msg).has_value());

  const auto OBJECT = nodeAs<ObjectNode>(graph, 10);
  EXPECT_EQ(OBJECT.class_name, "mug");
  EXPECT_EQ(OBJECT.frame_id, "map");
  ASSERT_EQ(OBJECT.crops.size(), 1U);
  EXPECT_EQ(OBJECT.crops[0], CROP);
  EXPECT_TRUE(OBJECT.workers.at("worker").processed);
}

TEST(SceneGraphTest, RetainObjectNodesKeepsConceptsReachableFromRetainedObjects)
{
  SceneGraph graph;
  const auto OBJECT1   = addObject(graph, "cup");
  const auto OBJECT2   = addObject(graph, "plate");
  const auto SHARED    = graph.addConceptNode("shared").value();
  const auto EXCLUSIVE = graph.addConceptNode("exclusive").value();
  const auto ANCESTOR  = graph.addConceptNode("ancestor").value();

  ASSERT_TRUE(graph.addEdge(OBJECT1, SHARED, "rel").has_value());
  ASSERT_TRUE(graph.addEdge(OBJECT2, SHARED, "rel").has_value());
  ASSERT_TRUE(graph.addEdge(OBJECT1, EXCLUSIVE, "rel").has_value());
  ASSERT_TRUE(graph.addEdge(SHARED, ANCESTOR, "rel").has_value());

  const auto RESULT = graph.retainObjectNodes({OBJECT2});
  ASSERT_TRUE(RESULT.has_value());

  EXPECT_TRUE(graph.hasNode(OBJECT2));
  EXPECT_TRUE(graph.hasNode(SHARED));
  EXPECT_TRUE(graph.hasNode(ANCESTOR));
  EXPECT_FALSE(graph.hasNode(OBJECT1));
  EXPECT_FALSE(graph.hasNode(EXCLUSIVE));
}

TEST(SceneGraphTest, RetainObjectNodesEmptyActiveSetEmptiesGraph)
{
  SceneGraph graph;
  const auto OBJECT_ID  = addObject(graph, "cup");
  const auto CONCEPT_ID = graph.addConceptNode("on").value();
  ASSERT_TRUE(graph.addEdge(OBJECT_ID, CONCEPT_ID, "rel").has_value());

  const auto RESULT = graph.retainObjectNodes({});
  ASSERT_TRUE(RESULT.has_value());

  EXPECT_TRUE(graph.getNodes().empty());
  EXPECT_TRUE(graph.getEdges().empty());
}

// Regression: the implicit copy kept vertex descriptors pointing into the source graph.
TEST(SceneGraphTest, MovedGraphOutlivesSource)
{
  auto source    = std::make_unique<SceneGraph>();
  const auto OBJ = addObject(*source, "cup");
  const auto RED = source->addConceptNode("red").value();
  ASSERT_TRUE(source->addEdge(OBJ, RED, "has_color").has_value());

  SceneGraph moved(std::move(*source));
  source.reset();
  ASSERT_TRUE(moved.findNode(OBJ).has_value());
  ASSERT_EQ(moved.getEdges().size(), 1U);
  EXPECT_TRUE(moved.hasEdge(OBJ, RED, "has_color"));

  auto other = std::make_unique<SceneGraph>();
  SceneGraph assigned;
  *other   = std::move(moved);
  assigned = std::move(*other);
  other.reset();
  ASSERT_TRUE(assigned.findNode(RED).has_value());
  EXPECT_TRUE(assigned.hasEdge(OBJ, RED, "has_color"));
  EXPECT_TRUE(assigned.addEdge(OBJ, assigned.addConceptNode("x").value(), "is").has_value());
}
