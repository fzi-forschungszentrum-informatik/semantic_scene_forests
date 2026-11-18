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
#include <memory>
#include <string>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>
#include <semantic_scene_forests/scene_graph.hpp>

#include "generation_common.hpp"
#include "tasks/task_utils.hpp"

namespace {

using fzi::semantic_scene_forests::SceneGraph;
using semantic_scene_forests_generation::applyLabels;
using semantic_scene_forests_generation::json;

int64_t addObject(SceneGraph& graph)
{
  const int64_t ID = graph.allocateId();
  EXPECT_TRUE(graph.addObject({.id = ID, .class_name = "cup"}));
  return ID;
}

TEST(ApplyLabelsTest, InvalidLabelIsSkippedAndTheOthersAreWritten)
{
  SceneGraph graph;
  const auto OBJECT = addObject(graph);

  // A non-string, an empty label and the root itself are rejected; red and blue go through.
  const auto RESULT = applyLabels(graph,
                                  rclcpp::get_logger("test"),
                                  OBJECT,
                                  "has_color",
                                  "color",
                                  json::array({"red", 42, "", "color", "blue"}));
  ASSERT_TRUE(RESULT);

  for (const std::string LABEL : {"red", "blue"})
  {
    const auto CONCEPTS = graph.findConceptNodesByName(LABEL);
    ASSERT_EQ(CONCEPTS.size(), 1U) << LABEL;
    EXPECT_TRUE(graph.hasEdge(OBJECT, CONCEPTS.front().id, "has_color"));
    const auto ROOT = graph.findConceptNodesByName("color");
    ASSERT_EQ(ROOT.size(), 1U);
    EXPECT_TRUE(graph.hasEdge(CONCEPTS.front().id, ROOT.front().id, "is_type_of"));
  }
  EXPECT_EQ(graph.getConceptNodes().size(), 3U); // red, blue, color
}

TEST(ApplyLabelsTest, LinksToAnExistingConceptOfAnotherObject)
{
  SceneGraph graph;
  const auto FIRST  = addObject(graph);
  const auto SECOND = addObject(graph);
  ASSERT_TRUE(applyLabels(
    graph, rclcpp::get_logger("test"), FIRST, "has_color", "color", json::array({"red"})));
  ASSERT_TRUE(applyLabels(
    graph, rclcpp::get_logger("test"), SECOND, "has_color", "color", json::array({"red"})));
  const auto RED = graph.findConcept("red");
  ASSERT_TRUE(RED);
  EXPECT_TRUE(graph.hasEdge(FIRST, *RED, "has_color"));
  EXPECT_TRUE(graph.hasEdge(SECOND, *RED, "has_color"));
  EXPECT_EQ(graph.getConceptNodes().size(), 2U); // red, color
}

TEST(ApplyLabelsTest, DependentStepIsSkippedWhenItsPredecessorFailed)
{
  SceneGraph graph;
  // The source does not exist (deleted meanwhile): the first step fails, so no root is created.
  const auto RESULT = applyLabels(
    graph, rclcpp::get_logger("test"), 9999, "has_color", "color", json::array({"red"}));
  ASSERT_TRUE(RESULT);
  EXPECT_TRUE(graph.getConceptNodes().empty());
}

TEST(ApplyLabelsTest, RerunKeepsExistingEdgesAndRejectsNonArray)
{
  SceneGraph graph;
  const auto OBJECT = addObject(graph);
  const auto LABELS = json::array({"red"});
  ASSERT_TRUE(applyLabels(graph, rclcpp::get_logger("test"), OBJECT, "has_color", "color", LABELS));
  ASSERT_TRUE(applyLabels(graph, rclcpp::get_logger("test"), OBJECT, "has_color", "color", LABELS));
  EXPECT_EQ(graph.getConceptNodes().size(), 2U);
  EXPECT_FALSE(
    applyLabels(graph, rclcpp::get_logger("test"), OBJECT, "has_color", "color", json("red")));
}

TEST(MarkRunResultsTest, MarksInputAndItsNewConceptsButNotOthersWriters)
{
  SceneGraph graph;
  const auto OBJECT = addObject(graph);
  const auto INPUT  = graph.addConceptEdge(OBJECT, "has_color", "red");
  ASSERT_TRUE(INPUT);
  const std::unordered_set<int64_t> EXISTING{INPUT->id};
  // The run categorizes "red", and another writer adds "plastic" meanwhile.
  ASSERT_TRUE(applyLabels(
    graph, rclcpp::get_logger("test"), INPUT->id, "is_type_of", "category", json::array({"warm"})));
  const auto OTHER = graph.addConceptEdge(OBJECT, "has_material", "plastic");
  ASSERT_TRUE(OTHER);

  semantic_scene_forests_generation::markRunResults(graph, "gen", {INPUT->id}, EXISTING);

  const auto PENDING    = graph.unprocessedBy("gen");
  const auto IS_PENDING = [&PENDING](int64_t id) {
    return std::ranges::find(PENDING, id) != PENDING.end();
  };
  EXPECT_FALSE(IS_PENDING(INPUT->id));
  EXPECT_FALSE(IS_PENDING(graph.findConceptNodesByName("warm").front().id));
  EXPECT_FALSE(IS_PENDING(graph.findConceptNodesByName("category").front().id));
  EXPECT_TRUE(IS_PENDING(OTHER->id));
}

} // namespace

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
