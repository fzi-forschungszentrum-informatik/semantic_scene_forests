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
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/time.hpp>
#include <semantic_scene_forests/scene_graph.hpp>

#include "semantic_scene_forests_manager/database.hpp"

namespace {

using fzi::semantic_scene_forests::SceneGraph;
using semantic_scene_forests_manager::Database;
using semantic_scene_forests_manager::TrackUpdate;
using semantic_scene_forests_manager::Tx;

class DatabaseGraphTest : public ::testing::Test
{
protected:
  int64_t addObject(const std::string& class_name = "cup")
  {
    return m_db.write([&](Tx& tx) {
      TrackUpdate update;
      update.id                    = tx.allocateObjectId();
      update.class_name            = class_name;
      update.detection.bbox.size.x = 0.4;
      tx.upsertTrack(update, rclcpp::Time(100, 0));
      return update.id;
    });
  }

  int64_t addConcept(const std::string& label)
  {
    return m_db.write([&](Tx& tx) { return tx.findOrCreateConcept(label); });
  }

  bool link(int64_t source, int64_t target, const std::string& label)
  {
    return m_db.write([&](Tx& tx) { return tx.addEdge(source, target, label); });
  }

  SceneGraph graph() const
  {
    return m_db.read([](const Tx& tx) { return tx.visibleGraph(); });
  }

  // Clones the visible graph so a test can edit it as a worker scratch graph.
  std::unique_ptr<SceneGraph> scratch() const
  {
    auto clone = graph().clone();
    EXPECT_TRUE(clone.has_value());
    return std::move(*clone);
  }

  void commit(const SceneGraph& before, const SceneGraph& after)
  {
    m_db.write([&](Tx& tx) { tx.commitScratch(before, after); });
  }

  static std::vector<std::string> conceptLabels(const SceneGraph& graph)
  {
    std::vector<std::string> labels;
    for (const auto& node : graph.getConceptNodes())
    {
      labels.push_back(node.concept_name);
    }
    std::ranges::sort(labels);
    return labels;
  }

  Database m_db;
};

TEST_F(DatabaseGraphTest, ConceptAndObjectIdsShareOneSequence)
{
  const int64_t OBJECT = addObject();
  EXPECT_GT(addConcept("red"), OBJECT);
  EXPECT_EQ(addConcept("red"), addConcept("red"));
}

TEST_F(DatabaseGraphTest, AddEdgeRejectsMissingEndpoint)
{
  const int64_t OBJECT = addObject();
  const int64_t RED    = addConcept("red");
  EXPECT_FALSE(link(OBJECT, 999, "has_color"));
  EXPECT_FALSE(link(999, RED, "has_color"));
  EXPECT_TRUE(link(OBJECT, RED, "has_color"));
  EXPECT_FALSE(link(OBJECT, RED, "has_color")); // Already present.
}

TEST_F(DatabaseGraphTest, DeleteObjectKeepsConcepts)
{
  const int64_t OBJECT = addObject();
  ASSERT_TRUE(link(OBJECT, addConcept("red"), "has_color"));
  m_db.write([&](Tx& tx) { tx.deleteObject(OBJECT); });
  // The concept row stays until pruning, but without a visible edge it is not shown.
  const auto GRAPH = graph();
  EXPECT_TRUE(conceptLabels(GRAPH).empty());
  EXPECT_TRUE(GRAPH.getEdges().empty());
  EXPECT_EQ(m_db.write([](Tx& tx) { return tx.pruneIsolatedConcepts(); }), 1);
}

TEST_F(DatabaseGraphTest, PruneRemovesOnlyEdgelessConcepts)
{
  const int64_t OBJECT = addObject();
  ASSERT_TRUE(link(OBJECT, addConcept("red"), "has_color"));
  addConcept("lonely");
  EXPECT_EQ(m_db.write([](Tx& tx) { return tx.pruneIsolatedConcepts(); }), 1);
  EXPECT_EQ(conceptLabels(graph()), std::vector<std::string>{"red"});
}

TEST_F(DatabaseGraphTest, VisibleGraphHidesBlockedObjectAndItsOnlyConcepts)
{
  const int64_t HIDDEN = addObject("cup");
  const int64_t SHOWN  = addObject("plate");
  const int64_t BIG    = addConcept("big");
  ASSERT_TRUE(link(HIDDEN, addConcept("red"), "has_color"));
  ASSERT_TRUE(link(HIDDEN, BIG, "has_size"));
  ASSERT_TRUE(link(SHOWN, BIG, "has_size"));
  addConcept("lonely"); // Edgeless concepts have no visible edge and are not shown.
  ASSERT_TRUE(m_db.write([&](Tx& tx) { return tx.block(HIDDEN); }));

  const auto GRAPH   = graph();
  const auto OBJECTS = GRAPH.getObjectNodes();
  ASSERT_EQ(OBJECTS.size(), 1U);
  EXPECT_EQ(OBJECTS[0].id, SHOWN);
  EXPECT_EQ(OBJECTS[0].class_name, "plate");
  ASSERT_TRUE(OBJECTS[0].detection);
  EXPECT_DOUBLE_EQ(OBJECTS[0].detection->bbox.size.x, 0.4);
  EXPECT_EQ(conceptLabels(GRAPH), std::vector<std::string>{"big"});
  EXPECT_TRUE(GRAPH.hasNode(BIG));
  ASSERT_EQ(GRAPH.getEdges().size(), 1U);
  EXPECT_TRUE(GRAPH.hasEdge(SHOWN, BIG, "has_size"));
}

TEST_F(DatabaseGraphTest, VisibleGraphHidesConceptChainOfBlockedObject)
{
  const int64_t HIDDEN = addObject("cup");
  const int64_t SHOWN  = addObject("plate");
  const int64_t RED    = addConcept("red");
  const int64_t BLUE   = addConcept("blue");
  const int64_t COLOR  = addConcept("color");
  ASSERT_TRUE(link(HIDDEN, RED, "has_color"));
  ASSERT_TRUE(link(RED, COLOR, "is_a"));
  ASSERT_TRUE(link(SHOWN, BLUE, "has_color"));
  ASSERT_TRUE(link(BLUE, COLOR, "is_a"));
  ASSERT_TRUE(m_db.write([&](Tx& tx) { return tx.block(HIDDEN); }));

  const auto GRAPH = graph();
  EXPECT_FALSE(GRAPH.hasNode(RED));
  EXPECT_EQ(conceptLabels(GRAPH), (std::vector<std::string>{"blue", "color"}));
  EXPECT_TRUE(GRAPH.hasEdge(BLUE, COLOR, "is_a"));
}

TEST_F(DatabaseGraphTest, CommitScratchReusesConceptByLabel)
{
  const int64_t FIRST  = addObject();
  const int64_t SECOND = addObject();
  const auto BEFORE    = graph();
  for (const int64_t OBJECT : {FIRST, SECOND})
  {
    auto after = scratch();
    auto red   = after->addConceptNode("red");
    ASSERT_TRUE(red.has_value());
    EXPECT_LT(*red, 0);
    ASSERT_TRUE(after->addEdge(OBJECT, *red, "has_color").has_value());
    commit(BEFORE, *after);
  }
  const auto GRAPH = graph();
  EXPECT_EQ(conceptLabels(GRAPH), std::vector<std::string>{"red"});
  EXPECT_EQ(GRAPH.getEdges().size(), 2U);
}

TEST_F(DatabaseGraphTest, CommitScratchSkipsEdgesToDeletedObject)
{
  const int64_t OBJECT = addObject();
  const auto BEFORE    = graph();
  auto after           = scratch();
  auto red             = after->addConceptNode("red");
  ASSERT_TRUE(red.has_value());
  ASSERT_TRUE(after->addEdge(OBJECT, *red, "has_color").has_value());
  m_db.write([&](Tx& tx) { tx.deleteObject(OBJECT); });
  commit(BEFORE, *after);
  EXPECT_TRUE(graph().getEdges().empty());
}

TEST_F(DatabaseGraphTest, CommitScratchRemovesAnyDroppedItem)
{
  // The graph is shared: a worker's scratch may drop items another writer added.
  const int64_t OBJECT = addObject();
  const int64_t RED    = addConcept("red");
  const int64_t BIG    = addConcept("big");
  ASSERT_TRUE(link(OBJECT, RED, "has_color"));
  ASSERT_TRUE(link(OBJECT, BIG, "has_size"));
  const auto NOTE = m_db.write([&](Tx& tx) { return tx.addContext(OBJECT, "kitchen"); });
  ASSERT_TRUE(NOTE.second);

  const auto BEFORE = graph();
  auto after        = scratch();
  ASSERT_TRUE(after->removeNode(NOTE.first).has_value());
  ASSERT_TRUE(after->removeEdge(OBJECT, RED, "has_color").has_value());
  commit(BEFORE, *after);

  const auto GRAPH = graph();
  EXPECT_EQ(conceptLabels(GRAPH), std::vector<std::string>{"big"});
  ASSERT_EQ(GRAPH.getEdges().size(), 1U);
  EXPECT_TRUE(GRAPH.hasEdge(OBJECT, BIG, "has_size"));
  // "kitchen" was removed as a concept; "red" only lost its edge and is left for pruning.
  EXPECT_EQ(m_db.write([](Tx& tx) { return tx.pruneIsolatedConcepts(); }), 1);
}

TEST_F(DatabaseGraphTest, AddContextReusesSameLabel)
{
  const int64_t OBJECT = addObject();
  const auto FIRST     = m_db.write([&](Tx& tx) { return tx.addContext(OBJECT, " kitchen "); });
  EXPECT_TRUE(FIRST.second);
  EXPECT_GT(FIRST.first, OBJECT);
  const auto SECOND = m_db.write([&](Tx& tx) { return tx.addContext(OBJECT, "kitchen"); });
  EXPECT_EQ(SECOND.first, FIRST.first);
  EXPECT_FALSE(SECOND.second);
  EXPECT_TRUE(graph().hasEdge(OBJECT, FIRST.first, "has_context"));

  // Unknown and blocked nodes are rejected.
  EXPECT_EQ(m_db.write([](Tx& tx) { return tx.addContext(999, "kitchen"); }),
            std::make_pair(int64_t{0}, false));
  ASSERT_TRUE(m_db.write([&](Tx& tx) { return tx.block(OBJECT); }));
  EXPECT_EQ(m_db.write([&](Tx& tx) { return tx.addContext(OBJECT, "hall"); }),
            std::make_pair(int64_t{0}, false));
}

TEST_F(DatabaseGraphTest, UnmarkedQueries)
{
  const int64_t OBJECT = addObject();
  const int64_t RED    = addConcept("red");
  EXPECT_EQ(m_db.read([](const Tx& tx) { return tx.visibleObjectsUnmarkedBy("a"); }),
            std::vector<int64_t>{OBJECT});
  EXPECT_EQ(m_db.read([](const Tx& tx) { return tx.conceptsUnmarkedBy("a"); }),
            std::vector<int64_t>{RED});
  m_db.write([&](Tx& tx) {
    tx.mark("a", OBJECT);
    tx.mark("a", RED);
  });
  EXPECT_TRUE(m_db.read([](const Tx& tx) { return tx.visibleObjectsUnmarkedBy("a"); }).empty());
  EXPECT_TRUE(m_db.read([](const Tx& tx) { return tx.conceptsUnmarkedBy("a"); }).empty());
  EXPECT_EQ(m_db.read([](const Tx& tx) { return tx.conceptsUnmarkedBy("b"); }),
            std::vector<int64_t>{RED});
}

} // namespace
