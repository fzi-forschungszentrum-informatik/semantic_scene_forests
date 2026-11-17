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

#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <expected>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boost/bimap.hpp>


#include <semantic_scene_forests/error.hpp>
#include <semantic_scene_forests/graph_edge.hpp>
#include <semantic_scene_forests/graph_nodes.hpp>
#include <semantic_scene_forests/visibility_control.hpp>

#include <boost/graph/adjacency_list.hpp>

#include <semantic_scene_forests_msgs/msg/concept_node.hpp>
#include <semantic_scene_forests_msgs/msg/edge.hpp>
#include <semantic_scene_forests_msgs/msg/object_node.hpp>
#include <semantic_scene_forests_msgs/msg/scene_graph.hpp>


namespace fzi::semantic_scene_forests {

using SceneGraphAdjList = boost::adjacency_list<boost::multisetS,
                                                boost::setS,
                                                boost::directedS,
                                                SceneGraphNode,
                                                SceneGraphEdge,
                                                boost::listS>;

//! Result of findOrAddConcept / addConceptEdge.
struct ConceptLookup
{
  int64_t id{0};
  bool created{false}; //!< True if this call created the concept.
};

//! The shared scene graph model. Thread-safe: every public method locks the graph once
//! (shared for const methods, exclusive for mutating ones) and is one transaction that
//! validates before it mutates. Getters return copies.
//!
//! There is no read-modify-write outside the class: a change that depends on what the graph
//! holds (find-then-add, add-then-roll-back, ...) must be a SceneGraph method, never a
//! sequence of calls by the caller. Independent changes are consecutive calls.
//!
//! Objects and concepts share one positive id space; allocateId() hands out ids.
// ponytail: one lock per graph; split it (e.g. per node kind) if contention ever shows.
class SceneGraph
{
public:
  SceneGraph();

  virtual ~SceneGraph();

  // Move-only: vertex descriptors are pointers into m_graph, so a member-wise copy would leave
  // m_id_map pointing into the source. Use clone() for an independent copy.
  SceneGraph(const SceneGraph&)            = delete;
  SceneGraph& operator=(const SceneGraph&) = delete;
  //! Both lock both graphs.
  SEMANTIC_SCENE_FORESTS_PUBLIC SceneGraph(SceneGraph&& other) noexcept;
  SEMANTIC_SCENE_FORESTS_PUBLIC SceneGraph& operator=(SceneGraph&& other) noexcept;

  //! Returns a fresh positive id for a node added later (atomic, no graph lock).
  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC int64_t allocateId() noexcept;

  //! Adds the object as given (usually with an id from allocateId()).
  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::expected<void, SceneGraphError>
  addObject(ObjectNode node) noexcept;

  //! Sets bbox and last_seen; last_updated moves to last_seen only if the bbox changed.
  //! NODE_DOES_NOT_EXIST if there is no object with this id (e.g. it was deleted).
  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::expected<void, SceneGraphError>
  updateObjectPose(int64_t id,
                   const vision_msgs::msg::BoundingBox3D& bbox,
                   const builtin_interfaces::msg::Time& last_seen) noexcept;

  //! Sets the crop policy used by offerCrop(). Default: 5 crops, 10 s.
  SEMANTIC_SCENE_FORESTS_PUBLIC void setCropPolicy(std::size_t max_crops,
                                                   std::chrono::nanoseconds min_interval) noexcept;

  //! Offers a crop to an object. Below max_crops it is appended if it is at least the policy
  //! interval newer than the newest crop (or the object has none). When full, it replaces the
  //! oldest crop only if it is at least the interval newer than both the oldest and the newest
  //! crop, so a full object never swaps crops every frame.
  //! Returns whether the crop was taken; NODE_DOES_NOT_EXIST for an unknown object.
  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::expected<bool, SceneGraphError>
  offerCrop(int64_t id, const CropPtr& crop) noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::expected<void, SceneGraphError>
  setObjectClass(int64_t id, const std::string& class_name) noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::expected<int64_t, SceneGraphError>
  addConceptNodeWithId(int64_t id, const std::string& label) noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::expected<int64_t, SceneGraphError>
  addConceptNode(const std::string& label) noexcept;

  //! Returns the concept with this label, creating it if absent. The label is trimmed of
  //! spaces/tabs and must be non-empty, at most 128 bytes, without control characters
  //! (INVALID_LABEL otherwise).
  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::expected<ConceptLookup, SceneGraphError>
  findOrAddConcept(const std::string& label) noexcept;

  //! findOrAddConcept(label), then the edge source_id --relation--> concept. If the edge is
  //! rejected (cycle, duplicate, unknown source), a concept created by this call is removed.
  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::expected<ConceptLookup, SceneGraphError>
  addConceptEdge(int64_t source_id, const std::string& relation, const std::string& label) noexcept;

  //! Sets workers[worker].processed on an object or concept.
  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::expected<void, SceneGraphError>
  markProcessed(int64_t id, const std::string& worker) noexcept;

  //! Ids of all nodes (objects and concepts) the worker has not marked processed.
  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::vector<int64_t>
  unprocessedBy(const std::string& worker) const noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::expected<SceneGraphNode, SceneGraphError>
  removeNode(int64_t id) noexcept;

  //! Removes every node, object or concept, without edges.
  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC
    std::expected<std::vector<SceneGraphNode>, SceneGraphError>
    removeIsolatedNodes() noexcept;

  //! Removes concepts without edges; objects stay. Returns the removed ids.
  SEMANTIC_SCENE_FORESTS_PUBLIC std::vector<int64_t> pruneIsolatedConcepts() noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::optional<SceneGraphNode>
  findNode(int64_t id) const noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC bool hasNode(int64_t id) const noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC
    std::expected<std::vector<SceneGraphEdge>, SceneGraphError>
    findEdges(int64_t from, int64_t to) const noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC bool
  hasEdge(int64_t from, int64_t to, const std::string& label) const noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::expected<SceneGraphEdge, SceneGraphError>
  addEdge(int64_t from, int64_t to, const std::string& label) noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::expected<SceneGraphEdge, SceneGraphError>
  removeEdge(int64_t from, int64_t to, const std::string& label) noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC
    std::expected<std::vector<std::pair<int64_t, SceneGraphEdge>>, SceneGraphError>
    outgoingEdges(int64_t id) const noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC
    std::expected<std::vector<std::pair<int64_t, SceneGraphEdge>>, SceneGraphError>
    incomingEdges(int64_t id) const noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::vector<ConceptNode>
  findConceptNodesByName(const std::string& name) const noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC
    std::vector<std::tuple<int64_t, int64_t, SceneGraphEdge>>
    findEdgesByLabel(const std::string& label) const noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::vector<ObjectNode>
  getObjectNodes() const noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::vector<ConceptNode>
  getConceptNodes() const noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::vector<SceneGraphNode> getNodes() const noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC
    std::vector<std::tuple<int64_t, int64_t, SceneGraphEdge>>
    getEdges() const noexcept;

  //! Objects get class, bbox, frame (header.frame_id), stamps and source; crops and worker
  //! state are not part of the message.
  static SEMANTIC_SCENE_FORESTS_PUBLIC std::expected<std::unique_ptr<SceneGraph>, SceneGraphError>
  fromMsg(const semantic_scene_forests_msgs::msg::SceneGraph& msg) noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC
    std::unique_ptr<semantic_scene_forests_msgs::msg::SceneGraph>
    toMsg() const noexcept;

  // Creates an independent graph copy with all node data.
  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC
    std::expected<std::unique_ptr<SceneGraph>, SceneGraphError>
    clone() const;

  // Exchanges graph storage, id mapping, id counter and crop policy; locks both graphs.
  SEMANTIC_SCENE_FORESTS_PUBLIC void swap(SceneGraph& other) noexcept;

  // Updates in place; on the first error, earlier successful changes remain. Existing objects
  // take the message's fields but keep their crops and worker state.
  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::expected<void, SceneGraphError>
  updateSceneGraphFromMsg(const semantic_scene_forests_msgs::msg::SceneGraph& msg) noexcept;

  // Retains the objects named in active_ids (ids that are not existing objects are ignored)
  // together with every node reachable from them through outgoing edges, and removes
  // everything else. An empty active set empties the graph.
  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::expected<void, SceneGraphError>
  retainObjectNodes(const std::unordered_set<int64_t>& active_ids) noexcept;

private:
  using IdVertexBimap = boost::bimap<int64_t, SceneGraphAdjList::vertex_descriptor>;
  mutable std::shared_mutex m_mutex;
  IdVertexBimap m_id_map;
  SceneGraphAdjList m_graph;
  std::atomic<int64_t> m_next_id{1};
  std::size_t m_max_crops{5};
  std::chrono::nanoseconds m_crop_min_interval{std::chrono::seconds(10)};

  // Unlocked helpers: the caller holds m_mutex.
  void reserveId(int64_t id) noexcept;
  ObjectNode* findObject(int64_t id) noexcept;
  std::expected<void, SceneGraphError> addObjectUnlocked(ObjectNode node) noexcept;
  std::expected<int64_t, SceneGraphError> addConceptUnlocked(int64_t id,
                                                             const std::string& label) noexcept;
  std::expected<ConceptLookup, SceneGraphError>
  findOrAddConceptUnlocked(const std::string& label) noexcept;
  std::expected<SceneGraphNode, SceneGraphError> removeNodeUnlocked(int64_t id) noexcept;
  std::vector<SceneGraphNode> removeIsolatedUnlocked(bool concepts_only) noexcept;
  std::expected<std::vector<SceneGraphEdge>, SceneGraphError>
  findEdgesUnlocked(int64_t from, int64_t to) const noexcept;
  std::expected<SceneGraphEdge, SceneGraphError>
  addEdgeUnlocked(int64_t from, int64_t to, const std::string& label) noexcept;
  std::expected<SceneGraphEdge, SceneGraphError>
  removeEdgeUnlocked(int64_t from, int64_t to, const std::string& label) noexcept;
  std::vector<std::tuple<int64_t, int64_t, SceneGraphEdge>> getEdgesUnlocked() const noexcept;

  std::expected<void, SceneGraphError>
  updateObjectNodeFromMsg(const semantic_scene_forests_msgs::msg::ObjectNode& object_msg) noexcept;

  std::expected<void, SceneGraphError> updateConceptNodeFromMsg(
    const semantic_scene_forests_msgs::msg::ConceptNode& concept_msg) noexcept;

  // Drops every edge absent from the message before adding the new ones, so a
  // reversed edge cannot trip cycle detection on its old self.
  std::expected<void, SceneGraphError>
  reconcileEdgesFromMsg(const semantic_scene_forests_msgs::msg::SceneGraph& msg) noexcept;
};
} // namespace fzi::semantic_scene_forests
