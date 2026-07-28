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

#include <expected>
#include <memory>
#include <tuple>
#include <unordered_map>
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
class SceneGraph
{
public:
  SceneGraph();

  virtual ~SceneGraph();

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::expected<int64_t, SceneGraphError>
  addObjectNodeWithId(int64_t id,
                      const std::string& class_name,
                      const vision_msgs::msg::Detection3D::SharedPtr& detection,
                      const std::array<sensor_msgs::msg::Image::SharedPtr, 5>& visuals) noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::expected<int64_t, SceneGraphError>
  addObjectNode(const std::string& class_name,
                const vision_msgs::msg::Detection3D::SharedPtr& detection,
                const std::array<sensor_msgs::msg::Image::SharedPtr, 5>& visuals) noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::expected<int64_t, SceneGraphError>
  addConceptNodeWithId(int64_t id, const std::string& label) noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::expected<int64_t, SceneGraphError>
  addConceptNode(const std::string& label) noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::expected<SceneGraphNode, SceneGraphError>
  removeNode(int64_t id) noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC
    std::expected<std::vector<SceneGraphNode>, SceneGraphError>
    removeIsolatedNodes() noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::optional<SceneGraphNode>
  findNode(int64_t id) noexcept;

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

  static SEMANTIC_SCENE_FORESTS_PUBLIC std::expected<std::unique_ptr<SceneGraph>, SceneGraphError>
  fromMsg(const semantic_scene_forests_msgs::msg::SceneGraph& msg) noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC
    std::unique_ptr<semantic_scene_forests_msgs::msg::SceneGraph>
    toMsg() noexcept;

  // Creates an independent graph snapshot while preserving object metadata.
  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC
    std::expected<std::unique_ptr<SceneGraph>, SceneGraphError>
    clone() const;

  // Exchanges graph storage and its descriptor-to-id mapping atomically at the caller level.
  SEMANTIC_SCENE_FORESTS_PUBLIC void swap(SceneGraph& other) noexcept;

  [[nodiscard]] SEMANTIC_SCENE_FORESTS_PUBLIC std::expected<void, SceneGraphError>
  updateSceneGraphFromMsg(const semantic_scene_forests_msgs::msg::SceneGraph& msg) noexcept;

private:
  using IdVertexBimap = boost::bimap<int64_t, SceneGraphAdjList::vertex_descriptor>;
  IdVertexBimap m_id_map;
  SceneGraphAdjList m_graph;
  int64_t m_next_object_id;
  int64_t m_next_concept_id;

  std::expected<void, SceneGraphError>
  updateObjectNodeFromMsg(const semantic_scene_forests_msgs::msg::ObjectNode& object_msg) noexcept;

  std::expected<void, SceneGraphError> updateConceptNodeFromMsg(
    const semantic_scene_forests_msgs::msg::ConceptNode& concept_msg) noexcept;
};
} // namespace fzi::semantic_scene_forests
