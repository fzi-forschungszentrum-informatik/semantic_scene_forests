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

#include <algorithm>
#include <utility>

#include <semantic_scene_forests/scene_graph.hpp>

#include <ranges>

#include <boost/graph/breadth_first_search.hpp>

namespace fzi::semantic_scene_forests {

SceneGraph::SceneGraph()
  : m_next_object_id(1)
  , m_next_concept_id(-1)
{
}

SceneGraph::~SceneGraph() = default;

std::expected<int64_t, SceneGraphError> SceneGraph::addObjectNodeWithId(
  int64_t id,
  const std::string& class_name,
  const vision_msgs::msg::Detection3D::SharedPtr& detection,
  const std::array<sensor_msgs::msg::Image::SharedPtr, 5>& visuals) noexcept
{
  if (this->m_id_map.left.count(id) > 0)
  {
    return std::unexpected(SceneGraphError::nodeAlreadyExists(id));
  }

  this->m_next_object_id = std::max(this->m_next_object_id, id + 1);
  auto object_node =
    ObjectNode{.id = id, .class_name = class_name, .detection = detection, .visuals = visuals};

  const SceneGraphNode GRAPH_NODE(std::in_place_type<ObjectNode>, std::move(object_node));

  SceneGraphAdjList::vertex_descriptor graph_index = boost::add_vertex(GRAPH_NODE, this->m_graph);

  this->m_id_map.insert({id, graph_index});

  return id;
}

std::expected<int64_t, SceneGraphError>
SceneGraph::addObjectNode(const std::string& class_name,
                          const vision_msgs::msg::Detection3D::SharedPtr& detection,
                          const std::array<sensor_msgs::msg::Image::SharedPtr, 5>& visuals) noexcept
{
  const int64_t NODE_ID = this->m_next_object_id;
  this->m_next_object_id += 1;
  return addObjectNodeWithId(NODE_ID, class_name, detection, visuals);
}

std::expected<int64_t, SceneGraphError>
SceneGraph::addConceptNodeWithId(int64_t id, const std::string& label) noexcept
{
  if (this->m_id_map.left.count(id) > 0)
  {
    return std::unexpected(SceneGraphError::nodeAlreadyExists(id));
  }

  this->m_next_concept_id = std::min(this->m_next_concept_id, id - 1);
  auto concept_node       = ConceptNode{.id = id, .concept_name = label};

  const SceneGraphNode GRAPH_NODE(std::in_place_type<ConceptNode>, std::move(concept_node));

  SceneGraphAdjList::vertex_descriptor graph_index = boost::add_vertex(GRAPH_NODE, this->m_graph);

  this->m_id_map.insert({id, graph_index});

  return id;
}

std::expected<int64_t, SceneGraphError>
SceneGraph::addConceptNode(const std::string& label) noexcept
{
  const int64_t NODE_ID = this->m_next_concept_id;
  this->m_next_concept_id -= 1;
  return addConceptNodeWithId(NODE_ID, label);
}

std::expected<SceneGraphNode, SceneGraphError> SceneGraph::removeNode(const int64_t id) noexcept
{
  const auto IT = this->m_id_map.left.find(id);
  if (IT == this->m_id_map.left.end())
  {
    return std::unexpected(SceneGraphError::nodeDoesNotExist(id));
  }

  auto* const VERTEX_DESC     = IT->second;
  SceneGraphNode removed_node = this->m_graph[VERTEX_DESC];

  this->m_id_map.left.erase(IT);
  boost::clear_vertex(VERTEX_DESC, this->m_graph);
  boost::remove_vertex(VERTEX_DESC, this->m_graph);

  return removed_node;
}

std::expected<std::vector<SceneGraphNode>, SceneGraphError>
SceneGraph::removeIsolatedNodes() noexcept
{
  std::vector<int64_t> isolated_ids;

  for (const auto& [id, vertex_desc] : this->m_id_map.left)
  {
    const bool HAS_OUTGOING = boost::out_degree(vertex_desc, this->m_graph) > 0;

    bool has_incoming = false;
    using EdgeIter    = boost::graph_traits<SceneGraphAdjList>::edge_iterator;
    EdgeIter ei;
    EdgeIter ei_end;
    boost::tie(ei, ei_end) = boost::edges(this->m_graph);
    for (; ei != ei_end; ++ei)
    {
      if (boost::target(*ei, this->m_graph) == vertex_desc)
      {
        has_incoming = true;
        break;
      }
    }

    if (!HAS_OUTGOING && !has_incoming)
    {
      isolated_ids.push_back(id);
    }
  }

  std::vector<SceneGraphNode> removed_nodes;
  for (const int64_t ID : isolated_ids)
  {
    auto result = removeNode(ID);
    if (!result)
    {
      return std::unexpected(result.error());
    }
    removed_nodes.push_back(std::move(*result));
  }

  return removed_nodes;
}

std::optional<SceneGraphNode> SceneGraph::findNode(const int64_t id) noexcept
{
  const auto IT = this->m_id_map.left.find(id);
  if (IT == this->m_id_map.left.end())
  {
    return std::nullopt;
  }
  return this->m_graph[IT->second];
}

bool SceneGraph::hasNode(const int64_t id) const noexcept
{
  return this->m_id_map.left.count(id) > 0;
}

std::expected<std::vector<SceneGraphEdge>, SceneGraphError>
SceneGraph::findEdges(const int64_t from, const int64_t to) const noexcept
{
  const auto FROM_IT = this->m_id_map.left.find(from);
  if (FROM_IT == this->m_id_map.left.end())
  {
    return std::unexpected(SceneGraphError::nodeDoesNotExist(from));
  }

  const auto TO_IT = this->m_id_map.left.find(to);
  if (TO_IT == this->m_id_map.left.end())
  {
    return std::unexpected(SceneGraphError::nodeDoesNotExist(to));
  }

  std::vector<SceneGraphEdge> edges;

  using OutEdgeIter = boost::graph_traits<SceneGraphAdjList>::out_edge_iterator;
  OutEdgeIter ei;
  OutEdgeIter ei_end;
  boost::tie(ei, ei_end) = boost::out_edges(FROM_IT->second, this->m_graph);

  for (; ei != ei_end; ++ei)
  {
    if (boost::target(*ei, this->m_graph) == TO_IT->second)
    {
      edges.push_back(this->m_graph[*ei]);
    }
  }

  return edges;
}

bool SceneGraph::hasEdge(const int64_t from,
                         const int64_t to,
                         const std::string& label) const noexcept
{
  const auto EDGES_RESULT = findEdges(from, to);
  if (!EDGES_RESULT)
  {
    return false;
  }

  const auto& edges = *EDGES_RESULT;
  return std::ranges::any_of(edges,
                             [&label](const SceneGraphEdge& edge) { return edge.label == label; });
}


std::expected<SceneGraphEdge, SceneGraphError>
SceneGraph::addEdge(const int64_t from, const int64_t to, const std::string& label) noexcept
{
  const auto FROM_IT = this->m_id_map.left.find(from);
  if (FROM_IT == this->m_id_map.left.end())
  {
    return std::unexpected(SceneGraphError::nodeDoesNotExist(from));
  }

  const auto TO_IT = this->m_id_map.left.find(to);
  if (TO_IT == this->m_id_map.left.end())
  {
    return std::unexpected(SceneGraphError::nodeDoesNotExist(to));
  }

  auto* const TARGET_VERTEX = TO_IT->second;

  if (const auto& target_node = this->m_graph[TARGET_VERTEX];
      std::holds_alternative<ObjectNode>(target_node))
  {
    return std::unexpected(SceneGraphError::invalidEdge(from, to, label));
  }

  using OutEdgeIter = boost::graph_traits<SceneGraphAdjList>::out_edge_iterator;
  OutEdgeIter ei;
  OutEdgeIter ei_end;
  boost::tie(ei, ei_end) = boost::out_edges(FROM_IT->second, this->m_graph);

  for (; ei != ei_end; ++ei)
  {
    if (boost::target(*ei, this->m_graph) == TARGET_VERTEX && this->m_graph[*ei].label == label)
    {
      return std::unexpected(SceneGraphError::edgeAlreadyExists(from, to));
    }
  }

  auto* const SOURCE_VERTEX = FROM_IT->second;

  using VertexDesc = boost::graph_traits<SceneGraphAdjList>::vertex_descriptor;

  struct CycleDetected : std::exception
  {
  };

  struct CycleVisitor : boost::default_bfs_visitor
  {
    VertexDesc source;
    explicit CycleVisitor(VertexDesc s)
      : source(s)
    {
    }
    void discover_vertex(VertexDesc v, const SceneGraphAdjList& /*g*/) const
    {
      if (v == source)
      {
        throw CycleDetected{};
      }
    }
  };

  std::map<VertexDesc, boost::default_color_type> color_map;
  auto color_pmap = boost::make_assoc_property_map(color_map);

  try
  {
    boost::breadth_first_search(this->m_graph,
                                TARGET_VERTEX,
                                boost::visitor(CycleVisitor{SOURCE_VERTEX}).color_map(color_pmap));
  }
  catch (const CycleDetected&)
  {
    return std::unexpected(SceneGraphError::cycle(from, to));
  }

  SceneGraphEdge edge{.label = label};
  auto [added_edge, success] = boost::add_edge(FROM_IT->second, TARGET_VERTEX, edge, this->m_graph);

  if (!success)
  {
    return std::unexpected(SceneGraphError::edgeDoesNotExist(from, to, label));
  }

  return this->m_graph[added_edge];
}

std::expected<SceneGraphEdge, SceneGraphError>
SceneGraph::removeEdge(const int64_t from, const int64_t to, const std::string& label) noexcept
{
  const auto FROM_IT = this->m_id_map.left.find(from);
  if (FROM_IT == this->m_id_map.left.end())
  {
    return std::unexpected(SceneGraphError::nodeDoesNotExist(from));
  }

  const auto TO_IT = this->m_id_map.left.find(to);
  if (TO_IT == this->m_id_map.left.end())
  {
    return std::unexpected(SceneGraphError::nodeDoesNotExist(to));
  }

  using OutEdgeIter = boost::graph_traits<SceneGraphAdjList>::out_edge_iterator;
  OutEdgeIter ei;
  OutEdgeIter ei_end;
  boost::tie(ei, ei_end) = boost::out_edges(FROM_IT->second, this->m_graph);

  for (; ei != ei_end; ++ei)
  {
    if (boost::target(*ei, this->m_graph) == TO_IT->second && this->m_graph[*ei].label == label)
    {
      SceneGraphEdge removed_edge = this->m_graph[*ei];
      boost::remove_edge(*ei, this->m_graph);
      return removed_edge;
    }
  }

  return std::unexpected(SceneGraphError::edgeDoesNotExist(from, to, label));
}

std::expected<std::vector<std::pair<int64_t, SceneGraphEdge>>, SceneGraphError>
SceneGraph::outgoingEdges(const int64_t id) const noexcept
{
  const auto IT = this->m_id_map.left.find(id);
  if (IT == this->m_id_map.left.end())
  {
    return std::unexpected(SceneGraphError::nodeDoesNotExist(id));
  }

  std::vector<std::pair<int64_t, SceneGraphEdge>> edges;

  using OutEdgeIter = boost::graph_traits<SceneGraphAdjList>::out_edge_iterator;
  OutEdgeIter ei;
  OutEdgeIter ei_end;
  boost::tie(ei, ei_end) = boost::out_edges(IT->second, this->m_graph);

  for (; ei != ei_end; ++ei)
  {
    auto* const TARGET_VERTEX = boost::target(*ei, this->m_graph);
    const auto TARGET_ID_IT   = this->m_id_map.right.find(TARGET_VERTEX);

    if (TARGET_ID_IT != this->m_id_map.right.end())
    {
      edges.emplace_back(TARGET_ID_IT->second, this->m_graph[*ei]);
    }
  }

  return edges;
}

std::expected<std::vector<std::pair<int64_t, SceneGraphEdge>>, SceneGraphError>
SceneGraph::incomingEdges(const int64_t id) const noexcept
{
  const auto TARGET_IT = this->m_id_map.left.find(id);
  if (TARGET_IT == this->m_id_map.left.end())
  {
    return std::unexpected(SceneGraphError::nodeDoesNotExist(id));
  }

  std::vector<std::pair<int64_t, SceneGraphEdge>> edges;
  auto* const TARGET_VERTEX = TARGET_IT->second;

  for (const auto& [source_id, source_vertex] : this->m_id_map.left)
  {
    using OutEdgeIter = boost::graph_traits<SceneGraphAdjList>::out_edge_iterator;
    OutEdgeIter ei;
    OutEdgeIter ei_end;
    boost::tie(ei, ei_end) = boost::out_edges(source_vertex, this->m_graph);

    for (; ei != ei_end; ++ei)
    {
      if (boost::target(*ei, this->m_graph) == TARGET_VERTEX)
      {
        edges.emplace_back(source_id, this->m_graph[*ei]);
      }
    }
  }

  return edges;
}

std::vector<ConceptNode> SceneGraph::findConceptNodesByName(const std::string& name) const noexcept
{
  std::vector<ConceptNode> result;

  for (const auto& [id, vertex_desc] : this->m_id_map.left)
  {
    const auto& node = this->m_graph[vertex_desc];
    if (const auto* concept_node = std::get_if<ConceptNode>(&node))
    {
      if (concept_node->concept_name == name)
      {
        result.push_back(*concept_node);
      }
    }
  }

  return result;
}

std::vector<std::tuple<int64_t, int64_t, SceneGraphEdge>>
SceneGraph::findEdgesByLabel(const std::string& label) const noexcept
{
  std::vector<std::tuple<int64_t, int64_t, SceneGraphEdge>> result;

  using EdgeIter = boost::graph_traits<SceneGraphAdjList>::edge_iterator;
  EdgeIter ei;
  EdgeIter ei_end;
  boost::tie(ei, ei_end) = boost::edges(this->m_graph);

  for (; ei != ei_end; ++ei)
  {
    const auto& edge = this->m_graph[*ei];
    if (edge.label == label)
    {
      auto* const SOURCE_VERTEX = boost::source(*ei, this->m_graph);
      auto* const TARGET_VERTEX = boost::target(*ei, this->m_graph);

      const auto SOURCE_ID_IT = this->m_id_map.right.find(SOURCE_VERTEX);
      const auto TARGET_ID_IT = this->m_id_map.right.find(TARGET_VERTEX);

      if (SOURCE_ID_IT != this->m_id_map.right.end() && TARGET_ID_IT != this->m_id_map.right.end())
      {
        result.emplace_back(SOURCE_ID_IT->second, TARGET_ID_IT->second, edge);
      }
    }
  }

  return result;
}

std::vector<ObjectNode> SceneGraph::getObjectNodes() const noexcept
{
  std::vector<ObjectNode> result;

  for (const auto& [id, vertex_desc] : this->m_id_map.left)
  {
    const auto& node = this->m_graph[vertex_desc];
    if (const auto* object = std::get_if<ObjectNode>(&node))
    {
      result.push_back(*object);
    }
  }

  return result;
}

std::vector<ConceptNode> SceneGraph::getConceptNodes() const noexcept
{
  std::vector<ConceptNode> result;

  for (const auto& [id, vertex_desc] : this->m_id_map.left)
  {
    const auto& node = this->m_graph[vertex_desc];
    if (const auto* concept_node = std::get_if<ConceptNode>(&node))
    {
      result.push_back(*concept_node);
    }
  }

  return result;
}

std::vector<SceneGraphNode> SceneGraph::getNodes() const noexcept
{
  std::vector<SceneGraphNode> result;

  for (const auto& [id, vertex_desc] : this->m_id_map.left)
  {
    result.push_back(this->m_graph[vertex_desc]);
  }

  return result;
}

std::vector<std::tuple<int64_t, int64_t, SceneGraphEdge>> SceneGraph::getEdges() const noexcept
{
  std::vector<std::tuple<int64_t, int64_t, SceneGraphEdge>> result;

  using EdgeIter = boost::graph_traits<SceneGraphAdjList>::edge_iterator;
  EdgeIter ei;
  EdgeIter ei_end;
  boost::tie(ei, ei_end) = boost::edges(this->m_graph);

  for (; ei != ei_end; ++ei)
  {
    const auto& edge          = this->m_graph[*ei];
    auto* const SOURCE_VERTEX = boost::source(*ei, this->m_graph);
    auto* const TARGET_VERTEX = boost::target(*ei, this->m_graph);

    const auto SOURCE_ID_IT = this->m_id_map.right.find(SOURCE_VERTEX);
    const auto TARGET_ID_IT = this->m_id_map.right.find(TARGET_VERTEX);

    if (SOURCE_ID_IT != this->m_id_map.right.end() && TARGET_ID_IT != this->m_id_map.right.end())
    {
      result.emplace_back(SOURCE_ID_IT->second, TARGET_ID_IT->second, edge);
    }
  }

  return result;
}

std::expected<std::unique_ptr<SceneGraph>, SceneGraphError>
SceneGraph::fromMsg(const semantic_scene_forests_msgs::msg::SceneGraph& msg) noexcept
{
  auto graph = std::make_unique<SceneGraph>();

  for (const auto& node_msg : msg.object_nodes)
  {
    auto detection = std::make_shared<vision_msgs::msg::Detection3D>();
    std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals{};

    auto result = graph->addObjectNodeWithId(node_msg.id, "", detection, visuals);

    if (!result)
    {
      return std::unexpected(result.error());
    }
  }

  for (const auto& node_msg : msg.concept_nodes)
  {
    if (auto result = graph->addConceptNodeWithId(node_msg.id, node_msg.class_name); !result)
    {
      return std::unexpected(result.error());
    }
  }

  for (const auto& edge_msg : msg.edges)
  {
    if (auto result = graph->addEdge(edge_msg.source_id, edge_msg.target_id, edge_msg.label);
        !result)
    {
      return std::unexpected(result.error());
    }
  }

  return graph;
}

std::unique_ptr<semantic_scene_forests_msgs::msg::SceneGraph> SceneGraph::toMsg() noexcept
{
  auto msg = std::make_unique<semantic_scene_forests_msgs::msg::SceneGraph>();

  for (const auto& [id, vertex_desc] : this->m_id_map.left)
  {
    const auto& node = this->m_graph[vertex_desc];
    if (const auto* object = std::get_if<ObjectNode>(&node))
    {
      semantic_scene_forests_msgs::msg::ObjectNode object_node_msg;
      object_node_msg.id = object->id;

      msg->object_nodes.push_back(object_node_msg);
    }
    else if (const auto* concept_node = std::get_if<ConceptNode>(&node))
    {
      semantic_scene_forests_msgs::msg::ConceptNode concept_node_msg;
      concept_node_msg.id         = concept_node->id;
      concept_node_msg.class_name = concept_node->concept_name;
      msg->concept_nodes.push_back(concept_node_msg);
    }
  }

  using EdgeIter = boost::graph_traits<SceneGraphAdjList>::edge_iterator;
  EdgeIter ei;
  EdgeIter ei_end;
  boost::tie(ei, ei_end) = boost::edges(this->m_graph);

  for (; ei != ei_end; ++ei)
  {
    const auto& edge          = this->m_graph[*ei];
    auto* const SOURCE_VERTEX = boost::source(*ei, this->m_graph);
    auto* const TARGET_VERTEX = boost::target(*ei, this->m_graph);

    const auto SOURCE_ID_IT = this->m_id_map.right.find(SOURCE_VERTEX);
    const auto TARGET_ID_IT = this->m_id_map.right.find(TARGET_VERTEX);

    if (SOURCE_ID_IT != this->m_id_map.right.end() && TARGET_ID_IT != this->m_id_map.right.end())
    {
      semantic_scene_forests_msgs::msg::Edge edge_msg;
      edge_msg.source_id = SOURCE_ID_IT->second;
      edge_msg.target_id = TARGET_ID_IT->second;
      edge_msg.label     = edge.label;
      msg->edges.push_back(edge_msg);
    }
  }

  return msg;
}

std::expected<std::unique_ptr<SceneGraph>, SceneGraphError> SceneGraph::clone() const
{
  auto result = std::make_unique<SceneGraph>();

  for (const auto& [id, vertex_desc] : this->m_id_map.left)
  {
    const auto& node = this->m_graph[vertex_desc];
    if (const auto* object = std::get_if<ObjectNode>(&node))
    {
      vision_msgs::msg::Detection3D::SharedPtr detection;
      if (object->detection)
      {
        detection = std::make_shared<vision_msgs::msg::Detection3D>(*object->detection);
      }

      std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals;
      for (std::size_t i = 0; i < object->visuals.size(); ++i)
      {
        if (object->visuals[i])
        {
          visuals[i] = std::make_shared<sensor_msgs::msg::Image>(*object->visuals[i]);
        }
      }

      if (auto add_result = result->addObjectNodeWithId(id, object->class_name, detection, visuals);
          !add_result)
      {
        return std::unexpected(add_result.error());
      }
    }
    else if (const auto* concept_node = std::get_if<ConceptNode>(&node))
    {
      if (auto add_result = result->addConceptNodeWithId(id, concept_node->concept_name);
          !add_result)
      {
        return std::unexpected(add_result.error());
      }
    }
  }

  for (const auto& [source_id, target_id, edge] : getEdges())
  {
    if (auto add_result = result->addEdge(source_id, target_id, edge.label); !add_result)
    {
      return std::unexpected(add_result.error());
    }
  }

  result->m_next_object_id  = m_next_object_id;
  result->m_next_concept_id = m_next_concept_id;
  return result;
}

void SceneGraph::swap(SceneGraph& other) noexcept
{
  m_id_map.swap(other.m_id_map);
  m_graph.swap(other.m_graph);
  std::swap(m_next_object_id, other.m_next_object_id);
  std::swap(m_next_concept_id, other.m_next_concept_id);
}

std::expected<void, SceneGraphError> SceneGraph::updateObjectNodeFromMsg(
  const semantic_scene_forests_msgs::msg::ObjectNode& object_msg) noexcept
{
  auto detection = std::make_shared<vision_msgs::msg::Detection3D>();
  std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals{};

  if (this->m_id_map.left.count(object_msg.id) > 0)
  {
    ObjectNode object_node{
      .id = object_msg.id, .class_name = "", .detection = detection, .visuals = visuals};
    SceneGraphNode node(std::in_place_type<ObjectNode>, std::move(object_node));

    if (const auto IT = this->m_id_map.left.find(object_msg.id); IT != this->m_id_map.left.end())
    {
      this->m_graph[IT->second] = std::move(node);
    }
  }
  else
  {
    if (auto result = addObjectNodeWithId(object_msg.id, "", detection, visuals); !result)
    {
      return std::unexpected(result.error());
    }
  }
  return {};
}

std::expected<void, SceneGraphError> SceneGraph::updateConceptNodeFromMsg(
  const semantic_scene_forests_msgs::msg::ConceptNode& concept_msg) noexcept
{
  if (this->m_id_map.left.count(concept_msg.id) > 0)
  {
    ConceptNode concept_node{.id = concept_msg.id, .concept_name = concept_msg.class_name};
    SceneGraphNode node(std::in_place_type<ConceptNode>, std::move(concept_node));

    if (const auto IT = this->m_id_map.left.find(concept_msg.id); IT != this->m_id_map.left.end())
    {
      this->m_graph[IT->second] = std::move(node);
    }
  }
  else
  {
    if (auto result = addConceptNodeWithId(concept_msg.id, concept_msg.class_name); !result)
    {
      return std::unexpected(result.error());
    }
  }
  return {};
}

std::expected<void, SceneGraphError> SceneGraph::updateSceneGraphFromMsg(
  const semantic_scene_forests_msgs::msg::SceneGraph& msg) noexcept
{
  std::unordered_set<int64_t> valid_node_ids;

  for (const auto& object_msg : msg.object_nodes)
  {
    valid_node_ids.insert(object_msg.id);
    if (auto result = updateObjectNodeFromMsg(object_msg); !result)
    {
      return result;
    }
  }

  for (const auto& concept_msg : msg.concept_nodes)
  {
    valid_node_ids.insert(concept_msg.id);
    if (auto result = updateConceptNodeFromMsg(concept_msg); !result)
    {
      return result;
    }
  }

  std::vector<int64_t> to_remove;
  for (const auto& [id, vertex_desc] : this->m_id_map.left)
  {
    if (!valid_node_ids.contains(id))
    {
      to_remove.push_back(id);
    }
  }

  for (const int64_t ID : to_remove)
  {
    if (auto result = removeNode(ID); !result)
    {
      return std::unexpected(result.error());
    }
  }

  for (const auto& edge_msg : msg.edges)
  {
    if (!hasEdge(edge_msg.source_id, edge_msg.target_id, edge_msg.label))
    {
      if (auto result = addEdge(edge_msg.source_id, edge_msg.target_id, edge_msg.label); !result)
      {
        return std::unexpected(result.error());
      }
    }
  }

  return {};
}

} // namespace fzi::semantic_scene_forests
