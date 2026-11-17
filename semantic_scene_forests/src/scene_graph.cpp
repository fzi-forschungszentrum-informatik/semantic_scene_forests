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
#include <limits>
#include <mutex>
#include <set>
#include <tuple>
#include <unordered_set>
#include <utility>

#include <semantic_scene_forests/scene_graph.hpp>

#include <ranges>

#include <boost/graph/breadth_first_search.hpp>

namespace fzi::semantic_scene_forests {

namespace {

std::chrono::nanoseconds toDuration(const builtin_interfaces::msg::Time& stamp)
{
  return std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nanosec);
}

std::chrono::nanoseconds cropStamp(const CropPtr& crop)
{
  return toDuration(crop->header.stamp);
}

//! Label rules ported from validateConceptLabel (generation) and conceptLabel (manager).
std::expected<std::string, SceneGraphError> conceptLabel(const std::string& raw)
{
  const auto FIRST = raw.find_first_not_of(" \t");
  if (FIRST == std::string::npos)
  {
    return std::unexpected(SceneGraphError::invalidLabel("must not be empty"));
  }
  std::string label = raw.substr(FIRST, raw.find_last_not_of(" \t") - FIRST + 1);
  if (label.size() > 128U)
  {
    return std::unexpected(SceneGraphError::invalidLabel("must not exceed 128 bytes"));
  }
  if (std::ranges::any_of(
        label, [](unsigned char character) { return character < 0x20U || character == 0x7FU; }))
  {
    return std::unexpected(SceneGraphError::invalidLabel("must not contain control characters"));
  }
  return label;
}

} // namespace

SceneGraph::SceneGraph() = default;

SceneGraph::~SceneGraph() = default;

// Both go through swap(), which locks both graphs and rebuilds their descriptor maps.
SceneGraph::SceneGraph(SceneGraph&& other) noexcept
  : SceneGraph()
{
  swap(other);
}

SceneGraph& SceneGraph::operator=(SceneGraph&& other) noexcept
{
  swap(other);
  return *this;
}

int64_t SceneGraph::allocateId() noexcept
{
  return m_next_id.fetch_add(1);
}

void SceneGraph::reserveId(const int64_t id) noexcept
{
  // INT64_MAX is the last allocatable id; anything beyond would wrap.
  if (id == std::numeric_limits<int64_t>::max())
  {
    return;
  }
  int64_t next = m_next_id.load();
  while (next <= id && !m_next_id.compare_exchange_weak(next, id + 1))
  {
  }
}

ObjectNode* SceneGraph::findObject(const int64_t id) noexcept
{
  const auto IT = m_id_map.left.find(id);
  if (IT == m_id_map.left.end())
  {
    return nullptr;
  }
  return std::get_if<ObjectNode>(&m_graph[IT->second]);
}

std::expected<void, SceneGraphError> SceneGraph::addObject(ObjectNode node) noexcept
{
  const std::unique_lock LOCK(m_mutex);
  return addObjectUnlocked(std::move(node));
}

std::expected<void, SceneGraphError> SceneGraph::addObjectUnlocked(ObjectNode node) noexcept
{
  if (m_id_map.left.count(node.id) > 0)
  {
    return std::unexpected(SceneGraphError::nodeAlreadyExists(node.id));
  }
  if (std::ranges::any_of(node.crops, [](const CropPtr& crop) { return !crop; }))
  {
    return std::unexpected(SceneGraphError::graphEditingFailed("null crop"));
  }
  reserveId(node.id);
  const int64_t ID = node.id;
  auto* const VERTEX =
    boost::add_vertex(SceneGraphNode(std::in_place_type<ObjectNode>, std::move(node)), m_graph);
  m_id_map.insert({ID, VERTEX});
  return {};
}

std::expected<void, SceneGraphError>
SceneGraph::updateObjectPose(const int64_t id,
                             const vision_msgs::msg::BoundingBox3D& bbox,
                             const builtin_interfaces::msg::Time& last_seen) noexcept
{
  const std::unique_lock LOCK(m_mutex);
  auto* object = findObject(id);
  if (object == nullptr)
  {
    return std::unexpected(SceneGraphError::nodeDoesNotExist(id));
  }
  if (object->bbox != bbox)
  {
    object->bbox         = bbox;
    object->last_updated = last_seen;
  }
  object->last_seen = last_seen;
  return {};
}

void SceneGraph::setCropPolicy(const std::size_t max_crops,
                               const std::chrono::nanoseconds min_interval) noexcept
{
  const std::unique_lock LOCK(m_mutex);
  m_max_crops         = max_crops;
  m_crop_min_interval = min_interval;
}

std::expected<bool, SceneGraphError> SceneGraph::offerCrop(const int64_t id,
                                                           const CropPtr& crop) noexcept
{
  const std::unique_lock LOCK(m_mutex);
  auto* object = findObject(id);
  if (object == nullptr)
  {
    return std::unexpected(SceneGraphError::nodeDoesNotExist(id));
  }
  if (!crop)
  {
    return std::unexpected(SceneGraphError::graphEditingFailed("null crop"));
  }

  auto& crops         = object->crops;
  const auto BY_STAMP = [](const CropPtr& a, const CropPtr& b) {
    return cropStamp(a) < cropStamp(b);
  };
  // The newest-crop gap is checked for appends and replacements alike; since the oldest crop is
  // never newer than the newest, it also covers the oldest-crop gap a replacement needs.
  if (!crops.empty() &&
      cropStamp(crop) < cropStamp(*std::ranges::max_element(crops, BY_STAMP)) + m_crop_min_interval)
  {
    return false;
  }
  if (crops.size() < m_max_crops)
  {
    crops.push_back(crop);
    return true;
  }
  if (crops.empty())
  {
    return false; // max_crops == 0
  }
  crops.erase(std::ranges::min_element(crops, BY_STAMP));
  crops.push_back(crop);
  return true;
}

std::expected<void, SceneGraphError>
SceneGraph::setObjectClass(const int64_t id, const std::string& class_name) noexcept
{
  const std::unique_lock LOCK(m_mutex);
  auto* object = findObject(id);
  if (object == nullptr)
  {
    return std::unexpected(SceneGraphError::nodeDoesNotExist(id));
  }
  object->class_name = class_name;
  return {};
}

std::expected<int64_t, SceneGraphError>
SceneGraph::addConceptNodeWithId(const int64_t id, const std::string& label) noexcept
{
  const std::unique_lock LOCK(m_mutex);
  return addConceptUnlocked(id, label);
}

std::expected<int64_t, SceneGraphError>
SceneGraph::addConceptNode(const std::string& label) noexcept
{
  const std::unique_lock LOCK(m_mutex);
  return addConceptUnlocked(allocateId(), label);
}

std::expected<int64_t, SceneGraphError>
SceneGraph::addConceptUnlocked(const int64_t id, const std::string& label) noexcept
{
  if (m_id_map.left.count(id) > 0)
  {
    return std::unexpected(SceneGraphError::nodeAlreadyExists(id));
  }
  reserveId(id);
  auto* const VERTEX = boost::add_vertex(
    SceneGraphNode(std::in_place_type<ConceptNode>, ConceptNode{.id = id, .concept_name = label}),
    m_graph);
  m_id_map.insert({id, VERTEX});
  return id;
}

std::expected<ConceptLookup, SceneGraphError>
SceneGraph::findOrAddConcept(const std::string& label) noexcept
{
  const std::unique_lock LOCK(m_mutex);
  return findOrAddConceptUnlocked(label);
}

std::expected<ConceptLookup, SceneGraphError>
SceneGraph::findOrAddConceptUnlocked(const std::string& label) noexcept
{
  auto clean = conceptLabel(label);
  if (!clean)
  {
    return std::unexpected(clean.error());
  }
  // The lowest id wins when several concepts share a label.
  for (const auto& [id, vertex_desc] : m_id_map.left)
  {
    if (const auto* concept_node = std::get_if<ConceptNode>(&m_graph[vertex_desc]);
        concept_node != nullptr && concept_node->concept_name == *clean)
    {
      return ConceptLookup{.id = id, .created = false};
    }
  }
  auto added = addConceptUnlocked(allocateId(), *clean);
  if (!added)
  {
    return std::unexpected(added.error());
  }
  return ConceptLookup{.id = *added, .created = true};
}

std::expected<ConceptLookup, SceneGraphError> SceneGraph::addConceptEdge(
  const int64_t source_id, const std::string& relation, const std::string& label) noexcept
{
  const std::unique_lock LOCK(m_mutex);
  auto concept_node = findOrAddConceptUnlocked(label);
  if (!concept_node)
  {
    return std::unexpected(concept_node.error());
  }
  if (auto edge = addEdgeUnlocked(source_id, concept_node->id, relation); !edge)
  {
    if (concept_node->created)
    {
      // A fresh concept has no edges, so removing it cannot fail or touch other data.
      (void)removeNodeUnlocked(concept_node->id);
    }
    return std::unexpected(edge.error());
  }
  return *concept_node;
}

std::expected<void, SceneGraphError> SceneGraph::markProcessed(const int64_t id,
                                                               const std::string& worker) noexcept
{
  const std::unique_lock LOCK(m_mutex);
  const auto IT = m_id_map.left.find(id);
  if (IT == m_id_map.left.end())
  {
    return std::unexpected(SceneGraphError::nodeDoesNotExist(id));
  }
  std::visit([&worker](auto& node) { node.workers[worker].processed = true; }, m_graph[IT->second]);
  return {};
}

std::vector<int64_t> SceneGraph::unprocessedBy(const std::string& worker) const noexcept
{
  const std::shared_lock LOCK(m_mutex);
  std::vector<int64_t> result;
  for (const auto& [id, vertex_desc] : m_id_map.left)
  {
    const bool PROCESSED = std::visit(
      [&worker](const auto& node) {
        const auto IT = node.workers.find(worker);
        return IT != node.workers.end() && IT->second.processed;
      },
      m_graph[vertex_desc]);
    if (!PROCESSED)
    {
      result.push_back(id);
    }
  }
  return result;
}

std::expected<SceneGraphNode, SceneGraphError> SceneGraph::removeNode(const int64_t id) noexcept
{
  const std::unique_lock LOCK(m_mutex);
  return removeNodeUnlocked(id);
}

std::expected<SceneGraphNode, SceneGraphError>
SceneGraph::removeNodeUnlocked(const int64_t id) noexcept
{
  const auto IT = m_id_map.left.find(id);
  if (IT == m_id_map.left.end())
  {
    return std::unexpected(SceneGraphError::nodeDoesNotExist(id));
  }

  auto* const VERTEX_DESC     = IT->second;
  SceneGraphNode removed_node = m_graph[VERTEX_DESC];

  m_id_map.left.erase(IT);
  boost::clear_vertex(VERTEX_DESC, m_graph);
  boost::remove_vertex(VERTEX_DESC, m_graph);

  return removed_node;
}

std::expected<std::vector<SceneGraphNode>, SceneGraphError>
SceneGraph::removeIsolatedNodes() noexcept
{
  const std::unique_lock LOCK(m_mutex);
  return removeIsolatedUnlocked(false);
}

std::vector<int64_t> SceneGraph::pruneIsolatedConcepts() noexcept
{
  const std::unique_lock LOCK(m_mutex);
  std::vector<int64_t> ids;
  for (const auto& node : removeIsolatedUnlocked(true))
  {
    ids.push_back(std::get<ConceptNode>(node).id);
  }
  return ids;
}

std::vector<SceneGraphNode> SceneGraph::removeIsolatedUnlocked(const bool concepts_only) noexcept
{
  std::unordered_set<SceneGraphAdjList::vertex_descriptor> with_incoming;
  for (auto [ei, ei_end] = boost::edges(m_graph); ei != ei_end; ++ei)
  {
    with_incoming.insert(boost::target(*ei, m_graph));
  }

  std::vector<int64_t> isolated_ids;
  for (const auto& [id, vertex_desc] : m_id_map.left)
  {
    if (boost::out_degree(vertex_desc, m_graph) == 0 && !with_incoming.contains(vertex_desc) &&
        (!concepts_only || std::holds_alternative<ConceptNode>(m_graph[vertex_desc])))
    {
      isolated_ids.push_back(id);
    }
  }

  std::vector<SceneGraphNode> removed_nodes;
  removed_nodes.reserve(isolated_ids.size());
  for (const int64_t ID : isolated_ids)
  {
    removed_nodes.push_back(*removeNodeUnlocked(ID)); // ids come from m_id_map
  }
  return removed_nodes;
}

std::optional<SceneGraphNode> SceneGraph::findNode(const int64_t id) const noexcept
{
  const std::shared_lock LOCK(m_mutex);
  const auto IT = m_id_map.left.find(id);
  if (IT == m_id_map.left.end())
  {
    return std::nullopt;
  }
  return m_graph[IT->second];
}

bool SceneGraph::hasNode(const int64_t id) const noexcept
{
  const std::shared_lock LOCK(m_mutex);
  return m_id_map.left.count(id) > 0;
}

std::expected<std::vector<SceneGraphEdge>, SceneGraphError>
SceneGraph::findEdges(const int64_t from, const int64_t to) const noexcept
{
  const std::shared_lock LOCK(m_mutex);
  return findEdgesUnlocked(from, to);
}

std::expected<std::vector<SceneGraphEdge>, SceneGraphError>
SceneGraph::findEdgesUnlocked(const int64_t from, const int64_t to) const noexcept
{
  const auto FROM_IT = m_id_map.left.find(from);
  if (FROM_IT == m_id_map.left.end())
  {
    return std::unexpected(SceneGraphError::nodeDoesNotExist(from));
  }

  const auto TO_IT = m_id_map.left.find(to);
  if (TO_IT == m_id_map.left.end())
  {
    return std::unexpected(SceneGraphError::nodeDoesNotExist(to));
  }

  std::vector<SceneGraphEdge> edges;
  for (auto [ei, ei_end] = boost::out_edges(FROM_IT->second, m_graph); ei != ei_end; ++ei)
  {
    if (boost::target(*ei, m_graph) == TO_IT->second)
    {
      edges.push_back(m_graph[*ei]);
    }
  }
  return edges;
}

bool SceneGraph::hasEdge(const int64_t from,
                         const int64_t to,
                         const std::string& label) const noexcept
{
  const std::shared_lock LOCK(m_mutex);
  const auto EDGES_RESULT = findEdgesUnlocked(from, to);
  return EDGES_RESULT && std::ranges::any_of(*EDGES_RESULT, [&label](const SceneGraphEdge& edge) {
           return edge.label == label;
         });
}

std::expected<SceneGraphEdge, SceneGraphError>
SceneGraph::addEdge(const int64_t from, const int64_t to, const std::string& label) noexcept
{
  const std::unique_lock LOCK(m_mutex);
  return addEdgeUnlocked(from, to, label);
}

std::expected<SceneGraphEdge, SceneGraphError>
SceneGraph::addEdgeUnlocked(const int64_t from, const int64_t to, const std::string& label) noexcept
{
  const auto FROM_IT = m_id_map.left.find(from);
  if (FROM_IT == m_id_map.left.end())
  {
    return std::unexpected(SceneGraphError::nodeDoesNotExist(from));
  }

  const auto TO_IT = m_id_map.left.find(to);
  if (TO_IT == m_id_map.left.end())
  {
    return std::unexpected(SceneGraphError::nodeDoesNotExist(to));
  }

  auto* const TARGET_VERTEX = TO_IT->second;

  if (std::holds_alternative<ObjectNode>(m_graph[TARGET_VERTEX]))
  {
    return std::unexpected(SceneGraphError::invalidEdge(from, to, label));
  }

  for (auto [ei, ei_end] = boost::out_edges(FROM_IT->second, m_graph); ei != ei_end; ++ei)
  {
    if (boost::target(*ei, m_graph) == TARGET_VERTEX && m_graph[*ei].label == label)
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
    boost::breadth_first_search(
      m_graph, TARGET_VERTEX, boost::visitor(CycleVisitor{SOURCE_VERTEX}).color_map(color_pmap));
  }
  catch (const CycleDetected&)
  {
    return std::unexpected(SceneGraphError::cycle(from, to));
  }

  SceneGraphEdge edge{.label = label};
  auto [added_edge, success] = boost::add_edge(SOURCE_VERTEX, TARGET_VERTEX, edge, m_graph);

  if (!success)
  {
    return std::unexpected(SceneGraphError::edgeDoesNotExist(from, to, label));
  }

  return m_graph[added_edge];
}

std::expected<SceneGraphEdge, SceneGraphError>
SceneGraph::removeEdge(const int64_t from, const int64_t to, const std::string& label) noexcept
{
  const std::unique_lock LOCK(m_mutex);
  return removeEdgeUnlocked(from, to, label);
}

std::expected<SceneGraphEdge, SceneGraphError> SceneGraph::removeEdgeUnlocked(
  const int64_t from, const int64_t to, const std::string& label) noexcept
{
  const auto FROM_IT = m_id_map.left.find(from);
  if (FROM_IT == m_id_map.left.end())
  {
    return std::unexpected(SceneGraphError::nodeDoesNotExist(from));
  }

  const auto TO_IT = m_id_map.left.find(to);
  if (TO_IT == m_id_map.left.end())
  {
    return std::unexpected(SceneGraphError::nodeDoesNotExist(to));
  }

  for (auto [ei, ei_end] = boost::out_edges(FROM_IT->second, m_graph); ei != ei_end; ++ei)
  {
    if (boost::target(*ei, m_graph) == TO_IT->second && m_graph[*ei].label == label)
    {
      SceneGraphEdge removed_edge = m_graph[*ei];
      boost::remove_edge(*ei, m_graph);
      return removed_edge;
    }
  }

  return std::unexpected(SceneGraphError::edgeDoesNotExist(from, to, label));
}

std::expected<std::vector<std::pair<int64_t, SceneGraphEdge>>, SceneGraphError>
SceneGraph::outgoingEdges(const int64_t id) const noexcept
{
  const std::shared_lock LOCK(m_mutex);
  const auto IT = m_id_map.left.find(id);
  if (IT == m_id_map.left.end())
  {
    return std::unexpected(SceneGraphError::nodeDoesNotExist(id));
  }

  std::vector<std::pair<int64_t, SceneGraphEdge>> edges;
  for (auto [ei, ei_end] = boost::out_edges(IT->second, m_graph); ei != ei_end; ++ei)
  {
    const auto TARGET_ID_IT = m_id_map.right.find(boost::target(*ei, m_graph));
    if (TARGET_ID_IT != m_id_map.right.end())
    {
      edges.emplace_back(TARGET_ID_IT->second, m_graph[*ei]);
    }
  }
  return edges;
}

std::expected<std::vector<std::pair<int64_t, SceneGraphEdge>>, SceneGraphError>
SceneGraph::incomingEdges(const int64_t id) const noexcept
{
  const std::shared_lock LOCK(m_mutex);
  const auto TARGET_IT = m_id_map.left.find(id);
  if (TARGET_IT == m_id_map.left.end())
  {
    return std::unexpected(SceneGraphError::nodeDoesNotExist(id));
  }

  std::vector<std::pair<int64_t, SceneGraphEdge>> edges;
  auto* const TARGET_VERTEX = TARGET_IT->second;

  for (const auto& [source_id, source_vertex] : m_id_map.left)
  {
    for (auto [ei, ei_end] = boost::out_edges(source_vertex, m_graph); ei != ei_end; ++ei)
    {
      if (boost::target(*ei, m_graph) == TARGET_VERTEX)
      {
        edges.emplace_back(source_id, m_graph[*ei]);
      }
    }
  }
  return edges;
}

std::vector<ConceptNode> SceneGraph::findConceptNodesByName(const std::string& name) const noexcept
{
  const std::shared_lock LOCK(m_mutex);
  std::vector<ConceptNode> result;
  for (const auto& [id, vertex_desc] : m_id_map.left)
  {
    if (const auto* concept_node = std::get_if<ConceptNode>(&m_graph[vertex_desc]);
        concept_node != nullptr && concept_node->concept_name == name)
    {
      result.push_back(*concept_node);
    }
  }
  return result;
}

std::vector<std::tuple<int64_t, int64_t, SceneGraphEdge>>
SceneGraph::findEdgesByLabel(const std::string& label) const noexcept
{
  const std::shared_lock LOCK(m_mutex);
  auto edges = getEdgesUnlocked();
  std::erase_if(edges, [&label](const auto& edge) { return std::get<2>(edge).label != label; });
  return edges;
}

std::vector<ObjectNode> SceneGraph::getObjectNodes() const noexcept
{
  const std::shared_lock LOCK(m_mutex);
  std::vector<ObjectNode> result;
  for (const auto& [id, vertex_desc] : m_id_map.left)
  {
    if (const auto* object = std::get_if<ObjectNode>(&m_graph[vertex_desc]))
    {
      result.push_back(*object);
    }
  }
  return result;
}

std::vector<ConceptNode> SceneGraph::getConceptNodes() const noexcept
{
  const std::shared_lock LOCK(m_mutex);
  std::vector<ConceptNode> result;
  for (const auto& [id, vertex_desc] : m_id_map.left)
  {
    if (const auto* concept_node = std::get_if<ConceptNode>(&m_graph[vertex_desc]))
    {
      result.push_back(*concept_node);
    }
  }
  return result;
}

std::vector<SceneGraphNode> SceneGraph::getNodes() const noexcept
{
  const std::shared_lock LOCK(m_mutex);
  std::vector<SceneGraphNode> result;
  for (const auto& [id, vertex_desc] : m_id_map.left)
  {
    result.push_back(m_graph[vertex_desc]);
  }
  return result;
}

std::vector<std::tuple<int64_t, int64_t, SceneGraphEdge>> SceneGraph::getEdges() const noexcept
{
  const std::shared_lock LOCK(m_mutex);
  return getEdgesUnlocked();
}

std::vector<std::tuple<int64_t, int64_t, SceneGraphEdge>>
SceneGraph::getEdgesUnlocked() const noexcept
{
  std::vector<std::tuple<int64_t, int64_t, SceneGraphEdge>> result;
  for (auto [ei, ei_end] = boost::edges(m_graph); ei != ei_end; ++ei)
  {
    const auto SOURCE_ID_IT = m_id_map.right.find(boost::source(*ei, m_graph));
    const auto TARGET_ID_IT = m_id_map.right.find(boost::target(*ei, m_graph));

    if (SOURCE_ID_IT != m_id_map.right.end() && TARGET_ID_IT != m_id_map.right.end())
    {
      result.emplace_back(SOURCE_ID_IT->second, TARGET_ID_IT->second, m_graph[*ei]);
    }
  }
  return result;
}

namespace {

ObjectNode objectFromMsg(const semantic_scene_forests_msgs::msg::ObjectNode& msg)
{
  return ObjectNode{.id           = msg.id,
                    .class_name   = msg.class_name,
                    .bbox         = msg.bbox,
                    .frame_id     = msg.header.frame_id,
                    .source       = msg.source,
                    .last_seen    = msg.last_seen,
                    .last_updated = msg.last_updated};
}

} // namespace

std::expected<std::unique_ptr<SceneGraph>, SceneGraphError>
SceneGraph::fromMsg(const semantic_scene_forests_msgs::msg::SceneGraph& msg) noexcept
{
  // The graph is not shared yet, so the unlocked helpers are safe.
  auto graph = std::make_unique<SceneGraph>();

  for (const auto& node_msg : msg.object_nodes)
  {
    if (auto result = graph->addObjectUnlocked(objectFromMsg(node_msg)); !result)
    {
      return std::unexpected(result.error());
    }
  }

  for (const auto& node_msg : msg.concept_nodes)
  {
    if (auto result = graph->addConceptUnlocked(node_msg.id, node_msg.class_name); !result)
    {
      return std::unexpected(result.error());
    }
  }

  for (const auto& edge_msg : msg.edges)
  {
    if (auto result =
          graph->addEdgeUnlocked(edge_msg.source_id, edge_msg.target_id, edge_msg.label);
        !result)
    {
      return std::unexpected(result.error());
    }
  }

  return graph;
}

std::unique_ptr<semantic_scene_forests_msgs::msg::SceneGraph> SceneGraph::toMsg() const noexcept
{
  const std::shared_lock LOCK(m_mutex);
  auto msg = std::make_unique<semantic_scene_forests_msgs::msg::SceneGraph>();

  for (const auto& [id, vertex_desc] : m_id_map.left)
  {
    const auto& node = m_graph[vertex_desc];
    if (const auto* object = std::get_if<ObjectNode>(&node))
    {
      semantic_scene_forests_msgs::msg::ObjectNode object_node_msg;
      object_node_msg.header.frame_id = object->frame_id;
      object_node_msg.header.stamp    = object->last_seen;
      object_node_msg.id              = object->id;
      object_node_msg.class_name      = object->class_name;
      object_node_msg.bbox            = object->bbox;
      object_node_msg.source          = object->source;
      object_node_msg.last_seen       = object->last_seen;
      object_node_msg.last_updated    = object->last_updated;
      msg->object_nodes.push_back(std::move(object_node_msg));
    }
    else if (const auto* concept_node = std::get_if<ConceptNode>(&node))
    {
      semantic_scene_forests_msgs::msg::ConceptNode concept_node_msg;
      concept_node_msg.id         = concept_node->id;
      concept_node_msg.class_name = concept_node->concept_name;
      msg->concept_nodes.push_back(std::move(concept_node_msg));
    }
  }

  for (const auto& [source_id, target_id, edge] : getEdgesUnlocked())
  {
    semantic_scene_forests_msgs::msg::Edge edge_msg;
    edge_msg.source_id = source_id;
    edge_msg.target_id = target_id;
    edge_msg.label     = edge.label;
    msg->edges.push_back(std::move(edge_msg));
  }

  return msg;
}

std::expected<std::unique_ptr<SceneGraph>, SceneGraphError> SceneGraph::clone() const
{
  const std::shared_lock LOCK(m_mutex);
  auto result = std::make_unique<SceneGraph>();

  // Nodes are copied whole; crops stay shared (they are immutable).
  for (const auto& [id, vertex_desc] : m_id_map.left)
  {
    result->m_id_map.insert({id, boost::add_vertex(m_graph[vertex_desc], result->m_graph)});
  }

  for (const auto& [source_id, target_id, edge] : getEdgesUnlocked())
  {
    if (auto add_result = result->addEdgeUnlocked(source_id, target_id, edge.label); !add_result)
    {
      return std::unexpected(add_result.error());
    }
  }

  result->m_next_id           = m_next_id.load();
  result->m_max_crops         = m_max_crops;
  result->m_crop_min_interval = m_crop_min_interval;
  return result;
}

void SceneGraph::swap(SceneGraph& other) noexcept
{
  if (this == &other)
  {
    return;
  }
  const std::scoped_lock LOCK(m_mutex, other.m_mutex);

  m_id_map.swap(other.m_id_map);
  m_graph.swap(other.m_graph);
  const int64_t NEXT_ID = m_next_id.load();
  m_next_id             = other.m_next_id.load();
  other.m_next_id       = NEXT_ID;
  std::swap(m_max_crops, other.m_max_crops);
  std::swap(m_crop_min_interval, other.m_crop_min_interval);

  // boost::adjacency_list::swap copies: every vertex descriptor is recreated,
  // so both id maps must be rebuilt from the (swapped) vertex properties.
  auto remap = [](SceneGraph& graph) {
    graph.m_id_map.clear();
    for (auto [it, end] = boost::vertices(graph.m_graph); it != end; ++it)
    {
      const auto ID = std::visit([](const auto& node) { return node.id; }, graph.m_graph[*it]);
      graph.m_id_map.insert({ID, *it});
    }
  };
  remap(*this);
  remap(other);
}

std::expected<void, SceneGraphError> SceneGraph::updateObjectNodeFromMsg(
  const semantic_scene_forests_msgs::msg::ObjectNode& object_msg) noexcept
{
  const auto IT = m_id_map.left.find(object_msg.id);
  if (IT == m_id_map.left.end())
  {
    return addObjectUnlocked(objectFromMsg(object_msg));
  }

  // An existing node of the wrong kind is a collision the caller must see.
  auto* object = std::get_if<ObjectNode>(&m_graph[IT->second]);
  if (object == nullptr)
  {
    return std::unexpected(SceneGraphError::nodeAlreadyExists(object_msg.id));
  }

  // Crops and worker state are not in the message; keep them.
  auto updated    = objectFromMsg(object_msg);
  updated.crops   = std::move(object->crops);
  updated.workers = std::move(object->workers);
  *object         = std::move(updated);
  return {};
}

std::expected<void, SceneGraphError> SceneGraph::updateConceptNodeFromMsg(
  const semantic_scene_forests_msgs::msg::ConceptNode& concept_msg) noexcept
{
  const auto IT = m_id_map.left.find(concept_msg.id);
  if (IT == m_id_map.left.end())
  {
    if (auto result = addConceptUnlocked(concept_msg.id, concept_msg.class_name); !result)
    {
      return std::unexpected(result.error());
    }
    return {};
  }

  // An existing node of the wrong kind is a collision the caller must see.
  auto* concept_node = std::get_if<ConceptNode>(&m_graph[IT->second]);
  if (concept_node == nullptr)
  {
    return std::unexpected(SceneGraphError::nodeAlreadyExists(concept_msg.id));
  }
  concept_node->concept_name = concept_msg.class_name;
  return {};
}

std::expected<void, SceneGraphError> SceneGraph::updateSceneGraphFromMsg(
  const semantic_scene_forests_msgs::msg::SceneGraph& msg) noexcept
{
  const std::unique_lock LOCK(m_mutex);
  // The live graph is edited in place: every operation that succeeds before an
  // invalid one keeps its effect, and only the offending operation is rejected
  // (its low-level check runs before any mutation).
  for (const auto& object_msg : msg.object_nodes)
  {
    if (auto result = updateObjectNodeFromMsg(object_msg); !result)
    {
      return result;
    }
  }
  for (const auto& concept_msg : msg.concept_nodes)
  {
    if (auto result = updateConceptNodeFromMsg(concept_msg); !result)
    {
      return result;
    }
  }

  // Nodes absent from the message disappear; explicitly supplied isolated
  // concepts stay because they are listed.
  std::unordered_set<int64_t> message_node_ids;
  for (const auto& object_msg : msg.object_nodes)
  {
    message_node_ids.insert(object_msg.id);
  }
  for (const auto& concept_msg : msg.concept_nodes)
  {
    message_node_ids.insert(concept_msg.id);
  }

  std::vector<int64_t> nodes_to_remove;
  for (const auto& [id, vertex_desc] : m_id_map.left)
  {
    if (!message_node_ids.contains(id))
    {
      nodes_to_remove.push_back(id);
    }
  }
  for (const int64_t ID : nodes_to_remove)
  {
    if (auto result = removeNodeUnlocked(ID); !result)
    {
      return std::unexpected(result.error());
    }
  }

  return reconcileEdgesFromMsg(msg);
}

std::expected<void, SceneGraphError>
SceneGraph::reconcileEdgesFromMsg(const semantic_scene_forests_msgs::msg::SceneGraph& msg) noexcept
{
  const auto CURRENT_EDGES = getEdgesUnlocked();

  std::set<std::tuple<int64_t, int64_t, std::string>> current_edge_set;
  std::set<std::tuple<int64_t, int64_t, std::string>> message_edges;
  for (const auto& [source_id, target_id, edge] : CURRENT_EDGES)
  {
    current_edge_set.emplace(source_id, target_id, edge.label);
  }
  for (const auto& edge_msg : msg.edges)
  {
    message_edges.emplace(edge_msg.source_id, edge_msg.target_id, edge_msg.label);
  }

  for (const auto& [source_id, target_id, edge] : CURRENT_EDGES)
  {
    if (!message_edges.contains({source_id, target_id, edge.label}))
    {
      if (auto result = removeEdgeUnlocked(source_id, target_id, edge.label); !result)
      {
        return std::unexpected(result.error());
      }
    }
  }

  // Add only edges the graph did not already hold before this update;
  // addEdge rejects an invalid edge before mutating anything.
  for (const auto& edge_msg : msg.edges)
  {
    if (!current_edge_set.contains({edge_msg.source_id, edge_msg.target_id, edge_msg.label}))
    {
      if (auto result = addEdgeUnlocked(edge_msg.source_id, edge_msg.target_id, edge_msg.label);
          !result)
      {
        return std::unexpected(result.error());
      }
    }
  }
  return {};
}

std::expected<void, SceneGraphError>
SceneGraph::retainObjectNodes(const std::unordered_set<int64_t>& active_ids) noexcept
{
  const std::unique_lock LOCK(m_mutex);
  std::unordered_set<int64_t> keep;
  std::vector<SceneGraphAdjList::vertex_descriptor> pending;

  for (const int64_t ID : active_ids)
  {
    const auto IT = m_id_map.left.find(ID);
    if (IT != m_id_map.left.end() && std::holds_alternative<ObjectNode>(m_graph[IT->second]))
    {
      keep.insert(ID);
      pending.push_back(IT->second);
    }
  }

  // Follow outgoing edges transitively: shared concepts and concept ancestry
  // reachable from a surviving object stay; everything else goes.
  while (!pending.empty())
  {
    auto* const VERTEX_DESC = pending.back();
    pending.pop_back();

    for (auto [ei, ei_end] = boost::out_edges(VERTEX_DESC, m_graph); ei != ei_end; ++ei)
    {
      auto* const TARGET_VERTEX = boost::target(*ei, m_graph);
      const auto TARGET_ID_IT   = m_id_map.right.find(TARGET_VERTEX);
      if (TARGET_ID_IT != m_id_map.right.end() && !keep.contains(TARGET_ID_IT->second))
      {
        keep.insert(TARGET_ID_IT->second);
        pending.push_back(TARGET_VERTEX);
      }
    }
  }

  std::vector<int64_t> to_remove;
  for (const auto& [id, vertex_desc] : m_id_map.left)
  {
    if (!keep.contains(id))
    {
      to_remove.push_back(id);
    }
  }

  for (const int64_t ID : to_remove)
  {
    if (auto result = removeNodeUnlocked(ID); !result)
    {
      return std::unexpected(result.error());
    }
  }

  return {};
}

} // namespace fzi::semantic_scene_forests
