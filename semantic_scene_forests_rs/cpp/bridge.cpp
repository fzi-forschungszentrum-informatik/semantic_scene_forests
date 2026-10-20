// Copyright (c) 2026 FZI Forschungszentrum Informatik
// SPDX-License-Identifier: BSD-3-Clause
#include "semantic_scene_forests_rs/src/ffi.rs.h"
#include <semantic_scene_forests/scene_graph.hpp>

namespace ssf_rs {
namespace {
Status success()
{
  return {.success = true, .kind = GraphErrorKind::GraphEditingFailed, .message = {}};
}

Status failure(const SceneGraphError& error)
{
  GraphErrorKind kind{};
  switch (error.kind)
  {
    case SceneGraphErrorKind::NODE_ALREADY_EXISTS:
      kind = GraphErrorKind::NodeAlreadyExists;
      break;
    case SceneGraphErrorKind::NODE_DOES_NOT_EXIST:
      kind = GraphErrorKind::NodeDoesNotExist;
      break;
    case SceneGraphErrorKind::EDGE_DOES_NOT_EXIST:
      kind = GraphErrorKind::EdgeDoesNotExist;
      break;
    case SceneGraphErrorKind::INVALID_EDGE:
      kind = GraphErrorKind::InvalidEdge;
      break;
    case SceneGraphErrorKind::EDGE_ALREADY_EXISTS:
      kind = GraphErrorKind::EdgeAlreadyExists;
      break;
    case SceneGraphErrorKind::CYCLE:
      kind = GraphErrorKind::Cycle;
      break;
    case SceneGraphErrorKind::GRAPH_EDITING_FAILED:
      kind = GraphErrorKind::GraphEditingFailed;
      break;
  }
  return {.success = false, .kind = kind, .message = error.message};
}

IdResult nodeResult(const std::expected<int64_t, SceneGraphError>& result)
{
  return result ? IdResult{.status = success(), .id = *result}
                : IdResult{.status = failure(result.error()), .id = 0};
}
} // namespace

NativeGraph::NativeGraph()
  : m_graph(std::make_unique<fzi::semantic_scene_forests::SceneGraph>())
{
}
NativeGraph::~NativeGraph() = default;

std::unique_ptr<NativeGraph> newGraph()
{
  return std::make_unique<NativeGraph>();
}

IdResult NativeGraph::addObjectNode(rust::Str name)
{
  return nodeResult(m_graph->addObjectNode(std::string(name), nullptr, {}));
}

IdResult NativeGraph::addConceptNode(rust::Str name)
{
  return nodeResult(m_graph->addConceptNode(std::string(name)));
}

bool NativeGraph::hasNode(int64_t id) const
{
  return m_graph->hasNode(id);
}

Status NativeGraph::addEdge(int64_t source, int64_t target, rust::Str label)
{
  const auto RESULT = m_graph->addEdge(source, target, std::string(label));
  return RESULT ? success() : failure(RESULT.error());
}

EdgeList NativeGraph::outgoingEdges(int64_t id) const
{
  const auto RESULT = m_graph->outgoingEdges(id);
  if (!RESULT)
  {
    return {.status = failure(RESULT.error()), .edges = {}};
  }
  EdgeList edges{.status = success(), .edges = {}};
  for (const auto& [target, edge] : *RESULT)
  {
    edges.edges.push_back({.source = id, .target = target, .label = edge.label});
  }
  return edges;
}
} // namespace ssf_rs
