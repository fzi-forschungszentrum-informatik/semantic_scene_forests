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

std_msgs::msg::Header toHeader(const HeaderRecord& record)
{
  std_msgs::msg::Header header;
  header.stamp.sec     = record.sec;
  header.stamp.nanosec = record.nanosec;
  header.frame_id      = std::string(record.frame_id);
  return header;
}

HeaderRecord fromHeader(const std_msgs::msg::Header& header)
{
  return {.sec = header.stamp.sec, .nanosec = header.stamp.nanosec, .frame_id = header.frame_id};
}

semantic_scene_forests_msgs::msg::SceneGraph toMsg(const GraphRecord& record)
{
  semantic_scene_forests_msgs::msg::SceneGraph message;
  message.header = toHeader(record.header);
  for (const auto& node : record.objects)
  {
    semantic_scene_forests_msgs::msg::ObjectNode object;
    object.header = toHeader(node.header);
    object.id     = node.id;
    message.object_nodes.push_back(std::move(object));
  }
  for (const auto& node : record.concepts)
  {
    semantic_scene_forests_msgs::msg::ConceptNode concept_node;
    concept_node.header     = toHeader(node.header);
    concept_node.id         = node.id;
    concept_node.class_name = std::string(node.class_name);
    message.concept_nodes.push_back(std::move(concept_node));
  }
  for (const auto& edge : record.edges)
  {
    semantic_scene_forests_msgs::msg::Edge edge_message;
    edge_message.source_id = edge.source;
    edge_message.target_id = edge.target;
    edge_message.label     = std::string(edge.label);
    message.edges.push_back(std::move(edge_message));
  }
  return message;
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

Status NativeGraph::loadMessage(const GraphRecord& message)
{
  auto result = fzi::semantic_scene_forests::SceneGraph::fromMsg(toMsg(message));
  if (!result)
  {
    return failure(result.error());
  }
  m_graph = std::move(*result);
  return success();
}

GraphRecord NativeGraph::toMessage()
{
  const auto MESSAGE = m_graph->toMsg();
  GraphRecord record{
    .header = fromHeader(MESSAGE->header), .objects = {}, .concepts = {}, .edges = {}};
  for (const auto& node : MESSAGE->object_nodes)
  {
    record.objects.push_back({.header = fromHeader(node.header), .id = node.id});
  }
  for (const auto& node : MESSAGE->concept_nodes)
  {
    record.concepts.push_back(
      {.header = fromHeader(node.header), .id = node.id, .class_name = node.class_name});
  }
  for (const auto& edge : MESSAGE->edges)
  {
    record.edges.push_back(
      {.source = edge.source_id, .target = edge.target_id, .label = edge.label});
  }
  return record;
}

Status NativeGraph::updateFromMessage(const GraphRecord& message)
{
  const auto RESULT = m_graph->updateSceneGraphFromMsg(toMsg(message));
  return RESULT ? success() : failure(RESULT.error());
}
} // namespace ssf_rs
