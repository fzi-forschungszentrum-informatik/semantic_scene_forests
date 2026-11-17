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
    case SceneGraphErrorKind::INVALID_LABEL:
      kind = GraphErrorKind::InvalidLabel;
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

geometry_msgs::msg::Pose toPose(const PoseRecord& record)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x    = record.position[0];
  pose.position.y    = record.position[1];
  pose.position.z    = record.position[2];
  pose.orientation.x = record.orientation[0];
  pose.orientation.y = record.orientation[1];
  pose.orientation.z = record.orientation[2];
  pose.orientation.w = record.orientation[3];
  return pose;
}

PoseRecord fromPose(const geometry_msgs::msg::Pose& pose)
{
  return {.position    = {pose.position.x, pose.position.y, pose.position.z},
          .orientation = {
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w}};
}

vision_msgs::msg::BoundingBox3D toBBox(const BBoxRecord& record)
{
  vision_msgs::msg::BoundingBox3D bbox;
  bbox.center = toPose(record.center);
  bbox.size.x = record.size[0];
  bbox.size.y = record.size[1];
  bbox.size.z = record.size[2];
  return bbox;
}

BBoxRecord fromBBox(const vision_msgs::msg::BoundingBox3D& bbox)
{
  return {.center = fromPose(bbox.center), .size = {bbox.size.x, bbox.size.y, bbox.size.z}};
}

builtin_interfaces::msg::Time toStamp(const StampRecord& record)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec     = record.sec;
  stamp.nanosec = record.nanosec;
  return stamp;
}

StampRecord fromStamp(const builtin_interfaces::msg::Time& stamp)
{
  return {.sec = stamp.sec, .nanosec = stamp.nanosec};
}

semantic_scene_forests_msgs::msg::SceneGraph toMsg(const GraphRecord& record)
{
  semantic_scene_forests_msgs::msg::SceneGraph message;
  message.header = toHeader(record.header);
  for (const auto& node : record.objects)
  {
    semantic_scene_forests_msgs::msg::ObjectNode object;
    object.header       = toHeader(node.header);
    object.id           = node.id;
    object.class_name   = std::string(node.class_name);
    object.bbox         = toBBox(node.bbox);
    object.source       = node.source;
    object.last_updated = toStamp(node.last_updated);
    object.last_seen    = toStamp(node.last_seen);
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

int64_t NativeGraph::allocateId()
{
  return m_graph->allocateId();
}

Status NativeGraph::addObject(const ObjectNodeRecord& node)
{
  fzi::semantic_scene_forests::ObjectNode object{.id           = node.id,
                                                 .class_name   = std::string(node.class_name),
                                                 .bbox         = toBBox(node.bbox),
                                                 .frame_id     = std::string(node.frame_id),
                                                 .source       = node.source,
                                                 .last_seen    = toStamp(node.last_seen),
                                                 .last_updated = toStamp(node.last_updated)};
  for (const auto& crop : node.crops)
  {
    auto image          = std::make_shared<sensor_msgs::msg::CompressedImage>();
    image->header.stamp = toStamp(crop.stamp);
    image->format       = "jpeg";
    image->data.assign(crop.jpeg.begin(), crop.jpeg.end());
    object.crops.push_back(std::move(image));
  }
  for (const auto& worker : node.workers)
  {
    object.workers[std::string(worker.name)] = {.processed = worker.processed};
  }
  const auto RESULT = m_graph->addObject(std::move(object));
  return RESULT ? success() : failure(RESULT.error());
}

rust::Vec<ObjectNodeRecord> NativeGraph::objectNodes() const
{
  rust::Vec<ObjectNodeRecord> objects;
  for (const auto& node : m_graph->getObjectNodes())
  {
    ObjectNodeRecord record{.id           = node.id,
                            .class_name   = node.class_name,
                            .bbox         = fromBBox(node.bbox),
                            .frame_id     = node.frame_id,
                            .source       = node.source,
                            .last_seen    = fromStamp(node.last_seen),
                            .last_updated = fromStamp(node.last_updated),
                            .crops        = {},
                            .workers      = {}};
    for (const auto& crop : node.crops)
    {
      CropRecord crop_record{.stamp = fromStamp(crop->header.stamp), .jpeg = {}};
      crop_record.jpeg.reserve(crop->data.size());
      for (const auto BYTE : crop->data)
      {
        crop_record.jpeg.push_back(BYTE);
      }
      record.crops.push_back(std::move(crop_record));
    }
    for (const auto& [name, state] : node.workers)
    {
      record.workers.push_back({.name = name, .processed = state.processed});
    }
    objects.push_back(std::move(record));
  }
  return objects;
}

Status NativeGraph::copyFrom(const NativeGraph& other)
{
  auto result = other.m_graph->clone();
  if (!result)
  {
    return failure(result.error());
  }
  m_graph = std::move(*result);
  return success();
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
    record.objects.push_back({.header       = fromHeader(node.header),
                              .id           = node.id,
                              .class_name   = node.class_name,
                              .bbox         = fromBBox(node.bbox),
                              .source       = node.source,
                              .last_updated = fromStamp(node.last_updated),
                              .last_seen    = fromStamp(node.last_seen)});
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
