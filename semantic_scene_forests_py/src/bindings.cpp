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

#include <chrono>
#include <cstring>
#include <expected>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <pybind11/chrono.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <semantic_scene_forests/scene_graph.hpp>

namespace py = pybind11;

namespace {

using fzi::semantic_scene_forests::ConceptNode;
using fzi::semantic_scene_forests::CropPtr;
using fzi::semantic_scene_forests::ObjectNode;
using fzi::semantic_scene_forests::SceneGraph;
using fzi::semantic_scene_forests::SceneGraphEdge;
using fzi::semantic_scene_forests::WorkerState;

PyObject* g_graph_error_type = nullptr;

[[noreturn]] void raiseGraphError(const SceneGraphError& error)
{
  const auto EXCEPTION_TYPE = py::reinterpret_borrow<py::object>(g_graph_error_type);
  auto exception            = EXCEPTION_TYPE(error.message);
  exception.attr("kind")    = py::cast(error.kind);
  PyErr_SetObject(g_graph_error_type, exception.ptr());
  throw py::error_already_set();
}

template <typename T> T unwrap(std::expected<T, SceneGraphError> result)
{
  if (!result)
  {
    raiseGraphError(result.error());
  }
  return std::move(*result);
}

void unwrap(std::expected<void, SceneGraphError> result)
{
  if (!result)
  {
    raiseGraphError(result.error());
  }
}

py::object messageClass(const char* module_name, const char* class_name)
{
  return py::module_::import(module_name).attr(class_name);
}

template <typename TMessage>
TMessage messageFromPython(py::handle message, const char* module_name, const char* class_name)
{
  const auto EXPECTED_CLASS = messageClass(module_name, class_name);
  if (!py::isinstance(message, EXPECTED_CLASS))
  {
    throw py::type_error(std::string("expected ") + module_name + "." + class_name);
  }

  const auto SERIALIZED =
    py::module_::import("rclpy.serialization").attr("serialize_message")(message).cast<py::bytes>();
  const std::string SERIALIZED_BYTES = SERIALIZED;

  rclcpp::SerializedMessage serialized_message(SERIALIZED_BYTES.size());
  auto& raw_message = serialized_message.get_rcl_serialized_message();
  if (!SERIALIZED_BYTES.empty())
  {
    std::memcpy(raw_message.buffer, SERIALIZED_BYTES.data(), SERIALIZED_BYTES.size());
  }
  raw_message.buffer_length = SERIALIZED_BYTES.size();

  TMessage result;
  rclcpp::Serialization<TMessage> serializer;
  serializer.deserialize_message(&serialized_message, &result);
  return result;
}

template <typename TMessage>
py::object messageToPython(const TMessage& message, const char* module_name, const char* class_name)
{
  rclcpp::SerializedMessage serialized_message;
  rclcpp::Serialization<TMessage> serializer;
  serializer.serialize_message(&message, &serialized_message);

  const auto& raw_message = serialized_message.get_rcl_serialized_message();
  const auto SERIALIZED   = py::bytes(reinterpret_cast<const char*>(raw_message.buffer),
                                      static_cast<py::ssize_t>(raw_message.buffer_length));
  return py::module_::import("rclpy.serialization")
    .attr("deserialize_message")(SERIALIZED, messageClass(module_name, class_name));
}

using BoundingBox = vision_msgs::msg::BoundingBox3D;
using Stamp       = builtin_interfaces::msg::Time;

BoundingBox bboxFromPython(py::handle bbox)
{
  return bbox.is_none() ? BoundingBox{}
                        : messageFromPython<BoundingBox>(bbox, "vision_msgs.msg", "BoundingBox3D");
}

Stamp stampFromPython(py::handle stamp)
{
  return stamp.is_none() ? Stamp{}
                         : messageFromPython<Stamp>(stamp, "builtin_interfaces.msg", "Time");
}

py::object stampToPython(const Stamp& stamp)
{
  return messageToPython(stamp, "builtin_interfaces.msg", "Time");
}

CropPtr cropFromPython(py::handle stamp, const std::string& jpeg)
{
  auto crop          = std::make_shared<sensor_msgs::msg::CompressedImage>();
  crop->header.stamp = stampFromPython(stamp);
  crop->format       = "jpeg";
  crop->data.assign(jpeg.begin(), jpeg.end());
  return crop;
}

//! Crops cross the boundary as a list of (builtin_interfaces.msg.Time, JPEG bytes).
std::vector<CropPtr> cropsFromPython(const std::vector<std::pair<py::object, std::string>>& crops)
{
  std::vector<CropPtr> result;
  result.reserve(crops.size());
  for (const auto& [stamp, jpeg] : crops)
  {
    result.push_back(cropFromPython(stamp, jpeg));
  }
  return result;
}

py::list cropsToPython(const std::vector<CropPtr>& crops)
{
  py::list result;
  for (const auto& crop : crops)
  {
    const std::string JPEG(crop->data.begin(), crop->data.end());
    result.append(py::make_tuple(stampToPython(crop->header.stamp), py::bytes(JPEG)));
  }
  return result;
}

std::map<std::string, bool> workersToPython(const std::map<std::string, WorkerState>& workers)
{
  std::map<std::string, bool> result;
  for (const auto& [name, state] : workers)
  {
    result[name] = state.processed;
  }
  return result;
}

std::map<std::string, WorkerState> workersFromPython(const std::map<std::string, bool>& workers)
{
  std::map<std::string, WorkerState> result;
  for (const auto& [name, processed] : workers)
  {
    result[name] = WorkerState{.processed = processed};
  }
  return result;
}

ObjectNode makeObjectNode(int64_t id,
                          const std::string& class_name,
                          py::handle bbox,
                          const std::string& frame_id,
                          uint8_t source,
                          py::handle last_seen,
                          py::handle last_updated,
                          const std::vector<std::pair<py::object, std::string>>& crops)
{
  return ObjectNode{.id           = id,
                    .class_name   = class_name,
                    .bbox         = bboxFromPython(bbox),
                    .frame_id     = frame_id,
                    .source       = source,
                    .last_seen    = stampFromPython(last_seen),
                    .last_updated = stampFromPython(last_updated),
                    .crops        = cropsFromPython(crops)};
}

} // namespace

PYBIND11_MODULE(_semantic_scene_forests, module)
{
  module.doc() = "Bindings for the semantic scene graph library.";

  py::enum_<SceneGraphErrorKind>(module, "GraphErrorKind")
    .value("NODE_ALREADY_EXISTS", SceneGraphErrorKind::NODE_ALREADY_EXISTS)
    .value("NODE_DOES_NOT_EXIST", SceneGraphErrorKind::NODE_DOES_NOT_EXIST)
    .value("EDGE_DOES_NOT_EXIST", SceneGraphErrorKind::EDGE_DOES_NOT_EXIST)
    .value("INVALID_EDGE", SceneGraphErrorKind::INVALID_EDGE)
    .value("EDGE_ALREADY_EXISTS", SceneGraphErrorKind::EDGE_ALREADY_EXISTS)
    .value("CYCLE", SceneGraphErrorKind::CYCLE)
    .value("GRAPH_EDITING_FAILED", SceneGraphErrorKind::GRAPH_EDITING_FAILED)
    .value("INVALID_LABEL", SceneGraphErrorKind::INVALID_LABEL)
    .export_values();

  g_graph_error_type =
    PyErr_NewException("semantic_scene_forests.GraphError", PyExc_RuntimeError, nullptr);
  module.attr("GraphError") = py::reinterpret_borrow<py::object>(g_graph_error_type);

  py::class_<ConceptNode>(module, "ConceptNode")
    .def(py::init<int64_t, std::string>(), py::arg("id"), py::arg("concept_name"))
    .def_readwrite("id", &ConceptNode::id)
    .def_readwrite("concept_name", &ConceptNode::concept_name)
    .def_property(
      "workers",
      [](const ConceptNode& node) { return workersToPython(node.workers); },
      [](ConceptNode& node, const std::map<std::string, bool>& workers) {
        node.workers = workersFromPython(workers);
      })
    .def("__repr__",
         [](const ConceptNode& node) {
           return "ConceptNode(id=" + std::to_string(node.id) + ", concept_name='" +
                  node.concept_name + "')";
         })
    .def(py::self == py::self);

  py::class_<ObjectNode>(module, "ObjectNode")
    .def(py::init(&makeObjectNode),
         py::arg("id"),
         py::arg("class_name"),
         py::arg("bbox")         = py::none(),
         py::arg("frame_id")     = "",
         py::arg("source")       = ObjectNode::SOURCE_CAMERA,
         py::arg("last_seen")    = py::none(),
         py::arg("last_updated") = py::none(),
         py::arg("crops")        = std::vector<std::pair<py::object, std::string>>{})
    .def_readonly_static("SOURCE_CAMERA", &ObjectNode::SOURCE_CAMERA)
    .def_readonly_static("SOURCE_MANUAL", &ObjectNode::SOURCE_MANUAL)
    .def_readwrite("id", &ObjectNode::id)
    .def_readwrite("class_name", &ObjectNode::class_name)
    .def_readwrite("frame_id", &ObjectNode::frame_id)
    .def_readwrite("source", &ObjectNode::source)
    .def_property(
      "bbox",
      [](const ObjectNode& node) {
        return messageToPython(node.bbox, "vision_msgs.msg", "BoundingBox3D");
      },
      [](ObjectNode& node, py::handle bbox) { node.bbox = bboxFromPython(bbox); })
    .def_property(
      "last_seen",
      [](const ObjectNode& node) { return stampToPython(node.last_seen); },
      [](ObjectNode& node, py::handle stamp) { node.last_seen = stampFromPython(stamp); })
    .def_property(
      "last_updated",
      [](const ObjectNode& node) { return stampToPython(node.last_updated); },
      [](ObjectNode& node, py::handle stamp) { node.last_updated = stampFromPython(stamp); })
    .def_property(
      "crops",
      [](const ObjectNode& node) { return cropsToPython(node.crops); },
      [](ObjectNode& node, const std::vector<std::pair<py::object, std::string>>& crops) {
        node.crops = cropsFromPython(crops);
      })
    .def_property(
      "workers",
      [](const ObjectNode& node) { return workersToPython(node.workers); },
      [](ObjectNode& node, const std::map<std::string, bool>& workers) {
        node.workers = workersFromPython(workers);
      })
    .def("__repr__",
         [](const ObjectNode& node) {
           return "ObjectNode(id=" + std::to_string(node.id) + ", class_name='" + node.class_name +
                  "')";
         })
    .def(py::self == py::self);

  py::class_<SceneGraphEdge>(module, "Edge")
    .def(py::init<std::string>(), py::arg("label"))
    .def_readwrite("label", &SceneGraphEdge::label)
    .def("__repr__", [](const SceneGraphEdge& edge) { return "Edge(label='" + edge.label + "')"; })
    .def(py::self == py::self);

  py::class_<SceneGraph>(module, "SceneGraph")
    .def(py::init<>())
    .def("allocate_id", &SceneGraph::allocateId)
    .def(
      "add_object",
      [](SceneGraph& graph, const ObjectNode& node) { unwrap(graph.addObject(node)); },
      py::arg("node"))
    .def(
      "update_object_pose",
      [](SceneGraph& graph, int64_t id, py::handle bbox, py::handle last_seen) {
        unwrap(graph.updateObjectPose(id, bboxFromPython(bbox), stampFromPython(last_seen)));
      },
      py::arg("id"),
      py::arg("bbox"),
      py::arg("last_seen"))
    .def("set_crop_policy",
         &SceneGraph::setCropPolicy,
         py::arg("max_crops"),
         py::arg("min_interval"),
         "min_interval is a datetime.timedelta")
    .def(
      "offer_crop",
      [](SceneGraph& graph, int64_t id, py::handle stamp, const std::string& jpeg) {
        return unwrap(graph.offerCrop(id, cropFromPython(stamp, jpeg)));
      },
      py::arg("id"),
      py::arg("stamp"),
      py::arg("jpeg"))
    .def(
      "set_object_class",
      [](SceneGraph& graph, int64_t id, const std::string& class_name) {
        unwrap(graph.setObjectClass(id, class_name));
      },
      py::arg("id"),
      py::arg("class_name"))
    .def(
      "find_or_add_concept",
      [](SceneGraph& graph, const std::string& label) {
        const auto LOOKUP = unwrap(graph.findOrAddConcept(label));
        return std::make_pair(LOOKUP.id, LOOKUP.created);
      },
      py::arg("label"))
    .def(
      "find_concept",
      [](const SceneGraph& graph, const std::string& label) { return graph.findConcept(label); },
      py::arg("label"))
    .def(
      "add_concept_edge",
      [](SceneGraph& graph,
         int64_t source_id,
         const std::string& relation,
         const std::string& label) {
        return unwrap(graph.addConceptEdge(source_id, relation, label)).id;
      },
      py::arg("source_id"),
      py::arg("relation"),
      py::arg("label"))
    .def(
      "mark_processed",
      [](SceneGraph& graph, int64_t id, const std::string& worker) {
        unwrap(graph.markProcessed(id, worker));
      },
      py::arg("id"),
      py::arg("worker"))
    .def("unprocessed_by", &SceneGraph::unprocessedBy, py::arg("worker"))
    .def("prune_isolated_concepts", &SceneGraph::pruneIsolatedConcepts)
    .def(
      "add_concept_node",
      [](SceneGraph& graph, const std::string& concept_name) {
        return unwrap(graph.addConceptNode(concept_name));
      },
      py::arg("concept_name"))
    .def(
      "add_concept_node_with_id",
      [](SceneGraph& graph, int64_t id, const std::string& concept_name) {
        return unwrap(graph.addConceptNodeWithId(id, concept_name));
      },
      py::arg("id"),
      py::arg("concept_name"))
    .def("remove_node", [](SceneGraph& graph, int64_t id) { return unwrap(graph.removeNode(id)); })
    .def("remove_isolated_nodes",
         [](SceneGraph& graph) { return unwrap(graph.removeIsolatedNodes()); })
    .def("find_node", &SceneGraph::findNode, py::arg("id"))
    .def("has_node", &SceneGraph::hasNode, py::arg("id"))
    .def("find_edges",
         [](const SceneGraph& graph, int64_t source_id, int64_t target_id) {
           return unwrap(graph.findEdges(source_id, target_id));
         })
    .def("has_edge",
         &SceneGraph::hasEdge,
         py::arg("source_id"),
         py::arg("target_id"),
         py::arg("label"))
    .def("add_edge",
         [](SceneGraph& graph, int64_t source_id, int64_t target_id, const std::string& label) {
           return unwrap(graph.addEdge(source_id, target_id, label));
         })
    .def("remove_edge",
         [](SceneGraph& graph, int64_t source_id, int64_t target_id, const std::string& label) {
           return unwrap(graph.removeEdge(source_id, target_id, label));
         })
    .def("outgoing_edges",
         [](const SceneGraph& graph, int64_t id) { return unwrap(graph.outgoingEdges(id)); })
    .def("incoming_edges",
         [](const SceneGraph& graph, int64_t id) { return unwrap(graph.incomingEdges(id)); })
    .def("find_concept_nodes_by_name", &SceneGraph::findConceptNodesByName, py::arg("concept_name"))
    .def("find_edges_by_label", &SceneGraph::findEdgesByLabel, py::arg("label"))
    .def_property_readonly("object_nodes", &SceneGraph::getObjectNodes)
    .def_property_readonly("concept_nodes", &SceneGraph::getConceptNodes)
    .def_property_readonly("nodes", &SceneGraph::getNodes)
    .def_property_readonly("edges", &SceneGraph::getEdges)
    .def_static("from_msg",
                [](py::handle message) {
                  return unwrap(SceneGraph::fromMsg(
                    messageFromPython<semantic_scene_forests_msgs::msg::SceneGraph>(
                      message, "semantic_scene_forests_msgs.msg", "SceneGraph")));
                })
    .def("to_msg",
         [](const SceneGraph& graph) {
           return messageToPython(*graph.toMsg(), "semantic_scene_forests_msgs.msg", "SceneGraph");
         })
    .def("update_from_msg",
         [](SceneGraph& graph, py::handle message) {
           unwrap(graph.updateSceneGraphFromMsg(
             messageFromPython<semantic_scene_forests_msgs::msg::SceneGraph>(
               message, "semantic_scene_forests_msgs.msg", "SceneGraph")));
         })
    .def(
      "retain_object_nodes",
      [](SceneGraph& graph, const std::unordered_set<int64_t>& active_ids) {
        unwrap(graph.retainObjectNodes(active_ids));
      },
      py::arg("active_ids"))
    .def("clone", [](const SceneGraph& graph) { return unwrap(graph.clone()); })
    .def("swap", &SceneGraph::swap, py::arg("other"));
}
