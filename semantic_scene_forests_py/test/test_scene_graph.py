# Copyright (c) 2026 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import pytest
from sensor_msgs.msg import Image
from semantic_scene_forests_msgs.msg import ConceptNode as SceneGraphConceptNodeMsg
from vision_msgs.msg import Detection3D

from semantic_scene_forests import (
    ConceptNode,
    Edge,
    GraphError,
    GraphErrorKind,
    ObjectNode,
    SceneGraph,
)


def make_payload() -> tuple[Detection3D, list[Image | None]]:
    detection = Detection3D()
    detection.id = "cup-detection"
    image = Image()
    image.header.frame_id = "camera"
    return detection, [image, None, None, None, None]


def test_node_value_types_preserve_ros_payloads() -> None:
    detection, visuals = make_payload()

    node = ObjectNode(7, "cup", detection, visuals)

    assert node.id == 7
    assert node.class_name == "cup"
    assert node.detection is not detection
    assert node.detection is not None
    assert node.detection.id == "cup-detection"
    assert len(node.visuals) == 5
    assert node.visuals[0] is not None
    assert node.visuals[0].header.frame_id == "camera"
    assert node.visuals[1] is None
    assert ConceptNode(-1, "on").concept_name == "on"
    assert Edge("relation").label == "relation"


def test_object_visual_validation() -> None:
    with pytest.raises(ValueError, match="exactly five"):
        ObjectNode(1, "cup", visuals=[])

    with pytest.raises(TypeError, match="sensor_msgs.msg.Image"):
        ObjectNode(1, "cup", visuals=[None, None, None, None, object()])


def test_graph_construction_queries_and_mutation() -> None:
    graph = SceneGraph()
    detection, visuals = make_payload()
    object_id = graph.add_object_node("cup", detection, visuals)
    concept_id = graph.add_concept_node("on")

    assert object_id == 1
    assert concept_id == -1
    assert graph.has_node(object_id)
    assert graph.find_node(999) is None
    assert isinstance(graph.find_node(object_id), ObjectNode)
    assert graph.add_edge(object_id, concept_id, "relation").label == "relation"
    assert graph.has_edge(object_id, concept_id, "relation")
    assert graph.find_edges(object_id, concept_id) == [Edge("relation")]
    assert graph.outgoing_edges(object_id) == [(concept_id, Edge("relation"))]
    assert graph.incoming_edges(concept_id) == [(object_id, Edge("relation"))]
    assert graph.find_concept_nodes_by_name("on") == [ConceptNode(concept_id, "on")]
    assert graph.find_edges_by_label("relation") == [
        (object_id, concept_id, Edge("relation"))
    ]
    assert graph.object_nodes[0].class_name == "cup"
    assert graph.concept_nodes == [ConceptNode(concept_id, "on")]
    assert len(graph.nodes) == 2
    assert graph.edges == [(object_id, concept_id, Edge("relation"))]
    assert graph.remove_edge(object_id, concept_id, "relation") == Edge("relation")
    assert graph.remove_isolated_nodes() == [
        ConceptNode(concept_id, "on"),
        ObjectNode(object_id, "cup", detection, visuals),
    ]


def test_graph_errors_keep_kind_and_message() -> None:
    graph = SceneGraph()
    first_id = graph.add_concept_node_with_id(-1, "first")

    with pytest.raises(GraphError) as duplicate:
        graph.add_concept_node_with_id(first_id, "duplicate")
    assert duplicate.value.kind == GraphErrorKind.NODE_ALREADY_EXISTS
    assert "already exists" in str(duplicate.value)

    with pytest.raises(GraphError) as missing:
        graph.remove_node(42)
    assert missing.value.kind == GraphErrorKind.NODE_DOES_NOT_EXIST

    object_id = graph.add_object_node("cup")
    second_object_id = graph.add_object_node("plate")
    with pytest.raises(GraphError) as invalid:
        graph.add_edge(object_id, second_object_id, "on")
    assert invalid.value.kind == GraphErrorKind.INVALID_EDGE

    graph.add_edge(object_id, first_id, "relation")
    with pytest.raises(GraphError) as duplicate_edge:
        graph.add_edge(object_id, first_id, "relation")
    assert duplicate_edge.value.kind == GraphErrorKind.EDGE_ALREADY_EXISTS

    second_id = graph.add_concept_node("second")
    graph.add_edge(first_id, second_id, "next")
    with pytest.raises(GraphError) as cycle:
        graph.add_edge(second_id, first_id, "previous")
    assert cycle.value.kind == GraphErrorKind.CYCLE


def test_clone_swap_and_explicit_id_allocation() -> None:
    graph = SceneGraph()
    detection, visuals = make_payload()
    object_id = graph.add_object_node_with_id(30, "cup", detection, visuals)
    concept_id = graph.add_concept_node_with_id(-50, "red")
    graph.add_edge(object_id, concept_id, "has_color")

    clone = graph.clone()
    cloned_object = clone.find_node(object_id)
    assert isinstance(cloned_object, ObjectNode)
    assert cloned_object.detection is not None
    assert cloned_object.detection.id == "cup-detection"
    clone.add_concept_node("blue")
    assert len(graph.concept_nodes) == 1
    assert len(clone.concept_nodes) == 2
    assert graph.add_object_node("plate") == 31
    assert graph.add_concept_node("auto") == -51

    other = SceneGraph()
    other_id = other.add_concept_node_with_id(-3, "other")
    graph.swap(other)
    assert graph.has_node(other_id)
    assert other.has_node(object_id)


def test_ros_message_conversion_preserves_current_wire_format() -> None:
    graph = SceneGraph()
    detection, visuals = make_payload()
    object_id = graph.add_object_node("cup", detection, visuals)
    concept_id = graph.add_concept_node("on")
    graph.add_edge(object_id, concept_id, "relation")

    message = graph.to_msg()
    assert len(message.object_nodes) == 1
    assert len(message.concept_nodes) == 1
    assert len(message.edges) == 1

    loaded = SceneGraph.from_msg(message)
    loaded_object = loaded.find_node(object_id)
    assert isinstance(loaded_object, ObjectNode)
    assert loaded_object.class_name == ""
    assert loaded_object.detection is not None
    assert loaded_object.detection.id == ""
    assert loaded.has_edge(object_id, concept_id, "relation")


def test_update_from_msg_reconciles_edges_and_rejects_collisions() -> None:
    graph = SceneGraph()
    detection, visuals = make_payload()
    object_id = graph.add_object_node_with_id(10, "cup", detection, visuals)
    concept_id = graph.add_concept_node_with_id(-10, "on")
    graph.add_edge(object_id, concept_id, "relation")

    # An edge missing from the message disappears.
    trimmed = graph.to_msg()
    trimmed.edges.clear()
    graph.update_from_msg(trimmed)
    assert graph.edges == []
    assert graph.has_node(object_id)
    assert graph.has_node(concept_id)

    # A reversed edge replaces the original without tripping cycle detection.
    # Objects can never receive edges, so the reversal uses two concept nodes.
    first = graph.add_concept_node_with_id(-20, "first")
    second = graph.add_concept_node_with_id(-21, "second")
    graph.add_edge(first, second, "rel")
    reversed_message = graph.to_msg()
    for edge in reversed_message.edges:
        if edge.source_id == first and edge.target_id == second and edge.label == "rel":
            edge.source_id = second
            edge.target_id = first
    graph.update_from_msg(reversed_message)
    assert graph.has_edge(second, first, "rel")
    assert not graph.has_edge(first, second, "rel")

    # A concept node reusing an existing object id must be rejected and leave
    # the original graph unchanged.
    collision_message = graph.to_msg()
    imposter = SceneGraphConceptNodeMsg()
    imposter.id = object_id
    imposter.class_name = "imposter"
    collision_message.concept_nodes.append(imposter)

    with pytest.raises(GraphError) as collision:
        graph.update_from_msg(collision_message)
    assert collision.value.kind == GraphErrorKind.NODE_ALREADY_EXISTS

    preserved = graph.find_node(object_id)
    assert isinstance(preserved, ObjectNode)
    assert preserved.detection is not None
    assert preserved.detection.id == "cup-detection"
    assert preserved.visuals[0].header.frame_id == "camera"


def test_retain_object_nodes_python_parity() -> None:
    graph = SceneGraph()
    detection, visuals = make_payload()
    object1 = graph.add_object_node_with_id(1, "cup", detection, visuals)
    object2 = graph.add_object_node_with_id(2, "plate", detection, visuals)
    shared = graph.add_concept_node_with_id(-1, "shared")
    exclusive = graph.add_concept_node_with_id(-2, "exclusive")
    ancestor = graph.add_concept_node_with_id(-3, "ancestor")

    graph.add_edge(object1, shared, "rel")
    graph.add_edge(object2, shared, "rel")
    graph.add_edge(object1, exclusive, "rel")
    graph.add_edge(shared, ancestor, "rel")

    graph.retain_object_nodes({object2})

    assert graph.has_node(object2)
    assert graph.has_node(shared)
    assert graph.has_node(ancestor)
    assert not graph.has_node(object1)
    assert not graph.has_node(exclusive)

    graph.retain_object_nodes(set())
    assert len(graph.nodes) == 0
    assert len(graph.edges) == 0
