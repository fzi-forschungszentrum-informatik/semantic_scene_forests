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
from datetime import timedelta

import pytest
from builtin_interfaces.msg import Time
from vision_msgs.msg import BoundingBox3D

from semantic_scene_forests import (
    ConceptNode,
    Edge,
    GraphError,
    GraphErrorKind,
    ObjectNode,
    SceneGraph,
)

JPEG = b"\xff\xd8jpeg\xff\xd9"


def make_bbox(x: float = 1.0) -> BoundingBox3D:
    bbox = BoundingBox3D()
    bbox.center.position.x = x
    bbox.size.z = 0.5
    return bbox


def new_object(graph: SceneGraph, name: str = "cup", sec: int = 10) -> int:
    object_id = graph.allocate_id()
    stamp = Time(sec=sec)
    graph.add_object(
        ObjectNode(
            object_id,
            name,
            make_bbox(),
            "map",
            last_seen=stamp,
            last_updated=stamp,
            crops=[(stamp, JPEG)],
        )
    )
    return object_id


def test_node_value_types_expose_new_fields() -> None:
    node = ObjectNode(
        7,
        "cup",
        make_bbox(),
        "map",
        ObjectNode.SOURCE_MANUAL,
        Time(sec=3),
        Time(sec=4),
        [(Time(sec=5), JPEG)],
    )

    assert node.id == 7
    assert node.class_name == "cup"
    assert node.bbox.center.position.x == 1.0
    assert node.frame_id == "map"
    assert node.source == ObjectNode.SOURCE_MANUAL
    assert node.last_seen.sec == 3
    assert node.last_updated.sec == 4
    assert node.crops[0][0].sec == 5
    assert node.crops[0][1] == JPEG
    assert node.workers == {}
    node.workers = {"describer": True}
    assert node.workers == {"describer": True}
    assert ConceptNode(1, "on").concept_name == "on"
    assert Edge("relation").label == "relation"


def test_graph_construction_queries_and_mutation() -> None:
    graph = SceneGraph()
    object_id = new_object(graph)
    concept_id = graph.add_concept_node("on")

    assert object_id > 0
    assert concept_id > object_id  # one shared positive id counter
    assert graph.has_node(object_id)
    assert graph.find_node(999) is None
    found = graph.find_node(object_id)
    assert isinstance(found, ObjectNode)
    assert found.frame_id == "map"
    assert found.crops[0][1] == JPEG
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
    assert len(graph.remove_isolated_nodes()) == 2


def test_graph_errors_keep_kind_and_message() -> None:
    graph = SceneGraph()
    first_id = graph.add_concept_node_with_id(1, "first")

    with pytest.raises(GraphError) as duplicate:
        graph.add_concept_node_with_id(first_id, "duplicate")
    assert duplicate.value.kind == GraphErrorKind.NODE_ALREADY_EXISTS
    assert "already exists" in str(duplicate.value)

    with pytest.raises(GraphError) as missing:
        graph.remove_node(42)
    assert missing.value.kind == GraphErrorKind.NODE_DOES_NOT_EXIST

    object_id = new_object(graph)
    second_object_id = new_object(graph, "plate")
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

    with pytest.raises(GraphError) as label:
        graph.find_or_add_concept("  ")
    assert label.value.kind == GraphErrorKind.INVALID_LABEL


def test_atomic_object_methods() -> None:
    graph = SceneGraph()
    object_id = new_object(graph, sec=10)

    graph.update_object_pose(object_id, make_bbox(2.0), Time(sec=20))
    node = graph.find_node(object_id)
    assert isinstance(node, ObjectNode)
    assert node.bbox.center.position.x == 2.0
    assert node.last_seen.sec == 20
    assert node.last_updated.sec == 20
    with pytest.raises(GraphError) as missing:
        graph.update_object_pose(999, make_bbox(), Time())
    assert missing.value.kind == GraphErrorKind.NODE_DOES_NOT_EXIST

    graph.set_crop_policy(2, timedelta(seconds=5))
    assert not graph.offer_crop(object_id, Time(sec=12), JPEG)  # too close to sec=10
    assert graph.offer_crop(object_id, Time(sec=16), b"b")
    assert graph.offer_crop(object_id, Time(sec=22), b"c")  # replaces oldest
    crops = graph.find_node(object_id).crops
    assert [data for _, data in crops] == [b"b", b"c"]

    graph.set_object_class(object_id, "mug")
    assert graph.find_node(object_id).class_name == "mug"


def test_concepts_and_worker_state() -> None:
    graph = SceneGraph()
    object_id = new_object(graph)

    concept_id, created = graph.find_or_add_concept(" red ")
    assert created
    assert graph.find_or_add_concept("red") == (concept_id, False)
    edge_concept = graph.add_concept_edge(object_id, "has_color", "blue")
    assert graph.find_concept("blue") == edge_concept
    assert graph.find_concept("green") is None
    with pytest.raises(GraphError):
        graph.add_concept_edge(
            object_id, "has_color", "blue"
        )  # existing label is an error
    assert graph.has_edge(object_id, edge_concept, "has_color")

    assert object_id in graph.unprocessed_by("describer")
    graph.mark_processed(object_id, "describer")
    assert object_id not in graph.unprocessed_by("describer")
    assert graph.find_node(object_id).workers == {"describer": True}

    # "red" has no edge: pruned; "blue" is connected and the object is kept
    assert graph.prune_isolated_concepts() == [concept_id]
    assert graph.has_node(object_id)


def test_clone_swap_and_explicit_id_allocation() -> None:
    graph = SceneGraph()
    stamp = Time(sec=1)
    graph.add_object(ObjectNode(30, "cup", make_bbox(), "map", crops=[(stamp, JPEG)]))
    concept_id = graph.add_concept_node_with_id(50, "red")
    graph.add_edge(30, concept_id, "has_color")

    clone = graph.clone()
    cloned_object = clone.find_node(30)
    assert isinstance(cloned_object, ObjectNode)
    assert cloned_object.crops[0][1] == JPEG
    clone.add_concept_node("blue")
    assert len(graph.concept_nodes) == 1
    assert len(clone.concept_nodes) == 2
    assert graph.allocate_id() == 51  # explicit ids advance the counter

    other = SceneGraph()
    other_id = other.add_concept_node_with_id(3, "other")
    graph.swap(other)
    assert graph.has_node(other_id)
    assert other.has_node(30)


def test_ros_message_conversion_roundtrip() -> None:
    graph = SceneGraph()
    object_id = new_object(graph)
    concept_id = graph.add_concept_node("on")
    graph.add_edge(object_id, concept_id, "relation")

    message = graph.to_msg()
    assert len(message.object_nodes) == 1
    assert len(message.concept_nodes) == 1
    assert len(message.edges) == 1
    assert message.object_nodes[0].header.frame_id == "map"
    assert message.object_nodes[0].bbox.center.position.x == 1.0

    loaded = SceneGraph.from_msg(message)
    loaded_object = loaded.find_node(object_id)
    assert isinstance(loaded_object, ObjectNode)
    assert loaded_object.frame_id == "map"
    assert loaded_object.last_seen.sec == 10
    assert loaded_object.crops == []  # crops are not on the wire
    assert loaded.has_edge(object_id, concept_id, "relation")

    graph.update_from_msg(message)
    assert graph.find_node(object_id).crops[0][1] == JPEG  # update keeps crops


def test_retain_object_nodes_binding_smoke() -> None:
    graph = SceneGraph()
    object1 = new_object(graph)
    object2 = new_object(graph, "plate")
    shared = graph.add_concept_node("shared")
    ancestor = graph.add_concept_node("ancestor")
    graph.add_edge(object1, shared, "rel")
    graph.add_edge(object2, shared, "rel")
    graph.add_edge(shared, ancestor, "rel")

    graph.retain_object_nodes({object2})

    assert graph.has_node(object2)
    assert graph.has_node(shared)
    assert graph.has_node(ancestor)
    assert not graph.has_node(object1)

    graph.retain_object_nodes(set())
    assert len(graph.nodes) == 0
    assert len(graph.edges) == 0
