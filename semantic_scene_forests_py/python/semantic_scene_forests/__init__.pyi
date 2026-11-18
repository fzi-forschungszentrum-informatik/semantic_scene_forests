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
from typing import ClassVar, TypeAlias

from builtin_interfaces.msg import Time
from vision_msgs.msg import BoundingBox3D

from semantic_scene_forests_msgs.msg import SceneGraph as SceneGraphMsg

Crop: TypeAlias = tuple[Time, bytes]  # (stamp, JPEG bytes)
Node: TypeAlias = ConceptNode | ObjectNode
NodeEdge: TypeAlias = tuple[int, Edge]
GraphEdge: TypeAlias = tuple[int, int, Edge]

class GraphErrorKind:
    NODE_ALREADY_EXISTS: GraphErrorKind
    NODE_DOES_NOT_EXIST: GraphErrorKind
    EDGE_DOES_NOT_EXIST: GraphErrorKind
    INVALID_EDGE: GraphErrorKind
    EDGE_ALREADY_EXISTS: GraphErrorKind
    CYCLE: GraphErrorKind
    GRAPH_EDITING_FAILED: GraphErrorKind
    INVALID_LABEL: GraphErrorKind

class GraphError(RuntimeError):
    kind: GraphErrorKind

class ConceptNode:
    id: int
    concept_name: str
    workers: dict[str, bool]  # worker name -> processed

    def __init__(self, id: int, concept_name: str) -> None: ...

class ObjectNode:
    SOURCE_CAMERA: ClassVar[int]
    SOURCE_MANUAL: ClassVar[int]
    id: int
    class_name: str
    bbox: BoundingBox3D
    frame_id: str
    source: int
    last_seen: Time
    last_updated: Time
    crops: list[Crop]
    workers: dict[str, bool]  # worker name -> processed

    def __init__(
        self,
        id: int,
        class_name: str,
        bbox: BoundingBox3D | None = None,
        frame_id: str = "",
        source: int = ...,
        last_seen: Time | None = None,
        last_updated: Time | None = None,
        crops: list[Crop] = ...,
    ) -> None: ...

class Edge:
    label: str

    def __init__(self, label: str) -> None: ...

class SceneGraph:
    def allocate_id(self) -> int: ...
    def add_object(self, node: ObjectNode) -> None: ...
    def update_object_pose(
        self, id: int, bbox: BoundingBox3D, last_seen: Time
    ) -> None: ...
    def set_crop_policy(self, max_crops: int, min_interval: timedelta) -> None: ...
    def offer_crop(self, id: int, stamp: Time, jpeg: bytes) -> bool: ...
    def set_object_class(self, id: int, class_name: str) -> None: ...
    def find_or_add_concept(self, label: str) -> tuple[int, bool]: ...
    def find_concept(self, label: str) -> int | None: ...
    def add_concept_edge(
        self, source_id: int, relation: str, label: str
    ) -> tuple[int, bool]: ...
    def mark_processed(self, id: int, worker: str) -> None: ...
    def unprocessed_by(self, worker: str) -> list[int]: ...
    def prune_isolated_concepts(self) -> list[int]: ...
    def add_concept_node(self, concept_name: str) -> int: ...
    def add_concept_node_with_id(self, id: int, concept_name: str) -> int: ...
    def remove_node(self, id: int) -> Node: ...
    def remove_isolated_nodes(self) -> list[Node]: ...
    def find_node(self, id: int) -> Node | None: ...
    def has_node(self, id: int) -> bool: ...
    def find_edges(self, source_id: int, target_id: int) -> list[Edge]: ...
    def has_edge(self, source_id: int, target_id: int, label: str) -> bool: ...
    def add_edge(self, source_id: int, target_id: int, label: str) -> Edge: ...
    def remove_edge(self, source_id: int, target_id: int, label: str) -> Edge: ...
    def outgoing_edges(self, id: int) -> list[NodeEdge]: ...
    def incoming_edges(self, id: int) -> list[NodeEdge]: ...
    def find_concept_nodes_by_name(self, concept_name: str) -> list[ConceptNode]: ...
    def find_edges_by_label(self, label: str) -> list[GraphEdge]: ...
    @property
    def object_nodes(self) -> list[ObjectNode]: ...
    @property
    def concept_nodes(self) -> list[ConceptNode]: ...
    @property
    def nodes(self) -> list[Node]: ...
    @property
    def edges(self) -> list[GraphEdge]: ...
    @staticmethod
    def from_msg(message: SceneGraphMsg) -> SceneGraph: ...
    def to_msg(self) -> SceneGraphMsg: ...
    def update_from_msg(self, message: SceneGraphMsg) -> None: ...
    def clone(self) -> SceneGraph: ...
    def swap(self, other: SceneGraph) -> None: ...
