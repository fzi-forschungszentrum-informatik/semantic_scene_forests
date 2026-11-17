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

#[cxx::bridge(namespace = "ssf_rs")]
pub(crate) mod bridge {
    #[derive(Debug, PartialEq, Eq)]
    #[repr(u8)]
    enum GraphErrorKind {
        NodeAlreadyExists,
        NodeDoesNotExist,
        EdgeDoesNotExist,
        InvalidEdge,
        EdgeAlreadyExists,
        Cycle,
        GraphEditingFailed,
        InvalidLabel,
    }

    struct Status {
        success: bool,
        kind: GraphErrorKind,
        message: String,
    }

    struct IdResult {
        status: Status,
        id: i64,
    }

    struct EdgeRecord {
        source: i64,
        target: i64,
        label: String,
    }

    struct EdgeList {
        status: Status,
        edges: Vec<EdgeRecord>,
    }

    #[derive(Default)]
    struct HeaderRecord {
        sec: i32,
        nanosec: u32,
        frame_id: String,
    }

    #[derive(Default)]
    struct StampRecord {
        sec: i32,
        nanosec: u32,
    }

    struct ObjectRecord {
        header: HeaderRecord,
        id: i64,
        class_name: String,
        bbox: BBoxRecord,
        source: u8,
        last_updated: StampRecord,
        last_seen: StampRecord,
    }

    struct ConceptRecord {
        header: HeaderRecord,
        id: i64,
        class_name: String,
    }

    struct GraphRecord {
        header: HeaderRecord,
        objects: Vec<ObjectRecord>,
        concepts: Vec<ConceptRecord>,
        edges: Vec<EdgeRecord>,
    }

    #[derive(Default)]
    struct PoseRecord {
        position: [f64; 3],
        orientation: [f64; 4],
    }

    #[derive(Default)]
    struct BBoxRecord {
        center: PoseRecord,
        size: [f64; 3],
    }

    struct CropRecord {
        stamp: StampRecord,
        jpeg: Vec<u8>,
    }

    struct WorkerRecord {
        name: String,
        processed: bool,
    }

    struct ObjectNodeRecord {
        id: i64,
        class_name: String,
        bbox: BBoxRecord,
        frame_id: String,
        source: u8,
        last_seen: StampRecord,
        last_updated: StampRecord,
        crops: Vec<CropRecord>,
        workers: Vec<WorkerRecord>,
    }

    unsafe extern "C++" {
        include!("semantic_scene_forests_rs/bridge.hpp");
        type NativeGraph;

        #[cxx_name = "newGraph"]
        fn new_graph() -> UniquePtr<NativeGraph>;
        #[cxx_name = "allocateId"]
        fn allocate_id(self: Pin<&mut NativeGraph>) -> i64;
        #[cxx_name = "addObject"]
        fn add_object(self: Pin<&mut NativeGraph>, node: &ObjectNodeRecord) -> Status;
        #[cxx_name = "objectNodes"]
        fn object_nodes(self: &NativeGraph) -> Vec<ObjectNodeRecord>;
        #[cxx_name = "copyFrom"]
        fn copy_from(self: Pin<&mut NativeGraph>, other: &NativeGraph) -> Status;
        #[cxx_name = "addConceptNode"]
        fn add_concept_node(self: Pin<&mut NativeGraph>, name: &str) -> IdResult;
        #[cxx_name = "hasNode"]
        fn has_node(self: &NativeGraph, id: i64) -> bool;
        #[cxx_name = "addEdge"]
        fn add_edge(self: Pin<&mut NativeGraph>, source: i64, target: i64, label: &str) -> Status;
        #[cxx_name = "outgoingEdges"]
        fn outgoing_edges(self: &NativeGraph, id: i64) -> EdgeList;
        #[cxx_name = "loadMessage"]
        fn load_message(self: Pin<&mut NativeGraph>, message: &GraphRecord) -> Status;
        #[cxx_name = "toMessage"]
        fn to_message(self: Pin<&mut NativeGraph>) -> GraphRecord;
        #[cxx_name = "updateFromMessage"]
        fn update_from_message(self: Pin<&mut NativeGraph>, message: &GraphRecord) -> Status;
    }
}

use crate::SceneGraphMsg;
use ros_env::{
    builtin_interfaces::msg::Time, semantic_scene_forests_msgs::msg, std_msgs::msg::Header,
};

impl From<&Time> for bridge::StampRecord {
    fn from(stamp: &Time) -> Self {
        Self {
            sec: stamp.sec,
            nanosec: stamp.nanosec,
        }
    }
}

impl From<bridge::StampRecord> for Time {
    fn from(stamp: bridge::StampRecord) -> Self {
        Self {
            sec: stamp.sec,
            nanosec: stamp.nanosec,
        }
    }
}

impl From<&Header> for bridge::HeaderRecord {
    fn from(header: &Header) -> Self {
        Self {
            sec: header.stamp.sec,
            nanosec: header.stamp.nanosec,
            frame_id: header.frame_id.clone(),
        }
    }
}

impl From<bridge::HeaderRecord> for Header {
    fn from(header: bridge::HeaderRecord) -> Self {
        Self {
            stamp: ros_env::builtin_interfaces::msg::Time {
                sec: header.sec,
                nanosec: header.nanosec,
            },
            frame_id: header.frame_id,
        }
    }
}

impl From<&SceneGraphMsg> for bridge::GraphRecord {
    fn from(message: &SceneGraphMsg) -> Self {
        Self {
            header: (&message.header).into(),
            objects: message
                .object_nodes
                .iter()
                .map(|node| bridge::ObjectRecord {
                    header: (&node.header).into(),
                    id: node.id,
                    class_name: node.class_name.clone(),
                    bbox: (&node.bbox).into(),
                    source: node.source,
                    last_updated: (&node.last_updated).into(),
                    last_seen: (&node.last_seen).into(),
                })
                .collect(),
            concepts: message
                .concept_nodes
                .iter()
                .map(|node| bridge::ConceptRecord {
                    header: (&node.header).into(),
                    id: node.id,
                    class_name: node.class_name.clone(),
                })
                .collect(),
            edges: message
                .edges
                .iter()
                .map(|edge| bridge::EdgeRecord {
                    source: edge.source_id,
                    target: edge.target_id,
                    label: edge.label.clone(),
                })
                .collect(),
        }
    }
}

impl From<bridge::GraphRecord> for SceneGraphMsg {
    fn from(message: bridge::GraphRecord) -> Self {
        Self {
            header: message.header.into(),
            object_nodes: message
                .objects
                .into_iter()
                .map(|node| msg::ObjectNode {
                    header: node.header.into(),
                    id: node.id,
                    class_name: node.class_name,
                    bbox: node.bbox.into(),
                    source: node.source,
                    last_updated: node.last_updated.into(),
                    last_seen: node.last_seen.into(),
                })
                .collect(),
            concept_nodes: message
                .concepts
                .into_iter()
                .map(|node| msg::ConceptNode {
                    header: node.header.into(),
                    id: node.id,
                    class_name: node.class_name,
                })
                .collect(),
            edges: message
                .edges
                .into_iter()
                .map(|edge| msg::Edge {
                    source_id: edge.source,
                    target_id: edge.target,
                    label: edge.label,
                })
                .collect(),
        }
    }
}
