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

    struct HeaderRecord {
        sec: i32,
        nanosec: u32,
        frame_id: String,
    }

    struct ObjectRecord {
        header: HeaderRecord,
        id: i64,
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

    unsafe extern "C++" {
        include!("semantic_scene_forests_rs/bridge.hpp");
        type NativeGraph;

        #[cxx_name = "newGraph"]
        fn new_graph() -> UniquePtr<NativeGraph>;
        #[cxx_name = "addObjectNode"]
        fn add_object_node(self: Pin<&mut NativeGraph>, name: &str) -> IdResult;
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
use ros_env::{semantic_scene_forests_msgs::msg, std_msgs::msg::Header};

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
