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
    }
}
