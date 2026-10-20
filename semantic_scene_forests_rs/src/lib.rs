//! Rust bindings for the existing C++ semantic scene graph library.

pub use ros_env::semantic_scene_forests_msgs::msg::SceneGraph as SceneGraphMsg;
pub use ros_env::{sensor_msgs::msg::Image, vision_msgs::msg::Detection3D};

mod ffi;
mod payloads;
pub use bridge::GraphErrorKind;
use ffi::bridge;

/// A graph operation rejected by the C++ library.
#[derive(Debug)]
pub struct GraphError {
    pub kind: GraphErrorKind,
    pub message: String,
}

impl std::fmt::Display for GraphError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.message)
    }
}
impl std::error::Error for GraphError {}

impl bridge::Status {
    fn into_result(self) -> Result<(), GraphError> {
        if self.success {
            Ok(())
        } else {
            Err(GraphError {
                kind: self.kind,
                message: self.message,
            })
        }
    }
}

/// An owned copy of an edge label.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Edge {
    pub label: String,
}

/// An owned copy of an object and its optional ROS payloads.
#[derive(Debug, Clone, PartialEq)]
pub struct ObjectNode {
    pub id: i64,
    pub class_name: String,
    pub detection: Option<Detection3D>,
    pub visuals: [Option<Image>; 5],
}

/// Owns a C++ graph. Mutation requires an exclusive Rust borrow.
pub struct SceneGraph {
    inner: cxx::UniquePtr<bridge::NativeGraph>,
}

impl Default for SceneGraph {
    fn default() -> Self {
        Self::new()
    }
}

impl SceneGraph {
    pub fn new() -> Self {
        Self {
            inner: bridge::new_graph(),
        }
    }

    /// Add an object, copying its detection and exactly five optional image slots.
    pub fn add_object_node(
        &mut self,
        class_name: &str,
        detection: Option<&Detection3D>,
        visuals: [Option<&Image>; 5],
    ) -> Result<i64, GraphError> {
        let payload = bridge::PayloadRecord::new(detection, visuals);
        let result = self.inner.pin_mut().add_object_node(class_name, &payload);
        result.status.into_result()?;
        Ok(result.id)
    }

    /// Return independent copies; editing these values does not mutate the graph.
    pub fn object_nodes(&self) -> Vec<ObjectNode> {
        self.inner
            .object_nodes()
            .into_iter()
            .map(Into::into)
            .collect()
    }

    pub fn add_concept_node(&mut self, name: &str) -> Result<i64, GraphError> {
        let result = self.inner.pin_mut().add_concept_node(name);
        result.status.into_result()?;
        Ok(result.id)
    }

    pub fn has_node(&self, id: i64) -> bool {
        self.inner.has_node(id)
    }

    pub fn add_edge(&mut self, source: i64, target: i64, label: &str) -> Result<(), GraphError> {
        self.inner
            .pin_mut()
            .add_edge(source, target, label)
            .into_result()
    }

    pub fn outgoing_edges(&self, id: i64) -> Result<Vec<(i64, Edge)>, GraphError> {
        let result = self.inner.outgoing_edges(id);
        result.status.into_result()?;
        Ok(result
            .edges
            .into_iter()
            .map(|e| (e.target, Edge { label: e.label }))
            .collect())
    }

    /// Reconstruct a graph from the existing ID-only object wire format.
    pub fn from_msg(message: &SceneGraphMsg) -> Result<Self, GraphError> {
        let mut graph = Self::new();
        graph
            .inner
            .pin_mut()
            .load_message(&message.into())
            .into_result()?;
        Ok(graph)
    }

    /// Export the graph; object payloads and message headers are not retained.
    pub fn to_msg(&mut self) -> SceneGraphMsg {
        self.inner.pin_mut().to_message().into()
    }

    /// Update in place. Successful earlier changes remain if a later operation fails.
    pub fn update_from_msg(&mut self, message: &SceneGraphMsg) -> Result<(), GraphError> {
        self.inner
            .pin_mut()
            .update_from_message(&message.into())
            .into_result()
    }
}
