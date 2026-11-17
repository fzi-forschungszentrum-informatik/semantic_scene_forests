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

//! Rust bindings for the existing C++ semantic scene graph library.

pub use ros_env::semantic_scene_forests_msgs::msg::SceneGraph as SceneGraphMsg;
pub use ros_env::{builtin_interfaces::msg::Time, vision_msgs::msg::BoundingBox3D};
use std::collections::BTreeMap;

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

/// A JPEG crop and the time it was taken.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Crop {
    pub stamp: Time,
    pub jpeg: Vec<u8>,
}

/// An owned copy of an object node. `bbox` is expressed in `frame_id`; `workers` maps a
/// worker name to its processed flag.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ObjectNode {
    pub id: i64,
    pub class_name: String,
    pub bbox: BoundingBox3D,
    pub frame_id: String,
    pub source: u8,
    pub last_seen: Time,
    pub last_updated: Time,
    pub crops: Vec<Crop>,
    pub workers: BTreeMap<String, bool>,
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

    /// Reserve an id from the graph's shared (object and concept) counter.
    pub fn allocate_id(&mut self) -> i64 {
        self.inner.pin_mut().allocate_id()
    }

    /// Insert an object under `node.id` (see [`allocate_id`](Self::allocate_id)), copying all fields.
    pub fn add_object(&mut self, node: &ObjectNode) -> Result<(), GraphError> {
        self.inner.pin_mut().add_object(&node.into()).into_result()
    }

    /// Return independent copies; editing these values does not mutate the graph.
    pub fn object_nodes(&self) -> Vec<ObjectNode> {
        self.inner
            .object_nodes()
            .into_iter()
            .map(Into::into)
            .collect()
    }

    /// Deep-copy the graph and all payloads using the fallible C++ snapshot operation.
    pub fn try_clone(&self) -> Result<Self, GraphError> {
        let mut snapshot = Self::new();
        snapshot
            .inner
            .pin_mut()
            .copy_from(&self.inner)
            .into_result()?;
        Ok(snapshot)
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

    /// Reconstruct a graph from a message; crops and worker state are not on the wire.
    pub fn from_msg(message: &SceneGraphMsg) -> Result<Self, GraphError> {
        let mut graph = Self::new();
        graph
            .inner
            .pin_mut()
            .load_message(&message.into())
            .into_result()?;
        Ok(graph)
    }

    /// Export the graph; crops and worker state are not part of the message.
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
