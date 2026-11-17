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

//! The object, query, error, snapshot, and ROS conversion example from the Python bindings.
use semantic_scene_forests_rs::{
    BoundingBox3D, Crop, GraphErrorKind, ObjectNode, SceneGraph, Time,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut graph = SceneGraph::new();
    let stamp = Time {
        sec: 12,
        nanosec: 0,
    };
    let mut bbox = BoundingBox3D::default();
    bbox.size.x = 0.1;
    let cup_node = ObjectNode {
        id: graph.allocate_id(),
        class_name: "cup".into(),
        bbox,
        frame_id: "map".into(),
        last_seen: stamp.clone(),
        last_updated: stamp.clone(),
        crops: vec![Crop {
            stamp,
            jpeg: vec![0xff, 0xd8, 0xff, 0xd9],
        }],
        ..Default::default()
    };
    let cup = cup_node.id;
    graph.add_object(&cup_node)?;
    let on = graph.add_concept_node("on")?;
    graph.add_edge(cup, on, "relation")?;

    for (target_id, edge) in graph.outgoing_edges(cup)? {
        println!("{target_id} {}", edge.label);
    }

    match graph.add_edge(on, cup, "invalid") {
        Err(error) if error.kind == GraphErrorKind::InvalidEdge => {
            println!("{:?}: {error}", error.kind);
        }
        Err(error) => return Err(error.into()),
        Ok(()) => return Err("invalid edge unexpectedly accepted".into()),
    }

    let snapshot = graph.try_clone()?;
    let message = graph.to_msg();
    let loaded = SceneGraph::from_msg(&message)?;
    assert_eq!(snapshot.object_nodes()[0], cup_node);
    assert_eq!(loaded.outgoing_edges(cup)?, graph.outgoing_edges(cup)?);
    Ok(())
}
