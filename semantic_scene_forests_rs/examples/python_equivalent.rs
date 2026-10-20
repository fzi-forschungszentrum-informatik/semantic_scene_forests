// Copyright (c) 2026 FZI Forschungszentrum Informatik
// SPDX-License-Identifier: BSD-3-Clause
//! The payload, query, error, snapshot, and ROS conversion example from the Python bindings.
use semantic_scene_forests_rs::{Detection3D, GraphErrorKind, Image, SceneGraph};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let detection = Detection3D {
        id: "cup-1".into(),
        ..Default::default()
    };
    let mut image = Image::default();
    image.header.frame_id = "camera".into();

    let mut graph = SceneGraph::new();
    let cup = graph.add_object_node(
        "cup",
        Some(&detection),
        [Some(&image), None, None, None, None],
    )?;
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
    assert_eq!(
        snapshot.object_nodes()[0].detection.as_ref(),
        Some(&detection)
    );
    assert_eq!(snapshot.object_nodes()[0].visuals[0].as_ref(), Some(&image));
    assert_eq!(loaded.outgoing_edges(cup)?, graph.outgoing_edges(cup)?);
    Ok(())
}
