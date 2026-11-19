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

use rclrs::MessageIDL;
use ros_env::sensor_msgs::msg::Image;
use semantic_scene_forests_rs::{
    BoundingBox3D, Crop, GraphErrorKind, ObjectNode, SceneGraph, SceneGraphMsg, Time,
};
use std::borrow::Cow;

fn cup_node(id: i64) -> ObjectNode {
    let mut bbox = BoundingBox3D::default();
    bbox.center.position.x = 1.0;
    bbox.center.orientation.w = 1.0;
    bbox.size.z = 0.5;
    let stamp = Time {
        sec: 12,
        nanosec: 345,
    };
    ObjectNode {
        id,
        class_name: "cup".into(),
        bbox,
        frame_id: "map".into(),
        source: 1,
        last_seen: stamp.clone(),
        last_updated: stamp.clone(),
        crops: vec![Crop {
            stamp,
            jpeg: vec![0xff, 0xd8, 1, 2, 0xff, 0xd9],
        }],
        workers: [("describer".to_string(), true)].into(),
    }
}

fn add_cup(graph: &mut SceneGraph) -> i64 {
    let id = graph.allocate_id();
    graph.add_object(&cup_node(id)).unwrap();
    id
}

#[test]
fn generated_messages_share_rclrs_runtime() {
    fn accepts_message<T: MessageIDL>() {}
    accepts_message::<SceneGraphMsg>();

    let mut message = SceneGraphMsg::default();
    assert!(message.object_nodes.is_empty());
    message.object_nodes.push(Default::default());
    message.object_nodes[0].id = 42;
    assert_eq!(message.object_nodes[0].id, 42);

    let image = Image {
        height: 1,
        width: 3,
        step: 3,
        data: vec![10, 20, 30],
        ..Default::default()
    };
    let native = Image::into_rmw_message(Cow::Owned(image)).into_owned();
    assert_eq!(Image::from_rmw_message(native).data, [10, 20, 30]);
}

#[test]
fn build_and_query_graph() {
    let mut graph = SceneGraph::new();
    let cup = add_cup(&mut graph);
    let on = graph.add_concept_node("on").unwrap();
    let table = graph.add_concept_node("table").unwrap();
    assert!(cup > 0 && on > 0 && table > 0 && on != table && cup != on);
    assert!(graph.has_node(cup));
    graph.add_edge(cup, on, "relation").unwrap();
    graph.add_edge(on, table, "target").unwrap();
    let edges = graph.outgoing_edges(cup).unwrap();
    assert_eq!(edges.len(), 1);
    assert_eq!(edges[0].0, on);
    assert_eq!(edges[0].1.label, "relation");
    assert_eq!(graph.outgoing_edges(on).unwrap()[0].0, table);
}

#[test]
fn cycle_error_preserves_graph() {
    let mut graph = SceneGraph::new();
    let first = graph.add_concept_node("first").unwrap();
    let second = graph.add_concept_node("second").unwrap();
    graph.add_edge(first, second, "relation").unwrap();
    let before = graph.outgoing_edges(first).unwrap();
    let error = graph.add_edge(second, first, "relation").unwrap_err();
    assert_eq!(error.kind, GraphErrorKind::Cycle);
    assert!(!error.message.is_empty());
    assert_eq!(graph.outgoing_edges(first).unwrap(), before);
    assert!(graph.outgoing_edges(second).unwrap().is_empty());
}

#[test]
fn message_roundtrip() {
    let mut empty = SceneGraph::new();
    let mut loaded_empty = SceneGraph::from_msg(&empty.to_msg()).unwrap();
    assert_eq!(loaded_empty.to_msg(), SceneGraphMsg::default());

    let mut graph = SceneGraph::new();
    let cup = add_cup(&mut graph);
    let on = graph.add_concept_node("on").unwrap();
    graph.add_edge(cup, on, "relation").unwrap();
    let message = graph.to_msg();
    assert_eq!(message.object_nodes[0].id, cup);
    assert_eq!(message.object_nodes[0].class_name, "cup");
    assert_eq!(message.object_nodes[0].header.frame_id, "map");
    assert_eq!(message.object_nodes[0].bbox.size.z, 0.5);
    assert_eq!(message.concept_nodes[0].id, on);
    assert_eq!(message.concept_nodes[0].class_name, "on");
    assert_eq!(message.edges[0].source_id, cup);
    assert_eq!(message.edges[0].target_id, on);
    assert_eq!(message.edges[0].label, "relation");
    let mut loaded = SceneGraph::from_msg(&message).unwrap();
    assert!(loaded.has_node(cup));
    assert_eq!(
        loaded.outgoing_edges(cup).unwrap(),
        graph.outgoing_edges(cup).unwrap()
    );
    assert_eq!(loaded.to_msg(), message);
}

#[test]
fn update_keeps_earlier_success_on_error() {
    use ros_env::semantic_scene_forests_msgs::msg::{ConceptNode, Edge as EdgeMsg};
    let mut graph = SceneGraph::new();
    let mut message = SceneGraphMsg::default();
    message.concept_nodes.push(ConceptNode {
        id: 42,
        class_name: "new concept".into(),
        ..Default::default()
    });
    message.edges.push(EdgeMsg {
        source_id: 42,
        target_id: 999,
        label: "missing".into(),
    });
    let error = graph.update_from_msg(&message).unwrap_err();
    assert_eq!(error.kind, GraphErrorKind::NodeDoesNotExist);
    assert!(graph.has_node(42));
    assert!(graph.outgoing_edges(42).unwrap().is_empty());

    message.edges.clear();
    graph.update_from_msg(&message).unwrap();
    graph.update_from_msg(&SceneGraphMsg::default()).unwrap();
    assert!(!graph.has_node(42));
}

#[test]
fn object_fields_roundtrip_and_are_owned_copies() {
    let mut graph = SceneGraph::new();
    let mut expected = cup_node(graph.allocate_id());
    graph.add_object(&expected).unwrap();
    expected.class_name = "changed after insertion".into();

    let mut objects = graph.object_nodes();
    assert_eq!(objects[0], cup_node(objects[0].id));
    objects[0].crops[0].jpeg[0] = 99;
    assert_eq!(graph.object_nodes()[0], cup_node(objects[0].id));

    let error = graph.add_object(&cup_node(objects[0].id)).unwrap_err();
    assert_eq!(error.kind, GraphErrorKind::NodeAlreadyExists);
}

#[test]
fn snapshot_keeps_crops_but_wire_format_does_not() {
    let mut graph = SceneGraph::new();
    let cup = add_cup(&mut graph);
    let on = graph.add_concept_node("on").unwrap();
    graph.add_edge(cup, on, "relation").unwrap();
    assert_eq!(
        graph.add_edge(on, cup, "invalid").unwrap_err().kind,
        GraphErrorKind::InvalidEdge
    );

    let snapshot = graph.try_clone().unwrap();
    assert_eq!(snapshot.object_nodes(), graph.object_nodes());
    let loaded = SceneGraph::from_msg(&graph.to_msg()).unwrap();
    assert_eq!(
        loaded.outgoing_edges(cup).unwrap(),
        graph.outgoing_edges(cup).unwrap()
    );
    let loaded_cup = &loaded.object_nodes()[0];
    assert_eq!(loaded_cup.frame_id, "map");
    assert_eq!(loaded_cup.bbox, cup_node(cup).bbox);
    assert!(loaded_cup.crops.is_empty() && loaded_cup.workers.is_empty());

    graph.update_from_msg(&SceneGraphMsg::default()).unwrap();
    assert!(!graph.has_node(cup));
    assert_eq!(snapshot.object_nodes()[0], cup_node(cup));
}
