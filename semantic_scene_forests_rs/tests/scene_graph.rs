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
use semantic_scene_forests_rs::{GraphErrorKind, SceneGraph, SceneGraphMsg};
use std::borrow::Cow;

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
    let cup = graph.add_object_node("cup", None, [None; 5]).unwrap();
    let on = graph.add_concept_node("on").unwrap();
    let table = graph.add_concept_node("table").unwrap();
    assert!(cup > 0 && on < 0 && table < 0 && on != table);
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
    let cup = graph.add_object_node("cup", None, [None; 5]).unwrap();
    let on = graph.add_concept_node("on").unwrap();
    graph.add_edge(cup, on, "relation").unwrap();
    let message = graph.to_msg();
    assert_eq!(message.object_nodes[0].id, cup);
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
        id: -42,
        class_name: "new concept".into(),
        ..Default::default()
    });
    message.edges.push(EdgeMsg {
        source_id: -42,
        target_id: -999,
        label: "missing".into(),
    });
    let error = graph.update_from_msg(&message).unwrap_err();
    assert_eq!(error.kind, GraphErrorKind::NodeDoesNotExist);
    assert!(graph.has_node(-42));
    assert!(graph.outgoing_edges(-42).unwrap().is_empty());

    message.edges.clear();
    graph.update_from_msg(&message).unwrap();
    graph.update_from_msg(&SceneGraphMsg::default()).unwrap();
    assert!(!graph.has_node(-42));
}

#[test]
fn payload_roundtrip_owns_complete_ros_messages() {
    use semantic_scene_forests_rs::{Detection3D, Image};
    let mut detection = Detection3D {
        id: "cup-1".into(),
        ..Default::default()
    };
    detection.header.frame_id = "camera".into();
    detection.header.stamp.sec = 12;
    detection.header.stamp.nanosec = 345;
    detection.bbox.center.position.x = 1.0;
    detection.bbox.center.position.y = 2.0;
    detection.bbox.center.position.z = 3.0;
    detection.bbox.center.orientation.x = 0.1;
    detection.bbox.center.orientation.y = 0.2;
    detection.bbox.center.orientation.z = 0.3;
    detection.bbox.center.orientation.w = 0.9;
    detection.bbox.size.x = 0.4;
    detection.bbox.size.y = 0.5;
    detection.bbox.size.z = 0.6;
    detection.results.push(Default::default());
    detection.results[0].hypothesis.class_id = "cup".into();
    detection.results[0].hypothesis.score = 0.95;
    detection.results[0].pose.pose = detection.bbox.center.clone();
    detection.results[0].pose.covariance = std::array::from_fn(|i| i as f64 / 10.0);
    let mut image = Image {
        header: detection.header.clone(),
        height: 1,
        width: 2,
        encoding: "rgb8".into(),
        is_bigendian: 1,
        step: 6,
        data: vec![1, 2, 3, 4, 5, 6],
    };
    let expected_detection = detection.clone();
    let expected_image = image.clone();
    let mut graph = SceneGraph::new();
    let cup = graph
        .add_object_node(
            "cup",
            Some(&detection),
            [Some(&image), None, None, None, Some(&image)],
        )
        .unwrap();
    detection.id = "changed after insertion".into();
    image.data[0] = 99;

    let mut objects = graph.object_nodes();
    assert_eq!(objects[0].id, cup);
    assert_eq!(objects[0].class_name, "cup");
    assert_eq!(objects[0].detection.as_ref(), Some(&expected_detection));
    assert_eq!(
        objects[0].visuals,
        [
            Some(expected_image.clone()),
            None,
            None,
            None,
            Some(expected_image)
        ]
    );
    objects[0].detection.as_mut().unwrap().id = "changed returned copy".into();
    assert_eq!(
        graph.object_nodes()[0].detection.as_ref(),
        Some(&expected_detection)
    );

    let plain = graph.add_object_node("plain", None, [None; 5]).unwrap();
    let plain = graph
        .object_nodes()
        .into_iter()
        .find(|node| node.id == plain)
        .unwrap();
    assert!(plain.detection.is_none());
    assert_eq!(plain.visuals, [None, None, None, None, None]);
}

#[test]
fn python_equivalent_snapshot_preserves_payloads_without_wire_serialization() {
    use semantic_scene_forests_rs::{Detection3D, Image};
    let detection = Detection3D {
        id: "cup-1".into(),
        ..Default::default()
    };
    let mut image = Image::default();
    image.header.frame_id = "camera".into();
    image.data = vec![42];
    let mut graph = SceneGraph::new();
    let cup = graph
        .add_object_node(
            "cup",
            Some(&detection),
            [Some(&image), None, None, None, None],
        )
        .unwrap();
    let on = graph.add_concept_node("on").unwrap();
    graph.add_edge(cup, on, "relation").unwrap();
    assert_eq!(
        graph.add_edge(on, cup, "invalid").unwrap_err().kind,
        GraphErrorKind::InvalidEdge
    );

    let snapshot = graph.try_clone().unwrap();
    assert_eq!(snapshot.object_nodes(), graph.object_nodes());
    assert_eq!(
        snapshot.outgoing_edges(cup).unwrap(),
        graph.outgoing_edges(cup).unwrap()
    );
    let message = graph.to_msg();
    let loaded = SceneGraph::from_msg(&message).unwrap();
    assert_eq!(
        loaded.outgoing_edges(cup).unwrap(),
        graph.outgoing_edges(cup).unwrap()
    );
    let loaded_objects = loaded.object_nodes();
    assert!(loaded_objects[0].class_name.is_empty());
    assert!(loaded_objects[0].detection.as_ref().unwrap().id.is_empty());
    assert!(loaded_objects[0].visuals.iter().all(Option::is_none));

    let extra = graph.add_concept_node("original only").unwrap();
    assert!(!snapshot.has_node(extra));
    graph.update_from_msg(&SceneGraphMsg::default()).unwrap();
    assert!(!graph.has_node(cup));
    let retained = snapshot.object_nodes();
    assert_eq!(retained[0].detection.as_ref(), Some(&detection));
    assert_eq!(retained[0].visuals[0].as_ref(), Some(&image));
    assert_eq!(snapshot.outgoing_edges(cup).unwrap()[0].0, on);
}
