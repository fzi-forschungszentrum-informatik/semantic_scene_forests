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
    let cup = graph.add_object_node("cup").unwrap();
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
