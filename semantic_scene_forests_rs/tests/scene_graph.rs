use rclrs::MessageIDL;
use ros_env::sensor_msgs::msg::Image;
use semantic_scene_forests_rs::SceneGraphMsg;
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
