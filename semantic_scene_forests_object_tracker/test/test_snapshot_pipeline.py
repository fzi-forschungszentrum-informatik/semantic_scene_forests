# Copyright (c) 2026 FZI Forschungszentrum Informatik
# SPDX-License-Identifier: BSD-3-Clause

"""One synthetic ROS pipeline, with only the external HTTP inference stubbed.

Use the same observation/merge sequence as test_tracking_lifecycle.cpp, but run
all four real components through their public ROS transport and action APIs.
"""

import json
import os
from pathlib import Path
import subprocess
import threading
import time
from contextlib import ExitStack
from http.server import BaseHTTPRequestHandler, HTTPServer

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from composition_interfaces.srv import LoadNode, UnloadNode
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from semantic_scene_forests_msgs.action import UpdateSceneGraph
from semantic_scene_forests_msgs.msg import SceneGraph
from semantic_scene_forests_object_msgs.msg import (
    ManagedObjectArray,
    ObjectCandidateArray,
)
from semantic_scene_forests_planning_msgs.action import FindObject
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)


def test_snapshot_membership_reaches_manager_graph_and_query(tmp_path):
    # Empty generation task lists still send a setup request. Return a fixed
    # acknowledgement; queries deliberately name a chosen ID, even if retired.
    # This tests membership enforcement, NOT model answer quality.
    class InferenceStub(BaseHTTPRequestHandler):
        answer_id = 1

        def log_message(self, *_args):
            pass

        def reply(self, body):
            payload = json.dumps(body).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            self.reply({"data": []})

        def do_POST(self):
            self.rfile.read(int(self.headers["Content-Length"]))
            content = json.dumps(
                {"found": True, "object_id": self.answer_id, "message": "fixed"}
            )
            self.reply(
                {
                    "id": "synthetic",
                    "object": "chat.completion",
                    "created": 0,
                    "model": "synthetic",
                    "choices": [
                        {
                            "index": 0,
                            "message": {"role": "assistant", "content": content},
                            "finish_reason": "stop",
                        }
                    ],
                    "usage": {
                        "prompt_tokens": 1,
                        "completion_tokens": 1,
                        "total_tokens": 2,
                    },
                }
            )

    with ExitStack() as cleanup:
        server = HTTPServer(("127.0.0.1", 0), InferenceStub)
        cleanup.callback(server.server_close)
        worker = threading.Thread(target=server.serve_forever)
        worker.start()
        cleanup.callback(worker.join)
        cleanup.callback(server.shutdown)
        # Register before init as well: try_shutdown tolerates incomplete init.
        cleanup.callback(rclpy.try_shutdown)
        rclpy.init()
        node = rclpy.create_node(
            "snapshot_pipeline_io", namespace=f"/task5_{os.getpid()}"
        )
        cleanup.callback(node.destroy_node)
        namespace = node.get_namespace()
        executable = (
            Path(get_package_prefix("rclcpp_components"))
            / "lib/rclcpp_components/component_container_mt"
        )
        log = cleanup.enter_context((tmp_path / "components.log").open("w+"))

        def print_log():
            log.seek(0)
            print(log.read())

        cleanup.callback(print_log)
        process = subprocess.Popen(
            [
                str(executable),
                "--ros-args",
                "-r",
                "__node:=pipeline",
                "-r",
                f"__ns:={namespace}",
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
        )

        def stop_container():
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()

        cleanup.callback(stop_container)
        latest = {}
        received = {"snapshot": 0}
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        def wait(predicate, timeout=8):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                assert process.poll() is None, "component container exited"
                rclpy.spin_once(node, timeout_sec=0.02)
            assert predicate(), "timed out waiting for pipeline state"

        def result(future):
            wait(future.done)
            return future.result()

        def record(key, msg):
            latest[key] = msg
            if key == "snapshot":
                received[key] += 1

        def action(client, goal):
            assert client.wait_for_server(timeout_sec=8)
            handle = result(client.send_goal_async(goal))
            assert handle.accepted
            return result(handle.get_result_async())

        def graph_ids():
            return {obj.id for obj in latest["graph"].object_nodes}

        def managed_ids():
            return {obj.id for obj in latest["managed"].objects}

        _subscriptions = [  # Keep handles alive for the whole sequence.
            node.create_subscription(
                msg_type, topic, lambda msg, key=key: record(key, msg), qos
            )
            for key, msg_type, topic in [
                ("snapshot", ObjectCandidateArray, "snapshots"),
                ("managed", ManagedObjectArray, "managed_objects"),
                ("graph", SceneGraph, "scene_graph"),
            ]
        ]
        load = node.create_client(LoadNode, "pipeline/_container/load_node")
        unload = node.create_client(UnloadNode, "pipeline/_container/unload_node")
        assert load.wait_for_service(timeout_sec=8)

        def component(package, plugin, params):
            request = LoadNode.Request(
                package_name=package, plugin_name=plugin, node_namespace=namespace
            )
            # Generation uses absolute names; explicitly keep this test isolated.
            request.remap_rules = [
                f"/{name}:={namespace}/{name}"
                for name in [
                    "managed_objects",
                    "scene_graph",
                    "update_scene_graph",
                    "get_object_visuals",
                ]
            ]
            request.remap_rules.append("object_candidate_snapshots:=snapshots")
            for name, value in params.items():
                if isinstance(value, list):
                    pv = ParameterValue(
                        type=ParameterType.PARAMETER_STRING_ARRAY,
                        string_array_value=value,
                    )
                elif isinstance(value, str):
                    pv = ParameterValue(
                        type=ParameterType.PARAMETER_STRING, string_value=value
                    )
                elif isinstance(value, int):
                    pv = ParameterValue(
                        type=ParameterType.PARAMETER_INTEGER, integer_value=value
                    )
                else:
                    pv = ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE, double_value=value
                    )
                request.parameters.append(Parameter(name=name, value=pv))
            response = result(load.call_async(request))
            assert response.success, response.error_message
            return response.unique_id

        component(
            "semantic_scene_forests_object_manager",
            "semantic_scene_forests_object_manager::ObjectManagerNode",
            {"frame_id": "map"},
        )
        endpoint = f"http://127.0.0.1:{server.server_port}/v1"
        component(
            "semantic_scene_forests_generation",
            "semantic_scene_forests_generation::SceneGraphGenerationNode",
            {
                "server_url": endpoint,
                "phase1_tasks": [],
                "phase2_tasks": [],
                "frame_id": "map",
            },
        )
        component(
            "semantic_scene_forests_query",
            "semantic_scene_forests_query::SceneGraphQueryNode",
            {"server_url": endpoint},
        )
        tracker_id = component(
            "semantic_scene_forests_object_tracker",
            "semantic_scene_forests_object_tracker::ObjectTrackerComponent",
            {
                "confirmation_threshold": 2,
                "measurement_noise_std": 0.01,
                "gate_threshold_confirmed": 1000.0,
                "gate_threshold_strict": 1000.0,
            },
        )
        two_pub = node.create_publisher(
            Detection2DArray, "detections_2d", qos_profile_sensor_data
        )
        three_pub = node.create_publisher(
            Detection3DArray, "detections_3d", qos_profile_sensor_data
        )
        image_pub = node.create_publisher(
            Image, "camera/image", qos_profile_sensor_data
        )
        tf_pub = node.create_publisher(TFMessage, "/tf_static", qos)
        transform = TransformStamped()
        transform.header.frame_id = "map"
        transform.child_frame_id = f"task5_camera_{os.getpid()}"
        transform.transform.translation.x = 1.0
        transform.transform.rotation.w = 1.0
        tf_pub.publish(TFMessage(transforms=[transform]))
        wait(
            lambda: all(
                pub.get_subscription_count() for pub in [two_pub, three_pub, image_pub]
            )
        )
        wait(lambda: "snapshot" in latest and "managed" in latest and "graph" in latest)
        assert not latest["snapshot"].objects

        def batch(seconds, observations):
            two, three = Detection2DArray(), Detection3DArray()
            three.header.frame_id = transform.child_frame_id
            three.header.stamp.sec = seconds
            three.header.stamp.nanosec = 123456789
            two.header = three.header
            for index, (x, label, size) in enumerate(observations):
                a, b = Detection2D(), Detection3D()
                a.header = b.header = three.header
                a.id = b.id = str(index + 1)
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = label
                hypothesis.hypothesis.score = 0.99
                a.results = b.results = [hypothesis]
                a.bbox.center.position.x = a.bbox.center.position.y = 16.0
                a.bbox.size_x = a.bbox.size_y = 8.0
                b.bbox.center.position.x, b.bbox.center.position.z = x, 1.0
                b.bbox.center.orientation.w = 1.0
                b.bbox.size.x = b.bbox.size.y = b.bbox.size.z = size
                two.detections.append(a)
                three.detections.append(b)
            image = Image(
                header=three.header,
                height=32,
                width=32,
                encoding="bgr8",
                step=96,
                data=[127] * (32 * 96),
            )
            before = received["snapshot"]
            two_pub.publish(two)
            three_pub.publish(three)
            image_pub.publish(image)
            wait(lambda: received["snapshot"] > before)
            expected = {obj.id: obj.last_seen for obj in latest["snapshot"].objects}
            wait(
                lambda: (
                    {obj.id: obj.last_seen for obj in latest["managed"].objects}
                    == expected
                )
            )

        update = ActionClient(node, UpdateSceneGraph, "update_scene_graph")
        find = ActionClient(node, FindObject, "find_object")

        def generate(expected):
            outcome = action(
                update, UpdateSceneGraph.Goal(phase=UpdateSceneGraph.Goal.PHASE_1)
            )
            assert (
                outcome.status == GoalStatus.STATUS_SUCCEEDED and outcome.result.success
            )
            wait(lambda: graph_ids() == expected)

        def query(object_id, active):
            InferenceStub.answer_id = object_id
            outcome = action(
                find, FindObject.Goal(object_description="return the fixed test ID")
            )
            if active:
                assert outcome.status == GoalStatus.STATUS_SUCCEEDED
                assert outcome.result.success and outcome.result.object_id == object_id
            else:
                assert outcome.status == GoalStatus.STATUS_ABORTED
                assert not outcome.result.success and outcome.result.object_id == -1
                assert "not in the scene snapshot" in outcome.result.message

        batch(10, [(0.0, "box", 0.4)])
        batch(11, [(0.0, "box", 0.8)])
        assert managed_ids() == {1}
        a = latest["snapshot"].objects[0]
        assert abs(a.detection.bbox.center.position.x - 1.0) < 1e-6
        assert abs(a.detection.bbox.size.x - 0.48) < 1e-6
        assert a.detection.header.frame_id == "map"
        assert a.last_seen == a.detection.header.stamp
        assert a.last_seen.sec == 11 and a.last_seen.nanosec == 123456789
        assert a.header.stamp == latest["snapshot"].header.stamp
        assert a.header.stamp != a.last_seen
        generate({1})
        query(1, True)
        batch(17, [])
        assert managed_ids() == {1} and graph_ids() == {1}
        assert latest["managed"].objects[0].last_seen == a.last_seen
        assert latest["snapshot"].objects[0].last_seen == a.last_seen
        batch(18, [(0.05, "chair", 0.4)])
        assert managed_ids() == {1}  # Tentative B cannot retire A.
        batch(19, [(0.05, "chair", 0.4)])
        assert managed_ids() == {2}
        assert {obj.id for obj in latest["snapshot"].objects} == {2}
        wait(lambda: graph_ids() == set())  # B needs an explicit generation update.
        query(1, False)
        generate({2})
        query(2, True)

        batch(20, [(3.0, "box", 0.4), (3.8, "box", 0.4)])
        batch(21, [(3.0, "box", 0.4), (3.8, "box", 0.4)])
        assert managed_ids() == {2, 3, 4}
        generate({2, 3, 4})
        query(4, True)
        batch(22, [(3.25, "box", 0.4), (3.55, "box", 0.4)])
        batch(23, [(3.38, "box", 0.4), (3.42, "box", 0.4)])
        assert managed_ids() == {2, 3}
        assert {obj.id for obj in latest["snapshot"].objects} == {2, 3}
        wait(lambda: graph_ids() == {2, 3})
        query(4, False)

        # A session reset is authoritative; unload the tracker first so there
        # remains exactly one snapshot publisher, never two competing owners.
        assert result(
            unload.call_async(UnloadNode.Request(unique_id=tracker_id))
        ).success
        clear_pub = node.create_publisher(ObjectCandidateArray, "snapshots", qos)
        wait(lambda: clear_pub.get_subscription_count() >= 2)
        clear_pub.publish(ObjectCandidateArray())
        wait(
            lambda: (
                not latest["snapshot"].objects and not managed_ids() and not graph_ids()
            )
        )
        query(2, False)
