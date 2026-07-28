# semantic_scene_forests_py

Python bindings for the `semantic_scene_forests` C++ graph library. Build the package in a ROS 2 workspace, source the result, then import `semantic_scene_forests`.

## Build

```bash
colcon build --symlink-install --packages-up-to semantic_scene_forests_py
source install/setup.bash
```

## Usage

```python
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection3D

from semantic_scene_forests import GraphError
from semantic_scene_forests import SceneGraph

detection = Detection3D()
detection.id = "cup-1"
image = Image()
image.header.frame_id = "camera"

graph = SceneGraph()
cup = graph.add_object_node("cup", detection, [image, None, None, None, None])
on = graph.add_concept_node("on")
graph.add_edge(cup, on, "relation")

for target_id, edge in graph.outgoing_edges(cup):
    print(target_id, edge.label)

try:
    graph.add_edge(on, cup, "invalid")
except GraphError as error:
    print(error.kind, error)

snapshot = graph.clone()
message = graph.to_msg()
loaded = SceneGraph.from_msg(message)
```

Methods use snake_case. Mutating operations return their successful C++ values and raise `GraphError` with a `GraphErrorKind` on graph failures. `find_node()` returns `None` for an absent ID.

`ObjectNode.detection` and each item of `ObjectNode.visuals` are copied between Python and C++; they do not share mutable message storage with the graph.

## ROS Message Conversion

`SceneGraph.to_msg()`, `SceneGraph.from_msg()`, and `SceneGraph.update_from_msg()` accept or return `semantic_scene_forests_msgs.msg.SceneGraph` objects. They preserve the current C++ wire format exactly: object graph messages contain object IDs only. Consequently, an object class name, `Detection3D`, and image visuals are intentionally not retained by a `to_msg()` then `from_msg()` round trip.
