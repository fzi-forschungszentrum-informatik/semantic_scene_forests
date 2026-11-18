# semantic_scene_forests_py

Python bindings for the `semantic_scene_forests` C++ graph library. Build the package in a ROS 2 workspace, source the result, then import `semantic_scene_forests`.

## Build

```bash
colcon build --symlink-install --packages-up-to semantic_scene_forests_py
source install/setup.bash
```

## Usage

```python
from builtin_interfaces.msg import Time
from vision_msgs.msg import BoundingBox3D

from semantic_scene_forests import GraphError, ObjectNode, SceneGraph

bbox = BoundingBox3D()
bbox.size.x = 0.1
stamp = Time(sec=12)

graph = SceneGraph()
cup = graph.allocate_id()
graph.add_object(ObjectNode(cup, "cup", bbox, "map", last_seen=stamp, last_updated=stamp))
graph.offer_crop(cup, stamp, jpeg_bytes)  # crops are (stamp, JPEG bytes)
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

`ObjectNode.bbox`, `last_seen`, `last_updated` and the stamps of `ObjectNode.crops` are copied between Python and C++; they do not share mutable message storage with the graph. `ObjectNode.workers` is a `dict[str, bool]` (worker name to processed flag). Atomic helpers (`update_object_pose`, `offer_crop`, `set_crop_policy`, `set_object_class`, `find_or_add_concept`, `find_concept`, `add_concept_edge`, `mark_processed`, `unprocessed_by`, `prune_isolated_concepts`) mirror the C++ methods. Object and concept ids are positive and come from one counter.

## ROS Message Conversion

`SceneGraph.to_msg()`, `SceneGraph.from_msg()`, and `SceneGraph.update_from_msg()` accept or return `semantic_scene_forests_msgs.msg.SceneGraph` objects. Object messages carry the id, class name, bbox, frame, source and timestamps; crops and worker state are not part of the wire format and are not retained by a `to_msg()` then `from_msg()` round trip.
