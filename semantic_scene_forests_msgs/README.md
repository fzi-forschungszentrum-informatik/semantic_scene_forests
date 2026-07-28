# semantic_scene_forests_msgs

ROS 2 interface package for scene graph data and graph update actions.

## Task

- Define the wire format shared by graph generation, query, and visualization packages.
- Carry the current scene graph and graph-update requests over ROS.

## Interfaces

### Messages

- `msg/SceneGraph.msg`
- `msg/Edge.msg`
- `msg/ObjectNode.msg`
- `msg/ConceptNode.msg`

### Actions

- `action/UpdateSceneGraph.action`
  - goal: choose `PHASE_1` or `PHASE_2`
  - result: `success`, `message`
  - feedback: current progress and status text

## Used By

- `semantic_scene_forests`
- `semantic_scene_forests_generation`
- `semantic_scene_forests_query`
- `semantic_scene_forests_visualization_rqt`
- `semantic_scene_forests_visualization_rviz`
