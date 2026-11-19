# semantic_scene_forests_msgs

ROS 2 interface package for scene graph data and worker actions.

## Task

- Define the wire format shared by graph generation, query, and visualization packages.
- Carry the current scene graph (including object data) and worker run requests over ROS.

## Interfaces

### Messages

- `msg/SceneGraph.msg`
- `msg/Edge.msg`
- `msg/ObjectNode.msg`
- `msg/ConceptNode.msg`

### Services

- `srv/AddConcept.srv`: create a new concept and connect it to `source_id` with `relation`; fails if
  a concept with the label already exists
- `srv/GetConcept.srv`: look up a concept id by label (`found=false` if there is none)
- `srv/AddEdge.srv`: add a labeled edge between two existing nodes
- `srv/SaveSceneGraph.srv`: save the graph as a `.sg` bundle under `~/.ros/scene_graphs/`; returns
  the written `path`

### Actions

- `action/RunWorker.action`: manual run of a manager worker (`attribute_objects`,
  `generate_concepts`)
  - goal: empty
  - result: `success`, `message`
  - feedback: `current`, `total`, status text

## Used By

- `semantic_scene_forests`
- `semantic_scene_forests_generation`
- `semantic_scene_forests_query`
- `semantic_scene_forests_visualization_rqt`
- `semantic_scene_forests_visualization_rviz`
