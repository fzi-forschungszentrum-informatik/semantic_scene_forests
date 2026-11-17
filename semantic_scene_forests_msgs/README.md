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

- `srv/AddConcept.srv`: find or create a concept and connect it to `source_id` with `relation`
- `srv/AddEdge.srv`: add a labeled edge between two existing nodes

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
