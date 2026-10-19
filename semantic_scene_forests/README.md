# semantic_scene_forests

Core C++ library for representing and serializing semantic scene graphs.

## Task

- Store object nodes, concept nodes, and labeled directed edges.
- Convert between the in-memory graph and `semantic_scene_forests_msgs/msg/SceneGraph`.
- Provide the graph API used by generation, query, and visualization packages.

## Exported Interface

- Shared library: `semantic_scene_forests`
- Main headers:
  - `include/semantic_scene_forests/scene_graph.hpp`
  - `include/semantic_scene_forests/graph_nodes.hpp`
  - `include/semantic_scene_forests/graph_edge.hpp`
  - `include/semantic_scene_forests/error.hpp`

## Key API

- `fzi::semantic_scene_forests::SceneGraph`
  - add/remove object and concept nodes
  - add/remove labeled edges
  - query nodes and edges
  - `toMsg()` and `fromMsg()` for ROS message conversion
  - `updateSceneGraphFromMsg` applies updates in place; on the first invalid operation,
    earlier valid changes remain (partial success, not a transaction)

## Python Bindings

The `semantic_scene_forests_py` ROS package provides a Python-native binding for this API. Build it with `colcon`, source the workspace, then use `import semantic_scene_forests`. See that package's README for usage and ROS message conversion details.

## Dependencies

- Consumes `semantic_scene_forests_msgs` as the wire format.
- Used directly by `semantic_scene_forests_generation`, `semantic_scene_forests_query`, and `semantic_scene_forests_visualization_rqt`.
