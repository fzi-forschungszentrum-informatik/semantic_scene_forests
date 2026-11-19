# semantic_scene_forests_visualization_rviz

RViz display plugin for showing managed objects and scene graph context in 3D.

## Task

- Visualize managed-object geometry in RViz.
- Provide scene-aware inspection support, including image viewing for selected objects.
- Give operators a spatial view that complements the RQT graph view.

## Plugin Interface

- Display class: `semantic_scene_forests_visualization_rviz::SceneGraphDisplay`
- Plugin id: `semantic_scene_forests_visualization_rviz/SceneGraph`

## Runtime Interface

| Kind | Name | Type |
|---|---|---|
| Display input | configurable RViz topic (default `scene_graph`) | `semantic_scene_forests_msgs/msg/SceneGraph` (object nodes drawn as boxes) |
| Action client | `attribute_objects` | `semantic_scene_forests_msgs/action/RunWorker` |
| Action client | `generate_concepts` | `semantic_scene_forests_msgs/action/RunWorker` |

## Entry Point

- Plugin manifest: `semantic_scene_forests_display_plugin.xml`
