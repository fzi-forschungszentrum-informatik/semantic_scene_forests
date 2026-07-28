# semantic_scene_forests_launch

Launch package for running the integrated semantic scene forests stack.

## Task

- Start the tracker, object manager, graph generation, and query components in one container.
- Provide shared parameter files and diagnostic aggregation setup.

## Main Entry Points

- Launch file: `launch/scene_graph_combined.launch.xml`
- Helper script: `scripts/launch_semantic_scene_forests.sh`

## Launch Surface

Key launch arguments exposed by `scene_graph_combined.launch.xml`:

- `scene_graph_frame`
- `camera_image_topic`
- `detections_2d_topic`
- `detections_3d_topic`
- `enable_diagnostics`
- `use_sim_time`

## Managed Components

- `semantic_scene_forests_object_tracker::ObjectTrackerComponent`
- `semantic_scene_forests_object_manager::ObjectManagerNode`
- `semantic_scene_forests_generation::SceneGraphGenerationNode`
- `semantic_scene_forests_query::SceneGraphQueryNode`
- `diagnostic_aggregator/aggregator_node` (optional)
