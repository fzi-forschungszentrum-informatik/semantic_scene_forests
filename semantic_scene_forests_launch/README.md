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
- `config_dir`
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

## Multi-camera tracker configuration

The default `config/object_tracker.yaml` defines one `default` camera with the
logical topics `detections_2d`, `detections_3d`, and `camera/image`. The
standalone tracker launch and the manager launch keep their existing remap
arguments for these names. Use those remaps for a single camera; a multi-camera
configuration selects each input directly in its `cameras.<name>` entries.

The tracker package includes a complete two-camera example at
`config/object_tracker_multi_camera.yaml`. To use it with the combined stack,
copy it into a configuration directory as `object_tracker.yaml`, keep the
other stack configuration files in that directory, and launch with:

```bash
ros2 launch semantic_scene_forests_launch scene_graph_combined.launch.xml \
  config_dir:=/path/to/config_dir
```

The manager launch can run the tracker with the same file directly:

```bash
ros2 launch semantic_scene_forests_object_manager object_manager.launch.xml \
  enable_tracker:=true \
  tracker_config_file:=/path/to/object_tracker_multi_camera.yaml
```

The combined launch's `scene_graph_frame` is applied to both the tracker and
manager, so it must be the calibrated common frame for every camera. All camera
streams must share a compatible clock and class labels. Inputs are startup-only;
restart the tracker and manager together when resetting tracker IDs.
