# semantic_scene_forests_object_tracker

Tracking package that converts detections into persistent object candidates for the rest of the stack.

## Task

- Synchronize 2D detections, 3D detections, and camera images.
- Maintain object tracks over time.
- Publish `ObjectCandidate` messages with geometry and image crops.

## Exported Components

- `semantic_scene_forests_object_tracker::ObjectTrackerComponent`

## Runtime Interface

### ObjectTrackerComponent

| Kind | Name | Type |
|---|---|---|
| Subscribe | `detections_2d` | `vision_msgs/msg/Detection2DArray` |
| Subscribe | `detections_3d` | `vision_msgs/msg/Detection3DArray` |
| Subscribe | `camera/image` | `sensor_msgs/msg/Image` |
| Publish | `object_candidates` | `semantic_scene_forests_object_msgs/msg/ObjectCandidate` |


## Entry Points

- Launch file: `launch/object_tracker.launch.xml`
- Default configs: `config/object_tracker.yaml`
