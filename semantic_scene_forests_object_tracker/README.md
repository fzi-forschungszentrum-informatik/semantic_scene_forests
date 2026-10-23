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

The tracker has one independently synchronized input triple per configured camera:
`detections_2d`, `detections_3d`, and `camera/image`. Each 2D and 3D array must
contain matching nonempty detector IDs, and an image is required for the
corresponding observation. IDs must be unique within each camera's batch;
they may change or repeat across frames and cameras. A silent camera does not
hold up other cameras; a missing transform skips that camera's batch after the
TF lookup timeout.

### Configuration

`camera_names` is a nonempty, unique startup-only list. Each name selects a
`cameras.<name>` entry with `detections_2d_topic`, `detections_3d_topic`, and
`image_topic`. The shipped single-camera configuration uses the `default`
camera and the logical topics `detections_2d`, `detections_3d`, and
`camera/image`, so the existing launch remaps continue to work.

The two-camera example is
`config/object_tracker_multi_camera.yaml`. It uses explicit absolute topics
for `front` and `rear`; the legacy single-camera remap arguments do not replace
these explicit camera topics. Camera inputs are read at startup and are not
hot-reloaded.

### Object identity and coordinate requirements

Detector IDs are local synchronization keys only. They are not persistent
identity: equal IDs from different cameras do not force an association, and a
detector may recycle an ID for a different object. The tracker allocates the
global IDs sent to the manager and associates
observations using class labels and calibrated geometry in `world_frame`. A
stationary object seen first by one camera and later by another can therefore
keep the same global ID even when the local detector IDs differ. A moving object
that changes world position beyond the geometric gate needs motion or appearance
evidence that this tracker does not provide.
Nearby same-class objects can still be confused by the geometric matcher.

All cameras must use a common clock, compatible class labels, and valid TF from
each detection frame to `world_frame`. Configure the manager with the same
frame. Crops retain their source image headers and are bounded by the shared
`max_crops_per_object` limit. The manager has no retirement/remapping protocol,
so two already-published global IDs are never silently merged; restart the
tracker and manager together when resetting the tracker ID allocator.


## Entry Points

- Launch file: `launch/object_tracker.launch.xml`
- Default config: `config/object_tracker.yaml`
- Multi-camera example: `config/object_tracker_multi_camera.yaml`

Run the standalone tracker with a custom configuration using:

```bash
ros2 launch semantic_scene_forests_object_tracker object_tracker.launch.xml \
  config_file:=/path/to/object_tracker_multi_camera.yaml
```
