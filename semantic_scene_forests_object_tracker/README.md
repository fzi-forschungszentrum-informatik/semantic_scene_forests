# semantic_scene_forests_object_tracker

Tracking package that converts detections into persistent object candidates for the rest of the stack.

## Task

- Synchronize 2D detections, 3D detections, and camera images.
- Maintain object tracks over time.
- Publish complete `ObjectCandidateArray` snapshots with geometry and bounded image crops.

## Exported Components

- `semantic_scene_forests_object_tracker::ObjectTrackerComponent`

## Runtime Interface

### ObjectTrackerComponent

| Kind | Name | Type |
|---|---|---|
| Subscribe | `detections_2d` | `vision_msgs/msg/Detection2DArray` |
| Subscribe | `detections_3d` | `vision_msgs/msg/Detection3DArray` |
| Subscribe | `camera/image` | `sensor_msgs/msg/Image` |
| Publish | `object_candidate_snapshots` | `semantic_scene_forests_object_msgs/msg/ObjectCandidateArray` |

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
`max_crops_per_object` limit. Complete snapshots reconcile retired IDs in the manager;
restart the tracker and manager together when resetting the tracker ID allocator.

Snapshots use reliable, transient-local, depth-1 QoS, including an initial empty snapshot.
One tracker owns this topic. Override it using `object_candidate_snapshots_topic` in the
tracker, manager or combined launch; the old per-candidate topic/argument is retired.

## Observation and identity semantics

- A batch is accepted only after synchronizing 2D/3D detections and the image. Identity
  aging uses the **3D observation stamp**, not wall time. Duplicate/out-of-order batches from the same camera
  are dropped without state changes. Restart tracker and manager together before rewinding
  or looping a recording; do not mix observation time domains.
- Candidate `last_seen` and `detection.header.stamp` retain the exact latest accepted 3D
  observation time. Detection geometry/header are in the world frame. Snapshot/candidate
  headers instead carry publication time. Prediction/publication never advance observation time.
- Confirmed tracks remain **recent** through the inclusive `reacquisition_timeout` boundary.
  Ordinary same-class association requires both the Mahalanobis gate and
  `max_association_distance`. Association age uses the incoming observation time, so a delayed
  camera can continue observing an object; prediction and snapshot aging use the latest
  accepted time across cameras. Brief occlusion can preserve an ID.
- Older confirmed tracks are **remembered**: they remain in snapshots, are not ordinarily
  matched, and stop accumulating prediction uncertainty. A new detection starts a new ID;
  an identical returning object and a physical replacement cannot reliably be distinguished.
- After association and confirmation, a recent newly confirmed candidate may retire a
  remembered object only within `replacement_distance` and `replacement_max_size_ratio`
  on every world-AABB dimension. Classes may differ. Recent/currently observed objects,
  tentative candidates, overlap alone, or ambiguous many-to-one/one-to-many pairs do not
  authorize removal. Unresolved pairs may be reconsidered while candidates remain recent.
  A successful replacement keeps the new ID and never inherits operator overrides.
- Duplicate merging applies only to recent tracks and preserves the existing survivor-ID
  rule, newest coherent observation state and bounded crop/history merge. The final snapshot
  follows replacement and merging, so retired IDs cannot linger downstream.
- The active-track cap counts tentative/recent tracks, not remembered tracks. Expired tentative
  tracks are pruned before admission; remembered objects are never evicted just to make room.
  Without positive replacement evidence, remembered storage can grow for the session lifetime.

## Initial tuning (not calibrated)

| Parameter | Initial value | Meaning |
|---|---|---|
| `reacquisition_timeout` | 5.0 s | Inclusive ordinary identity-reuse age |
| `max_association_distance` | 0.5 m | Hard physical association cap |
| `replacement_distance` | 0.2 m | Maximum replacement center displacement |
| `replacement_max_size_ratio` | 2.0 | Maximum per-axis larger/smaller AABB dimension ratio |
| `bbox_size_alpha` | 0.2 | Measurement weight in dimension filtering |

Confirmation count and unconfirmed timeout remain unchanged. Tune distance/size thresholds
against recordings for large furniture versus small tabletop objects; there is no per-class
parameter machinery. The world-frame box transform algorithm remains unchanged.

## Validation

`test_tracking_lifecycle.cpp` covers focused real-ROS lifecycle cases.
`test_snapshot_pipeline.py` runs all four real components with synchronized synthetic
observations and a translated static camera TF. Only HTTP inference is stubbed; empty task
lists still need generation's setup acknowledgement. The sequence checks remembered A's
unchanged timestamp, confirmed replacement B/new ID, duplicate retirement, explicit graph
generation, rejected retired-ID queries and an authoritative empty snapshot. It does not
measure model quality or replace dataset/hardware calibration.

## Entry Points

- Launch file: `launch/object_tracker.launch.xml`
- Default config: `config/object_tracker.yaml`
- Multi-camera example: `config/object_tracker_multi_camera.yaml`

Run the standalone tracker with a custom configuration using:

```bash
ros2 launch semantic_scene_forests_object_tracker object_tracker.launch.xml \
  config_file:=/path/to/object_tracker_multi_camera.yaml
```
