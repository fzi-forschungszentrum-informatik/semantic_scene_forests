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
observations class-agnostically using calibrated geometry in `world_frame`; detector labels
flicker, so each track's label is a majority vote over its observations. A
stationary object seen first by one camera and later by another can therefore
keep the same global ID even when the local detector IDs differ. A moving object
that changes world position beyond the geometric gate needs motion or appearance
evidence that this tracker does not provide.
Nearby objects of comparable size can still be confused by the geometric matcher.

All cameras must use a common clock, compatible class labels, and valid TF from
each detection frame to `world_frame`. Configure the manager with the same
frame. Crops retain their source image headers and are bounded by the shared
`max_crops_per_object` limit. Complete snapshots reconcile retired IDs in the manager;
restart the tracker and manager together when resetting the tracker ID allocator.

Snapshots use reliable, transient-local, depth-1 QoS, including an initial empty snapshot.
They are published at `snapshot_rate_hz` (default 2 Hz) rather than per camera frame, because
each snapshot carries every confirmed object with its crops; `0` publishes after every
synchronized frame (used by the tests). The combined launch enables intra-process comms so
the snapshot moves to the manager without serialization.

TODO: if the container still drops input, the six raw camera image streams arriving over
zenoh are the next suspect; co-locate the tracker with the detector nodes or use compressed
images.
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
  Ordinary association requires the Mahalanobis gate, `max_association_distance` and a
  similar size (largest dimensions within `association_max_size_ratio`); labels are not
  compared. Association age uses the incoming observation time, so a delayed camera can
  continue observing an object; prediction and snapshot aging use the latest accepted time
  across cameras. Brief occlusion can preserve an ID.
- Overlap is the fraction of the smaller box inside the other, with extents padded to
  `overlap_min_extent`: depth often yields near-zero extents along the viewing ray, which makes
  IoU and raw volumes meaningless. Same-frame detections of one object (YOLOE often reports
  several labels) are reduced to the highest-scoring one before association.
- Older confirmed tracks are **stale**: they remain in snapshots, are not matched directly and
  stop accumulating prediction uncertainty. When a confirmed recent track appears at a stale
  track's pose:
  - similar size and overlap >= `merge_overlap_threshold`: the stale track is **reactivated**
    (keeps its ID, takes the new state, crops and labels);
  - otherwise, with the arbiter enabled, the Decisions API compares both tracks' crops and
    either reactivates (same object) or **replaces** the stale track (something else);
  - without an arbiter answer (disabled or failed), the stale track is replaced when the new
    track covers at least `replacement_overlap_threshold` of it.
- Recent tracks of similar size overlapping by at least `merge_overlap_threshold` are merged
  (older ID survives, newest observation state, bounded crop/history merge). Partially
  overlapping recent pairs (`arbiter.min_overlap` < overlap < `merge_overlap_threshold`) go to
  the arbiter. With p(same) >= `arbiter.same_object_probability` they are merged and the
  chosen label sticks until a new label is observed. The arbiter has one request in flight at
  a time, at most one per `arbiter.min_interval_seconds` (1 s), and asks each pair once. The
  key is passed by the combined launch from `OPENROUTER_API_KEY`. The final snapshot follows
  reactivation, replacement and merging, so retired IDs cannot linger downstream.
- The active-track cap counts tentative/recent tracks, not stale tracks. Expired tentative
  tracks are pruned before admission; stale objects are never evicted just to make room.

## Initial tuning (not calibrated)

| Parameter | Initial value | Meaning |
|---|---|---|
| `reacquisition_timeout` | 5.0 s | Inclusive ordinary identity-reuse age |
| `max_association_distance` | 0.5 m | Hard physical association cap |
| `association_max_size_ratio` | 2.0 | Maximum ratio of largest box dimensions for "similar" |
| `merge_overlap_threshold` | 0.5 | Overlap of the smaller box that means one object |
| `replacement_overlap_threshold` | 0.5 | Coverage of a stale box for replacement without arbiter |
| `overlap_min_extent` | 0.1 m | Box extent padding for overlap computations |
| `bbox_size_alpha` | 0.2 | Measurement weight in dimension filtering |

Confirmation count and unconfirmed timeout remain unchanged. Tune distance/size thresholds
against recordings for large furniture versus small tabletop objects; there is no per-class
parameter machinery. The world-frame box transform algorithm remains unchanged.

## Validation

`test_bbox_transform.cpp`, `test_tracked_object.cpp` and `test_tracking_lifecycle.cpp`
cover geometry, observation bookkeeping and the tracker node's lifecycle using synthetic
inputs. `test_multi_camera_tracker.cpp` covers camera handover, delayed observations, snapshot
retirement and manager integration. Generation and query nodes are not launched. No dataset
or hardware calibration is implied.

## Entry Points

- Launch file: `launch/object_tracker.launch.xml`
- Default config: `config/object_tracker.yaml`
- Multi-camera example: `config/object_tracker_multi_camera.yaml`

Run the standalone tracker with a custom configuration using:

```bash
ros2 launch semantic_scene_forests_object_tracker object_tracker.launch.xml \
  config_file:=/path/to/object_tracker_multi_camera.yaml
```
