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

Snapshots use reliable, transient-local, depth-1 QoS, including an initial empty snapshot.
One tracker owns this topic. Override it using `object_candidate_snapshots_topic` in the
tracker, manager or combined launch; the old per-candidate topic/argument is retired.

## Observation and identity semantics

- A batch is accepted only after synchronizing 2D/3D detections and the image. Identity
  aging uses the **3D observation stamp**, not wall time. Duplicate/out-of-order batches
  are dropped without state changes. Restart tracker and manager together before rewinding
  or looping a recording; do not mix observation time domains.
- Candidate `last_seen` and `detection.header.stamp` retain the exact latest accepted 3D
  observation time. Detection geometry/header are in the world frame. Snapshot/candidate
  headers instead carry publication time. Prediction/publication never advance observation time.
- Confirmed tracks remain **recent** through the inclusive `reacquisition_timeout` boundary.
  Ordinary same-class association requires both the Mahalanobis gate and
  `max_association_distance`. Brief occlusion can preserve an ID.
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

`test_bbox_transform.cpp`, `test_tracked_object.cpp` and `test_tracking_lifecycle.cpp`
cover geometry, observation bookkeeping and the tracker node's lifecycle using synthetic
inputs. They do not launch the manager, generation or query nodes. No dataset or hardware
calibration is implied.

## Entry Points

- Launch file: `launch/object_tracker.launch.xml`
- Default configs: `config/object_tracker.yaml`
