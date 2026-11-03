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

The tracker has one input triple per configured camera: `detections_2d`, `detections_3d` and
`camera/image`. The three messages of a frame must carry the **identical** stamp (exact-time
sync, queue `sync_queue_size`); mismatched frames are dropped. Each 2D and 3D array must contain
matching nonempty detector IDs; IDs only pair the 2D and 3D detections of one frame. A silent
camera does not hold up other cameras; a missing transform skips that camera's frame after the
TF lookup timeout.

### Configuration

`camera_names` is a nonempty, unique startup-only list. Each name selects a
`cameras.<name>` entry with `detections_2d_topic`, `detections_3d_topic`, `image_topic` and
`image_rotation` (0/90/180/270 degrees clockwise). The rotation is applied to the image and the
2D boxes before cropping, so crops come out upright; 3D boxes are TF geometry and unaffected.
The shipped single-camera configuration uses the `default` camera and the logical topics
`detections_2d`, `detections_3d`, and `camera/image`, so the existing launch remaps continue to
work.

The two-camera example is
`config/object_tracker_multi_camera.yaml`. It uses explicit absolute topics
for `front` and `rear`; the legacy single-camera remap arguments do not replace
these explicit camera topics. Camera inputs are read at startup and are not
hot-reloaded.

### Object identity and coordinate requirements

Detector IDs are local synchronization keys only. The tracker allocates the global IDs sent to
the manager. Labels never gate a geometric match inside a camera; each track's label is a vote
over its observations. Across cameras and revisits, labels and the arbiter decide (see below).
A moving object that changes world position beyond the geometric gate needs motion or
appearance evidence that this tracker does not provide.

All cameras must use a common clock, compatible class labels, and valid TF from
each detection frame to `world_frame`. Configure the manager with the same
frame. Crops retain their source image headers and are bounded by the shared
`max_crops_per_object` limit. Complete snapshots reconcile retired IDs in the manager;
restart the tracker and manager together when resetting the tracker ID allocator.

Snapshots use reliable, transient-local, depth-1 QoS, including an initial empty snapshot.
They are published by the global cycle at `snapshot_rate_hz` (default 5 Hz, must be > 0) and
contain every global track with its crops. The combined launch enables intra-process comms so
the snapshot moves to the manager without serialization.

TODO: if the container still drops input, the six raw camera image streams arriving over
zenoh are the next suspect; co-locate the tracker with the detector nodes or use compressed
images.
One tracker owns this topic. Override it using `object_candidate_snapshots_topic` in the
tracker, manager or combined launch; the old per-candidate topic/argument is retired.

## Architecture and identity semantics

```
 camera A (own callback group)        camera B ...           global cycle (own group, timer)
 ExactTime(2D, 3D, image)                                     collect all mailboxes
 CameraTracker: rotate, filter, TF,                           GlobalTracker: link / merge /
 dedup, predict, associate, crops  ─► mailbox (tracklets) ─►  reactivate / replace / arbiter
                                                              publish snapshot
                                     ArbiterWorker thread ◄─► request / result queues
```

- **Per camera** (`CameraTracker`): a frame is accepted only if its stamp is newer than the
  camera's last accepted frame (restart tracker and manager together before rewinding a
  recording). Detections below `confidence_threshold`, outside `class_whitelist` or without a 3D
  box are dropped; boxes are transformed to `world_frame`; same-frame duplicates (similar size,
  overlap >= `merge_overlap_threshold`) keep the highest score. Association is class-agnostic:
  confirmed tracks first (gate `gate_threshold_confirmed`), then tentative ones
  (`gate_threshold_strict`), both within `max_association_distance` and similar size
  (largest dimensions within `association_max_size_ratio`). Tracks confirm after
  `confirmation_threshold` observations; tentative tracks end after `unconfirmed_track_timeout`,
  confirmed ones `local_track_timeout` after their last observation.
- **Tracklets**: each accepted frame posts the camera's confirmed tracks to its mailbox.
- **Global** (`GlobalTracker`, every cycle):
  - a tracklet stays linked to its global track while it lives; several tracklets (cameras) may
    link to one global track, which takes one Kalman update per new tracklet observation and
    collects the crops;
  - a new tracklet links to a global track (stale or not) with similar size, overlap >=
    `merge_overlap_threshold` and an accepted label; otherwise it starts a new global track;
  - two global tracks of similar size overlapping by more than `arbiter.min_overlap` merge
    (older ID survives) when a label is accepted by the other and the overlap is clear;
    otherwise (label conflict or partial overlap) the pair goes to the arbiter. Without an
    arbiter, or after a failed request, clear overlaps merge and partial ones stay apart;
  - a global track is **stale** after `stale_timeout` (60 s) unseen. A similar track at its pose
    reactivates it (via the rules above); a differently sized track covering at least
    `replacement_overlap_threshold` of it **replaces** it.
- **Label check** (arbiter only): a new global track is held back from the snapshot until the
  arbiter answers two yes/no questions in one call: "do the crops show a <label>?" and "is
  <label> a physical object rather than a room or area category (kitchen, office, hallway)?".
  If both reach `arbiter.label_probability`, the label is confirmed (it sticks until a new label
  appears); otherwise the track is removed as a false positive. A failed request publishes the track with its voted label. Reactivated stale tracks
  are not checked again.
- **Arbiter** (`ArbiterWorker`, `arbiter.enabled`): a background thread works through a request
  queue (label checks and stale pairs first) back to back, without a rate limit; the global cycle only enqueues
  and drains. It sends both tracks' crops (base64) plus labels and sizes and asks whether they
  are the same object and which label fits. p(same) >= `arbiter.same_object_probability`
  merges the pair and records both labels as accepted, so later tracklets with either label link
  without a new request; otherwise the pair is recorded as distinct and never merged (a stale
  one is replaced). Each pair is asked once. `arbiter.provider` selects `openai`, `openrouter`
  or `ollama`; `server_url` and `model` must match it. An empty `api_key` falls back to
  `OPENAI_API_KEY` / `OPENROUTER_API_KEY` (Ollama needs none).

## Initial tuning (not calibrated)

| Parameter | Initial value | Meaning |
|---|---|---|
| `local_track_timeout` | 5.0 s | End of a camera's track after its last observation |
| `stale_timeout` | 60.0 s | Global track becomes stale (reactivate or replace on revisit) |
| `sync_queue_size` | 100 | Exact-sync queue; must cover detector latency at the image rate |
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

`test_bbox_transform.cpp`, `test_tracked_object.cpp`, `test_image_rotation.cpp`,
`test_camera_tracker.cpp` and `test_global_tracker.cpp` cover the
plain classes with synthetic inputs (no ROS threads, no HTTP). `test_multi_camera_tracker.cpp`
covers exact sync, camera handover, delayed observations, snapshot retirement and manager
integration. Generation and query nodes are not launched. No dataset
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
