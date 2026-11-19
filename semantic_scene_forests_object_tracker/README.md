# semantic_scene_forests_object_tracker

Tracking package that converts detections into persistent objects in the shared scene graph of the `scene_graph_manager`.

## Task

- Synchronize 2D detections, 3D detections, and camera images.
- Maintain object tracks over time.
- Write every global track as an object node (box, `last_seen`, bounded JPEG crops) to the scene graph each global cycle. The tracker keeps all track state; class votes after creation never touch the graph, and a node removed by `delete_object` is not recreated by camera sightings; an `add_observations` sighting of it (or a merge with a live track) brings it back under the same ID.

## Exported Worker Plugin

- `semantic_scene_forests_object_tracker::TrackerWorker`, loaded by
  `semantic_scene_forests_manager::SceneGraphManagerNode`. Its parameters live under the worker's
  configured name (`tracker.*` in `semantic_scene_forests_launch/config/scene_graph_manager.yaml`).

## Runtime Interface

### TrackerWorker (topics and services on the manager node)

| Kind | Name | Type |
|---|---|---|
| Subscribe | `detections_2d` | `vision_msgs/msg/Detection2DArray` |
| Subscribe | `detections_3d` | `vision_msgs/msg/Detection3DArray` |
| Subscribe | `camera/image` | `sensor_msgs/msg/Image` |
| Service | `add_observations` | `semantic_scene_forests_object_msgs/srv/AddObservations` |
| Service | `add_object` | `semantic_scene_forests_object_msgs/srv/AddObject` |

The tracker has one input triple per configured camera: `detections_2d`, `detections_3d` and
`camera/image`. The three messages of a frame must carry the **identical** stamp (exact-time
sync, queue `sync_queue_size`); mismatched frames are dropped. Each 2D and 3D array must contain
matching nonempty detector IDs; IDs only pair the 2D and 3D detections of one frame. A silent
camera does not hold up other cameras; a missing transform skips that camera's frame after the
TF lookup timeout.

### Manual observations

`add_observations` registers sightings from a source other than the configured cameras, e.g.
an agent's camera search, so that every object, wherever it was seen, gets its ID from this
tracker. Each observation (class name, framed 3D box with positive size, optional crops) is
transformed into `world_frame` using TF at its stamp (a zero stamp means now) and joins the next
global cycle as a tracklet of the reserved source `<manual>`. Negative timestamps and
unavailable transforms reject that observation. The usual global rules
apply: it links to a clearly overlapping, similar-sized track with an accepted label (updating
its pose and crops), or it starts a new global track, held for the arbiter's label check when
the arbiter is on. The response waits until every observation of the request is decided:
until then its tracklets are reposted each cycle (an unchanged stamp is not applied twice),
so their links follow merges and end when a label check removes the track. It is sent after
the graph write of the deciding cycle, with one global ID (0 = rejected), the published class
name and a `created` flag per observation. Global tracks are never timed out, so a manual
track stays after its tracklet ends. `class_whitelist` and
`confidence_threshold` do not apply; labels rejected for the session do. The service shares
the global callback group, so requests never race the cycle.

### Configuration

`camera_names` is a nonempty, unique startup-only list. Each name selects a
`cameras.<name>` entry with `detections_2d_topic`, `detections_3d_topic`, `image_topic` and
`image_rotation` (0/90/180/270 degrees clockwise). The rotation is applied to the image and the
2D boxes before cropping, so crops come out upright; 3D boxes are TF geometry and unaffected.
The parameter defaults define one `default` camera with the logical topics `detections_2d`,
`detections_3d`, and `camera/image`. The shipped stack configuration
(`semantic_scene_forests_launch/config/scene_graph_manager.yaml`) lists explicit absolute topics
per camera. Camera inputs are read at startup and are not hot-reloaded.

### Object identity and coordinate requirements

Detector IDs are local synchronization keys only. The tracker allocates the global IDs sent to
the manager. Labels never gate a geometric match inside a camera; each track's label is a vote
over its observations. Across cameras and revisits, labels and the arbiter decide (see below).
A moving object that changes world position beyond the geometric gate needs motion or
appearance evidence that this tracker does not provide.

All cameras must use a common clock, compatible class labels, and valid TF from
each detection frame to `world_frame`. The manager's `frame_id` must name the same frame (the
combined launch sets both from `scene_graph_frame`). Crops retain their source image headers and
are bounded by the shared `max_crops_per_object` limit. Crops are stored as JPEG (`crop_jpeg_quality`); the graph takes a newer crop only if it is at
least `crop_replace_min_interval_seconds` newer than the node's newest one. Crops given at creation (`add_object`) are stored as-is up to the limit; later crops follow that interval rule. Global track IDs are
scene graph node IDs; the graph lives in memory, so restarting the manager starts a new ID space.

`add_object` starts a manual track that is confirmed at once (no confirmation count, no label
check) and returns its ID; the node appears through the normal write and the track is then
updated, merged and retired like any other.

The global cycle runs at `snapshot_rate_hz` (default 5 Hz, must be > 0) and writes every changed global
track to the scene graph (one atomic graph call per change).

TODO: if the container still drops input, the six raw camera image streams arriving over
zenoh are the next suspect; co-locate the tracker with the detector nodes or use compressed
images.

## Architecture and identity semantics

```
 camera A (own callback group)        camera B ...           global cycle (own group, timer)
 ExactTime(2D, 3D, image)                                     collect all mailboxes
 CameraTracker: rotate, filter, TF,                           GlobalTracker: link / merge /
 dedup, predict, associate, crops  ─► mailbox (tracklets) ─►  reactivate / replace / arbiter
                                                              write to graph
                                     ArbiterWorker thread ◄─► request / result queues
```

- **Per camera** (`CameraTracker`): a frame is accepted only if its stamp is newer than the
  camera's last accepted frame (restart the manager before rewinding a recording). Detections below `confidence_threshold`, outside `class_whitelist` or without a 3D
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
- **Label check** (arbiter only): a new global track is held back from the graph until the
  arbiter answers two yes/no questions in one call: "do the crops show a <label>?" and "is
  <label> a physical object rather than a room or area category (kitchen, office, hallway)?".
  If both reach `arbiter.label_probability`, the label is confirmed (it sticks until a new label
  appears); otherwise the track is removed as a false positive. If p(physical object) is also below the threshold, the label is rejected for the session: all its tracks are removed and its detections are dropped like non-whitelisted classes. A failed request publishes the track with its voted label. Reactivated stale tracks
  are not checked again.
- **Arbiter** (`ArbiterWorker`, `arbiter.enabled`): a background thread works through a request
  queue (label checks and stale pairs first) back to back, without a rate limit; the global cycle only enqueues
  and drains. It sends both tracks' crops (base64) plus labels and sizes and asks whether they
  are the same object and which label fits. p(same) >= `arbiter.same_object_probability`
  merges the pair and records both labels as accepted, so later tracklets with either label link
  without a new request; otherwise the pair is recorded as distinct and never merged (a stale
  one is replaced). Each pair is asked once. `arbiter.provider` selects `openai`, `openrouter`,
  `ollama` or `vllm` (`vllm serve vllm-sr/d3-nano`, `http://localhost:8000/v1`); `server_url`
  and `model` must match it. `api_key` is required for `openai` and `openrouter` (Ollama and vLLM
  need none).

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
covers exact sync, camera handover and delayed observations through the worker on the real
in-memory SceneGraph. Generation and query are not run. No dataset
or hardware calibration is implied.

## Entry Points

The tracker runs only as a manager worker; start it with
`ros2 launch semantic_scene_forests_launch scene_graph_combined.launch.xml`.
