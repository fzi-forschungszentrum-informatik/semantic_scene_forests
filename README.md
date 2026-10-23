# semantic_scene_forests

ROS 2 monorepo for building, querying, and visualizing semantic scene graphs from tracked objects.

## Pipeline

1. `semantic_scene_forests_object_tracker` turns detections and images into authoritative `ObjectCandidateArray` snapshots.
2. `semantic_scene_forests_object_manager` reconciles these snapshots with operator overrides and publishes `ManagedObjectArray`.
3. `semantic_scene_forests_generation` turns managed objects into a `SceneGraph` and serves graph updates.
4. `semantic_scene_forests_query` answers language-driven queries against the current graph.
5. `semantic_scene_forests_visualization_rqt` and `semantic_scene_forests_visualization_rviz` expose operator-facing views.

## Main Runtime Interfaces

| Interface | Type | Role |
|---|---|---|
| `object_candidate_snapshots` | `semantic_scene_forests_object_msgs/msg/ObjectCandidateArray` | Complete confirmed tracker state |
| `managed_objects` | `semantic_scene_forests_object_msgs/msg/ManagedObjectArray` | Complete managed object state |
| `/scene_graph` | `semantic_scene_forests_msgs/msg/SceneGraph` | Current semantic graph |
| `/update_scene_graph` | `semantic_scene_forests_msgs/action/UpdateSceneGraph` | Trigger phase 1 or phase 2 graph updates |
| `/get_object_visuals` | `semantic_scene_forests_object_msgs/srv/GetObjectVisuals` | Fetch image crops for object IDs |
| `find_object` | `semantic_scene_forests_planning_msgs/action/FindObject` | Resolve an object from a natural-language description |
| `prompt_llm` | `semantic_scene_forests_planning_msgs/action/PromptLLM` | General graph-aware LLM query |

## Packages

| Package | Task | Primary interface |
|---|---|---|
| `semantic_scene_forests` | In-memory scene graph library | C++ headers and shared library |
| `semantic_scene_forests_py` | Python bindings for the in-memory graph | `import semantic_scene_forests` |
| [`semantic_scene_forests_rs`](semantic_scene_forests_rs/README.md) | Initial Rust bindings for the same C++ graph | Cargo library and in-package rclrs example |
| `semantic_scene_forests_generation` | Builds semantic graphs from managed objects | `ManagedObjectArray` in, `SceneGraph` out |
| `semantic_scene_forests_launch` | Starts the integrated stack | `scene_graph_combined.launch.xml` |
| `semantic_scene_forests_msgs` | Graph message and action definitions | `SceneGraph`, `Edge`, `UpdateSceneGraph` |
| `semantic_scene_forests_object_manager` | Maintains managed object lifecycle | Services plus `managed_objects` topic |
| `semantic_scene_forests_object_msgs` | Object message and service definitions | `ManagedObject`, `GetObjectVisuals`, edits/blocking |
| `semantic_scene_forests_object_tracker` | Tracks objects across observations | Detections in, `ObjectCandidateArray` out |
| `semantic_scene_forests_planning_msgs` | Query/planning action definitions | `FindObject`, `PromptLLM` |
| `semantic_scene_forests_query` | Language query interface over the graph | Action servers on current graph state |
| `semantic_scene_forests_utils` | Shared utility headers | Diagnostic helpers |
| `semantic_scene_forests_visualization_rqt` | Interactive graph inspection UI | RQT plugin |
| `semantic_scene_forests_visualization_rviz` | 3D RViz display | RViz display plugin |

## Build

```bash
colcon build --symlink-install --packages-up-to semantic_scene_forests_launch
```

## Notes

- `object_candidate_snapshots` replaces the former per-object `object_candidates` topic;
  launch overrides now use `object_candidate_snapshots_topic`. Rebuild downstream message
  consumers and source the rebuilt overlay: the object message layout has changed.
- Tracker snapshots and managed arrays use reliable, transient-local, depth-1 QoS.
  Each is the complete current set: omitted IDs are retired; an empty array clears it.
  Absence from a detection batch is different: confirmed objects remain remembered.
- `last_seen` and the detection header stamp are the latest accepted 3D observation time.
  Snapshot/candidate headers describe publication time; managed `last_updated` describes
  state modification. Prediction, republication and manual edits do not advance `last_seen`.
- Brief occlusion can retain an ID; after `reacquisition_timeout`, a new observation starts
  a new ID. A nearby, similarly sized confirmed object may replace an unambiguously matched
  remembered object, even across classes. An identical returning object cannot be distinguished
  from a replacement. See [tracker semantics and tuning](semantic_scene_forests_object_tracker/README.md).
- Generation removes retired object nodes (and unreachable concepts); query sanitizes graph
  membership against managed state and rejects stale/fabricated positive IDs. New IDs still
  need the existing `/update_scene_graph` phase-1 workflow; no automatic inference is scheduled.
- One tracker owns the snapshot topic. Restart tracker and manager together for a new session,
  including recording rewind/loop; do not mix observation time domains. There is no persistence
  or ID epoch. Remembered objects can accumulate for the lifetime of a session.
- Tracker and manager tests exercise their own logic and node interfaces independently.
  No full-stack test, dataset or hardware validation is included; the initial tuning values
  require calibration on recordings/hardware.

- `semantic_scene_forests_docker/` contains container and dev environment support and is intentionally not a ROS package.
- Each package has its own brief `README.md` describing its task and interface.
