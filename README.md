# semantic_scene_forests

ROS 2 monorepo for building, querying, and visualizing semantic scene graphs from tracked objects.

## Pipeline

One `scene_graph_manager` node (`semantic_scene_forests_manager`) owns the thread-safe `SceneGraph`,
the single shared model, and hosts worker plugins that read and write it:

1. `tracker` (`semantic_scene_forests_object_tracker::TrackerWorker`) keeps the track state, turns
   detections and images into object nodes with geometry and crops, and serves `add_object`.
2. `object_attribution` (`semantic_scene_forests_generation::ObjectAttributionWorker`) describes
   new objects with per-object LLM tasks.
3. `concept_generation` (`semantic_scene_forests_generation::ConceptGenerationWorker`) adds
   higher-level concepts with whole-graph LLM tasks.
4. `pruning` (`semantic_scene_forests_manager::PruningWorker`) deletes concepts without edges.

The manager publishes the graph, objects included, on `/scene_graph` and serves the object
services. `semantic_scene_forests_query` answers language-driven queries against that graph, and
`semantic_scene_forests_visualization_rqt` / `semantic_scene_forests_visualization_rviz` expose
operator-facing views.

## Main Runtime Interfaces

| Interface | Type | Role |
|---|---|---|
| `/scene_graph` | `semantic_scene_forests_msgs/msg/SceneGraph` | Current semantic graph, including object nodes with class, bbox and timestamps |
| `/get_objects` | `semantic_scene_forests_object_msgs/srv/GetObjects` | Current object nodes (`semantic_scene_forests_msgs/ObjectNode[]`) |
| `/get_object_visuals` | `semantic_scene_forests_object_msgs/srv/GetObjectVisuals` | Fetch image crops for object IDs (JPEG decoded to `sensor_msgs/Image`) |
| `/delete_object` | `semantic_scene_forests_object_msgs/srv/DeleteObject` | Remove an object node; camera sightings do not recreate it; only an explicit `add_observations` sighting, or a merge with a live track, revives it under the same id |
| `/edit_object` | `semantic_scene_forests_object_msgs/srv/EditObject` | Set an object's class name |
| `/add_object` | `semantic_scene_forests_object_msgs/srv/AddObject` | Tracker: add a manual object as a confirmed track. Returns `object_id` at once; the node appears at the next tracker cycle (~200 ms at 5 Hz) |
| `/add_observations` | `semantic_scene_forests_object_msgs/srv/AddObservations` | Register sightings from a non-camera source (tracker worker) |
| `/add_concept` | `semantic_scene_forests_msgs/srv/AddConcept` | Create a new concept and connect it to a source node; fails if the label exists |
| `/get_concept` | `semantic_scene_forests_msgs/srv/GetConcept` | Look up a concept id by label |
| `/add_edge` | `semantic_scene_forests_msgs/srv/AddEdge` | Add a labeled edge between two existing nodes |
| `/attribute_objects`, `/generate_concepts` | `semantic_scene_forests_msgs/action/RunWorker` | Manually rerun phase 1 or phase 2 |
| `find_object` | `semantic_scene_forests_planning_msgs/action/FindObject` | Resolve an object from a natural-language description |
| `prompt_llm` | `semantic_scene_forests_planning_msgs/action/PromptLLM` | General graph-aware LLM query |

### Adding concepts with `add_concept`

`/add_concept` accepts `source_id`, `relation` and `label`. It creates a new concept
and adds the edge `source_id --relation--> concept` as one atomic call. It returns `concept_id`,
`success` and `message`. If a concept with that label already exists it fails and changes
nothing: look the id up with `/get_concept` (`label` -> `concept_id`, `found`, `message`) and
link it with `/add_edge`. A new concept always comes with its edge, so it is never orphaned: if
the edge is rejected (cycle, unknown source), the new concept is removed again. Labels are trimmed, nonempty,
at most 128 UTF-8 bytes, and free of control characters. `/add_edge` adds a labeled edge
between two existing nodes. No delete, rename, or replacement operation is exposed.

The graph lives in memory: restarting the manager does not persist concepts or edges.

## Packages

| Package | Task | Primary interface |
|---|---|---|
| `semantic_scene_forests` | In-memory scene graph library | C++ headers and shared library |
| `semantic_scene_forests_py` | Python bindings for the in-memory graph | `import semantic_scene_forests` |
| [`semantic_scene_forests_rs`](semantic_scene_forests_rs/README.md) | Initial Rust bindings for the same C++ graph | Cargo library and in-package rclrs example |
| `semantic_scene_forests_generation` | LLM workers that add attributes and concepts | `ObjectAttributionWorker`, `ConceptGenerationWorker` |
| `semantic_scene_forests_launch` | Starts the integrated stack | `scene_graph_combined.launch.xml` |
| `semantic_scene_forests_manager` | `scene_graph_manager` node, shared graph and pruning worker | `/scene_graph`, object services, worker plugins |
| `semantic_scene_forests_msgs` | Graph message, service and action definitions | `SceneGraph`, `ObjectNode`, `AddConcept`, `GetConcept`, `AddEdge`, `RunWorker` |
| `semantic_scene_forests_object_msgs` | Object message and service definitions | `ObjectCandidate`, `GetObjects`, `GetObjectVisuals`, `AddObject`, `DeleteObject`, `EditObject` |
| `semantic_scene_forests_object_tracker` | Tracks objects across observations | Detections in, graph object nodes out |
| `semantic_scene_forests_planning_msgs` | Query/planning action definitions | `FindObject`, `PromptLLM` |
| `semantic_scene_forests_query` | Language query interface over the graph | Action servers on current graph state |
| `semantic_scene_forests_utils` | Shared utility headers | Diagnostic helpers |
| `semantic_scene_forests_visualization_rqt` | Interactive graph inspection UI | RQT plugin |
| `semantic_scene_forests_visualization_rviz` | 3D RViz display | RViz display plugin |

## Build

```bash
colcon build --symlink-install --packages-up-to semantic_scene_forests_launch
```

## Pre-commit checks

With `pre-commit` installed, run this once per clone to enable checks on commits:

```bash
pre-commit install
```

Run the same checks as CI manually with:

```bash
pre-commit run --all-files --show-diff-on-failure
```

Review and stage any formatting fixes made by the hooks, then rerun the checks.
The hooks check formatting and file hygiene; use `colcon build` to verify compilation.

## Notes

- The old `object_manager` and `semantic_scene_forests_generation` nodes, the
  `object_candidate_snapshots` / `managed_objects` topics and the `/update_scene_graph` action are
  gone. Objects are published inside `/scene_graph`; clients read them from there or from
  `/get_objects`. Phase 1 and phase 2 reruns are the `/attribute_objects` and
  `/generate_concepts` actions.
- All IDs come from one counter: objects and concepts share one positive ID space that is never
  reused within a session.
- `delete_object` removes a node and its edges. The tracker stops writing that track until it
  retires, and never recreates the node. A corrected position comes in through `add_observations`
  or `add_object`; there is no pose override.
- `last_seen` is the latest accepted 3D observation time; `last_updated` is the observation
  time of the most recent pose/bounding-box change. Label edits and crop updates change neither.
- Brief occlusion can retain an ID; a nearby, similarly sized object may replace a stale one,
  even across classes. See [tracker semantics and tuning](semantic_scene_forests_object_tracker/README.md).
- The graph lives in memory: restart the manager for a new session, including recording
  rewind/loop; do not mix observation time domains. There is no persistence.
- Tests cover the SceneGraph, the manager node and each worker on the real in-memory graph. No
  full-stack test, dataset or hardware validation is included; the initial tuning values
  require calibration on recordings/hardware.

- `semantic_scene_forests_docker/` contains container and dev environment support and is intentionally not a ROS package.
- Each package has its own brief `README.md` describing its task and interface.
