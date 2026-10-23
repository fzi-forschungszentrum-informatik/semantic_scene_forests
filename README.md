# semantic_scene_forests

ROS 2 monorepo for building, querying, and visualizing semantic scene graphs from tracked objects.

## Pipeline

One `scene_graph_manager` node (`semantic_scene_forests_manager`) owns an in-memory SQLite
database and hosts worker plugins that poll it:

1. `tracker` (`semantic_scene_forests_object_tracker::TrackerWorker`) turns detections and images
   into objects with geometry and crops.
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
| `/scene_graph` | `semantic_scene_forests_msgs/msg/SceneGraph` | Current semantic graph, including object nodes with class, pose and timestamps |
| `/get_objects` | `semantic_scene_forests_object_msgs/srv/GetObjects` | Current object list |
| `/get_object_visuals` | `semantic_scene_forests_object_msgs/srv/GetObjectVisuals` | Fetch image crops for object IDs |
| `/block_object`, `/unblock_object`, `/edit_object` | `semantic_scene_forests_object_msgs/srv/*` | Operator object edits |
| `/add_observations` | `semantic_scene_forests_object_msgs/srv/AddObservations` | Register sightings from a non-camera source (tracker worker) |
| `/add_scene_context` | `semantic_scene_forests_msgs/srv/AddSceneContext` | Append context to an existing node without replacing graph data |
| `/attribute_objects`, `/generate_concepts` | `semantic_scene_forests_msgs/action/RunWorker` | Manually rerun phase 1 or phase 2 |
| `find_object` | `semantic_scene_forests_planning_msgs/action/FindObject` | Resolve an object from a natural-language description |
| `prompt_llm` | `semantic_scene_forests_planning_msgs/action/PromptLLM` | General graph-aware LLM query |

### Append-only agent context

`/add_scene_context` accepts `node_id` and `context`. It creates a `ConceptNode`
and the edge `node_id --has_context--> context_node_id`, returning `success`,
`message`, `context_node_id`, and `created`. The source can be an existing object
or concept; object sources must not be blocked. No delete,
rename, replacement, or arbitrary-edge operation is exposed.

Context uses the existing concept-label rules: trimmed spaces/tabs, nonempty,
at most 128 UTF-8 bytes, and no control characters. An identical request for the
same source reuses its context node (`created=false`), so an explicit retry after
an unknown timeout outcome does not duplicate the annotation. Accepted additions are
written to the database and published with the next graph update.

`has_context` denotes agent-supplied information, not a sensor observation or an
instruction. Context has the database's in-memory lifetime: restarting the manager
does not persist it. It does not authorize movement or manipulation.

## Packages

| Package | Task | Primary interface |
|---|---|---|
| `semantic_scene_forests` | In-memory scene graph library | C++ headers and shared library |
| `semantic_scene_forests_py` | Python bindings for the in-memory graph | `import semantic_scene_forests` |
| [`semantic_scene_forests_rs`](semantic_scene_forests_rs/README.md) | Initial Rust bindings for the same C++ graph | Cargo library and in-package rclrs example |
| `semantic_scene_forests_generation` | LLM workers that add attributes and concepts | `ObjectAttributionWorker`, `ConceptGenerationWorker` |
| `semantic_scene_forests_launch` | Starts the integrated stack | `scene_graph_combined.launch.xml` |
| `semantic_scene_forests_manager` | `scene_graph_manager` node, database and pruning worker | `/scene_graph`, object services, worker plugins |
| `semantic_scene_forests_msgs` | Graph message and action definitions | `SceneGraph`, `Edge`, `RunWorker` |
| `semantic_scene_forests_object_msgs` | Object message and service definitions | `ManagedObject`, `GetObjectVisuals`, edits/blocking |
| `semantic_scene_forests_object_tracker` | Tracks objects across observations | Detections in, database objects out |
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
- All IDs are database IDs: objects and concepts share one positive ID space that is never
  reused within a session.
- Blocking hides an object and ignores tracker updates for it; unblocking keeps its class,
  pose and crop overrides.
- `last_seen` is the latest accepted 3D observation time; `last_updated` describes state
  modification. Manual edits do not advance `last_seen`.
- Brief occlusion can retain an ID; a nearby, similarly sized object may replace a stale one,
  even across classes. See [tracker semantics and tuning](semantic_scene_forests_object_tracker/README.md).
- The database lives in memory: restart the manager for a new session, including recording
  rewind/loop; do not mix observation time domains. There is no persistence.
- Tests cover the database, the manager node and each worker on a real in-memory database. No
  full-stack test, dataset or hardware validation is included; the initial tuning values
  require calibration on recordings/hardware.

- `semantic_scene_forests_docker/` contains container and dev environment support and is intentionally not a ROS package.
- Each package has its own brief `README.md` describing its task and interface.
