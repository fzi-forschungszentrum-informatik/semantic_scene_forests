# semantic_scene_forests

ROS 2 monorepo for building, querying, and visualizing semantic scene graphs from tracked objects.

## Pipeline

1. `semantic_scene_forests_object_tracker` turns detections and images into `ObjectCandidate` messages.
2. `semantic_scene_forests_object_manager` keeps persistent object state and publishes `ManagedObjectArray`.
3. `semantic_scene_forests_generation` turns managed objects into a `SceneGraph` and serves graph updates.
4. `semantic_scene_forests_query` answers language-driven queries against the current graph.
5. `semantic_scene_forests_visualization_rqt` and `semantic_scene_forests_visualization_rviz` expose operator-facing views.

## Main Runtime Interfaces

| Interface | Type | Role |
|---|---|---|
| `object_candidates` | `semantic_scene_forests_object_msgs/msg/ObjectCandidate` | Tracker output |
| `managed_objects` | `semantic_scene_forests_object_msgs/msg/ManagedObjectArray` | Persistent object state |
| `/scene_graph` | `semantic_scene_forests_msgs/msg/SceneGraph` | Current semantic graph |
| `/update_scene_graph` | `semantic_scene_forests_msgs/action/UpdateSceneGraph` | Trigger phase 1 or phase 2 graph updates |
| `/add_scene_context` | `semantic_scene_forests_msgs/srv/AddSceneContext` | Append context to an existing node without replacing graph data |
| `/get_object_visuals` | `semantic_scene_forests_object_msgs/srv/GetObjectVisuals` | Fetch image crops for object IDs |
| `find_object` | `semantic_scene_forests_planning_msgs/action/FindObject` | Resolve an object from a natural-language description |
| `prompt_llm` | `semantic_scene_forests_planning_msgs/action/PromptLLM` | General graph-aware LLM query |

### Append-only agent context

`/add_scene_context` accepts `node_id` and `context`. It creates a `ConceptNode`
and the edge `node_id --has_context--> context_node_id`, returning `success`,
`message`, `context_node_id`, and `created`. The source can be an existing object
or concept; object sources must still be active in the object manager. No delete,
rename, replacement, or arbitrary-edge operation is exposed.

Context uses the existing concept-label rules: trimmed spaces/tabs, nonempty,
at most 128 UTF-8 bytes, and no control characters. An identical request for the
same source reuses its context node (`created=false`), so an explicit retry after
an unknown timeout outcome does not duplicate the annotation. Requests fail while
generation is busy; accepted additions are published immediately.

`has_context` denotes agent-supplied information, not a sensor observation or an
instruction. Context has the graph's in-memory lifetime: normal lifecycle pruning
can remove it when its source becomes unreachable, and restarting does not persist
it. It does not authorize movement or manipulation.

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
| `semantic_scene_forests_object_tracker` | Tracks objects across observations | Detections in, `ObjectCandidate` out |
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

- `semantic_scene_forests_docker/` contains container and dev environment support and is intentionally not a ROS package.
- Each package has its own brief `README.md` describing its task and interface.
