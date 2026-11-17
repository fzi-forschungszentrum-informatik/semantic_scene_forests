# semantic_scene_forests_generation

Worker plugins for the `scene_graph_manager` that add semantics to the objects in the shared SceneGraph with
a two-phase VLM/LLM pipeline.

## Task

- `ObjectAttributionWorker` (phase 1): run per-object tasks for instance attributes.
- `ConceptGenerationWorker` (phase 2): run whole-graph tasks for higher-level semantics.
- Write results to the manager's shared SceneGraph; the manager publishes `/scene_graph`.

## Runtime Interface

Both workers are loaded by `semantic_scene_forests_manager::SceneGraphManagerNode` and serve their
action on the manager node. Parameters live under the worker's configured name
(`object_attribution.*`, `concept_generation.*`).

| Kind | Name | Type |
|---|---|---|
| Action server | `attribute_objects` | `semantic_scene_forests_msgs/action/RunWorker` |
| Action server | `generate_concepts` | `semantic_scene_forests_msgs/action/RunWorker` |

## Update Behavior

- `ObjectAttributionWorker` polls every `poll_interval_seconds` (default 1.0) for visible objects
  it has not described yet and describes them one at a time with its `tasks`, reading crops from
  the graph. A failed object is logged, marked, and only retried by a manual run.
- `ConceptGenerationWorker` runs every `interval_seconds` (default 120) over concepts it has not
  processed yet, and skips the LLM if there are none. A failed run retries at the next tick.
- `attribute_objects` reruns phase 1 over all visible objects; `generate_concepts` reruns phase 2
  over all visible concepts. Each action is reject-while-busy, one goal at a time.
- Tasks work on the live shared graph. Each concept+edge write is one atomic call, made after the
  LLM returns; no lock is held during an LLM call. A rejected or invalid write is logged and skipped.
  An object deleted before its turn is skipped. The graph is a shared workspace: no worker owns
  concepts or edges.
- LLM connection and request timeouts are `llm_connect_timeout_seconds` and
  `llm_request_timeout_seconds`. Cancellation is checked at task boundaries; an in-flight HTTP
  call must return first.
- Teardown stops the workers cooperatively at the next checkpoint and waits for the current
  blocking call; a goal caught in teardown receives no terminal result.

## Exported Plugins

- Workers (`semantic_scene_forests_generation_workers.xml`):
  - `semantic_scene_forests_generation::ObjectAttributionWorker`
  - `semantic_scene_forests_generation::ConceptGenerationWorker`
- Task plugin base classes:
  - `semantic_scene_forests_generation::IObjectTask`
  - `semantic_scene_forests_generation::IGraphTask`

## Built-in Task Plugins

- Phase 1: `ColorTask`, `MaterialTask`, `ManipulationTask`
- Phase 2: `CategoryTask`, `AffordanceTask`

## Entry Points

- Task plugin description: `semantic_scene_forests_generation_plugins.xml`
- Started by `ros2 launch semantic_scene_forests_launch scene_graph_combined.launch.xml`

## Limitations

- Action-result requests racing ROS shutdown can still trigger an upstream `rclcpp_action`
  executor exception; no final result is delivered after shutdown.
