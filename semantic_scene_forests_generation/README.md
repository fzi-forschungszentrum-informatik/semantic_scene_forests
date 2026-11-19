# semantic_scene_forests_generation

ROS 2 component that generates a semantic scene graph from managed objects with a two-phase VLM/LLM pipeline.

## Task

- Consume persistent object state from the object manager.
- Run per-object tasks for instance attributes.
- Run whole-graph tasks for higher-level semantics.
- Publish the current graph and expose an action to refresh it.

## Runtime Interface

| Kind | Name | Type |
|---|---|---|
| Subscribe | `/managed_objects` | `semantic_scene_forests_object_msgs/msg/ManagedObjectArray` |
| Publish | `/scene_graph` | `semantic_scene_forests_msgs/msg/SceneGraph` |
| Action server | `/update_scene_graph` | `semantic_scene_forests_msgs/action/UpdateSceneGraph` |
| Service client | `/get_object_visuals` | `semantic_scene_forests_object_msgs/srv/GetObjectVisuals` |

The graph is published at 1 Hz with reliable, transient-local QoS.

## Update Behavior

- Each new managed object joins a queue that a background worker describes with the Phase 1 tasks,
  one object at a time. A failed object is logged and counted in diagnostics, and is only retried
  by a manual Phase 1.
- Every `phase2_interval_seconds` (default 120) the worker runs Phase 2 over concepts it has not
  processed yet, and skips the LLM if there are none. A failed run retries at the next tick.
- `/update_scene_graph` is the manual rerun: Phase 1 redescribes all objects and Phase 2 reprocesses
  all concepts. The worker pauses after its current step while a goal runs, and only one goal runs
  at a time.
- `/add_scene_context` reports busy while an update step is in flight.
- Tasks run against a private graph snapshot. The live graph is locked only while a snapshot is
  taken or a successful update is committed.
- LLM connection, LLM request, and object-visuals request timeouts are configurable through
  `llm_connect_timeout_seconds`, `llm_request_timeout_seconds`, and
  `visuals_request_timeout_seconds`.
- Cancellation is checked at worker checkpoints. An in-flight HTTP call or plugin task must
  return before the goal can finish cancellation.
- Teardown stops the worker cooperatively at the next checkpoint and waits for the current
  blocking call; a goal caught in teardown receives no terminal result.
- When the managed-object array changes, the graph is pruned to the retained objects: absent
  objects, their incident edges, and concepts no longer reachable from any retained object are
  removed, while concepts shared with retained objects and their transitive ancestors survive.
  An empty managed array empties the graph.
- Updates keep running while objects come and go: each commit drops objects that were removed
  meanwhile, and Phase 1 skips objects that are gone before their turn.

## Exported Components

- Component: `semantic_scene_forests_generation::SceneGraphGenerationNode`
- Auto-generated executable: `semantic_scene_forests_generation_node`
- Plugin base classes:
  - `semantic_scene_forests_generation::IObjectTask`
  - `semantic_scene_forests_generation::IGraphTask`

## Built-in Task Plugins

- Phase 1: `ColorTask`, `MaterialTask`, `ManipulationTask`
- Phase 2: `CategoryTask`, `AffordanceTask`

## Entry Points

- Launch file: `launch/semantic_scene_forests_generation.launch.xml`
- Plugin description: `semantic_scene_forests_generation_plugins.xml`

## Limitations

- Action-result requests racing ROS shutdown can still trigger an upstream `rclcpp_action`
  executor exception; no final result is delivered after shutdown.
