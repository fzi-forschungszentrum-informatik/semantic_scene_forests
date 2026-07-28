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

- Only one `/update_scene_graph` goal runs at a time.
- Tasks run against a private graph snapshot. The live graph is locked only while a snapshot is
  taken or a successful update is committed.
- LLM connection, LLM request, and object-visuals request timeouts are configurable through
  `llm_connect_timeout_seconds`, `llm_request_timeout_seconds`, and
  `visuals_request_timeout_seconds`.
- Cancellation is checked between external requests. An in-flight LLM request completes or times
  out before the goal can finish cancellation.

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
