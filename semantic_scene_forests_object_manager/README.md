# semantic_scene_forests_object_manager

Stateful middleware component that turns tracker output into managed object state.

## Task

- Accept tracked object candidates.
- Keep the latest managed object set.
- Provide editing and blocking operations for operators or downstream tools.
- Serve stored image crops for graph generation and inspection.

## Runtime Interface

| Kind | Name | Type |
|---|---|---|
| Subscribe | `object_candidates` | `semantic_scene_forests_object_msgs/msg/ObjectCandidate` |
| Publish | `managed_objects` | `semantic_scene_forests_object_msgs/msg/ManagedObjectArray` |
| Service | `get_objects` | `semantic_scene_forests_object_msgs/srv/GetObjects` |
| Service | `block_object` | `semantic_scene_forests_object_msgs/srv/BlockObject` |
| Service | `edit_object` | `semantic_scene_forests_object_msgs/srv/EditObject` |
| Service | `unblock_object` | `semantic_scene_forests_object_msgs/srv/UnblockObject` |
| Service | `get_object_visuals` | `semantic_scene_forests_object_msgs/srv/GetObjectVisuals` |

`managed_objects` is published with reliable, transient-local QoS so late subscribers receive the latest state.

## Semantics

- Blocking an active object deletes its payload; only the ID is kept in a process-local
  suppression set. Candidates for a blocked ID are rejected until it is unblocked.
- Unblocking does not restore the old payload — the next candidate for that ID creates a
  fresh object.
- The published `managed_objects` array is authoritative: consumers must treat it as the
  complete active set. An empty array means an empty scene.

## Exported Component

- `semantic_scene_forests_object_manager::ObjectManagerNode`

Downstream C++ consumers can `find_package(semantic_scene_forests_object_manager)` and link
`semantic_scene_forests_object_manager::object_manager_component`; the public headers and the
generated parameter header are installed with the package.

## Entry Points

- Launch file: `launch/object_manager.launch.xml`
- Default config: `config/object_manager.yaml`
