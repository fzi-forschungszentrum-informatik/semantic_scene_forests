# semantic_scene_forests_object_manager

Stateful middleware component that turns tracker output into managed object state.

## Task

- Reconcile authoritative tracked object snapshots.
- Keep the latest managed object set.
- Provide editing and blocking operations for operators or downstream tools.
- Serve stored image crops for graph generation and inspection.

## Runtime Interface

| Kind | Name | Type |
|---|---|---|
| Subscribe | `object_candidate_snapshots` | `semantic_scene_forests_object_msgs/msg/ObjectCandidateArray` |
| Publish | `managed_objects` | `semantic_scene_forests_object_msgs/msg/ManagedObjectArray` |
| Service | `get_objects` | `semantic_scene_forests_object_msgs/srv/GetObjects` |
| Service | `block_object` | `semantic_scene_forests_object_msgs/srv/BlockObject` |
| Service | `edit_object` | `semantic_scene_forests_object_msgs/srv/EditObject` |
| Service | `unblock_object` | `semantic_scene_forests_object_msgs/srv/UnblockObject` |
| Service | `get_object_visuals` | `semantic_scene_forests_object_msgs/srv/GetObjectVisuals` |

Both snapshot input and `managed_objects` output use reliable, transient-local, depth-1
QoS so late subscribers receive the latest state. The launch override is
`object_candidate_snapshots_topic` (the former per-candidate interface is retired).

## Semantics

- Blocking an active object deletes its payload; only the ID is kept in a process-local
  suppression set. Snapshot rows for a blocked ID are rejected until it is unblocked.
- Unblocking does not restore the old payload — the next snapshot containing that ID creates a
  fresh object.
- The published `managed_objects` array is authoritative: consumers must treat it as the
  complete active set. An empty array means an empty scene.
- The tracker snapshot is also authoritative: IDs omitted from it lose payload, visuals and
  overrides. No second per-candidate ingestion API is supported. A confirmed object missing
  from detections is still present in the tracker's remembered state, not automatically removed.
- Manual class and pose edits remain sticky independently while other tracker fields refresh;
  a class override also updates hypothesis class IDs. Curated crop selections (including an
  empty selection) survive tracker refreshes. Overrides end when an ID is retired/blocked;
  a replacement/new ID starts without the old object's overrides.
- `last_seen` and the world-frame detection header stamp remain the exact last accepted 3D
  observation time. Manual edits never advance them. `last_updated` is managed-state modification
  time, and the array header is publication time; republication alone changes neither per-object
  timestamp. A manually overridden pose intentionally need not equal the last measured pose.
- One tracker owns the input. Restart tracker and manager together for each new session or
  recording rewind/loop; IDs, overrides and the suppression set are process-local, not persistent.

## Exported Component

- `semantic_scene_forests_object_manager::ObjectManagerNode`

Downstream C++ consumers can `find_package(semantic_scene_forests_object_manager)` and link
`semantic_scene_forests_object_manager::object_manager_component`; the public headers and the
generated parameter header are installed with the package.

## Entry Points

- Launch file: `launch/object_manager.launch.xml`
- Default config: `config/object_manager.yaml`
