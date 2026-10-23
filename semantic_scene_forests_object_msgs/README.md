# semantic_scene_forests_object_msgs

ROS 2 interface package for tracked objects and object-management services.

## Task

- Define the object messages used by the manager, its workers, query, and visualization.
- Expose services for fetching visuals and editing object state.

## Messages

- `msg/ObjectCandidate.msg`: one manual observation for `AddObservations`
- `msg/ManagedObject.msg`: object state with blocking and edit metadata (returned by `GetObjects`)
- `msg/ObjectVisuals.msg`: image crops associated with an object ID

## Services

- `srv/AddObservations.srv`: register sightings from a non-camera source with the tracker
- `srv/GetObjects.srv`: return the current object list
- `srv/GetObjectVisuals.srv`: fetch visuals for one or more object IDs
- `srv/BlockObject.srv`: suppress an object by ID
- `srv/EditObject.srv`: update class, pose, or stored crops
- `srv/UnblockObject.srv`: allow a blocked object again

## Used By

- `semantic_scene_forests_object_tracker`
- `semantic_scene_forests_manager`
- `semantic_scene_forests_generation`
- `semantic_scene_forests_query`
- `semantic_scene_forests_visualization_rqt`
- `semantic_scene_forests_visualization_rviz`
