# semantic_scene_forests_object_msgs

ROS 2 interface package for tracked objects and object-management services.

## Task

- Define the object messages used by the manager, its workers, query, and visualization.
- Expose services for fetching visuals and editing object state.

## Messages

- `msg/ObjectCandidate.msg`: one manual observation for `AddObservations`
- `msg/ObjectVisuals.msg`: image crops associated with an object ID

## Services

- `srv/AddObservations.srv`: register sightings from a non-camera source with the tracker
- `srv/AddObject.srv`: add a manual object with the tracker (served by the tracker, returns `object_id`)
- `srv/GetObjects.srv`: return the current object nodes (`semantic_scene_forests_msgs/ObjectNode[]`)
- `srv/GetObjectVisuals.srv`: fetch visuals for one or more object IDs
- `srv/DeleteObject.srv`: remove an object node by ID
- `srv/EditObject.srv`: set an object's class name

## Used By

- `semantic_scene_forests_object_tracker`
- `semantic_scene_forests_manager`
- `semantic_scene_forests_generation`
- `semantic_scene_forests_query`
- `semantic_scene_forests_visualization_rqt`
- `semantic_scene_forests_visualization_rviz`
