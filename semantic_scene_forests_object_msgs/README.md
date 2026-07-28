# semantic_scene_forests_object_msgs

ROS 2 interface package for tracked objects, managed objects, and object-management services.

## Task

- Define the messages passed between tracking, object management, generation, query, and visualization.
- Expose services for fetching visuals and editing object state.

## Messages

- `msg/ObjectCandidate.msg`: tracker output before lifecycle management
- `msg/ManagedObject.msg`: persistent object state with blocking and edit metadata
- `msg/ManagedObjectArray.msg`: current managed object set
- `msg/ObjectVisuals.msg`: image crops associated with an object ID

## Services

- `srv/GetObjects.srv`: return the current managed object list
- `srv/GetObjectVisuals.srv`: fetch visuals for one or more object IDs
- `srv/BlockObject.srv`: suppress an object by ID
- `srv/EditObject.srv`: update class, pose, or stored crops
- `srv/UnblockObject.srv`: allow a blocked object again

## Used By

- `semantic_scene_forests_object_tracker`
- `semantic_scene_forests_object_manager`
- `semantic_scene_forests_generation`
- `semantic_scene_forests_query`
- `semantic_scene_forests_visualization_rqt`
- `semantic_scene_forests_visualization_rviz`
