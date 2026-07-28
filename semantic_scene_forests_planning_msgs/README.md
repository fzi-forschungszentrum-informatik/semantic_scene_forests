# semantic_scene_forests_planning_msgs

ROS 2 action definitions for language-driven scene graph queries.

## Task

- Define the action contracts used to query the current graph from operator tools and UIs.

## Interfaces

- `action/FindObject.action`
  - input: `object_description`
  - output: success flag, message, resolved `object_id`
  - feedback: expanded node IDs and status text
- `action/PromptLLM.action`
  - input: free-form `query`
  - output: success flag, message, text `response`
  - feedback: expanded node IDs and status text

## Used By

- `semantic_scene_forests_query` provides the action servers.
- `semantic_scene_forests_visualization_rqt` acts as a client.
