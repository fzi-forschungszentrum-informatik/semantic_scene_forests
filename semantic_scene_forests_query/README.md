# semantic_scene_forests_query

ROS 2 component that answers language queries against the current scene graph and object state.

## Task

- Cache the latest scene graph and managed objects.
- Expose action servers for object lookup and free-form LLM prompting.
- Fetch object visuals when a query needs image evidence.

## Runtime Interface

| Kind | Name | Type |
|---|---|---|
| Subscribe | `/scene_graph` | `semantic_scene_forests_msgs/msg/SceneGraph` |
| Subscribe | `managed_objects` | `semantic_scene_forests_object_msgs/msg/ManagedObjectArray` |
| Action server | `find_object` | `semantic_scene_forests_planning_msgs/action/FindObject` |
| Action server | `prompt_llm` | `semantic_scene_forests_planning_msgs/action/PromptLLM` |
| Service client | `get_object_visuals` | `semantic_scene_forests_object_msgs/srv/GetObjectVisuals` |

## Exported Component

- Component: `semantic_scene_forests_query::SceneGraphQueryNode`
- Auto-generated executable: `semantic_scene_forests_query_node`

## Entry Points

- Launch file: `launch/semantic_scene_forests_query.launch.xml`
- Default config: `config/semantic_scene_forests_query.yaml`

## Prompt Resources

- `resources/find_object/`
- `resources/prompt_llm/`
- `resources/common/`
