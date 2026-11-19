# semantic_scene_forests_visualization_rqt

RQT plugin for interactive inspection of the semantic scene graph.

## Task

- Display the graph as either a force-directed view or a layered DAG view.
- Show managed objects and image crops for selected nodes.
- Let operators trigger graph updates and language queries from the UI.

## Plugin Interface

- Plugin class: `semantic_scene_forests_visualization_rqt::SceneGraphPlugin`
- Plugin id: `semantic_scene_forests_visualization_rqt/SceneGraphView`

## Runtime Interface

| Kind | Name | Type |
|---|---|---|
| Subscribe | `/scene_graph` | `semantic_scene_forests_msgs/msg/SceneGraph` (object node class names label the view) |
| Action client | `attribute_objects` | `semantic_scene_forests_msgs/action/RunWorker` |
| Action client | `generate_concepts` | `semantic_scene_forests_msgs/action/RunWorker` |
| Action client | `find_object` | `semantic_scene_forests_planning_msgs/action/FindObject` |
| Action client | `prompt_llm` | `semantic_scene_forests_planning_msgs/action/PromptLLM` |
| Service client | `get_object_visuals` | `semantic_scene_forests_object_msgs/srv/GetObjectVisuals` |

## Entry Point

- Plugin manifest: `plugin.xml`
