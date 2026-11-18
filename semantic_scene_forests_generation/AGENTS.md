# semantic_scene_forests_generation

C++23 `scene_graph_manager` worker plugins that add semantics to the objects in the shared SceneGraph using a two-phase VLM/LLM pipeline.

## Project Configuration

**IMPORTANT**: This is a monorepo. All project-wide configuration (code style, git conventions, graphify usage) is defined in the root [`AGENTS.md`](../AGENTS.md).

Refer to the root `AGENTS.md` for:
- C/C++ code formatting and linting
- Git commit conventions
- Graphify knowledge graph usage
- Context7 documentation lookup

## Architecture

| Target | Contents |
|---|---|
| `generation_common` | LLM client, template and task plugin loading, shared goal/thread handling (`WorkerThreads`) |
| `object_attribution_worker` | `ObjectAttributionWorker` (phase 1, `IObjectTask` plugins) |
| `concept_generation_worker` | `ConceptGenerationWorker` (phase 2, `IGraphTask` plugins) |
| `semantic_scene_forests_generation_tasks` | Five default pluginlib task plugins |

The workers are `semantic_scene_forests_manager::IWorker` plugins
(`semantic_scene_forests_generation_workers.xml`) loaded by the manager node.

## Key Dependencies

| Dependency | How resolved |
|---|---|
| `oai_api` | Workspace package (plain CMake, found via colcon) |
| `minja` | **FetchContent** from `https://github.com/ochafik/minja.git` @ `143465a` |
| `nlohmann_json` | System install (`/usr/share/cmake/nlohmann_json`) |
| `pluginlib` | ROS2 package |
| `generate_parameter_library` | ROS2 build tool |
| `cv_bridge` | ROS2 package (ROS `sensor_msgs/Image` → `cv::Mat`) |

## ROS Interface (on the manager node)

```
Act   attribute_objects   semantic_scene_forests_msgs/RunWorker   (ObjectAttributionWorker, rerun all objects)
Act   generate_concepts   semantic_scene_forests_msgs/RunWorker   (ConceptGenerationWorker, rerun all concepts)
```

Objects, crops and the graph are read from and written to the manager's shared SceneGraph
(`std::shared_ptr<SceneGraph>`); the manager publishes `/scene_graph`.

## Plugin System

**Phase 1 plugins** implement `semantic_scene_forests_generation::IObjectTask` (`include/semantic_scene_forests_generation/task_interface.hpp`):
- Called once per object
- Receive a pre-populated chat history (system message + multimodal object-setup message)
- Call `chatTurn(client, history, prompt, logger)` from `src/tasks/task_utils.hpp`
- Write concepts and edges through the graph's atomic methods (e.g. `addConceptEdge`), with no lock held during the LLM call

**Phase 2 plugins** implement `semantic_scene_forests_generation::IGraphTask`:
- Called once per Phase 2 run
- Receive the input concepts (unprocessed ones, or all on a manual run), the shared `std::shared_ptr<SceneGraph>`, and a pre-populated meta-setup history

### Adding a new task plugin

1. Create `src/tasks/my_task.hpp` + `src/tasks/my_task.cpp` inheriting from `IObjectTask` or `IGraphTask`
2. Add `PLUGINLIB_EXPORT_CLASS(...)` to `src/tasks/task_plugins.cpp`
3. Add an entry to `semantic_scene_forests_generation_plugins.xml`
4. Add the `.cpp` to `semantic_scene_forests_generation_tasks` in `CMakeLists.txt`
5. Add the plugin class name to `object_attribution.tasks` or `concept_generation.tasks` in the launch config

## Parameters

`src/object_attribution_parameters.yaml` (prefix `object_attribution.`) and
`src/concept_generation_parameters.yaml` (prefix `concept_generation.`) share `server_url`,
`api_key`, `extra_body`, `llm_connect_timeout_seconds`, `llm_request_timeout_seconds`, `prompt_dir`,
`model` and `tasks`. Attribution adds `poll_interval_seconds` (1.0), concept generation
`interval_seconds` (120).

## Prompt Templates

Jinja2 templates live in `resources/{instance,meta}/*.j2`. At runtime they are loaded from `prompt_dir` and keyed by relative path without extension (e.g., `"instance/single_attribute"`). Context variables are passed as `nlohmann::json` objects via `minja::Context::make(minja::Value(json_ctx))`.

## Update Execution

- **Workers poll**: attribution describes objects it has not marked yet; concept
  generation processes unmarked concepts on its interval. Both run on owned `std::jthread`s.
- **Goals are serialized**: each action accepts one goal at a time (reject-while-busy); a goal
  caught in teardown receives no terminal result.
- **Cooperative stopping**: stop tokens are checked before external calls, per task, and before
  commit. Waiting is bounded by the per-call timeouts.
- **Plan, then per item**: a worker plans its items (`unprocessedBy`). For each item it reads the
  current context with short locked getter calls, calls the LLM with no lock held, and applies
  each result through atomic SceneGraph methods. A rejected or invalid call is logged and skipped;
  a dependent step runs only if its predecessor succeeded. An object deleted meanwhile is skipped.

## Build

```bash
colcon build --packages-select semantic_scene_forests_generation
```

minja is fetched automatically via FetchContent on first configure; subsequent builds use the CMake cache.

## graphify

**IMPORTANT**: Use the graphify knowledge graph in the **monorepo root** (`graphify-out/`), not in this package subdirectory.

When the user types `/graphify`, invoke the `skill` tool with `skill: "graphify"` before doing anything else.

Rules:
- For codebase questions, first run `graphify query "<question>"` when graphify-out/graph.json exists. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. These return a scoped subgraph, usually much smaller than GRAPH_REPORT.md or raw grep output.
- Dirty graphify-out/ files are expected after hooks or incremental updates; dirty graph files are not a reason to skip graphify. Only skip graphify if the task is about stale or incorrect graph output, or the user explicitly says not to use it.
- If graphify-out/wiki/index.md exists, use it for broad navigation instead of raw source browsing.
- Read graphify-out/GRAPH_REPORT.md only for broad architecture review or when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` to keep the graph current (AST-only, no API cost).
