# semantic_scene_forests_launch

Launch package for running the integrated semantic scene forests stack.

## Task

- Start the `scene_graph_manager` (tracker, object attribution, concept generation and pruning
  workers) and the query node in one container.
- Provide shared parameter files and diagnostic aggregation setup.

## Main Entry Points

- Launch file: `launch/scene_graph_combined.launch.xml`
- Helper script: `scripts/launch_semantic_scene_forests.sh` (catmux: zenoh router plus the
  combined launch)

## Launch Surface

Key launch arguments exposed by `scene_graph_combined.launch.xml`:

- `scene_graph_frame` (sets the manager's `frame_id` and `tracker.world_frame`)
- `config_dir`
- `enable_diagnostics`
- `use_sim_time`

## Managed Components

- `semantic_scene_forests_manager::SceneGraphManagerNode` (`config/scene_graph_manager.yaml`)
- `semantic_scene_forests_query::SceneGraphQueryNode` (`config/semantic_scene_forests_query.yaml`)
- `diagnostic_aggregator/aggregator_node` (optional)

## Configuration

`config/scene_graph_manager.yaml` lists the workers to load (`workers`, `worker_types.<name>`)
and their parameters under each worker's name (`tracker.*`, `object_attribution.*`,
`concept_generation.*`, `pruning.*`). Camera and detection topics are set per camera under
`tracker.camera_names` / `tracker.cameras.<name>`; there are no topic remap arguments.

Configuration comes from ROS parameters only, never from environment variables. Keep API keys
out of the committed file: put them in a local copy of the config directory and pass it with
`config_dir`. The tracker arbiter refuses to start without `tracker.arbiter.api_key` when enabled
with `openai` or `openrouter`.

`scene_graph_frame` must be the calibrated common frame for every camera. All camera streams
must share a compatible clock and class labels. Inputs are startup-only; restart the manager
to reset object IDs.
