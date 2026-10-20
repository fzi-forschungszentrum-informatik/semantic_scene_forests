# semantic_scene_forests_rs

Rust bindings for the existing C++ scene graph library. This package is an initial
proof of concept, not yet equivalent to the Python API.

## Prerequisites

Source a ROS 2 Lyrical workspace containing `semantic_scene_forests` and generated
Rust messages (`rosidl_generator_rs`). Rust 1.89 or newer, C++23, CMake, and the
`colcon-cargo`/`colcon-ros-cargo` plugins are required.

The verified current releases are `rclrs 0.8.0`, `ros-env 0.3.0`, and
`rosidl_runtime_rs 0.7.0`. The lockfile records the tested versions. `rclrs` is a
development dependency only: graph operations do not create a ROS node or executor.
The current CXX release requires Rust 1.88; the locked rclrs development dependency
tree raises the minimum for the complete package and examples to Rust 1.89.

`ros-env` includes generated messages from the sourced environment. It lets the
installed message source use runtime 0.7 even when its generated Cargo manifest
still names runtime 0.6. A local smoke test verified both message trait identity
and image conversion; C/Rust Image and byte-sequence sizes matched on Lyrical.

## Test

```bash
cargo test --manifest-path /path/to/semantic_scene_forests_rs/Cargo.toml
```

## Build and ROS example

From a sourced ROS workspace:

```bash
colcon build --packages-up-to semantic_scene_forests_rs
source install/setup.bash
cargo run --manifest-path /path/to/semantic_scene_forests_rs/Cargo.toml \
  --example scene_graph_roundtrip
```

The example lives in this package's `examples/` directory; it is not installed as
a `ros2 run` executable. It repeatedly publishes on `/scene_graph_rust_demo`,
reconstructs and modifies the received graph in an rclrs callback, and exits after
10 seconds. It fails if no valid message arrived or an operation failed.

While it runs, a non-Rust observer can inspect the same message:

```bash
ros2 topic echo --once /scene_graph_rust_demo semantic_scene_forests_msgs/msg/SceneGraph
```

Both processes need the same middleware/domain. The local verification used
temporary `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and `ROS_DOMAIN_ID=183` settings;
Zenoh users need their router running. No saved middleware settings are changed.

The native adapter is built with CMake against the exported C++ target. Its
sources/headers are installed alongside the Rust sources so downstream Cargo
crates can also build against the installed package.

Older colcon workspaces may contain a stale `.cargo/config.toml` with patches for
removed message packages. Cargo reads configuration from the working directory's
ancestors, even with `--manifest-path`. Run the command from outside that workspace
with the absolute manifest path rather than editing unrelated configuration.
Colcon runs Cargo from the package's source directory, so that case additionally
requires regenerating the stale workspace configuration or building from a clean
source checkout. Verification here used a clean temporary copy for colcon and
left the pre-existing workspace configuration unchanged.

## Graph operations

```rust
use semantic_scene_forests_rs::SceneGraph;

let mut graph = SceneGraph::new();
let cup = graph.add_object_node("cup")?;
let on = graph.add_concept_node("on")?;
graph.add_edge(cup, on, "relation")?;
assert_eq!(graph.outgoing_edges(cup)?[0].0, on);
```

Graph operations use the C++ implementation and return `GraphError` with its
original error kind and message. Queries return owned Rust values. Object creation
currently accepts a class name only, without detection/image payloads. The graph
does not implement `Clone`, `Send`, or `Sync` in this first version.

## ROS message conversion

`SceneGraph::from_msg(&message)`, `graph.to_msg()`, and
`graph.update_from_msg(&message)` use generated Rust SG messages and delegate to
the corresponding C++ methods. Object graph messages contain IDs only: class
names, detections, and image visuals do not survive a message round trip. The
C++ graph also does not retain message headers.

Updates edit the live graph in place. On an error, successful earlier changes
remain; this is not an all-or-nothing transaction. Nodes absent from a successful
update message are removed. Sending an empty message clears the graph.
