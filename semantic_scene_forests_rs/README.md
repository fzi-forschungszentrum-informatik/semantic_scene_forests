# semantic_scene_forests_rs

Rust bindings for the existing C++ scene graph library. This package is an initial
proof of concept, not yet equivalent to the Python API.

## Prerequisites

Source a ROS 2 Lyrical workspace containing `semantic_scene_forests` and generated
Rust messages (`rosidl_generator_rs`). Rust 1.85 or newer, C++23, CMake, and the
`colcon-cargo`/`colcon-ros-cargo` plugins are required.

The verified current releases are `rclrs 0.8.0`, `ros-env 0.3.0`, and
`rosidl_runtime_rs 0.7.0`. The lockfile records the tested versions. `rclrs` is a
development dependency only: graph operations do not create a ROS node or executor.

`ros-env` includes generated messages from the sourced environment. It lets the
installed message source use runtime 0.7 even when its generated Cargo manifest
still names runtime 0.6. A local smoke test verified both message trait identity
and image conversion; C/Rust Image and byte-sequence sizes matched on Lyrical.

## Test

```bash
cargo test --manifest-path /path/to/semantic_scene_forests_rs/Cargo.toml
```

Older colcon workspaces may contain a stale `.cargo/config.toml` with patches for
removed message packages. Cargo reads configuration from the working directory's
ancestors, even with `--manifest-path`. Run the command from outside that workspace
with the absolute manifest path rather than editing unrelated configuration.
