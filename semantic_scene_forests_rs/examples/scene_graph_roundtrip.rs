// Copyright (c) 2026 FZI Forschungszentrum Informatik
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//! Publish, receive, and mutate a C++-backed graph in an rclrs node.
use rclrs::{Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions};
use semantic_scene_forests_rs::{SceneGraph, SceneGraphMsg};
use std::{
    error::Error,
    sync::{Arc, Mutex},
    time::Duration,
};

fn inspect_message(message: &SceneGraphMsg, cup: i64, on: i64) -> Result<(), Box<dyn Error>> {
    let mut graph = SceneGraph::from_msg(message)?;
    let edges = graph.outgoing_edges(cup)?;
    if edges.len() != 1 || edges[0].0 != on || edges[0].1.label != "relation" {
        return Err("received graph does not contain the expected cup relation".into());
    }
    let table = graph.add_concept_node("table")?;
    graph.add_edge(on, table, "target")?;
    if graph.outgoing_edges(on)?[0].0 != table {
        return Err("received graph could not be modified".into());
    }
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut graph = SceneGraph::new();
    let cup = graph.add_object_node("cup", None, [None; 5])?;
    let on = graph.add_concept_node("on")?;
    graph.add_edge(cup, on, "relation")?;
    let mut message = graph.to_msg();
    message.header.frame_id = "rust_demo".into();

    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("scene_graph_rust_roundtrip")?;
    let publisher = node.create_publisher::<SceneGraphMsg>("/scene_graph_rust_demo")?;
    let outcome = Arc::new(Mutex::new(None::<Result<(), String>>));
    let received = outcome.clone();
    let _subscription =
        node.create_subscription("/scene_graph_rust_demo", move |message: SceneGraphMsg| {
            let result = inspect_message(&message, cup, on).map_err(|error| error.to_string());
            received.lock().unwrap().get_or_insert(result);
        })?;
    let published = outcome.clone();
    let _timer = node.create_timer_repeating(Duration::from_millis(250), move || {
        if let Err(error) = publisher.publish(&message) {
            published
                .lock()
                .unwrap()
                .get_or_insert(Err(error.to_string()));
        }
    })?;

    executor
        .spin(SpinOptions::default().timeout(Duration::from_secs(10)))
        .timeout_ok()
        .first_error()?;
    outcome
        .lock()
        .unwrap()
        .take()
        .ok_or("no SG message received within 10 seconds")??;
    println!("Published, received, queried, and modified the C++ scene graph successfully.");
    Ok(())
}
