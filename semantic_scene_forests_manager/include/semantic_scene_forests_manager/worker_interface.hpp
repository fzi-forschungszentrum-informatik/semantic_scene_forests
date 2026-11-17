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

#pragma once

#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/node.hpp>

#include "semantic_scene_forests/scene_graph.hpp"

namespace semantic_scene_forests_manager {

//! A pluginlib plugin hosted by the scene graph manager. Loaded by class name from
//! `worker_types.<name>`; creates its own ROS interfaces on the host node and runs on its own
//! threads.
class IWorker
{
public:
  virtual ~IWorker() = default;
  //! `name` is the worker's parameter prefix and its key in `WorkerState`. `diagnostics` is
  //! the manager's shared updater; workers add their status tasks to it.
  virtual void initialize(rclcpp::Node& node,
                          std::shared_ptr<fzi::semantic_scene_forests::SceneGraph> graph,
                          const std::string& name,
                          diagnostic_updater::Updater& diagnostics) = 0;
  virtual void start()                                              = 0;
  //! Requests stop and joins; idempotent.
  virtual void stop() = 0;
};

} // namespace semantic_scene_forests_manager
