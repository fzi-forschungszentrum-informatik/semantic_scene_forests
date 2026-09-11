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

#include <expected>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>

#include <oai_api/client.hpp>
#include <oai_api/types.hpp>

#include <minja/minja.hpp>

#include <semantic_scene_forests/scene_graph.hpp>
#include <semantic_scene_forests_object_msgs/msg/managed_object.hpp>

#include <rclcpp/rclcpp.hpp>

#include "semantic_scene_forests_generation/visibility_control.hpp"

namespace semantic_scene_forests_generation {

using TemplateMap = std::unordered_map<std::string, std::shared_ptr<minja::TemplateNode>>;
using TaskResult  = std::expected<void, std::string>;

// ---------------------------------------------------------------------------
// IObjectTask — Phase 1 plugin base class
//
// Executed once per managed object in Phase 1. The history vector is
// pre-populated with the system message and the object-setup user message
// (with image crops). Tasks append their own user prompt, call
// client.complete(history), and update the scene graph.
// ---------------------------------------------------------------------------

class SEMANTIC_SCENE_FORESTS_GENERATION_PUBLIC IObjectTask
{
public:
  IObjectTask()          = default;
  virtual ~IObjectTask() = default;

  virtual void initialize(rclcpp::Node* node, const TemplateMap& templates) = 0;

  virtual TaskResult execute(oai::OpenAIClient& client,
                             std::vector<oai::Message>& history,
                             int64_t object_id,
                             const std::vector<cv::Mat>& crops,
                             fzi::semantic_scene_forests::SceneGraph& graph) = 0;
};

// ---------------------------------------------------------------------------
// IGraphTask — Phase 2 plugin base class
//
// Executed once per Phase 2 run on the full scene graph. The history vector
// is pre-populated with the system message and the meta-setup user message.
// ---------------------------------------------------------------------------

class SEMANTIC_SCENE_FORESTS_GENERATION_PUBLIC IGraphTask
{
public:
  IGraphTask()          = default;
  virtual ~IGraphTask() = default;

  virtual void initialize(rclcpp::Node* node, const TemplateMap& templates) = 0;

  // NOTE: `current_graph` and `graph` may alias the same SceneGraph instance (the
  // current call site always passes the same object for both). Implementations
  // MUST snapshot any container/iterator obtained from `current_graph` (e.g. via
  // getConceptNodes()) into a local value BEFORE mutating `graph` — do not hold
  // live references or iterators into `current_graph` across a `graph` mutation.
  // See category_task.cpp / affordance_task.cpp for the correct snapshot pattern.
  virtual TaskResult execute(oai::OpenAIClient& client,
                             std::vector<oai::Message>& history,
                             const fzi::semantic_scene_forests::SceneGraph& current_graph,
                             fzi::semantic_scene_forests::SceneGraph& graph) = 0;
};

} // namespace semantic_scene_forests_generation
