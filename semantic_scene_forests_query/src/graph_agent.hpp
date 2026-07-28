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

#include <cstdint>
#include <expected>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <oai_api/tool.hpp>

#include <semantic_scene_forests/scene_graph.hpp>

namespace semantic_scene_forests_query {

// ---------------------------------------------------------------------------
// Object information sourced from the object manager (NOT the scene graph).
//
// Scene-graph ObjectNodes carry only an id (see semantic_scene_forests_msgs/ObjectNode and
// SceneGraph::fromMsg, which sets class_name to ""), so the class name and pose
// must be resolved through the /managed_objects cache supplied by the node.
// ---------------------------------------------------------------------------

struct ObjectInfo
{
  std::string class_name;
  // Bounding-box centre, in the frame published by the object manager.
  double position_x{};
  double position_y{};
  double position_z{};
  // Bounding-box size.
  double size_x{};
  double size_y{};
  double size_z{};
};

// Resolve object-manager metadata for a scene-graph object id.
using ObjectLookup = std::function<std::optional<ObjectInfo>(int64_t id)>;

// Run a one-shot vision completion over an object's image crops (fetched from
// the object manager via /get_object_visuals) and return a textual description.
// `focus` lets the caller bias the description toward what the agent cares about.
using VisualInspector =
  std::function<std::expected<std::string, std::string>(int64_t id, std::string_view focus)>;

// Notified with the cumulative, de-duplicated list of expanded node ids whenever
// the set grows. Used to publish action feedback.
using ExpandCallback = std::function<void(const std::vector<int64_t>&)>;

// Optional sink for human-readable trace lines (one per tool call). Lets the
// node surface what the agent is doing via the ROS logger.
using LogCallback = std::function<void(const std::string&)>;

// ---------------------------------------------------------------------------
// GraphAgentSession
//
// Owns a single immutable snapshot of the scene graph for the duration of one
// action goal, exposes graph-traversal tools to an oai::Agent, and tracks which
// nodes the agent has inspected ("expanded").
// ---------------------------------------------------------------------------

class GraphAgentSession
{
public:
  GraphAgentSession(std::unique_ptr<fzi::semantic_scene_forests::SceneGraph> graph,
                    ObjectLookup object_lookup,
                    VisualInspector inspect,
                    ExpandCallback on_expand,
                    LogCallback on_log = {});

  // Build the tool registry bound to this session. The returned tools capture
  // `this`; the session must outlive any agent built from them.
  [[nodiscard]] oai::ToolRegistry buildTools();

  [[nodiscard]] const std::vector<int64_t>& expandedNodes() const { return m_expanded; }

private:
  // Record `id` as expanded and fire the callback if the set grew.
  void markExpanded(int64_t id);

  // Add `id` to the expanded set WITHOUT firing the callback; returns true if the
  // set grew. Used to batch a parent + all its neighbours into a single callback.
  bool recordExpanded(int64_t id);

  // Emit a one-line trace for a finished tool call (no-op if no log sink).
  void logTool(std::string_view name,
               std::string_view args,
               const std::expected<std::string, std::string>& result) const;

  // Render a node as {id, type, name}; name for objects comes from ObjectLookup.
  [[nodiscard]] std::optional<std::string> nodeName(int64_t id) const;

  // Tool implementations (all return a JSON string; errors as std::unexpected).
  // Entry points: object nodes (bottom) and root concept nodes (top). All other
  // movement is stepwise via get_children/get_parents so the agent cannot start
  // mid-graph.
  // Entry-point listings mark every node they reveal as expanded.
  [[nodiscard]] std::expected<std::string, std::string> toolListObjects();
  [[nodiscard]] std::expected<std::string, std::string> toolListRootConcepts();
  // Walk one level DOWN (incoming edges) / UP (outgoing edges) the hierarchy.
  // get_children marks the parent and every returned child as expanded;
  // get_parents is symmetric (parent node + every returned parent).
  [[nodiscard]] std::expected<std::string, std::string> toolGetChildren(std::string_view args);
  [[nodiscard]] std::expected<std::string, std::string> toolGetParents(std::string_view args);
  [[nodiscard]] std::expected<std::string, std::string> toolGetObjectInfo(std::string_view args);
  [[nodiscard]] std::expected<std::string, std::string>
  toolInspectObjectVisual(std::string_view args);

  std::unique_ptr<fzi::semantic_scene_forests::SceneGraph> m_graph;
  ObjectLookup m_object_lookup;
  VisualInspector m_inspect;
  ExpandCallback m_on_expand;
  LogCallback m_on_log;
  std::vector<int64_t> m_expanded;
};

} // namespace semantic_scene_forests_query
