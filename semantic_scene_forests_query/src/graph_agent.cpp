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

#include "graph_agent.hpp"

#include <algorithm>
#include <cstddef>
#include <utility>
#include <variant>

#include <fmt/format.h>

#include <nlohmann/json.hpp>

#include <semantic_scene_forests/graph_nodes.hpp>

namespace semantic_scene_forests_query {

using json = nlohmann::json;

namespace {

// Parse the tool arguments and extract an integer "id" field. Never throws:
// uses the non-throwing parser and validates the field explicitly.
std::expected<int64_t, std::string> parseId(std::string_view args)
{
  const json PARSED = json::parse(args, nullptr, /*allow_exceptions=*/false);
  if (PARSED.is_discarded() || !PARSED.is_object())
  {
    return std::unexpected{"arguments must be a JSON object"};
  }
  const auto IT = PARSED.find("id");
  if (IT == PARSED.end() || !IT->is_number_integer())
  {
    return std::unexpected{"missing or non-integer 'id' field"};
  }
  return IT->get<int64_t>();
}

// Extract an optional string field; returns "" when absent.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): fixed parameter order
std::string parseOptionalString(std::string_view args, std::string_view key)
{
  const json PARSED = json::parse(args, nullptr, /*allow_exceptions=*/false);
  if (PARSED.is_discarded() || !PARSED.is_object())
  {
    return {};
  }
  const auto IT = PARSED.find(key);
  if (IT == PARSED.end() || !IT->is_string())
  {
    return {};
  }
  return IT->get<std::string>();
}

} // namespace

GraphAgentSession::GraphAgentSession(std::unique_ptr<fzi::semantic_scene_forests::SceneGraph> graph,
                                     VisualInspector inspect,
                                     ExpandCallback on_expand,
                                     StopCheck should_stop,
                                     LogCallback on_log)
  : m_graph(std::move(graph))
  , m_inspect(std::move(inspect))
  , m_on_expand(std::move(on_expand))
  , m_should_stop(std::move(should_stop))
  , m_on_log(std::move(on_log))
{
}

void GraphAgentSession::throwIfStopping() const
{
  if (m_should_stop && m_should_stop())
  {
    throw AgentStopped{"agent interrupted between steps"};
  }
}

void GraphAgentSession::logTool(std::string_view name,
                                std::string_view args,
                                const std::expected<std::string, std::string>& result) const
{
  if (!m_on_log)
  {
    return;
  }
  constexpr std::size_t K_MAX = 200;
  std::string outcome;
  if (result)
  {
    outcome = result->size() > K_MAX ? result->substr(0, K_MAX) + "…" : *result;
  }
  else
  {
    outcome = "ERROR: " + result.error();
  }
  m_on_log(fmt::format("tool {}({}) -> {}", name, args, outcome));
}

bool GraphAgentSession::recordExpanded(int64_t id)
{
  if (std::ranges::find(m_expanded, id) != m_expanded.end())
  {
    return false; // already recorded
  }
  m_expanded.push_back(id);
  return true;
}

void GraphAgentSession::markExpanded(int64_t id)
{
  if (recordExpanded(id) && m_on_expand)
  {
    m_on_expand(m_expanded);
  }
}

std::optional<std::string> GraphAgentSession::nodeName(int64_t id) const
{
  const auto NODE = m_graph->findNode(id);
  if (!NODE)
  {
    return std::nullopt;
  }
  return std::visit(
    [](const auto& n) -> std::string {
      using T = std::decay_t<decltype(n)>;
      if constexpr (std::is_same_v<T, fzi::semantic_scene_forests::ConceptNode>)
      {
        return n.concept_name;
      }
      else
      {
        return n.class_name;
      }
    },
    *NODE);
}

// ── Tools ─────────────────────────────────────────────────────────────────────

std::expected<std::string, std::string> GraphAgentSession::toolListObjects()
{
  throwIfStopping();
  bool grew = false;
  json out  = json::array();
  for (const auto& obj : m_graph->getObjectNodes())
  {
    grew |= recordExpanded(obj.id);
    out.push_back({{"id", obj.id}, {"class_name", obj.class_name}});
  }
  if (grew && m_on_expand)
  {
    m_on_expand(m_expanded);
  }
  return out.dump();
}

std::expected<std::string, std::string> GraphAgentSession::toolListRootConcepts()
{
  throwIfStopping();
  // Top-down entry points: concept nodes with no outgoing edges sit at the top of
  // the concept hierarchy (e.g. "color", "material"). From here the agent walks
  // DOWN via get_children.
  bool grew = false;
  json out  = json::array();
  for (const auto& concept_node : m_graph->getConceptNodes())
  {
    const auto OUTGOING = m_graph->outgoingEdges(concept_node.id);
    if (OUTGOING && OUTGOING->empty())
    {
      grew |= recordExpanded(concept_node.id);
      out.push_back({{"id", concept_node.id}, {"concept_name", concept_node.concept_name}});
    }
  }
  if (grew && m_on_expand)
  {
    m_on_expand(m_expanded);
  }
  return out.dump();
}

std::expected<std::string, std::string> GraphAgentSession::toolGetChildren(std::string_view args)
{
  throwIfStopping();
  const auto ID = parseId(args);
  if (!ID)
  {
    return std::unexpected{ID.error()};
  }
  if (!m_graph->hasNode(*ID))
  {
    return std::unexpected{"node " + std::to_string(*ID) + " does not exist"};
  }

  // Children are one level DOWN the hierarchy: nodes whose edges point INTO this
  // node (e.g. the attributes that `is_type_of` a root, or the objects that
  // `has_*` an attribute). Mark the parent and every child as expanded, firing a
  // single feedback callback for the whole batch.
  bool grew = recordExpanded(*ID);

  json out            = json::array();
  const auto INCOMING = m_graph->incomingEdges(*ID);
  if (INCOMING)
  {
    for (const auto& [child_id, edge] : *INCOMING)
    {
      grew |= recordExpanded(child_id);
      out.push_back(
        {{"label", edge.label}, {"id", child_id}, {"name", nodeName(child_id).value_or("")}});
    }
  }
  if (grew && m_on_expand)
  {
    m_on_expand(m_expanded);
  }

  return out.dump();
}

std::expected<std::string, std::string> GraphAgentSession::toolGetParents(std::string_view args)
{
  throwIfStopping();
  const auto ID = parseId(args);
  if (!ID)
  {
    return std::unexpected{ID.error()};
  }
  if (!m_graph->hasNode(*ID))
  {
    return std::unexpected{"node " + std::to_string(*ID) + " does not exist"};
  }

  // Parents are one level UP the hierarchy: nodes this node's edges point TO
  // (outgoing). Symmetric to get_children — mark this node and every parent.
  bool grew = recordExpanded(*ID);

  json out            = json::array();
  const auto OUTGOING = m_graph->outgoingEdges(*ID);
  if (OUTGOING)
  {
    for (const auto& [parent_id, edge] : *OUTGOING)
    {
      grew |= recordExpanded(parent_id);
      out.push_back(
        {{"label", edge.label}, {"id", parent_id}, {"name", nodeName(parent_id).value_or("")}});
    }
  }
  if (grew && m_on_expand)
  {
    m_on_expand(m_expanded);
  }

  return out.dump();
}

std::expected<std::string, std::string> GraphAgentSession::toolGetObjectInfo(std::string_view args)
{
  throwIfStopping();
  const auto ID = parseId(args);
  if (!ID)
  {
    return std::unexpected{ID.error()};
  }
  if (!m_graph->hasNode(*ID))
  {
    return std::unexpected{"node " + std::to_string(*ID) + " does not exist"};
  }
  markExpanded(*ID);

  const auto NODE = m_graph->findNode(*ID);
  const auto* obj = NODE ? std::get_if<fzi::semantic_scene_forests::ObjectNode>(&*NODE) : nullptr;
  if (!obj)
  {
    return std::unexpected{"node " + std::to_string(*ID) + " is not an object"};
  }
  const auto& c  = obj->bbox.center.position;
  const auto& sz = obj->bbox.size;
  const json OUT = {{"id", *ID},
                    {"class_name", obj->class_name},
                    {"position", {c.x, c.y, c.z}},
                    {"size", {sz.x, sz.y, sz.z}}};
  return OUT.dump();
}

std::expected<std::string, std::string>
GraphAgentSession::toolInspectObjectVisual(std::string_view args)
{
  throwIfStopping();
  const auto ID = parseId(args);
  if (!ID)
  {
    return std::unexpected{ID.error()};
  }
  if (!m_graph->hasNode(*ID))
  {
    return std::unexpected{"node " + std::to_string(*ID) + " does not exist"};
  }
  markExpanded(*ID);

  if (!m_inspect)
  {
    return std::unexpected{"visual inspection unavailable"};
  }
  const std::string FOCUS = parseOptionalString(args, "question");
  auto description        = m_inspect(*ID, FOCUS);
  if (!description)
  {
    return std::unexpected{description.error()};
  }
  const json OUT = {{"id", *ID}, {"description", *description}};
  return OUT.dump();
}

// ── Tool registry ───────────────────────────────────────────────────────────--

oai::ToolRegistry GraphAgentSession::buildTools()
{
  oai::ToolRegistry registry;

  registry.add(oai::FunctionTool::make(
    {.name = "list_objects",
     .description =
       "BOTTOM entry point. List all object nodes with their scene-graph class "
       "names. Start here for bottom-up (object) search, then walk UP via get_parents.",
     .parameters_schema_json = R"({"type":"object","properties":{},"required":[]})"},
    [this](std::string_view args) {
      auto r = toolListObjects();
      logTool("list_objects", args, r);
      return r;
    }));

  registry.add(oai::FunctionTool::make(
    {.name = "list_root_concepts",
     .description =
       "TOP entry point. List the root concept nodes (top of the concept hierarchy, "
       "e.g. 'color', 'material'). Start here for top-down (concept) search, then walk "
       "DOWN via get_children.",
     .parameters_schema_json = R"({"type":"object","properties":{},"required":[]})"},
    [this](std::string_view args) {
      auto r = toolListRootConcepts();
      logTool("list_root_concepts", args, r);
      return r;
    }));

  registry.add(oai::FunctionTool::make(
    {.name = "get_children",
     .description =
       "Walk one level DOWN: return ALL children of a node (the nodes whose edges point "
       "into it) as a list of {label, id, name}. E.g. children of 'color' are the "
       "specific colours; children of a colour are the objects that have it. The node "
       "and all returned children are marked as explored.",
     .parameters_schema_json =
       R"({"type":"object","properties":{"id":{"type":"integer"}},"required":["id"]})"},
    [this](std::string_view args) {
      auto r = toolGetChildren(args);
      logTool("get_children", args, r);
      return r;
    }));

  registry.add(oai::FunctionTool::make(
    {.name = "get_parents",
     .description =
       "Walk one level UP: return ALL parents of a node (the nodes its edges point to) "
       "as a list of {label, id, name}. E.g. parents of an object are its attribute "
       "concepts (has_color, has_material); the parent of a colour is 'color'.",
     .parameters_schema_json =
       R"({"type":"object","properties":{"id":{"type":"integer"}},"required":["id"]})"},
    [this](std::string_view args) {
      auto r = toolGetParents(args);
      logTool("get_parents", args, r);
      return r;
    }));

  registry.add(oai::FunctionTool::make(
    {.name        = "get_object_info",
     .description = "Get scene-graph object data (class name, 3D position and size) for an object "
                    "node.",
     .parameters_schema_json =
       R"({"type":"object","properties":{"id":{"type":"integer"}},"required":["id"]})"},
    [this](std::string_view args) {
      auto r = toolGetObjectInfo(args);
      logTool("get_object_info", args, r);
      return r;
    }));

  registry.add(oai::FunctionTool::make(
    {.name        = "inspect_object_visual",
     .description = "Look at an object's image crops and return a description. Optionally pass a "
                    "'question' to focus the description.",
     .parameters_schema_json =
       R"({"type":"object","properties":{"id":{"type":"integer"},"question":{"type":"string"}},"required":["id"]})"},
    [this](std::string_view args) {
      auto r = toolInspectObjectVisual(args);
      logTool("inspect_object_visual", args, r);
      return r;
    }));

  return registry;
}

} // namespace semantic_scene_forests_query
