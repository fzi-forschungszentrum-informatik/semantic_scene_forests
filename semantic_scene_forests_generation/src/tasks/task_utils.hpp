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

#include <algorithm>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include <oai_api/builder.hpp>
#include <oai_api/client.hpp>
#include <oai_api/types.hpp>

#include <rclcpp/rclcpp.hpp>

#include "semantic_scene_forests_generation/task_interface.hpp"

namespace semantic_scene_forests_generation {

using json = nlohmann::json;

inline constexpr std::size_t MAX_LLM_RESPONSE_BYTES = 64U * 1024U;
inline constexpr std::size_t MAX_TASK_LABELS        = 16U;
inline constexpr std::size_t MAX_TASK_SOURCE_IDS    = 128U;

// Extract a JSON object or array from an LLM response that may contain
// markdown code fences (```json ... ```) or bare JSON.
inline std::string extractJson(const std::string& content)
{
  const auto fence_start = content.find("```");
  if (fence_start != std::string::npos)
  {
    const auto code_start = content.find('\n', fence_start);
    if (code_start != std::string::npos)
    {
      const auto fence_end = content.find("```", code_start);
      if (fence_end != std::string::npos)
      {
        return content.substr(code_start + 1, fence_end - code_start - 1);
      }
    }
  }

  const auto start = content.find('{');
  const auto end   = content.rfind('}');
  if (start != std::string::npos && end != std::string::npos && end > start)
  {
    return content.substr(start, end - start + 1);
  }
  return content;
}

inline std::expected<json, std::string> parseJsonObject(const std::string& content)
{
  if (content.size() > MAX_LLM_RESPONSE_BYTES)
  {
    return std::unexpected("LLM response exceeds the maximum accepted size");
  }

  try
  {
    auto parsed = json::parse(extractJson(content));
    if (!parsed.is_object())
    {
      return std::unexpected("LLM response must be a JSON object");
    }
    return parsed;
  }
  catch (const json::exception& e)
  {
    return std::unexpected("failed to parse LLM JSON: " + std::string(e.what()));
  }
}

inline std::expected<std::string, std::string> validateConceptLabel(const json& value)
{
  if (!value.is_string())
  {
    return std::unexpected("concept labels must be strings");
  }

  const std::string raw = value.get<std::string>();
  const auto is_space   = [](unsigned char character) {
    return character == ' ' || character == '\t';
  };
  const auto first = std::find_if_not(raw.begin(), raw.end(), is_space);
  const auto last  = std::find_if_not(raw.rbegin(), raw.rend(), is_space).base();
  if (first == raw.end())
  {
    return std::unexpected("concept labels must not be empty");
  }

  std::string label{first, last};
  if (label.size() > 128U)
  {
    return std::unexpected("concept labels must not exceed 128 bytes");
  }
  if (std::any_of(label.begin(), label.end(), [](unsigned char character) {
        return character < 0x20U || character == 0x7FU;
      }))
  {
    return std::unexpected("concept labels must not contain control characters");
  }
  return label;
}

struct SceneContextAddition
{
  int64_t context_node_id;
  bool created;
};

inline std::expected<SceneContextAddition, std::string> appendSceneContext(
  fzi::semantic_scene_forests::SceneGraph& graph, int64_t node_id, const std::string& context)
{
  auto label = validateConceptLabel(context);
  if (!label)
  {
    return std::unexpected(label.error());
  }
  auto outgoing = graph.outgoingEdges(node_id);
  if (!outgoing)
  {
    return std::unexpected(outgoing.error().message);
  }
  for (const auto& [target_id, edge] : *outgoing)
  {
    if (edge.label != "has_context")
    {
      continue;
    }
    const auto TARGET = graph.findNode(target_id);
    if (TARGET)
    {
      const auto* context_node = std::get_if<fzi::semantic_scene_forests::ConceptNode>(&*TARGET);
      if (context_node && context_node->concept_name == *label)
      {
        return SceneContextAddition{.context_node_id = target_id, .created = false};
      }
    }
  }
  auto added = graph.addConceptNode(*label);
  if (!added)
  {
    return std::unexpected(added.error().message);
  }
  if (auto edge = graph.addEdge(node_id, *added, "has_context"); !edge)
  {
    // Roll back only this request's new node, never pre-existing graph data.
    if (auto rollback = graph.removeNode(*added); !rollback)
    {
      return std::unexpected(edge.error().message +
                             "; rollback failed: " + rollback.error().message);
    }
    return std::unexpected(edge.error().message);
  }
  return SceneContextAddition{.context_node_id = *added, .created = true};
}

inline TaskResult addUniqueEdge(fzi::semantic_scene_forests::SceneGraph& graph,
                                int64_t source_id,
                                int64_t target_id,
                                const std::string& label)
{
  const auto result = graph.addEdge(source_id, target_id, label);
  if (!result && result.error().kind != SceneGraphErrorKind::EDGE_ALREADY_EXISTS)
  {
    return std::unexpected(result.error().message);
  }
  return {};
}

inline std::expected<int64_t, std::string>
findOrCreateConcept(fzi::semantic_scene_forests::SceneGraph& graph, const std::string& label)
{
  const auto existing = graph.findConceptNodesByName(label);
  if (!existing.empty())
  {
    return existing.front().id;
  }

  auto result = graph.addConceptNode(label);
  if (!result)
  {
    return std::unexpected(result.error().message);
  }
  return *result;
}

inline std::expected<int64_t, std::string>
parseAllowedConceptId(std::string_view value, const std::unordered_set<int64_t>& allowed_ids)
{
  int64_t id{};
  const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), id);
  if (error != std::errc{} || end != value.data() + value.size())
  {
    return std::unexpected("LLM response contains an invalid concept id");
  }
  if (!allowed_ids.contains(id))
  {
    return std::unexpected("LLM response references a concept that was not requested");
  }
  return id;
}

// Send a user prompt, append the response to history, and return the response
// content. The caller receives the request error so a failed task cannot be
// reported as a successful scene-graph update.
inline std::expected<std::string, std::string> chatTurn(oai::OpenAIClient& client,
                                                        std::vector<oai::Message>& history,
                                                        const std::string& user_text,
                                                        const rclcpp::Logger& logger)
{
  auto user_message = oai::MessageBuilder::user().text(user_text).build();
  if (!user_message)
  {
    return std::unexpected("failed to build LLM request: " + user_message.error());
  }
  history.push_back(std::move(*user_message));

  try
  {
    auto response = client.complete(history);
    if (!response)
    {
      RCLCPP_WARN(logger,
                  "LLM request failed (%d): %s",
                  response.error().http_status,
                  response.error().message.c_str());
      history.pop_back();
      return std::unexpected("LLM request failed: " + response.error().message);
    }

    std::string content    = response->choice.content.value_or("");
    auto assistant_message = oai::MessageBuilder::assistant().text(content).build();
    if (!assistant_message)
    {
      history.pop_back();
      return std::unexpected("failed to build LLM response message: " + assistant_message.error());
    }
    history.push_back(std::move(*assistant_message));
    return content;
  }
  catch (const std::exception& e)
  {
    history.pop_back();
    return std::unexpected("LLM request threw an exception: " + std::string(e.what()));
  }
}

inline TaskResult runObjectAttributeTask(oai::OpenAIClient& client,
                                         std::vector<oai::Message>& history,
                                         int64_t object_id,
                                         fzi::semantic_scene_forests::SceneGraph& graph,
                                         const TemplateMap& templates,
                                         const rclcpp::Logger& logger,
                                         std::string_view attribute,
                                         std::string_view relation,
                                         std::string_view category)
{
  try
  {
    const auto prompt_template     = templates.find("instance/single_attribute");
    const auto formatting_template = templates.find("instance/formatting");
    if (prompt_template == templates.end() || formatting_template == templates.end())
    {
      return std::unexpected("required object attribute template is missing");
    }

    auto prompt_context = minja::Value::object();
    prompt_context.set("attribute", minja::Value(std::string{attribute}));
    const auto context       = minja::Context::make(std::move(prompt_context));
    const std::string prompt = prompt_template->second->render(context);

    const auto apply_response = [&](const std::string& raw) -> TaskResult {
      auto parsed = parseJsonObject(raw);
      if (!parsed)
      {
        return std::unexpected(parsed.error());
      }

      const auto values = parsed->find(std::string{attribute});
      if (values == parsed->end() || !values->is_array())
      {
        return std::unexpected("LLM response is missing the expected '" + std::string(attribute) +
                               "' array");
      }
      if (values->size() > MAX_TASK_LABELS)
      {
        return std::unexpected("LLM response contains too many attribute labels");
      }

      for (const auto& value : *values)
      {
        auto label = validateConceptLabel(value);
        if (!label)
        {
          return std::unexpected(label.error());
        }
        if (*label == category)
        {
          return std::unexpected("attribute values must not equal their category");
        }

        auto concept_id = findOrCreateConcept(graph, *label);
        if (!concept_id)
        {
          return std::unexpected(concept_id.error());
        }
        if (auto edge_result = addUniqueEdge(graph, object_id, *concept_id, std::string{relation});
            !edge_result)
        {
          return edge_result;
        }

        auto category_id = findOrCreateConcept(graph, std::string{category});
        if (!category_id)
        {
          return std::unexpected(category_id.error());
        }
        if (auto edge_result = addUniqueEdge(graph, *concept_id, *category_id, "is_type_of");
            !edge_result)
        {
          return edge_result;
        }
      }
      return {};
    };

    auto response = chatTurn(client, history, prompt, logger);
    if (!response)
    {
      return std::unexpected(response.error());
    }
    auto graph_before_response = graph.clone();
    if (!graph_before_response)
    {
      return std::unexpected(graph_before_response.error().message);
    }
    if (auto apply_result = apply_response(*response); apply_result)
    {
      return {};
    }
    else
    {
      // Discard any valid prefix from the malformed response before retrying.
      graph.swap(**graph_before_response);
      auto formatting_context = minja::Value::object();
      formatting_context.set("response", minja::Value(*response));
      formatting_context.set("attribute", minja::Value(std::string{attribute}));
      const auto formatting_minja_context = minja::Context::make(std::move(formatting_context));
      const std::string formatting_prompt =
        formatting_template->second->render(formatting_minja_context);
      auto retry = chatTurn(client, history, formatting_prompt, logger);
      if (!retry)
      {
        return std::unexpected(retry.error());
      }
      if (auto retry_result = apply_response(*retry); !retry_result)
      {
        return std::unexpected("invalid LLM response after formatting retry: " +
                               retry_result.error());
      }
    }
  }
  catch (const std::exception& e)
  {
    return std::unexpected("object attribute task failed: " + std::string(e.what()));
  }
  return {};
}

} // namespace semantic_scene_forests_generation
