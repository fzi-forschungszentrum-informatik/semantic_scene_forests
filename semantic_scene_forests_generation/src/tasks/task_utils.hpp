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
#include <memory>
#include <optional>
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

inline constexpr std::size_t MAX_LLM_RESPONSE_BYTES = static_cast<std::size_t>(64U) * 1024U;
inline constexpr std::size_t MAX_TASK_LABELS        = 16U;
inline constexpr std::size_t MAX_TASK_SOURCE_IDS    = 128U;

// Extract a JSON object or array from an LLM response that may contain
// markdown code fences (```json ... ```) or bare JSON.
inline std::string extractJson(const std::string& content)
{
  const auto FENCE_START = content.find("```");
  if (FENCE_START != std::string::npos)
  {
    const auto CODE_START = content.find('\n', FENCE_START);
    if (CODE_START != std::string::npos)
    {
      const auto FENCE_END = content.find("```", CODE_START);
      if (FENCE_END != std::string::npos)
      {
        return content.substr(CODE_START + 1, FENCE_END - CODE_START - 1);
      }
    }
  }

  const auto START = content.find('{');
  const auto END   = content.rfind('}');
  if (START != std::string::npos && END != std::string::npos && END > START)
  {
    return content.substr(START, END - START + 1);
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

//! Writes `source --relation--> label --is_type_of--> root` for every string in `labels`. Each
//! step is its own atomic addConceptEdge: a rejected label (invalid, cycle, unknown source) is
//! logged and skipped, and the is_type_of step runs only if its predecessor succeeded. An edge
//! that already exists counts as success. Errors only if `labels` is not a bounded array, before
//! anything is written.
inline TaskResult
applyLabels(fzi::semantic_scene_forests::SceneGraph& graph,
            const rclcpp::Logger& logger,
            int64_t source_id,
            // NOLINTNEXTLINE(bugprone-easily-swappable-parameters): fixed parameter order
            const std::string& relation,
            const std::string& root,
            const json& labels)
{
  if (!labels.is_array() || labels.size() > MAX_TASK_LABELS)
  {
    return std::unexpected("label values must be an array with a bounded size");
  }

  // Concept id of the target if the edge now exists, nullopt if it was rejected.
  const auto LINK = [&](int64_t source,
                        const std::string& edge_relation,
                        const std::string& label) -> std::optional<int64_t> {
    // Links `source` to an existing concept; an edge that is already there counts as success.
    const auto ATTACH = [&](int64_t concept_id) -> std::expected<int64_t, SceneGraphError> {
      const auto EDGE = graph.addEdge(source, concept_id, edge_relation);
      if (EDGE || EDGE.error().kind == SceneGraphErrorKind::EDGE_ALREADY_EXISTS)
      {
        return concept_id;
      }
      return std::unexpected(EDGE.error());
    };
    std::expected<int64_t, SceneGraphError> result =
      std::unexpected(SceneGraphError::graphEditingFailed("unreachable"));
    if (const auto EXISTING = graph.findConcept(label))
    {
      result = ATTACH(*EXISTING);
    }
    else if (const auto ADDED = graph.addConceptEdge(source, edge_relation, label); ADDED)
    {
      result = ADDED->id;
    }
    else if (const auto RACED = graph.findConcept(label);
             RACED && ADDED.error().kind == SceneGraphErrorKind::NODE_ALREADY_EXISTS)
    {
      result = ATTACH(*RACED); // another writer created the concept meanwhile
    }
    else
    {
      result = std::unexpected(ADDED.error());
    }
    if (result)
    {
      return *result;
    }
    RCLCPP_WARN(logger,
                "Skipping '%s' --%s--> '%s': %s",
                std::to_string(source).c_str(),
                edge_relation.c_str(),
                label.c_str(),
                result.error().message.c_str());
    return std::nullopt;
  };

  for (const auto& value : labels)
  {
    if (!value.is_string() || value.get<std::string>() == root)
    {
      RCLCPP_WARN(
        logger, "Skipping a label that is not a string or equals its root '%s'", root.c_str());
      continue;
    }
    const auto LABEL_ID = LINK(source_id, relation, value.get<std::string>());
    if (LABEL_ID)
    {
      LINK(*LABEL_ID, "is_type_of", root);
    }
  }
  return {};
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
    auto response = client.chat({.messages = history});
    if (!response)
    {
      RCLCPP_WARN(logger,
                  "LLM request failed (%d): %s",
                  response.error().http_status,
                  response.error().message.c_str());
      history.pop_back();
      return std::unexpected("LLM request failed: " + response.error().message);
    }

    std::string content    = response->text();
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

inline TaskResult runObjectAttributeTask(
  oai::OpenAIClient& client,
  std::vector<oai::Message>& history,
  int64_t object_id,
  const std::shared_ptr<fzi::semantic_scene_forests::SceneGraph>& graph,
  const TemplateMap& templates,
  const rclcpp::Logger& logger,
  // NOLINTNEXTLINE(bugprone-easily-swappable-parameters): fixed parameter order
  std::string_view attribute,
  std::string_view relation,
  std::string_view category)
{
  try
  {
    const auto PROMPT_TEMPLATE     = templates.find("instance/single_attribute");
    const auto FORMATTING_TEMPLATE = templates.find("instance/formatting");
    if (PROMPT_TEMPLATE == templates.end() || FORMATTING_TEMPLATE == templates.end())
    {
      return std::unexpected("required object attribute template is missing");
    }

    auto prompt_context = minja::Value::object();
    prompt_context.set("attribute", minja::Value(std::string{attribute}));
    const auto CONTEXT       = minja::Context::make(std::move(prompt_context));
    const std::string PROMPT = PROMPT_TEMPLATE->second->render(CONTEXT);

    const auto APPLY_RESPONSE = [&](const std::string& raw) -> TaskResult {
      auto parsed = parseJsonObject(raw);
      if (!parsed)
      {
        return std::unexpected(parsed.error());
      }
      const auto VALUES = parsed->find(std::string{attribute});
      if (VALUES == parsed->end() || !VALUES->is_array())
      {
        return std::unexpected("LLM response is missing the expected '" + std::string(attribute) +
                               "' array");
      }
      return applyLabels(
        *graph, logger, object_id, std::string{relation}, std::string{category}, *VALUES);
    };

    auto response = chatTurn(client, history, PROMPT, logger);
    if (!response)
    {
      return std::unexpected(response.error());
    }
    if (APPLY_RESPONSE(*response))
    {
      return {};
    }
    // Nothing was written for a malformed response, so ask once for the right format.
    auto formatting_context = minja::Value::object();
    formatting_context.set("response", minja::Value(*response));
    formatting_context.set("attribute", minja::Value(std::string{attribute}));
    const auto FORMATTING_MINJA_CONTEXT = minja::Context::make(std::move(formatting_context));
    const std::string FORMATTING_PROMPT =
      FORMATTING_TEMPLATE->second->render(FORMATTING_MINJA_CONTEXT);
    auto retry = chatTurn(client, history, FORMATTING_PROMPT, logger);
    if (!retry)
    {
      return std::unexpected(retry.error());
    }
    if (auto retry_result = APPLY_RESPONSE(*retry); !retry_result)
    {
      return std::unexpected("invalid LLM response after formatting retry: " +
                             retry_result.error());
    }
    return {};
  }
  catch (const std::exception& e)
  {
    return std::unexpected("object attribute task failed: " + std::string(e.what()));
  }
  return {};
}

} // namespace semantic_scene_forests_generation
