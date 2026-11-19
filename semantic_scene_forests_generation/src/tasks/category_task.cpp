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

#include "category_task.hpp"
#include "task_utils.hpp"

#include <minja/minja.hpp>
#include <nlohmann/json.hpp>

#include <unordered_set>

namespace semantic_scene_forests_generation {

void CategoryTask::initialize(rclcpp::Node* node, const TemplateMap& templates)
{
  m_logger    = node->get_logger().get_child("CategoryTask");
  m_templates = &templates;
}

TaskResult
CategoryTask::execute(oai::OpenAIClient& client,
                      std::vector<oai::Message>& history,
                      const std::vector<fzi::semantic_scene_forests::ConceptNode>& concepts,
                      const std::shared_ptr<fzi::semantic_scene_forests::SceneGraph>& graph)
{
  if (concepts.empty())
  {
    return {};
  }

  const auto CATEGORY_TEMPLATE = m_templates->find("meta/category");
  const auto FOLLOWUP_TEMPLATE = m_templates->find("meta/category_follow_up");
  if (CATEGORY_TEMPLATE == m_templates->end() || FOLLOWUP_TEMPLATE == m_templates->end())
  {
    throw std::runtime_error("required category template is missing");
  }

  std::unordered_set<int64_t> allowed_ids;
  allowed_ids.reserve(concepts.size());
  auto input_list = minja::Value::array();
  for (const auto& concept_node : concepts)
  {
    allowed_ids.insert(concept_node.id);
    auto item = minja::Value::object();
    item.set("id", minja::Value(concept_node.id));
    item.set("concept_name", minja::Value(concept_node.concept_name));
    input_list.push_back(item);
  }
  auto prompt_context = minja::Value::object();
  prompt_context.set("input_list", input_list);
  const auto CONTEXT       = minja::Context::make(std::move(prompt_context));
  const std::string PROMPT = CATEGORY_TEMPLATE->second->render(CONTEXT);

  const auto APPLY_RESPONSE = [&](const std::string& raw) -> TaskResult {
    auto parsed = parseJsonObject(raw);
    if (!parsed)
    {
      return std::unexpected(parsed.error());
    }
    if (parsed->size() > MAX_TASK_SOURCE_IDS)
    {
      return std::unexpected("LLM response contains too many concept ids");
    }

    for (const auto& [id_string, categories] : parsed->items())
    {
      const auto SOURCE_ID = parseAllowedConceptId(id_string, allowed_ids);
      if (!SOURCE_ID)
      {
        RCLCPP_WARN(m_logger, "Skipping '%s': %s", id_string.c_str(), SOURCE_ID.error().c_str());
        continue;
      }
      if (auto applied =
            applyLabels(*graph, m_logger, *SOURCE_ID, "is_type_of", "category", categories);
          !applied)
      {
        RCLCPP_WARN(m_logger, "Skipping '%s': %s", id_string.c_str(), applied.error().c_str());
      }
    }
    return {};
  };

  auto content = chatTurn(client, history, PROMPT, m_logger);
  if (APPLY_RESPONSE(content))
  {
    return {};
  }

  auto followup_context = minja::Value::object();
  auto category_list    = minja::Value::array();
  for (const auto& concept_node : concepts)
  {
    category_list.push_back(minja::Value(concept_node.concept_name));
  }
  followup_context.set("categories", category_list);
  const auto FOLLOWUP_MINJA_CONTEXT = minja::Context::make(std::move(followup_context));
  const std::string FOLLOWUP        = FOLLOWUP_TEMPLATE->second->render(FOLLOWUP_MINJA_CONTEXT);
  auto retry                        = chatTurn(client, history, FOLLOWUP, m_logger);
  if (auto retry_result = APPLY_RESPONSE(retry); !retry_result)
  {
    return std::unexpected("invalid category response after follow-up: " + retry_result.error());
  }
  return {};
}

} // namespace semantic_scene_forests_generation
