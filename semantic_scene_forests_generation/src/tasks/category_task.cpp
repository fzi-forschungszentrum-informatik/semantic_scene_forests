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

TaskResult CategoryTask::execute(oai::OpenAIClient& client,
                                 std::vector<oai::Message>& history,
                                 const fzi::semantic_scene_forests::SceneGraph& current_graph,
                                 fzi::semantic_scene_forests::SceneGraph& graph)
{
  try
  {
    const auto CONCEPTS = current_graph.getConceptNodes();
    if (CONCEPTS.empty())
    {
      return {};
    }

    const auto category_template = m_templates->find("meta/category");
    const auto followup_template = m_templates->find("meta/category_follow_up");
    if (category_template == m_templates->end() || followup_template == m_templates->end())
    {
      return std::unexpected("required category template is missing");
    }

    std::unordered_set<int64_t> allowed_ids;
    allowed_ids.reserve(CONCEPTS.size());
    auto input_list = minja::Value::array();
    for (const auto& concept_node : CONCEPTS)
    {
      allowed_ids.insert(concept_node.id);
      auto item = minja::Value::object();
      item.set("id", minja::Value(concept_node.id));
      item.set("concept_name", minja::Value(concept_node.concept_name));
      input_list.push_back(item);
    }
    auto prompt_context = minja::Value::object();
    prompt_context.set("input_list", input_list);
    const auto context       = minja::Context::make(std::move(prompt_context));
    const std::string prompt = category_template->second->render(context);

    const auto apply_response = [&](const std::string& raw) -> TaskResult {
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
        auto source_id = parseAllowedConceptId(id_string, allowed_ids);
        if (!source_id)
        {
          return std::unexpected(source_id.error());
        }
        if (!categories.is_array() || categories.size() > MAX_TASK_LABELS)
        {
          return std::unexpected("category values must be an array with a bounded size");
        }

        for (const auto& value : categories)
        {
          auto category_label = validateConceptLabel(value);
          if (!category_label)
          {
            return std::unexpected(category_label.error());
          }
          if (*category_label == "category")
          {
            return std::unexpected("categories must not contain the category root");
          }

          auto category_id = findOrCreateConcept(graph, *category_label);
          if (!category_id)
          {
            return std::unexpected(category_id.error());
          }
          if (auto edge_result = addUniqueEdge(graph, *source_id, *category_id, "is_type_of");
              !edge_result)
          {
            return edge_result;
          }

          auto root_id = findOrCreateConcept(graph, "category");
          if (!root_id)
          {
            return std::unexpected(root_id.error());
          }
          if (auto edge_result = addUniqueEdge(graph, *category_id, *root_id, "is_type_of");
              !edge_result)
          {
            return edge_result;
          }
        }
      }
      return {};
    };

    auto content = chatTurn(client, history, prompt, m_logger);
    if (!content)
    {
      return std::unexpected(content.error());
    }
    auto graph_before_response = graph.clone();
    if (!graph_before_response)
    {
      return std::unexpected(graph_before_response.error().message);
    }
    if (auto apply_result = apply_response(*content); apply_result)
    {
      return {};
    }
    graph.swap(**graph_before_response);

    auto followup_context = minja::Value::object();
    auto category_list    = minja::Value::array();
    for (const auto& concept_node : CONCEPTS)
    {
      category_list.push_back(minja::Value(concept_node.concept_name));
    }
    followup_context.set("categories", category_list);
    const auto followup_minja_context = minja::Context::make(std::move(followup_context));
    const std::string followup        = followup_template->second->render(followup_minja_context);
    auto retry                        = chatTurn(client, history, followup, m_logger);
    if (!retry)
    {
      return std::unexpected(retry.error());
    }
    if (auto retry_result = apply_response(*retry); !retry_result)
    {
      return std::unexpected("invalid category response after follow-up: " + retry_result.error());
    }
    return {};
  }
  catch (const std::exception& e)
  {
    return std::unexpected("category task failed: " + std::string(e.what()));
  }
}

} // namespace semantic_scene_forests_generation
