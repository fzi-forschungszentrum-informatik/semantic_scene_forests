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

#include "affordance_task.hpp"
#include "task_utils.hpp"

#include <minja/minja.hpp>
#include <nlohmann/json.hpp>

#include <unordered_set>

namespace semantic_scene_forests_generation {

void AffordanceTask::initialize(rclcpp::Node* node, const TemplateMap& templates)
{
  m_logger    = node->get_logger().get_child("AffordanceTask");
  m_templates = &templates;
}

TaskResult AffordanceTask::execute(oai::OpenAIClient& client,
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

    const auto affordance_template = m_templates->find("meta/affordance");
    if (affordance_template == m_templates->end())
    {
      return std::unexpected("required affordance template is missing");
    }

    std::unordered_set<int64_t> allowed_ids;
    allowed_ids.reserve(CONCEPTS.size());
    auto input_list = minja::Value::array();
    for (const auto& concept_node : CONCEPTS)
    {
      allowed_ids.insert(concept_node.id);
      auto item = minja::Value::object();
      item.set("id", minja::Value(concept_node.id));
      item.set("class_name", minja::Value(concept_node.concept_name));
      input_list.push_back(item);
    }
    auto prompt_context = minja::Value::object();
    prompt_context.set("input_list", input_list);
    const auto context       = minja::Context::make(std::move(prompt_context));
    const std::string prompt = affordance_template->second->render(context);

    auto content = chatTurn(client, history, prompt, m_logger);
    if (!content)
    {
      return std::unexpected(content.error());
    }
    auto parsed = parseJsonObject(*content);
    if (!parsed)
    {
      return std::unexpected(parsed.error());
    }
    if (parsed->size() > MAX_TASK_SOURCE_IDS)
    {
      return std::unexpected("LLM response contains too many concept ids");
    }

    for (const auto& [id_string, affordances] : parsed->items())
    {
      auto source_id = parseAllowedConceptId(id_string, allowed_ids);
      if (!source_id)
      {
        return std::unexpected(source_id.error());
      }
      if (!affordances.is_array() || affordances.size() > MAX_TASK_LABELS)
      {
        return std::unexpected("affordance values must be an array with a bounded size");
      }

      for (const auto& value : affordances)
      {
        auto affordance_label = validateConceptLabel(value);
        if (!affordance_label)
        {
          return std::unexpected(affordance_label.error());
        }
        if (*affordance_label == "affordance")
        {
          return std::unexpected("affordances must not contain the affordance root");
        }

        auto affordance_id = findOrCreateConcept(graph, *affordance_label);
        if (!affordance_id)
        {
          return std::unexpected(affordance_id.error());
        }
        if (auto edge_result = addUniqueEdge(graph, *source_id, *affordance_id, "has_affordance");
            !edge_result)
        {
          return edge_result;
        }

        auto root_id = findOrCreateConcept(graph, "affordance");
        if (!root_id)
        {
          return std::unexpected(root_id.error());
        }
        if (auto edge_result = addUniqueEdge(graph, *affordance_id, *root_id, "is_type_of");
            !edge_result)
        {
          return edge_result;
        }
      }
    }
    return {};
  }
  catch (const std::exception& e)
  {
    return std::unexpected("affordance task failed: " + std::string(e.what()));
  }
}

} // namespace semantic_scene_forests_generation
