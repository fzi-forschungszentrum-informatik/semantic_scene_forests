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
#include "concept_generation_worker.hpp"

#include <exception>
#include <functional>
#include <optional>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <minja/minja.hpp>
#include <oai_api/builder.hpp>
#include <oai_api/types.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <semantic_scene_forests/scene_graph.hpp>

#include "generation_common.hpp"
#include "semantic_scene_forests_generation/concept_generation_parameters.hpp"

namespace semantic_scene_forests_generation {

using fzi::semantic_scene_forests::SceneGraph;
using semantic_scene_forests_manager::Tx;

ConceptGenerationWorker::~ConceptGenerationWorker()
{
  stop();
}

void ConceptGenerationWorker::initialize(rclcpp::Node& node,
                                         semantic_scene_forests_manager::Database& db,
                                         const std::string& name,
                                         diagnostic_updater::Updater& diagnostics)
{
  m_db     = &db;
  m_name   = name;
  m_logger = node.get_logger().get_child(name);
  const auto PARAMS =
    concept_generation::ParamListener(node.get_node_parameters_interface(), m_logger, name + ".")
      .get_params();
  m_interval = std::chrono::seconds(PARAMS.interval_seconds);

  m_templates   = loadTemplates(PARAMS.prompt_dir, m_logger);
  m_task_loader = std::make_unique<pluginlib::ClassLoader<IGraphTask>>(
    "semantic_scene_forests_generation", "semantic_scene_forests_generation::IGraphTask");
  m_tasks  = loadTasks(*m_task_loader, PARAMS.tasks, node, m_templates);
  m_client = std::make_unique<oai::OpenAIClient>(
    makeClient({.server_url              = PARAMS.server_url,
                .api_key                 = PARAMS.api_key,
                .extra_body              = PARAMS.extra_body,
                .model                   = PARAMS.model,
                .connect_timeout_seconds = PARAMS.llm_connect_timeout_seconds,
                .request_timeout_seconds = PARAMS.llm_request_timeout_seconds}));

  m_action_server =
    m_threads.serve(node,
                    "generate_concepts",
                    m_logger,
                    "Concept generation",
                    [this](const std::shared_ptr<WorkerThreads::GoalHandle>& /*goal_handle*/,
                           const std::function<bool()>& should_stop) -> std::optional<std::string> {
                      Outcome outcome{};
                      try
                      {
                        outcome = generate(false, should_stop);
                      }
                      catch (const std::exception& e)
                      {
                        m_stats.failed(e.what());
                        throw;
                      }
                      switch (outcome)
                      {
                        case Outcome::GENERATED:
                          return "Concept generation complete";
                        case Outcome::SKIPPED:
                          return "Skipped: no concepts are available";
                        case Outcome::STOPPED:
                          break;
                      }
                      return std::nullopt;
                    });
  diagnostics.add(name + " health", [this](diagnostic_updater::DiagnosticStatusWrapper& stat) {
    m_stats.report(stat);
    stat.add("Pending concepts",
             m_db->read([this](const Tx& tx) { return tx.conceptsUnmarkedBy(m_name); }).size());
    stat.add("Goal active", m_threads.goalActive());
  });
  RCLCPP_INFO(m_logger,
              "Concept generation with %zu tasks every %llds (%s)",
              m_tasks.size(),
              static_cast<long long>(m_interval.count()),
              PARAMS.model.c_str());
}

void ConceptGenerationWorker::start()
{
  m_threads.startBackground([this](const std::stop_token& stop) { runBackground(stop); });
}

void ConceptGenerationWorker::stop()
{
  m_threads.stop();
}

void ConceptGenerationWorker::runBackground(const std::stop_token& stop)
{
  const auto SHOULD_STOP = [&stop] { return stop.stop_requested(); };
  while (m_threads.waitForTick(stop, m_interval))
  {
    // A failed run keeps its concepts unmarked for the next tick.
    try
    {
      generate(true, SHOULD_STOP);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(m_logger, "Background concept generation failed: %s", e.what());
      m_stats.failed(e.what());
    }
    catch (...)
    {
      // A pluginlib-loaded task can throw a type that is not a std::exception.
      RCLCPP_ERROR(m_logger, "Background concept generation failed: unknown exception");
      m_stats.failed("unknown exception");
    }
  }
}

ConceptGenerationWorker::Outcome
ConceptGenerationWorker::generate(bool only_unmarked, const std::function<bool()>& should_stop)
{
  const std::scoped_lock LOCK{m_generate_mutex};
  if (should_stop())
  {
    return Outcome::STOPPED;
  }
  const SceneGraph INPUT = readInput(only_unmarked);
  const auto CONCEPTS    = INPUT.getConceptNodes();
  if (CONCEPTS.empty())
  {
    return Outcome::SKIPPED;
  }

  const auto SYSTEM_TEMPLATE = m_templates.find("meta/meta_system_prompt");
  const auto SETUP_TEMPLATE  = m_templates.find("meta/meta_setup_prompt");
  if (SYSTEM_TEMPLATE == m_templates.end() || SETUP_TEMPLATE == m_templates.end())
  {
    throw std::runtime_error("required prompt template is missing");
  }

  std::vector<oai::Message> history;
  auto system_message =
    oai::MessageBuilder::system()
      .text(SYSTEM_TEMPLATE->second->render(minja::Context::make(minja::Value::object())))
      .build();
  if (!system_message)
  {
    throw std::runtime_error("failed to build the system message: " + system_message.error());
  }
  history.push_back(std::move(*system_message));

  auto setup_ctx    = minja::Value::object();
  auto objects_list = minja::Value::array();
  for (const auto& concept_node : CONCEPTS)
  {
    objects_list.push_back(minja::Value(concept_node.concept_name));
  }
  setup_ctx.set("input_objects", objects_list);
  auto setup_message =
    oai::MessageBuilder::user()
      .text(SETUP_TEMPLATE->second->render(minja::Context::make(std::move(setup_ctx))))
      .build();
  if (!setup_message)
  {
    throw std::runtime_error("failed to build the setup message: " + setup_message.error());
  }
  history.push_back(std::move(*setup_message));

  auto& client = *m_client;
  auto ack     = client.chat({.messages = history});
  if (!ack)
  {
    throw std::runtime_error("setup request failed: " + ack.error().message);
  }
  auto acknowledgement = oai::MessageBuilder::assistant().text(ack->text()).build();
  if (!acknowledgement)
  {
    throw std::runtime_error("failed to build the setup acknowledgement: " +
                             acknowledgement.error());
  }
  history.push_back(std::move(*acknowledgement));

  auto after = INPUT.clone();
  if (!after)
  {
    throw std::runtime_error("failed to clone the scratch graph: " + after.error().message);
  }
  for (auto& task : m_tasks)
  {
    if (should_stop())
    {
      return Outcome::STOPPED;
    }
    if (auto task_result = task->execute(client, history, INPUT, **after); !task_result)
    {
      throw std::runtime_error("task failed: " + task_result.error());
    }
  }
  if (should_stop())
  {
    return Outcome::STOPPED;
  }

  commit(INPUT, **after);
  RCLCPP_INFO(m_logger, "Processed %zu concepts", CONCEPTS.size());
  m_stats.succeeded();
  return Outcome::GENERATED;
}

SceneGraph ConceptGenerationWorker::readInput(bool only_unmarked) const
{
  // The input holds the concepts with their database ids and no edges.
  // TODO: concepts hidden by visibleGraph() (not reachable from a visible object) stay unmarked and
  // are retried each tick at no LLM cost; revisit if Tx gains a concept-label query.
  return m_db->read([&](const Tx& tx) {
    const auto GRAPH = tx.visibleGraph();
    std::unordered_set<int64_t> wanted;
    if (only_unmarked)
    {
      const auto IDS = tx.conceptsUnmarkedBy(m_name);
      wanted.insert(IDS.begin(), IDS.end());
    }
    SceneGraph concepts;
    for (const auto& concept_node : GRAPH.getConceptNodes())
    {
      if (only_unmarked && !wanted.contains(concept_node.id))
      {
        continue;
      }
      if (auto added = concepts.addConceptNodeWithId(concept_node.id, concept_node.concept_name);
          !added)
      {
        throw std::runtime_error("failed to collect a concept: " + added.error().message);
      }
    }
    return concepts;
  });
}

void ConceptGenerationWorker::commit(const SceneGraph& input, const SceneGraph& after)
{
  // Concepts this run created count as processed, or the next tick would categorize its own
  // categories.
  // TODO: the scratch graph has no edges, so SceneGraph's cycle rejection does not see the
  // database's edges and Tx::addEdge has no cycle check; add one if cyclic is_type_of shows up.
  m_db->write([&](Tx& tx) {
    tx.commitScratch(input, after);
    std::unordered_set<int64_t> input_ids;
    for (const auto& concept_node : input.getConceptNodes())
    {
      input_ids.insert(concept_node.id);
      tx.mark(m_name, concept_node.id);
    }
    for (const auto& concept_node : after.getConceptNodes())
    {
      if (!input_ids.contains(concept_node.id))
      {
        // Returns the id commitScratch merged the scratch concept into.
        tx.mark(m_name, tx.findOrCreateConcept(concept_node.concept_name));
      }
    }
  });
}

} // namespace semantic_scene_forests_generation

PLUGINLIB_EXPORT_CLASS(semantic_scene_forests_generation::ConceptGenerationWorker,
                       semantic_scene_forests_manager::IWorker)
