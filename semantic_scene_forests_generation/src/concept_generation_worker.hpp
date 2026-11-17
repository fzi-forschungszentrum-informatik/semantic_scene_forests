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

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stop_token>
#include <string>
#include <vector>

#include <oai_api/client.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <semantic_scene_forests/scene_graph.hpp>
#include <semantic_scene_forests_msgs/action/run_worker.hpp>

#include "generation_common.hpp"
#include "semantic_scene_forests_generation/task_interface.hpp"
#include "semantic_scene_forests_manager/worker_interface.hpp"

namespace semantic_scene_forests_generation {

//! Runs the IGraphTask plugins every `<name>.interval_seconds` on the concepts it has not
//! processed yet and writes the results to the shared graph, marking the input as processed by
//! `<name>`. The `generate_concepts` action runs them on all concepts.
class ConceptGenerationWorker final : public semantic_scene_forests_manager::IWorker
{
public:
  ConceptGenerationWorker() = default;
  ~ConceptGenerationWorker() override;
  ConceptGenerationWorker(const ConceptGenerationWorker&)            = delete;
  ConceptGenerationWorker& operator=(const ConceptGenerationWorker&) = delete;
  ConceptGenerationWorker(ConceptGenerationWorker&&)                 = delete;
  ConceptGenerationWorker& operator=(ConceptGenerationWorker&&)      = delete;

  void initialize(rclcpp::Node& node,
                  std::shared_ptr<fzi::semantic_scene_forests::SceneGraph> graph,
                  const std::string& name,
                  diagnostic_updater::Updater& diagnostics) override;
  void start() override;
  void stop() override;

private:
  enum class Outcome : std::uint8_t
  {
    GENERATED,
    SKIPPED,
    STOPPED
  };

  void runBackground(const std::stop_token& stop);
  //! Runs the tasks on the unprocessed concepts (all concepts if !only_unprocessed), then marks
  //! them. Throws on failure, leaving the concepts unprocessed.
  Outcome generate(bool only_unprocessed, const std::function<bool()>& should_stop);
  //! The ids of the concepts to process.
  [[nodiscard]] std::vector<int64_t> planInput(bool only_unprocessed) const;

  std::shared_ptr<fzi::semantic_scene_forests::SceneGraph> m_graph;
  std::string m_name;
  rclcpp::Logger m_logger{rclcpp::get_logger("concept_generation")};
  std::chrono::seconds m_interval{120};

  TemplateMap m_templates;
  // Declared before the tasks: the loader must outlive the instances it created.
  std::unique_ptr<pluginlib::ClassLoader<IGraphTask>> m_task_loader;
  std::vector<std::shared_ptr<IGraphTask>> m_tasks;
  std::unique_ptr<oai::OpenAIClient> m_client;
  // The background loop and a goal may overlap for one run: one generates at a time.
  std::mutex m_generate_mutex;

  RunStats m_stats;
  WorkerThreads m_threads;

  // Declared last so it is destroyed first, before the state its callbacks use.
  rclcpp_action::Server<WorkerThreads::RunWorker>::SharedPtr m_action_server;
};

} // namespace semantic_scene_forests_generation
