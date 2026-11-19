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
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <oai_api/client.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <semantic_scene_forests_msgs/action/run_worker.hpp>

#include "semantic_scene_forests_generation/task_interface.hpp"

namespace semantic_scene_forests_generation {

//! LLM connection settings shared by the generation parameters.
struct LlmSettings
{
  std::string server_url;
  std::string api_key;
  //! JSON object merged into every request body; empty sends nothing extra.
  std::string extra_body;
  std::string model;
  int64_t connect_timeout_seconds{5};
  int64_t request_timeout_seconds{120};
};

//! Builds an OpenAI-compatible client. Throws std::invalid_argument for a non-object extra_body.
oai::OpenAIClient makeClient(const LlmSettings& settings);

//! Marks only `input_ids` as processed by `worker`; connected concepts stay unprocessed.
//! Ids deleted meanwhile are ignored.
void markRunResults(fzi::semantic_scene_forests::SceneGraph& graph,
                    const std::string& worker,
                    const std::vector<int64_t>& input_ids);

//! Parses every `.j2` file below prompt_dir (the package's share/resources when empty), keyed by
//! its relative path without extension, e.g. "instance/instance_system_prompt".
TemplateMap loadTemplates(const std::string& prompt_dir, const rclcpp::Logger& logger);

//! Creates and initializes the named task plugins; a plugin that fails to load is logged and
//! skipped. The loader must outlive the returned tasks.
template <class TTask>
std::vector<std::shared_ptr<TTask>> loadTasks(pluginlib::ClassLoader<TTask>& loader,
                                              const std::vector<std::string>& names,
                                              rclcpp::Node& node,
                                              const TemplateMap& templates)
{
  std::vector<std::shared_ptr<TTask>> tasks;
  for (const auto& name : names)
  {
    try
    {
      auto task = loader.createSharedInstance(name);
      task->initialize(&node, templates);
      tasks.push_back(std::move(task));
      RCLCPP_INFO(node.get_logger(), "Loaded task: %s", name.c_str());
    }
    catch (const pluginlib::PluginlibException& e)
    {
      RCLCPP_ERROR(node.get_logger(), "Failed to load task '%s': %s", name.c_str(), e.what());
    }
  }
  return tasks;
}

//! Success/failure counters of a worker's runs for its diagnostics; thread-safe.
class RunStats
{
public:
  void succeeded();
  void failed(std::string error);
  //! WARN while the most recent run failed, OK otherwise.
  void report(diagnostic_updater::DiagnosticStatusWrapper& stat);

private:
  std::mutex m_mutex;
  int64_t m_succeeded{0};
  int64_t m_failed{0};
  std::string m_last_error;
  bool m_last_failed{false};
};

//! A worker's background thread and its single manual goal thread: reject-while-busy admission,
//! no queuing, cooperative stop (AGENTS.md threading policy).
class WorkerThreads
{
public:
  using RunWorker  = semantic_scene_forests_msgs::action::RunWorker;
  using GoalHandle = rclcpp_action::ServerGoalHandle<RunWorker>;
  //! Runs a goal. Returns the success message, nullopt if should_stop() ended it early; throws on
  //! failure.
  using GoalBody = std::function<std::optional<std::string>(
    const std::shared_ptr<GoalHandle>&, const std::function<bool()>& should_stop)>;

  //! Serves `action`, running body on the goal thread. A client cancel ends as canceled(), a
  //! goal caught during teardown gets no result, a throw aborts with "<what> failed: ...".
  //! Keep the returned server declared after the state body uses, so it is destroyed first.
  rclcpp_action::Server<RunWorker>::SharedPtr serve(rclcpp::Node& node,
                                                    const std::string& action,
                                                    const rclcpp::Logger& logger,
                                                    const std::string& what,
                                                    GoalBody body);
  void startBackground(std::function<void(const std::stop_token&)> loop);
  //! Sleeps `interval` (stop-aware) until no goal is active; false once stop is requested.
  bool waitForTick(const std::stop_token& stop, std::chrono::duration<double> interval);
  [[nodiscard]] bool goalActive();
  //! Stops and joins both threads; every later goal is rejected. Idempotent.
  void stop();

private:
  void runGoal(const std::shared_ptr<GoalHandle>& goal_handle,
               const std::stop_token& stop,
               const std::string& what,
               const rclcpp::Logger& logger,
               const GoalBody& body);

  std::mutex m_mutex;
  bool m_goal_active{false};
  bool m_stopping{false};
  std::jthread m_goal_thread;
  std::condition_variable_any m_wake;
  std::jthread m_background_thread;
};

} // namespace semantic_scene_forests_generation
