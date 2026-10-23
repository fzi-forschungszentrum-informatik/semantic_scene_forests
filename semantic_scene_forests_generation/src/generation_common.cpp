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

#include "generation_common.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <utility>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <minja/minja.hpp>
#include <nlohmann/json.hpp>
#include <oai_api/builder.hpp>

namespace semantic_scene_forests_generation {

namespace fs = std::filesystem;

oai::OpenAIClient makeClient(const LlmSettings& settings)
{
  // oai-api silently ignores a non-object extra body, so reject typos at startup.
  nlohmann::json extra_body;
  if (!settings.extra_body.empty())
  {
    extra_body = nlohmann::json::parse(settings.extra_body, nullptr, false);
    if (!extra_body.is_object())
    {
      throw std::invalid_argument(
        "extra_body must be a JSON object (double quotes, lowercase true/false): " +
        settings.extra_body);
    }
  }
  return oai::ClientBuilder{}
    .baseUrl(settings.server_url)
    .apiKey(settings.api_key)
    .model(settings.model)
    // OpenRouter app attribution: the referer URL is the app identity, the title its name.
    .header("HTTP-Referer",
            "https://github.com/fzi-forschungszentrum-informatik/semantic_scene_forests/tree/"
            "main/semantic_scene_forests_generation")
    .header("X-OpenRouter-Title", "semantic_scene_forests_generation")
    .extraBody(extra_body)
    .connectTimeout(std::chrono::seconds{std::max<int64_t>(1, settings.connect_timeout_seconds)})
    .requestTimeout(std::chrono::seconds{std::max<int64_t>(1, settings.request_timeout_seconds)})
    .build();
}

TemplateMap loadTemplates(const std::string& prompt_dir, const rclcpp::Logger& logger)
{
  const fs::path ROOT =
    prompt_dir.empty()
      ? ament_index_cpp::get_package_share_path("semantic_scene_forests_generation") / "resources"
      : fs::path(prompt_dir);
  RCLCPP_INFO(logger, "Root template dir: %s", ROOT.c_str());
  const minja::Options OPTS{
    .trim_blocks = true, .lstrip_blocks = true, .keep_trailing_newline = false};

  TemplateMap templates;
  for (const auto& entry : fs::recursive_directory_iterator(ROOT))
  {
    if (!entry.is_regular_file() || entry.path().extension() != ".j2")
    {
      continue;
    }
    // Key: relative path without extension, e.g. "instance/single_attribute"
    const std::string KEY = entry.path()
                              .lexically_normal()
                              .lexically_relative(ROOT.lexically_normal())
                              .replace_extension()
                              .string();

    std::ifstream file{entry.path()};
    const std::string CONTENT{std::istreambuf_iterator<char>(file),
                              std::istreambuf_iterator<char>()};

    templates[KEY] = minja::Parser::parse(CONTENT, OPTS);
    RCLCPP_INFO(logger, "Loaded template: %s", KEY.c_str());
  }
  RCLCPP_INFO(logger, "Loaded %zu templates from %s", templates.size(), ROOT.c_str());
  return templates;
}

namespace {

//! Contains the rclcpp_action exception when teardown races a terminal goal transition.
template <class TFn> void terminal(const rclcpp::Logger& logger, TFn&& transition)
{
  try
  {
    std::forward<TFn>(transition)();
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(logger, "Terminal goal transition failed: %s", e.what());
  }
}

} // namespace

rclcpp_action::Server<WorkerThreads::RunWorker>::SharedPtr
WorkerThreads::serve(rclcpp::Node& node,
                     const std::string& action,
                     const rclcpp::Logger& logger,
                     const std::string& what,
                     GoalBody body)
{
  return rclcpp_action::create_server<RunWorker>(
    &node,
    action,
    [this, action, logger](const rclcpp_action::GoalUUID& /*uuid*/,
                           const std::shared_ptr<const RunWorker::Goal>& /*goal*/) {
      const std::scoped_lock LOCK{m_mutex};
      if (m_stopping || m_goal_active)
      {
        RCLCPP_WARN(logger, "Rejecting %s: a goal is active or stopping", action.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
      m_goal_active = true;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<GoalHandle>& /*goal_handle*/) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this, what, logger, body = std::move(body)](const std::shared_ptr<GoalHandle>& goal_handle) {
      {
        // Joins the finished previous goal thread outside the lock; stop() joins it otherwise.
        std::jthread previous;
        const std::scoped_lock LOCK{m_mutex};
        if (!m_stopping)
        {
          previous = std::move(m_goal_thread);
        }
      }
      const std::scoped_lock LOCK{m_mutex};
      if (m_stopping)
      {
        // A goal caught during teardown receives no result.
        m_goal_active = false;
        return;
      }
      m_goal_thread =
        std::jthread([this, goal_handle, what, logger, body](const std::stop_token& stop) {
          runGoal(goal_handle, stop, what, logger, body);
        });
    });
}

void WorkerThreads::runGoal(const std::shared_ptr<GoalHandle>& goal_handle,
                            const std::stop_token& stop,
                            const std::string& what,
                            const rclcpp::Logger& logger,
                            const GoalBody& body)
{
  const auto SHOULD_STOP = [&] { return stop.stop_requested() || goal_handle->is_canceling(); };
  auto result            = std::make_shared<RunWorker::Result>();
  try
  {
    if (auto message = body(goal_handle, SHOULD_STOP))
    {
      result->success = true;
      result->message = std::move(*message);
      terminal(logger, [&] { goal_handle->succeed(result); });
      RCLCPP_INFO(logger, "%s", result->message.c_str());
    }
    else if (!stop.stop_requested())
    {
      // A client cancellation; a requested stop token means teardown, which gets no result.
      result->message = "Canceled";
      terminal(logger, [&] { goal_handle->canceled(result); });
    }
  }
  catch (const std::exception& e)
  {
    result->message = what + " failed: " + e.what();
    terminal(logger, [&] { goal_handle->abort(result); });
    RCLCPP_ERROR(logger, "%s", result->message.c_str());
  }
  catch (...)
  {
    // A pluginlib-loaded task can throw a type that is not a std::exception.
    result->message = what + " failed: unknown exception";
    terminal(logger, [&] { goal_handle->abort(result); });
    RCLCPP_ERROR(logger, "%s", result->message.c_str());
  }
  const std::scoped_lock LOCK{m_mutex};
  m_goal_active = false;
}

void WorkerThreads::startBackground(std::function<void(const std::stop_token&)> loop)
{
  m_background_thread = std::jthread(std::move(loop));
}

bool WorkerThreads::waitForTick(const std::stop_token& stop, std::chrono::duration<double> interval)
{
  std::unique_lock lock{m_mutex};
  while (true)
  {
    m_wake.wait_for(lock, stop, interval, [] { return false; });
    if (stop.stop_requested())
    {
      return false;
    }
    if (!m_goal_active)
    {
      return true; // Otherwise a manual goal owns the pipeline.
    }
  }
}

bool WorkerThreads::goalActive()
{
  const std::scoped_lock LOCK{m_mutex};
  return m_goal_active;
}

void WorkerThreads::stop()
{
  std::jthread goal;
  {
    const std::scoped_lock LOCK{m_mutex};
    m_stopping = true;
    goal       = std::move(m_goal_thread);
  }
  goal.request_stop();
  m_background_thread.request_stop();
  if (goal.joinable())
  {
    goal.join();
  }
  if (m_background_thread.joinable())
  {
    m_background_thread.join();
  }
}

void RunStats::succeeded()
{
  const std::scoped_lock LOCK{m_mutex};
  ++m_succeeded;
  m_last_failed = false;
}

void RunStats::failed(std::string error)
{
  const std::scoped_lock LOCK{m_mutex};
  ++m_failed;
  m_last_error  = std::move(error);
  m_last_failed = true;
}

void RunStats::report(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  const std::scoped_lock LOCK{m_mutex};
  stat.add("Succeeded", m_succeeded);
  stat.add("Failed", m_failed);
  stat.add("Last error", m_last_error);
  if (m_last_failed)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Last run failed: " + m_last_error);
    return;
  }
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
               std::to_string(m_succeeded) + " succeeded, " + std::to_string(m_failed) + " failed");
}

} // namespace semantic_scene_forests_generation
