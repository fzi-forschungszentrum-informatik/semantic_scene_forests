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

#include <chrono>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "semantic_scene_forests_manager/database.hpp"
#include "semantic_scene_forests_manager/worker_interface.hpp"

namespace semantic_scene_forests_manager {

//! Deletes edgeless concepts every `<name>.interval_seconds`.
class PruningWorker : public IWorker
{
public:
  void initialize(rclcpp::Node& node,
                  Database& db,
                  const std::string& name,
                  diagnostic_updater::Updater& diagnostics) override
  {
    m_db     = &db;
    m_logger = node.get_logger().get_child(name);
    m_interval =
      std::chrono::duration<double>(node.declare_parameter(name + ".interval_seconds", 10.0));
    if (m_interval.count() <= 0.0)
    {
      throw std::invalid_argument(name + ".interval_seconds must be positive");
    }
    diagnostics.add(name + " health", this, &PruningWorker::onDiagnostic);
  }

  void start() override
  {
    m_thread = std::jthread([this](const std::stop_token& stop) { run(stop); });
  }

  void stop() override
  {
    m_thread.request_stop();
    if (m_thread.joinable())
    {
      m_thread.join();
    }
  }

private:
  void run(const std::stop_token& stop)
  {
    std::mutex mutex;
    std::condition_variable_any wake;
    std::unique_lock lock{mutex};
    while (true)
    {
      wake.wait_for(lock, stop, m_interval, [] { return false; });
      if (stop.stop_requested())
      {
        return;
      }
      try
      {
        const int PRUNED = m_db->write([](Tx& tx) { return tx.pruneIsolatedConcepts(); });
        RCLCPP_DEBUG(m_logger, "Pruned %d isolated concepts", PRUNED);
        const std::scoped_lock STATS_LOCK{m_stats_mutex};
        m_pruned += PRUNED;
        m_last_error.clear();
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(m_logger, "Pruning failed: %s", e.what());
        const std::scoped_lock STATS_LOCK{m_stats_mutex};
        m_last_error = e.what();
      }
    }
  }

  void onDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    const std::scoped_lock LOCK{m_stats_mutex};
    stat.add("Pruned concepts", m_pruned);
    if (!m_last_error.empty())
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                   "Last pruning failed: " + m_last_error);
      return;
    }
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                 "Pruned " + std::to_string(m_pruned) + " isolated concepts");
  }

  Database* m_db{nullptr};
  rclcpp::Logger m_logger{rclcpp::get_logger("pruning_worker")};
  std::mutex m_stats_mutex;
  int64_t m_pruned{0};
  std::string m_last_error;
  std::chrono::duration<double> m_interval{10.0};
  std::jthread m_thread;
};

} // namespace semantic_scene_forests_manager

PLUGINLIB_EXPORT_CLASS(semantic_scene_forests_manager::PruningWorker,
                       semantic_scene_forests_manager::IWorker)
