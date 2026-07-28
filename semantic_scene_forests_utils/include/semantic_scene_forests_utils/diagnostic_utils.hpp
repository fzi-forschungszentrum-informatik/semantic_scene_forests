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

#ifndef SEMANTIC_SCENE_FORESTS_UTILS__DIAGNOSTIC_UTILS_HPP_
#define SEMANTIC_SCENE_FORESTS_UTILS__DIAGNOSTIC_UTILS_HPP_

#include <chrono>
#include <mutex>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

namespace semantic_scene_forests_utils {

// ---------------------------------------------------------------------------
// ErrorCounter
//
// Small thread-safe helper for tracking a cumulative count of failures (TF
// lookups, crops, LLM/service calls, aborted goals, ...). Reports the count,
// the most recent error message and the time since the last error.
//
// Topic freshness and latency are intentionally NOT provided here: use the
// built-in diagnostic_updater primitives (HeaderlessTopicDiagnostic /
// TopicDiagnostic) for those.
// ---------------------------------------------------------------------------
class ErrorCounter
{
public:
  void increment()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    error_count_++;
    last_error_time_ = std::chrono::steady_clock::now();
  }

  void increment(const std::string& last_error_msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    error_count_++;
    last_error_time_    = std::chrono::steady_clock::now();
    last_error_message_ = last_error_msg;
  }

  size_t count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return error_count_;
  }

  // Adds the count / last-error fields to an existing status wrapper without
  // touching the summary, so several counters can share one diagnostic task.
  void report(diagnostic_updater::DiagnosticStatusWrapper& stat, const std::string& label) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stat.add(label + " error count", error_count_);
    if (!last_error_message_.empty())
    {
      stat.add(label + " last error", last_error_message_);
    }
    if (error_count_ > 0)
    {
      auto age =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_error_time_).count();
      stat.add(label + " time since last error (s)", age);
    }
  }

  void reset()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    error_count_ = 0;
    last_error_message_.clear();
  }

private:
  mutable std::mutex mutex_;
  size_t error_count_{0};
  std::chrono::time_point<std::chrono::steady_clock> last_error_time_;
  std::string last_error_message_;
};

} // namespace semantic_scene_forests_utils

#endif // SEMANTIC_SCENE_FORESTS_UTILS__DIAGNOSTIC_UTILS_HPP_
