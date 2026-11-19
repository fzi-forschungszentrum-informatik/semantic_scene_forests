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

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <expected>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <oai_api/client.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <semantic_scene_forests_object_tracker/object_tracker_parameters.hpp>

namespace semantic_scene_forests_object_tracker {

// Either "do A's crops show an a_label?" (label_check, only A is set) or "are global tracks A
// and B the same physical object?". A snapshot of the track(s).
struct ArbiterRequest
{
  bool label_check{false};
  bool stale_replacement{false}; // A is stale, B is seen now at its pose
  uint64_t a_id{0};
  uint64_t b_id{0};
  std::string a_label;
  std::string b_label;
  std::shared_ptr<const std::vector<sensor_msgs::msg::Image>> a_crops;
  std::shared_ptr<const std::vector<sensor_msgs::msg::Image>> b_crops;
  Eigen::Vector3d a_size;
  Eigen::Vector3d b_size;
  double center_distance{0.0};
};

struct ArbiterResult
{
  bool label_check{false};
  bool stale_replacement{false};
  uint64_t a_id{0};
  uint64_t b_id{0};
  // Label check: min(p(label correct), p(physical object, not a room)); nullopt on error.
  std::optional<double> same_probability;
  std::optional<double> object_probability; // label check only: p(physical object, not a room)
  std::string label;                        // chosen label, empty if none/invalid
  std::string error;
};

using DecideFn =
  std::function<std::expected<oai::DecisionResponse, oai::ApiError>(const oai::DecisionRequest&)>;

// "openai" | "openrouter" | "ollama" | "vllm" -> wire format; nullopt for anything else.
std::optional<oai::DecisionFormat> parseDecisionProvider(std::string_view provider);

// Builds the Decisions API client from the arbiter.* parameters. Throws std::invalid_argument
// for an unknown provider, an invalid extra_body or a missing key (openai/openrouter).
DecideFn makeDecisionClient(const object_tracker::Params::Arbiter& params);

// Runs arbiter requests on an owned thread for the node's lifetime: label checks (they hold a
// track back from publishing) and stale replacements first, then the rest in arrival order,
// back to back. Callers enqueue and drain without
// blocking; dedup is the caller's job. Destruction stops and joins the thread (an in-flight
// call is bounded by the client's request timeout); queued requests are discarded.
class ArbiterWorker
{
public:
  ArbiterWorker(DecideFn decide, oai::DecisionFormat format, std::string model);
  ArbiterWorker(const ArbiterWorker&)            = delete;
  ArbiterWorker& operator=(const ArbiterWorker&) = delete;
  ~ArbiterWorker();

  void enqueue(ArbiterRequest request);
  std::vector<ArbiterResult> drainResults();
  size_t pending() const;

  static ArbiterResult decide(const DecideFn& decide,
                              oai::DecisionFormat format,
                              const std::string& model,
                              const ArbiterRequest& request);

private:
  void run(const std::stop_token& stop);

  DecideFn m_decide;
  oai::DecisionFormat m_format;
  std::string m_model;
  mutable std::mutex m_mutex;
  std::condition_variable_any m_wake;
  std::deque<ArbiterRequest> m_priority_queue; // guarded by m_mutex
  std::deque<ArbiterRequest> m_queue;          // guarded by m_mutex
  std::vector<ArbiterResult> m_results;        // guarded by m_mutex
  bool m_busy{false};                          // guarded by m_mutex
  // Declared last: stopped and joined before the members it uses are destroyed.
  std::jthread m_thread;
};

} // namespace semantic_scene_forests_object_tracker
