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

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

#include "arbiter_worker.hpp"
#include "camera_tracker.hpp"
#include "semantic_scene_forests_object_tracker/tracked_object.hpp"
#include <semantic_scene_forests_object_tracker/object_tracker_parameters.hpp>

namespace semantic_scene_forests_object_tracker {

// Reconciles the tracklets of all cameras into global tracks. Owned by one thread (the
// global cycle). Identity rules:
//  - a tracklet stays linked to its global track for its lifetime (many tracklets, e.g. from
//    several cameras, may link to one global track);
//  - a new tracklet links to a global track (stale or not) that overlaps clearly, has a
//    similar size and an accepted label; otherwise it becomes a new global track;
//  - overlapping similar-sized global tracks merge when a label is accepted by the other and
//    the overlap is clear; label conflicts and partial overlaps go to the arbiter;
//  - a stale track (unseen > stale_timeout) covered by a differently sized track is replaced.
// Arbiter answers are recorded for the session: "same" merges and accepts both labels,
// "different" keeps the pair apart for good. With the arbiter, a new global track is held back
// (not published) until a label check confirms its label; a rejected label removes it as a
// false positive, a failed check publishes it with the voted label.
class GlobalTracker
{
public:
  using TrackPair = std::pair<uint64_t, uint64_t>; // {lower id, higher id}

  struct Stats
  {
    size_t tracks{0};
    size_t stale{0};
    size_t links{0};
    size_t decisions{0};
    size_t merges{0};
    size_t replacements{0};
    size_t held{0};
    size_t label_rejections{0};
    size_t decision_errors{0};
    std::string last_error;
  };

  explicit GlobalTracker(rclcpp::Logger logger);

  // One reconciliation cycle. `results` are finished arbiter answers; returns the new
  // arbiter requests (always empty when arbiter_enabled is false).
  std::vector<ArbiterRequest> cycle(const std::vector<Tracklet>& tracklets,
                                    const std::vector<ArbiterResult>& results,
                                    const object_tracker::Params& params,
                                    bool arbiter_enabled);

  const std::vector<std::unique_ptr<TrackedObject>>& tracks() const { return m_tracks; }
  bool isStale(const TrackedObject& track, const object_tracker::Params& params) const;
  // True while a new track waits for its label check; held tracks are not published.
  bool isHeld(const TrackedObject& track) const { return m_held.contains(track.track_id); }
  Stats stats(const object_tracker::Params& params) const;

private:
  using LinkKey = std::pair<std::string, uint64_t>; // {camera, local id}
  struct Link
  {
    uint64_t global_id;
    rclcpp::Time last_applied;
  };

  TrackedObject* find(uint64_t id);
  bool labelAccepted(const TrackedObject& track, const std::string& label) const;
  // Starts a global track; with the arbiter it is held and a label check is returned.
  std::optional<ArbiterRequest>
  createFrom(const Tracklet& tracklet, const object_tracker::Params& params, bool arbiter_enabled);
  // Merges the younger identity into the older one and moves links and records over.
  uint64_t merge(uint64_t a_id, uint64_t b_id);
  void retire(uint64_t id);
  void applyResult(const ArbiterResult& result, const object_tracker::Params& params);
  std::vector<ArbiterRequest> reconcilePairs(const object_tracker::Params& params,
                                             bool arbiter_enabled);
  void eraseRemoved();

  rclcpp::Logger m_logger;
  std::vector<std::unique_ptr<TrackedObject>> m_tracks;
  std::map<LinkKey, Link> m_links;
  std::map<uint64_t, std::set<std::string>> m_accepted_labels;
  std::set<TrackPair> m_distinct;       // arbiter: different objects, never merged
  std::set<TrackPair> m_asked;          // queued or answered, never asked again
  std::set<TrackPair> m_failed;         // request failed: fall back to the overlap rule
  std::set<uint64_t> m_removed;         // merged/retired this cycle, erased at safe points
  std::set<uint64_t> m_held;            // new tracks waiting for their label check
  std::optional<rclcpp::Time> m_latest; // latest observation time across all cameras
  uint64_t m_next_id{1};
  size_t m_decisions{0};
  size_t m_merges{0};
  size_t m_replacements{0};
  size_t m_label_rejections{0};
  size_t m_decision_errors{0};
  std::string m_last_error;
};

} // namespace semantic_scene_forests_object_tracker
