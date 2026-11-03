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

#include "global_tracker.hpp"

#include <algorithm>

#include <rclcpp/logging.hpp>

namespace semantic_scene_forests_object_tracker {

namespace {

DetectionAdapter toAdapter(const Tracklet& tracklet, const std::string& world_frame)
{
  DetectionAdapter adapter;
  adapter.class_name                  = tracklet.class_name;
  adapter.class_id                    = 0;
  adapter.score                       = tracklet.score;
  adapter.frame_id                    = world_frame;
  adapter.bbox3d.center.position.x    = tracklet.position.x();
  adapter.bbox3d.center.position.y    = tracklet.position.y();
  adapter.bbox3d.center.position.z    = tracklet.position.z();
  adapter.bbox3d.center.orientation.w = 1.0;
  adapter.bbox3d.size.x               = tracklet.size.x();
  adapter.bbox3d.size.y               = tracklet.size.y();
  adapter.bbox3d.size.z               = tracklet.size.z();
  return adapter;
}

GlobalTracker::TrackPair pairOf(uint64_t a, uint64_t b)
{
  return std::minmax(a, b);
}

void updateFrom(TrackedObject& track,
                const Tracklet& tracklet,
                const object_tracker::Params& params)
{
  // ponytail: one Kalman update per new tracklet observation; tracklet positions are already
  // filtered, so this double-filters. Fuse raw detections globally if positions lag on revisits.
  track.predict(std::max(0.0, (tracklet.last_seen - track.last_seen).seconds()));
  track.update(toAdapter(tracklet, params.world_frame), tracklet.last_seen);
  if (!tracklet.crops)
  {
    return;
  }
  // Tracklets repost their crop history every frame: add only crops not stored yet.
  const auto EXISTING = track.getCrops();
  for (const auto& crop : *tracklet.crops)
  {
    if (std::ranges::none_of(EXISTING, [&crop](const auto& stored) {
          return stored.header.stamp == crop.header.stamp &&
                 stored.header.frame_id == crop.header.frame_id;
        }))
    {
      track.addCrop(crop);
    }
  }
}

} // namespace

GlobalTracker::GlobalTracker(rclcpp::Logger logger)
  : m_logger(std::move(logger))
{
}

bool GlobalTracker::isStale(const TrackedObject& track, const object_tracker::Params& params) const
{
  return m_latest &&
         *m_latest - track.last_seen > rclcpp::Duration::from_seconds(params.stale_timeout);
}

TrackedObject* GlobalTracker::find(uint64_t id)
{
  if (m_removed.contains(id))
  {
    return nullptr;
  }
  const auto IT =
    std::ranges::find_if(m_tracks, [id](const auto& track) { return track->track_id == id; });
  return IT == m_tracks.end() ? nullptr : IT->get();
}

bool GlobalTracker::labelAccepted(const TrackedObject& track, const std::string& label) const
{
  if (label == track.class_name)
  {
    return true;
  }
  const auto IT = m_accepted_labels.find(track.track_id);
  return IT != m_accepted_labels.end() && IT->second.contains(label);
}

std::optional<ArbiterRequest> GlobalTracker::createFrom(const Tracklet& tracklet,
                                                        const object_tracker::Params& params,
                                                        bool arbiter_enabled)
{
  auto track = std::make_unique<TrackedObject>(toAdapter(tracklet, params.world_frame),
                                               tracklet.last_seen,
                                               m_next_id++,
                                               params.measurement_noise_std,
                                               params.process_noise_std,
                                               1,
                                               static_cast<size_t>(params.max_crops_per_object),
                                               1.0); // Tracklet sizes are already filtered.
  if (tracklet.crops)
  {
    for (const auto& crop : *tracklet.crops)
    {
      track->addCrop(crop);
    }
  }
  std::optional<ArbiterRequest> check;
  if (arbiter_enabled)
  {
    m_held.insert(track->track_id);
    check.emplace();
    check->label_check = true;
    check->a_id        = track->track_id;
    check->a_label     = track->class_name;
    check->a_crops     = tracklet.crops;
    check->a_size      = track->bbox3d_size;
  }
  m_tracks.push_back(std::move(track));
  return check;
}

uint64_t GlobalTracker::merge(uint64_t a_id, uint64_t b_id)
{
  const auto [SURVIVOR, REMOVED] = pairOf(a_id, b_id);
  auto* survivor                 = find(SURVIVOR);
  auto* removed                  = find(REMOVED);
  if (!survivor || !removed)
  {
    return SURVIVOR;
  }
  survivor->mergeFrom(*removed);
  if (!m_held.contains(REMOVED))
  {
    m_held.erase(SURVIVOR); // Same object as an already checked track.
  }
  m_held.erase(REMOVED);
  if (const auto IT = m_accepted_labels.find(REMOVED); IT != m_accepted_labels.end())
  {
    m_accepted_labels[SURVIVOR].insert(IT->second.begin(), IT->second.end());
    m_accepted_labels.erase(IT);
  }
  for (auto& [key, link] : m_links)
  {
    if (link.global_id == REMOVED)
    {
      link.global_id = SURVIVOR;
    }
  }
  std::set<TrackPair> distinct;
  for (const auto& [low, high] : m_distinct)
  {
    const auto LOW  = low == REMOVED ? SURVIVOR : low;
    const auto HIGH = high == REMOVED ? SURVIVOR : high;
    if (LOW != HIGH)
    {
      distinct.insert(pairOf(LOW, HIGH));
    }
  }
  m_distinct = std::move(distinct);
  m_removed.insert(REMOVED);
  ++m_merges;
  return SURVIVOR;
}

void GlobalTracker::retire(uint64_t id)
{
  m_removed.insert(id);
  m_held.erase(id);
  std::erase_if(m_links, [id](const auto& entry) { return entry.second.global_id == id; });
  m_accepted_labels.erase(id);
  std::erase_if(m_distinct,
                [id](const auto& pair) { return pair.first == id || pair.second == id; });
}

void GlobalTracker::eraseRemoved()
{
  std::erase_if(m_tracks,
                [this](const auto& track) { return m_removed.contains(track->track_id); });
  m_removed.clear();
}

void GlobalTracker::applyResult(const ArbiterResult& result, const object_tracker::Params& params)
{
  if (result.label_check)
  {
    auto* track = find(result.a_id);
    if (!track || m_held.erase(result.a_id) == 0)
    {
      return; // Merged or retired meanwhile.
    }
    if (!result.same_probability)
    {
      ++m_decision_errors;
      m_last_error = result.error;
      RCLCPP_WARN(m_logger,
                  "Label check for track %lu failed (%s); publishing with label '%s'",
                  result.a_id,
                  result.error.c_str(),
                  track->class_name.c_str());
      return;
    }
    ++m_decisions;
    const bool CORRECT = *result.same_probability >= params.arbiter.label_probability;
    RCLCPP_INFO(m_logger,
                "Label check for track %lu '%s': p=%.2f (object, not room: %.2f) -> %s",
                result.a_id,
                track->class_name.c_str(),
                *result.same_probability,
                result.object_probability.value_or(1.0),
                CORRECT ? "confirmed" : "rejected, removing");
    if (CORRECT)
    {
      track->setDecidedLabel(track->class_name);
    }
    else
    {
      retire(result.a_id);
      ++m_label_rejections;
    }
    if (!CORRECT && result.object_probability &&
        *result.object_probability < params.arbiter.label_probability &&
        m_rejected_labels.insert(track->class_name).second)
    {
      RCLCPP_WARN(m_logger,
                  "Label '%s' is not a physical object; ignoring it from now on",
                  track->class_name.c_str());
      for (const auto& other : m_tracks)
      {
        if (other->class_name == track->class_name && !m_removed.contains(other->track_id))
        {
          retire(other->track_id);
        }
      }
    }
    return;
  }
  const auto PAIR = pairOf(result.a_id, result.b_id);
  if (!result.same_probability)
  {
    m_failed.insert(PAIR);
    ++m_decision_errors;
    m_last_error = result.error;
    RCLCPP_WARN(m_logger,
                "Identity decision for tracks %lu/%lu failed: %s",
                result.a_id,
                result.b_id,
                result.error.c_str());
    return;
  }
  ++m_decisions;
  auto* a         = find(result.a_id);
  auto* b         = find(result.b_id);
  const bool SAME = *result.same_probability >= params.arbiter.same_object_probability;
  RCLCPP_INFO(m_logger,
              "Identity decision for %s tracks %lu/%lu: p(same)=%.2f label='%s' -> %s",
              result.stale_replacement ? "stale/recent" : "overlapping",
              result.a_id,
              result.b_id,
              *result.same_probability,
              result.label.c_str(),
              SAME ? "merge" : (result.stale_replacement ? "replace" : "keep both"));
  if (SAME)
  {
    if (!a || !b)
    {
      return;
    }
    auto labels = a->labels();
    labels.merge(b->labels());
    const auto SURVIVOR = merge(result.a_id, result.b_id);
    m_accepted_labels[SURVIVOR].merge(labels);
    if (!result.label.empty())
    {
      find(SURVIVOR)->setDecidedLabel(result.label);
    }
    return;
  }
  m_distinct.insert(PAIR);
  if (result.stale_replacement && a && isStale(*a, params))
  {
    retire(result.a_id); // Something else now occupies the stale track's pose.
    ++m_replacements;
  }
}

std::vector<ArbiterRequest> GlobalTracker::reconcilePairs(const object_tracker::Params& params,
                                                          bool arbiter_enabled)
{
  std::vector<ArbiterRequest> requests;
  const double MIN_EXTENT = params.overlap_min_extent;
  for (size_t i = 0; i < m_tracks.size(); ++i)
  {
    for (size_t j = i + 1; j < m_tracks.size(); ++j)
    {
      auto& a = *m_tracks[i];
      auto& b = *m_tracks[j];
      if (m_removed.contains(a.track_id))
      {
        break;
      }
      const auto PAIR = pairOf(a.track_id, b.track_id);
      if (m_removed.contains(b.track_id) || m_distinct.contains(PAIR))
      {
        continue;
      }
      const bool STALE_A = isStale(a, params);
      const bool STALE_B = isStale(b, params);
      if (!TrackedObject::sizesCompatible(
            a.bbox3d_size, b.bbox3d_size, params.association_max_size_ratio))
      {
        // Something of a different size now occupies a stale track's pose: replace it.
        if (STALE_A != STALE_B)
        {
          auto& stale  = STALE_A ? a : b;
          auto& recent = STALE_A ? b : a;
          if (TrackedObject::coveredFraction(recent.getPosition(),
                                             recent.bbox3d_size,
                                             stale.getPosition(),
                                             stale.bbox3d_size,
                                             MIN_EXTENT) >= params.replacement_overlap_threshold)
          {
            RCLCPP_INFO(m_logger,
                        "Replacing stale track %lu with track %lu",
                        stale.track_id,
                        recent.track_id);
            retire(stale.track_id);
            ++m_replacements;
          }
        }
        continue;
      }
      const double OVERLAP = TrackedObject::overlapOfSmaller(a, b, MIN_EXTENT);
      if (OVERLAP <= params.arbiter.min_overlap)
      {
        continue;
      }
      const bool SAME_LABEL = labelAccepted(a, b.class_name) || labelAccepted(b, a.class_name);
      const bool CLEAR      = OVERLAP >= params.merge_overlap_threshold;
      const bool FALLBACK   = !arbiter_enabled || m_failed.contains(PAIR);
      if (CLEAR && (SAME_LABEL || FALLBACK))
      {
        merge(a.track_id, b.track_id); // Older identity survives; a stale one is reactivated.
        continue;
      }
      if (FALLBACK || !m_asked.insert(PAIR).second)
      {
        continue;
      }
      // Label conflict or partial overlap: the arbiter decides. A stale track goes first.
      const bool SWAP    = STALE_B && !STALE_A;
      const auto& first  = SWAP ? b : a;
      const auto& second = SWAP ? a : b;
      RCLCPP_INFO(m_logger,
                  "Queueing identity decision for tracks %lu ('%s') / %lu ('%s'), overlap %.2f",
                  first.track_id,
                  first.class_name.c_str(),
                  second.track_id,
                  second.class_name.c_str(),
                  OVERLAP);
      requests.push_back(ArbiterRequest{
        .stale_replacement = STALE_A != STALE_B,
        .a_id              = first.track_id,
        .b_id              = second.track_id,
        .a_label           = first.class_name,
        .b_label           = second.class_name,
        .a_crops = std::make_shared<const std::vector<sensor_msgs::msg::Image>>(first.getCrops()),
        .b_crops = std::make_shared<const std::vector<sensor_msgs::msg::Image>>(second.getCrops()),
        .a_size  = first.bbox3d_size,
        .b_size  = second.bbox3d_size,
        .center_distance = (first.getPosition() - second.getPosition()).norm()});
    }
  }
  return requests;
}

std::vector<ArbiterRequest> GlobalTracker::cycle(const std::vector<Tracklet>& tracklets,
                                                 const std::vector<ArbiterResult>& results,
                                                 const object_tracker::Params& params,
                                                 bool arbiter_enabled)
{
  for (const auto& tracklet : tracklets)
  {
    if (!m_latest || tracklet.last_seen > *m_latest)
    {
      m_latest = tracklet.last_seen;
    }
  }
  for (const auto& result : results)
  {
    applyResult(result, params);
  }
  eraseRemoved();

  std::set<LinkKey> alive;
  std::vector<ArbiterRequest> label_checks;
  for (const auto& tracklet : tracklets)
  {
    if (m_rejected_labels.contains(tracklet.class_name))
    {
      continue; // Its link (if any) is dropped below.
    }
    const LinkKey KEY{tracklet.camera, tracklet.local_id};
    alive.insert(KEY);
    if (const auto IT = m_links.find(KEY); IT != m_links.end())
    {
      if (auto* track = find(IT->second.global_id))
      {
        if (tracklet.last_seen > IT->second.last_applied)
        {
          updateFrom(*track, tracklet, params);
          IT->second.last_applied = tracklet.last_seen;
        }
        continue;
      }
      m_links.erase(IT);
    }
    // Unlinked: join a clearly overlapping, similar-sized track with an accepted label
    // (another camera's view, a revisit, or a stale track being reactivated), else start one.
    TrackedObject* best = nullptr;
    double best_overlap = 0.0;
    for (const auto& track : m_tracks)
    {
      if (!TrackedObject::sizesCompatible(
            tracklet.size, track->bbox3d_size, params.association_max_size_ratio) ||
          !labelAccepted(*track, tracklet.class_name))
      {
        continue;
      }
      const double OVERLAP = std::max(TrackedObject::coveredFraction(track->getPosition(),
                                                                     track->bbox3d_size,
                                                                     tracklet.position,
                                                                     tracklet.size,
                                                                     params.overlap_min_extent),
                                      TrackedObject::coveredFraction(tracklet.position,
                                                                     tracklet.size,
                                                                     track->getPosition(),
                                                                     track->bbox3d_size,
                                                                     params.overlap_min_extent));
      if (OVERLAP >= params.merge_overlap_threshold && OVERLAP > best_overlap)
      {
        best_overlap = OVERLAP;
        best         = track.get();
      }
    }
    if (best)
    {
      updateFrom(*best, tracklet, params);
      m_links[KEY] = Link{.global_id = best->track_id, .last_applied = tracklet.last_seen};
    }
    else
    {
      if (auto check = createFrom(tracklet, params, arbiter_enabled))
      {
        label_checks.push_back(std::move(*check));
      }
      m_links[KEY] =
        Link{.global_id = m_tracks.back()->track_id, .last_applied = tracklet.last_seen};
    }
  }
  std::erase_if(m_links, [&alive](const auto& entry) { return !alive.contains(entry.first); });

  auto requests = std::move(label_checks);
  auto pairs    = reconcilePairs(params, arbiter_enabled);
  requests.insert(requests.end(), pairs.begin(), pairs.end());
  eraseRemoved();
  return requests;
}

GlobalTracker::Stats GlobalTracker::stats(const object_tracker::Params& params) const
{
  Stats stats{.tracks           = m_tracks.size(),
              .stale            = 0,
              .links            = m_links.size(),
              .decisions        = m_decisions,
              .merges           = m_merges,
              .replacements     = m_replacements,
              .held             = m_held.size(),
              .label_rejections = m_label_rejections,
              .decision_errors  = m_decision_errors,
              .last_error       = m_last_error};
  stats.stale = static_cast<size_t>(
    std::ranges::count_if(m_tracks, [&](const auto& track) { return isStale(*track, params); }));
  return stats;
}

} // namespace semantic_scene_forests_object_tracker
