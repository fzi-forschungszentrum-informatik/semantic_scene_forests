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
#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>

#include "semantic_scene_forests_object_tracker/tracked_object.hpp"

namespace semantic_scene_forests_object_tracker {
namespace detail {

// Transforms a bounding box from its source frame into the target frame of
// `transform`: world center = R_tf * center + translation, world extents from
// the composed rotation R_tf * R_box, and an identity orientation because the
// resulting extents are world-axis-aligned. Returns nullopt when the geometry
// is non-finite, degenerate (nonpositive sizes) or either quaternion has a
// zero/nonfinite norm.
std::optional<vision_msgs::msg::BoundingBox3D>
transformBoundingBoxToFrame(const vision_msgs::msg::BoundingBox3D& bbox,
                            const geometry_msgs::msg::TransformStamped& transform);

using TrackPair = std::pair<uint64_t, uint64_t>; // {lower id, higher id}

// First pair of eligible tracks that geometry cannot settle: similar sizes and an overlap of
// the smaller box in (min_overlap, max_overlap) (clear overlaps merge, the rest stay separate).
// Pairs in `decided` were already arbitrated and are never returned again.
std::optional<std::pair<size_t, size_t>>
findAmbiguousPair(const std::vector<std::unique_ptr<TrackedObject>>& tracks,
                  const std::function<bool(const TrackedObject&)>& eligible,
                  double min_overlap,
                  double max_overlap,
                  double max_size_ratio,
                  double min_extent,
                  const std::set<TrackPair>& decided);

} // namespace detail
} // namespace semantic_scene_forests_object_tracker
