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

#include <compare>
#include <cstdint>
#include <map>
#include <memory>
#include <ostream>
#include <string>
#include <variant>
#include <vector>

#include <fmt/ostream.h>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>

namespace fzi::semantic_scene_forests {

//! A JPEG crop of an object, stamped in header.stamp. Shared, so node copies are cheap.
using CropPtr = std::shared_ptr<const sensor_msgs::msg::CompressedImage>;

//! Per-worker state on a node. Not published; new worker-only fields go here.
struct WorkerState
{
  bool processed{false};
};

// NOLINTBEGIN(readability-redundant-member-init): the `{}` initializers are required, removing them
// breaks the build
struct ConceptNode
{
  int64_t id{0};
  std::string concept_name{};
  std::map<std::string, WorkerState> workers{};

  std::strong_ordering operator<=>(const ConceptNode& other) const { return id <=> other.id; }
  bool operator==(const ConceptNode& other) const { return id == other.id; }
};

inline std::ostream& operator<<(std::ostream& os, const ConceptNode& n)
{
  fmt::print(os, "Concept (id: {}, name: \'{}\')", n.id, n.concept_name);
  return os;
}

struct ObjectNode
{
  //! Values of `source`: the origin of the track (informational only).
  static constexpr uint8_t SOURCE_CAMERA = 0;
  static constexpr uint8_t SOURCE_MANUAL = 1;

  int64_t id{0};
  std::string class_name{};
  vision_msgs::msg::BoundingBox3D bbox{};
  std::string frame_id{}; //!< Frame of bbox.
  uint8_t source{SOURCE_CAMERA};
  builtin_interfaces::msg::Time last_seen{};
  builtin_interfaces::msg::Time last_updated{}; //!< Observation time of the last bbox change.
  std::vector<CropPtr> crops{};
  std::map<std::string, WorkerState> workers{};

  std::strong_ordering operator<=>(const ObjectNode& other) const { return id <=> other.id; }
  bool operator==(const ObjectNode& other) const { return id == other.id; }
};
// NOLINTEND(readability-redundant-member-init)

inline std::ostream& operator<<(std::ostream& os, const ObjectNode& n)
{
  fmt::print(os, "Object (id: {}, name: \'{}\')", n.id, n.class_name);
  return os;
}

using SceneGraphNode = std::variant<ConceptNode, ObjectNode>;

inline std::ostream& operator<<(std::ostream& os, const SceneGraphNode& node)
{
  std::visit([&os](const auto& n) { os << n; }, node);
  return os;
}

}; // namespace fzi::semantic_scene_forests
