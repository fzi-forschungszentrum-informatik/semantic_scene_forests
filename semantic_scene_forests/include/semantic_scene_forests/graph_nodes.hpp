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
#include <ostream>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

#include <fmt/ostream.h>

#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection3_d.hpp>

namespace fzi::semantic_scene_forests {

using ImageMsg       = sensor_msgs::msg::Image;
using Detection3DMsg = vision_msgs::msg::Detection3D;

struct ConceptNode
{
  int64_t id;
  std::string concept_name;

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
  int64_t id;
  std::string class_name;
  vision_msgs::msg::Detection3D::SharedPtr detection;
  std::array<sensor_msgs::msg::Image::SharedPtr, 5> visuals;

  std::strong_ordering operator<=>(const ObjectNode& other) const { return id <=> other.id; }
  bool operator==(const ObjectNode& other) const { return id == other.id; }
};

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
