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
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/time.hpp>
#include <semantic_scene_forests_object_msgs/msg/managed_object.hpp>
#include <semantic_scene_forests_object_msgs/msg/object_candidate.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection3_d.hpp>

namespace semantic_scene_forests_object_manager {

struct ManagedObject
{
  int64_t id{0};
  std::string class_name;
  vision_msgs::msg::Detection3D detection;
  std::vector<sensor_msgs::msg::Image> image_crops;
  bool is_blocked{false};
  uint8_t source{0};
  rclcpp::Time last_updated;
  builtin_interfaces::msg::Time last_seen;
  bool class_overridden{false};
  bool pose_overridden{false};
  bool crops_curated{false};

  static ManagedObject
  fromCandidate(const semantic_scene_forests_object_msgs::msg::ObjectCandidate& candidate,
                const rclcpp::Time& now);

  void
  updateFromCandidate(const semantic_scene_forests_object_msgs::msg::ObjectCandidate& candidate,
                      const rclcpp::Time& now);

  [[nodiscard]] semantic_scene_forests_object_msgs::msg::ManagedObject
  toMessage(const std::string& frame_id) const;
};

} // namespace semantic_scene_forests_object_manager
