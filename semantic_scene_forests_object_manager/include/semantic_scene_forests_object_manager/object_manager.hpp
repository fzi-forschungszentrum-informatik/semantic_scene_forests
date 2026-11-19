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
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/time.hpp>
#include <semantic_scene_forests_object_msgs/msg/managed_object.hpp>
#include <semantic_scene_forests_object_msgs/msg/managed_object_array.hpp>
#include <semantic_scene_forests_object_msgs/msg/object_candidate_array.hpp>
#include <semantic_scene_forests_object_msgs/msg/object_visuals.hpp>
#include <semantic_scene_forests_object_msgs/srv/edit_object.hpp>

#include "semantic_scene_forests_object_manager/managed_object.hpp"

namespace semantic_scene_forests_object_manager {

class ObjectManager
{
public:
  enum class EditFailure
  {
    SUCCESS,
    UNKNOWN_OBJECT,
    INVALID_CROP_INDEX
  };

  void processObjectSnapshot(
    const semantic_scene_forests_object_msgs::msg::ObjectCandidateArray& snapshot,
    const rclcpp::Time& now);

  bool blockObject(int64_t id);

  bool unblockObject(int64_t id);

  bool editObject(const semantic_scene_forests_object_msgs::srv::EditObject::Request& request,
                  const rclcpp::Time& now);

  bool editObject(const semantic_scene_forests_object_msgs::srv::EditObject::Request& request,
                  const rclcpp::Time& now,
                  EditFailure& failure);

  [[nodiscard]] std::vector<semantic_scene_forests_object_msgs::msg::ManagedObject>
  getManagedObjects(const std::string& frame_id) const;

  [[nodiscard]] std::vector<semantic_scene_forests_object_msgs::msg::ObjectVisuals>
  getObjectVisuals(const std::vector<int64_t>& ids) const;

  [[nodiscard]] semantic_scene_forests_object_msgs::msg::ManagedObjectArray
  buildMessage(const std::string& frame_id, const rclcpp::Time& now) const;

  [[nodiscard]] size_t getBlockedObjectCount() const;

private:
  mutable std::mutex m_objects_mutex;
  std::unordered_map<int64_t, ManagedObject> m_objects;
  std::unordered_set<int64_t> m_blocked_ids;
};

} // namespace semantic_scene_forests_object_manager
