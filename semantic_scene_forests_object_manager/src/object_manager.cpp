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

#include "semantic_scene_forests_object_manager/object_manager.hpp"

#include <algorithm>

namespace semantic_scene_forests_object_manager {

bool ObjectManager::processObjectCandidate(
  const semantic_scene_forests_object_msgs::msg::ObjectCandidate& candidate,
  const rclcpp::Time& now)
{
  if (m_blocked_ids.contains(candidate.id))
  {
    return false;
  }
  m_objects[candidate.id] = ManagedObject::fromCandidate(candidate, now);
  return true;
}

bool ObjectManager::blockObject(int64_t id)
{
  auto it = m_objects.find(id);
  if (it == m_objects.end())
  {
    return false;
  }
  m_objects.erase(it);
  m_blocked_ids.insert(id);
  return true;
}

bool ObjectManager::unblockObject(int64_t id)
{
  m_blocked_ids.erase(id);
  return true;
}

bool ObjectManager::editObject(
  const semantic_scene_forests_object_msgs::srv::EditObject::Request& request,
  const rclcpp::Time& now)
{
  auto it = m_objects.find(request.object_id);
  if (it == m_objects.end())
  {
    return false;
  }

  ManagedObject& obj = it->second;

  if (request.update_class_name)
  {
    obj.class_name = request.class_name;
  }

  if (request.update_pose)
  {
    obj.detection.bbox.center.position    = request.pose.position;
    obj.detection.bbox.center.orientation = request.pose.orientation;
  }

  auto indices = request.remove_crop_indices;
  std::ranges::sort(indices, std::greater<uint32_t>());
  for (const uint32_t IDX : indices)
  {
    if (static_cast<size_t>(IDX) < obj.image_crops.size())
    {
      obj.image_crops.erase(obj.image_crops.begin() + static_cast<std::ptrdiff_t>(IDX));
    }
  }

  obj.source       = 1;
  obj.last_updated = now;
  return true;
}

std::vector<semantic_scene_forests_object_msgs::msg::ManagedObject>
ObjectManager::getManagedObjects(const std::string& frame_id) const
{
  std::vector<semantic_scene_forests_object_msgs::msg::ManagedObject> result;
  result.reserve(m_objects.size());
  for (const auto& [id, obj] : m_objects)
  {
    result.push_back(obj.toMessage(frame_id));
  }
  return result;
}

std::vector<semantic_scene_forests_object_msgs::msg::ObjectVisuals>
ObjectManager::getObjectVisuals(const std::vector<int64_t>& ids) const
{
  std::vector<semantic_scene_forests_object_msgs::msg::ObjectVisuals> result;
  const auto LOOKUP = [&](int64_t id) {
    auto it = m_objects.find(id);
    if (it == m_objects.end())
    {
      return;
    }
    semantic_scene_forests_object_msgs::msg::ObjectVisuals visuals;
    visuals.object_id   = id;
    visuals.image_crops = it->second.image_crops;
    result.push_back(std::move(visuals));
  };

  if (ids.empty())
  {
    for (const auto& [id, _] : m_objects)
    {
      LOOKUP(id);
    }
  }
  else
  {
    for (const int64_t ID : ids)
    {
      LOOKUP(ID);
    }
  }
  return result;
}

semantic_scene_forests_object_msgs::msg::ManagedObjectArray
ObjectManager::buildMessage(const std::string& frame_id, const rclcpp::Time& now) const
{
  semantic_scene_forests_object_msgs::msg::ManagedObjectArray msg;
  msg.header.stamp    = now;
  msg.header.frame_id = frame_id;
  msg.objects         = getManagedObjects(frame_id);
  return msg;
}

} // namespace semantic_scene_forests_object_manager
