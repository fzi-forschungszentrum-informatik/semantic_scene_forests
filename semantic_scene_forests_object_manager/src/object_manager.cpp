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
#include <ranges>

namespace semantic_scene_forests_object_manager {

void ObjectManager::processObjectSnapshot(
  const semantic_scene_forests_object_msgs::msg::ObjectCandidateArray& snapshot,
  const rclcpp::Time& now)
{
  const std::scoped_lock LOCK{m_objects_mutex};
  std::unordered_set<int64_t> present_ids;
  present_ids.reserve(snapshot.objects.size());
  for (const auto& candidate : snapshot.objects)
  {
    present_ids.insert(candidate.id);
    if (!m_blocked_ids.contains(candidate.id))
    {
      auto it = m_objects.find(candidate.id);
      if (it == m_objects.end())
      {
        m_objects.emplace(candidate.id, ManagedObject::fromCandidate(candidate, now));
      }
      else
      {
        it->second.updateFromCandidate(candidate, now);
      }
    }
  }
  // Retiring an ID also retires all its overrides and curated visuals. Blocking
  // IDs are separate and survive even an empty authoritative snapshot.
  std::erase_if(m_objects,
                [&present_ids](const auto& entry) { return !present_ids.contains(entry.first); });
}

bool ObjectManager::blockObject(int64_t id)
{
  const std::scoped_lock LOCK{m_objects_mutex};
  if (m_blocked_ids.contains(id))
  {
    return true;
  }
  if (m_objects.erase(id) == 0U)
  {
    return false;
  }
  m_blocked_ids.insert(id);
  return true;
}

bool ObjectManager::unblockObject(int64_t id)
{
  const std::scoped_lock LOCK{m_objects_mutex};
  return m_blocked_ids.erase(id) > 0;
}

bool ObjectManager::editObject(
  const semantic_scene_forests_object_msgs::srv::EditObject::Request& request,
  const rclcpp::Time& now)
{
  EditFailure failure;
  return editObject(request, now, failure);
}

bool ObjectManager::editObject(
  const semantic_scene_forests_object_msgs::srv::EditObject::Request& request,
  const rclcpp::Time& now,
  EditFailure& failure)
{
  const std::scoped_lock LOCK{m_objects_mutex};
  auto it = m_objects.find(request.object_id);
  if (it == m_objects.end())
  {
    failure = EditFailure::UNKNOWN_OBJECT;
    return false;
  }

  ManagedObject& obj = it->second;

  auto indices = request.remove_crop_indices;
  std::ranges::sort(indices);
  const auto REMOVED = std::ranges::unique(indices);
  indices.erase(REMOVED.begin(), REMOVED.end());
  for (const uint32_t IDX : indices)
  {
    if (static_cast<size_t>(IDX) >= obj.image_crops.size())
    {
      failure = EditFailure::INVALID_CROP_INDEX;
      return false;
    }
  }

  bool changed = false;
  if (request.update_class_name)
  {
    changed              = !obj.class_overridden || obj.class_name != request.class_name;
    obj.class_name       = request.class_name;
    obj.class_overridden = true;
    for (auto& result : obj.detection.results)
    {
      result.hypothesis.class_id = obj.class_name;
    }
  }

  if (request.update_pose)
  {
    changed = changed || !obj.pose_overridden || obj.detection.bbox.center != request.pose;
    obj.detection.bbox.center = request.pose;
    obj.pose_overridden       = true;
  }

  for (const uint32_t IDX : std::views::reverse(indices))
  {
    obj.image_crops.erase(obj.image_crops.begin() + static_cast<std::ptrdiff_t>(IDX));
  }

  if (!indices.empty())
  {
    obj.crops_curated = true;
    changed           = true;
  }
  if (changed)
  {
    obj.source       = 1;
    obj.last_updated = now;
  }
  failure = EditFailure::SUCCESS;
  return true;
}

std::vector<semantic_scene_forests_object_msgs::msg::ManagedObject>
ObjectManager::getManagedObjects(const std::string& frame_id) const
{
  const std::scoped_lock LOCK{m_objects_mutex};
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
  const std::scoped_lock LOCK{m_objects_mutex};
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

size_t ObjectManager::getBlockedObjectCount() const
{
  const std::scoped_lock LOCK{m_objects_mutex};
  return m_blocked_ids.size();
}

} // namespace semantic_scene_forests_object_manager
