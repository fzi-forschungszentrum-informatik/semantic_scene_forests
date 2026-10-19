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

#include "semantic_scene_forests_object_manager/managed_object.hpp"

namespace semantic_scene_forests_object_manager {

ManagedObject ManagedObject::fromCandidate(
  const semantic_scene_forests_object_msgs::msg::ObjectCandidate& candidate,
  const rclcpp::Time& now)
{
  ManagedObject obj;
  obj.id           = candidate.id;
  obj.class_name   = candidate.class_name;
  obj.detection    = candidate.detection;
  obj.image_crops  = candidate.image_crops;
  obj.is_blocked   = false;
  obj.source       = 0;
  obj.last_updated = now;
  return obj;
}

semantic_scene_forests_object_msgs::msg::ManagedObject
ManagedObject::toMessage(const std::string& frame_id) const
{
  semantic_scene_forests_object_msgs::msg::ManagedObject msg;
  msg.id              = id;
  msg.class_name      = class_name;
  msg.detection       = detection;
  msg.is_blocked      = is_blocked;
  msg.source          = source;
  msg.last_updated    = last_updated;
  msg.header.stamp    = last_updated;
  msg.header.frame_id = frame_id;
  return msg;
}

} // namespace semantic_scene_forests_object_manager
