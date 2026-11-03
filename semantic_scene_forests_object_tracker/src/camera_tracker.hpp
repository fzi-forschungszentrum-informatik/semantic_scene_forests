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
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "semantic_scene_forests_object_tracker/tracked_object.hpp"
#include <semantic_scene_forests_object_tracker/object_tracker_parameters.hpp>

namespace semantic_scene_forests_object_tracker {

// A confirmed per-camera track as handed to the global tracker.
struct Tracklet
{
  std::string camera;
  uint64_t local_id{0};
  rclcpp::Time last_seen;
  Eigen::Vector3d position;
  Eigen::Vector3d size;
  std::string class_name;
  double score{0.0};
  std::shared_ptr<const std::vector<sensor_msgs::msg::Image>> crops;
};

// Tracks the detections of one camera in the world frame. Not thread-safe: one camera's
// callback group owns it. Tracks end local_track_timeout after their last observation;
// identity across revisits lives in the GlobalTracker.
class CameraTracker
{
public:
  using TransformLookup = std::function<std::optional<geometry_msgs::msg::TransformStamped>()>;
  enum class FrameResult : std::uint8_t
  {
    ACCEPTED,
    OUT_OF_ORDER, // stamp not newer than the last accepted frame of this camera
    BAD_IMAGE,    // image not convertible to bgr8
    NO_TRANSFORM, // a detection needed TF to the world frame, lookup failed
  };

  CameraTracker(std::string camera, int rotation_cw, rclcpp::Logger logger);

  // One exactly synchronized frame. Rejected frames leave the tracker unchanged.
  FrameResult process(const vision_msgs::msg::Detection2DArray& det2d_msg,
                      const vision_msgs::msg::Detection3DArray& det3d_msg,
                      const sensor_msgs::msg::Image& image_msg,
                      const TransformLookup& world_transform,
                      const object_tracker::Params& params);

  std::vector<Tracklet> tracklets() const;
  size_t trackCount() const { return m_tracks.size(); }
  size_t cropErrors() const { return m_crop_errors; }

private:
  std::string m_camera;
  int m_rotation;
  rclcpp::Logger m_logger;
  std::vector<std::unique_ptr<TrackedObject>> m_tracks;
  std::optional<rclcpp::Time> m_last_time;
  uint64_t m_next_id{1};
  size_t m_crop_errors{0};
};

} // namespace semantic_scene_forests_object_tracker
