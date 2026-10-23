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
#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection3_d.hpp>

namespace semantic_scene_forests_object_tracker {

struct DetectionAdapter
{
  std::string class_name;
  int class_id;
  double score;
  vision_msgs::msg::BoundingBox3D bbox3d;
  vision_msgs::msg::BoundingBox2D bbox2d;
  std::string frame_id;
};

class TrackedObject
{
public:
  TrackedObject(const DetectionAdapter& initial_detection,
                double initial_time,
                uint64_t track_id,
                double measurement_std,
                double process_noise_std,
                int confirmation_threshold,
                size_t max_crops);

  void predict(double dt);
  void update(const DetectionAdapter& detection, double current_time);

  Eigen::Vector3d getPosition() const;
  Eigen::Matrix3d getPositionCovariance() const;
  double mahalanobisDistance(const DetectionAdapter& detection);

  static double bbox3dOverlap(const TrackedObject& a, const TrackedObject& b);
  void mergeFrom(const TrackedObject& other);

  vision_msgs::msg::Detection3D toDetection3D() const;

  void addCrop(const sensor_msgs::msg::Image& crop);
  std::vector<sensor_msgs::msg::Image> getCrops() const;

  uint64_t track_id;
  int class_id;
  std::string class_name;
  std::string bbox3d_frame_id;
  Eigen::Vector3d bbox3d_size;
  double score;
  bool confirmed;
  bool published{false};
  int observation_count;
  double last_seen;
  double time_since_update{0.0};
  DetectionAdapter last_detection;

private:
  Eigen::Vector3d m_position;
  Eigen::Matrix3d m_covariance;
  Eigen::Matrix3d m_measurement_noise;
  double m_process_noise_std;
  int m_confirmation_threshold;

  std::unordered_map<std::string, int> m_class_history;
  std::unordered_map<std::string, int> m_class_id_map;

  std::deque<sensor_msgs::msg::Image> m_crops;
  size_t m_max_crops;
};

} // namespace semantic_scene_forests_object_tracker
