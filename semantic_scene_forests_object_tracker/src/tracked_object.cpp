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

#include "semantic_scene_forests_object_tracker/tracked_object.hpp"

#include <algorithm>
#include <cmath>

#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

namespace semantic_scene_forests_object_tracker {

TrackedObject::TrackedObject(const DetectionAdapter& initial_detection,
                             double initial_time,
                             uint64_t track_id,
                             double measurement_std,
                             double process_noise_std,
                             int confirmation_threshold,
                             size_t max_crops)
  : track_id(track_id)
  , class_id(initial_detection.class_id)
  , class_name(initial_detection.class_name)
  , bbox3d_frame_id(initial_detection.frame_id)
  , score(initial_detection.score)
  , confirmed(false)
  , observation_count(1)
  , last_seen(initial_time)
  , last_detection(initial_detection)
  , m_process_noise_std(process_noise_std)
  , m_confirmation_threshold(confirmation_threshold)
  , m_max_crops(max_crops)
{
  bbox3d_size = Eigen::Vector3d(initial_detection.bbox3d.size.x,
                                initial_detection.bbox3d.size.y,
                                initial_detection.bbox3d.size.z);

  m_position = Eigen::Vector3d(initial_detection.bbox3d.center.position.x,
                               initial_detection.bbox3d.center.position.y,
                               initial_detection.bbox3d.center.position.z);

  m_covariance        = Eigen::Matrix3d::Identity();
  m_measurement_noise = Eigen::Matrix3d::Identity() * (measurement_std * measurement_std);

  m_class_history[class_name] = 1;
  m_class_id_map[class_name]  = class_id;
}

void TrackedObject::predict(double dt)
{
  double q = m_process_noise_std * dt;
  m_covariance += Eigen::Matrix3d::Identity() * (q * q);
}

void TrackedObject::update(const DetectionAdapter& detection, double current_time)
{
  Eigen::Vector3d z(detection.bbox3d.center.position.x,
                    detection.bbox3d.center.position.y,
                    detection.bbox3d.center.position.z);

  Eigen::Vector3d innovation = z - m_position;
  Eigen::Matrix3d s          = m_covariance + m_measurement_noise;
  Eigen::Matrix3d k          = m_covariance * s.inverse();

  m_position += k * innovation;
  m_covariance = (Eigen::Matrix3d::Identity() - k) * m_covariance;

  m_class_history[detection.class_name]++;
  m_class_id_map[detection.class_name] = detection.class_id;

  int max_count = 0;
  for (const auto& [name, count] : m_class_history)
  {
    if (count > max_count)
    {
      max_count  = count;
      class_name = name;
    }
  }
  class_id = m_class_id_map[class_name];

  Eigen::Vector3d cur_pos = getPosition();
  Eigen::Vector3d cur_min = cur_pos - bbox3d_size / 2.0;
  Eigen::Vector3d cur_max = cur_pos + bbox3d_size / 2.0;

  Eigen::Vector3d det_size(
    detection.bbox3d.size.x, detection.bbox3d.size.y, detection.bbox3d.size.z);
  Eigen::Vector3d det_min = z - det_size / 2.0;
  Eigen::Vector3d det_max = z + det_size / 2.0;

  Eigen::Vector3d union_min  = cur_min.cwiseMin(det_min);
  Eigen::Vector3d union_max  = cur_max.cwiseMax(det_max);
  Eigen::Vector3d union_size = union_max - union_min;

  double alpha = 1.0 / (observation_count + 1);
  bbox3d_size  = (1.0 - alpha) * bbox3d_size + alpha * union_size;
  score        = detection.score;

  last_detection = detection;
  last_seen      = current_time;
  observation_count++;

  if (observation_count >= m_confirmation_threshold)
  {
    confirmed = true;
  }
}

Eigen::Vector3d TrackedObject::getPosition() const
{
  return m_position;
}

Eigen::Matrix3d TrackedObject::getPositionCovariance() const
{
  return m_covariance;
}

double TrackedObject::mahalanobisDistance(const DetectionAdapter& detection)
{
  Eigen::Vector3d z(detection.bbox3d.center.position.x,
                    detection.bbox3d.center.position.y,
                    detection.bbox3d.center.position.z);

  Eigen::Vector3d innovation = z - m_position;
  Eigen::Matrix3d s          = m_covariance + m_measurement_noise;

  Eigen::LLT<Eigen::Matrix3d> llt(s);
  if (llt.info() != Eigen::Success)
  {
    return innovation.norm();
  }

  Eigen::Vector3d solved = llt.solve(innovation);
  double dist_sq         = innovation.dot(solved);
  return std::sqrt(std::max(0.0, dist_sq));
}

double TrackedObject::bbox3dOverlap(const TrackedObject& a, const TrackedObject& b)
{
  Eigen::Vector3d a_pos = a.getPosition();
  Eigen::Vector3d b_pos = b.getPosition();

  Eigen::Vector3d a_min = a_pos - a.bbox3d_size / 2.0;
  Eigen::Vector3d a_max = a_pos + a.bbox3d_size / 2.0;
  Eigen::Vector3d b_min = b_pos - b.bbox3d_size / 2.0;
  Eigen::Vector3d b_max = b_pos + b.bbox3d_size / 2.0;

  Eigen::Vector3d inter_min  = a_min.cwiseMax(b_min);
  Eigen::Vector3d inter_max  = a_max.cwiseMin(b_max);
  Eigen::Vector3d inter_size = (inter_max - inter_min).cwiseMax(0.0);
  double inter_vol           = inter_size.x() * inter_size.y() * inter_size.z();

  double a_vol     = a.bbox3d_size.x() * a.bbox3d_size.y() * a.bbox3d_size.z();
  double b_vol     = b.bbox3d_size.x() * b.bbox3d_size.y() * b.bbox3d_size.z();
  double union_vol = a_vol + b_vol - inter_vol;

  if (union_vol <= 0.0)
  {
    return 0.0;
  }
  return inter_vol / union_vol;
}

void TrackedObject::mergeFrom(const TrackedObject& other)
{
  for (const auto& [name, count] : other.m_class_history)
  {
    m_class_history[name] += count;
    if (!m_class_id_map.contains(name))
    {
      m_class_id_map[name] = other.m_class_id_map.at(name);
    }
  }

  for (const auto& crop : other.getCrops())
  {
    addCrop(crop);
  }

  observation_count = std::max(observation_count, other.observation_count);

  int max_count = 0;
  for (const auto& [name, count] : m_class_history)
  {
    if (count > max_count)
    {
      max_count  = count;
      class_name = name;
    }
  }
  class_id = m_class_id_map[class_name];
}

vision_msgs::msg::Detection3D TrackedObject::toDetection3D() const
{
  vision_msgs::msg::Detection3D det;

  det.header.frame_id = bbox3d_frame_id;
  det.id              = std::to_string(track_id);

  det.bbox.center.position.x  = m_position.x();
  det.bbox.center.position.y  = m_position.y();
  det.bbox.center.position.z  = m_position.z();
  det.bbox.center.orientation = last_detection.bbox3d.center.orientation;
  det.bbox.size.x             = bbox3d_size.x();
  det.bbox.size.y             = bbox3d_size.y();
  det.bbox.size.z             = bbox3d_size.z();

  vision_msgs::msg::ObjectHypothesisWithPose hyp;
  hyp.hypothesis.class_id = class_name;
  hyp.hypothesis.score    = score;
  det.results.push_back(hyp);

  return det;
}

void TrackedObject::addCrop(const sensor_msgs::msg::Image& crop)
{
  if (m_crops.size() >= m_max_crops)
  {
    m_crops.pop_front();
  }
  m_crops.push_back(crop);
}

std::vector<sensor_msgs::msg::Image> TrackedObject::getCrops() const
{
  return {m_crops.begin(), m_crops.end()};
}

} // namespace semantic_scene_forests_object_tracker
