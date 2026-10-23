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

#include "camera_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <utility>

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/logging.hpp>

#include "image_rotation.hpp"
#include "tracker_geometry.hpp"

namespace semantic_scene_forests_object_tracker {

namespace {

struct DetectionPair
{
  const vision_msgs::msg::Detection2D* det2d = nullptr;
  const vision_msgs::msg::Detection3D* det3d = nullptr;
};

// Pairs 2D and 3D detections of one frame by detector ID (a per-frame key only).
std::map<std::string, DetectionPair> pairById(const vision_msgs::msg::Detection2DArray& det2d_msg,
                                              const vision_msgs::msg::Detection3DArray& det3d_msg)
{
  std::map<std::string, DetectionPair> pairs;
  for (const auto& det2d : det2d_msg.detections)
  {
    if (!det2d.id.empty())
    {
      pairs[det2d.id].det2d = &det2d;
    }
  }
  for (const auto& det3d : det3d_msg.detections)
  {
    if (!det3d.id.empty() && pairs.contains(det3d.id))
    {
      pairs[det3d.id].det3d = &det3d;
    }
  }
  return pairs;
}

bool has3dBox(const vision_msgs::msg::Detection3D& det)
{
  const auto& b = det.bbox;
  return b.center.position.x != 0.0 || b.center.position.y != 0.0 || b.center.position.z != 0.0 ||
         b.size.x != 0.0 || b.size.y != 0.0 || b.size.z != 0.0;
}

Eigen::Vector3d center(const DetectionAdapter& det)
{
  const auto& p = det.bbox3d.center.position;
  return {p.x, p.y, p.z};
}

Eigen::Vector3d extent(const DetectionAdapter& det)
{
  const auto& s = det.bbox3d.size;
  return {s.x, s.y, s.z};
}

std::optional<sensor_msgs::msg::Image>
cropImagePatch(const cv::Mat& image, const vision_msgs::msg::BoundingBox2D& bbox, int padding)
{
  const double CX = bbox.center.position.x;
  const double CY = bbox.center.position.y;
  const int X1    = std::max(0, static_cast<int>(std::round(CX - (bbox.size_x / 2.0))) - padding);
  const int Y1    = std::max(0, static_cast<int>(std::round(CY - (bbox.size_y / 2.0))) - padding);
  const int X2 =
    std::min(image.cols, static_cast<int>(std::round(CX + (bbox.size_x / 2.0))) + padding);
  const int Y2 =
    std::min(image.rows, static_cast<int>(std::round(CY + (bbox.size_y / 2.0))) + padding);
  if (X2 <= X1 || Y2 <= Y1)
  {
    return std::nullopt;
  }
  const cv::Mat ROI = image(cv::Rect(X1, Y1, X2 - X1, Y2 - Y1)).clone();
  return *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", ROI).toImageMsg();
}

} // namespace

CameraTracker::CameraTracker(std::string camera, int rotation_cw, rclcpp::Logger logger)
  : m_camera(std::move(camera))
  , m_rotation(rotation_cw)
  , m_logger(std::move(logger))
{
  if (!detail::isValidRotation(m_rotation))
  {
    throw std::invalid_argument("Camera '" + m_camera + "': image_rotation must be 0/90/180/270");
  }
}

CameraTracker::FrameResult
CameraTracker::process(const vision_msgs::msg::Detection2DArray& det2d_msg,
                       const vision_msgs::msg::Detection3DArray& det3d_msg,
                       const sensor_msgs::msg::Image& image_msg,
                       const TransformLookup& world_transform,
                       const object_tracker::Params& params,
                       const std::set<std::string>& rejected_labels)
{
  const rclcpp::Time CURRENT_TIME(det3d_msg.header.stamp);
  // shortcut: source clocks must be monotonic; restart with the manager after replay time resets.
  if (m_last_time && CURRENT_TIME <= *m_last_time)
  {
    return FrameResult::OUT_OF_ORDER;
  }

  cv::Mat image;
  try
  {
    image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
  }
  catch (const cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(m_logger, "Camera '%s': cv_bridge exception: %s", m_camera.c_str(), e.what());
    return FrameResult::BAD_IMAGE;
  }

  // ── Filter and transform detections into the world frame ────────────────
  const auto& whitelist = params.class_whitelist;
  std::optional<geometry_msgs::msg::TransformStamped> transform;
  std::vector<DetectionAdapter> detections;
  for (const auto& [id, pair] : pairById(det2d_msg, det3d_msg))
  {
    if (!pair.det2d || !pair.det3d || pair.det2d->results.empty() || pair.det3d->results.empty())
    {
      continue;
    }
    const auto& hypothesis = pair.det2d->results[0].hypothesis;
    if (hypothesis.score < params.confidence_threshold ||
        (!whitelist.empty() &&
         std::ranges::find(whitelist, hypothesis.class_id) == whitelist.end()) ||
        rejected_labels.contains(hypothesis.class_id) || !has3dBox(*pair.det3d))
    {
      continue;
    }
    if (!transform)
    {
      transform = world_transform();
      if (!transform)
      {
        return FrameResult::NO_TRANSFORM;
      }
    }
    // Whole-box world transform; invalid geometry is skipped with a warning.
    const auto WORLD_BOX = detail::transformBoundingBoxToFrame(pair.det3d->bbox, *transform);
    if (!WORLD_BOX)
    {
      RCLCPP_WARN(m_logger,
                  "Camera '%s': non-finite or degenerate bbox geometry - skipping detection ID: %s",
                  m_camera.c_str(),
                  id.c_str());
      continue;
    }
    detections.push_back(DetectionAdapter{.class_name = hypothesis.class_id,
                                          .class_id   = 0,
                                          .score      = hypothesis.score,
                                          .bbox3d     = *WORLD_BOX,
                                          .bbox2d     = pair.det2d->bbox,
                                          .frame_id   = params.world_frame});
  }

  // YOLOE often reports one object several times per frame under different labels. Keep the
  // highest-scoring detection of each similar-sized, clearly overlapping group.
  std::ranges::sort(detections, std::greater{}, &DetectionAdapter::score);
  std::vector<DetectionAdapter> kept;
  for (const auto& det : detections)
  {
    const bool DUPLICATE = std::ranges::any_of(kept, [&](const auto& other) {
      const double OVERLAP = std::max(
        TrackedObject::coveredFraction(
          center(det), extent(det), center(other), extent(other), params.overlap_min_extent),
        TrackedObject::coveredFraction(
          center(other), extent(other), center(det), extent(det), params.overlap_min_extent));
      return TrackedObject::sizesCompatible(
               extent(det), extent(other), params.association_max_size_ratio) &&
             OVERLAP >= params.merge_overlap_threshold;
    });
    if (!DUPLICATE)
    {
      kept.push_back(det);
    }
  }
  detections = std::move(kept);

  // ── Age, expire and predict tracks ───────────────────────────────────────
  const double DT          = m_last_time ? (CURRENT_TIME - *m_last_time).seconds() : 0.0;
  m_last_time              = CURRENT_TIME;
  const auto LOCAL_TIMEOUT = rclcpp::Duration::from_seconds(params.local_track_timeout);
  std::erase_if(m_tracks, [&](const auto& obj) {
    return obj->confirmed ? CURRENT_TIME - obj->last_seen > LOCAL_TIMEOUT
                          : obj->time_since_update + DT > params.unconfirmed_track_timeout;
  });
  for (auto& obj : m_tracks)
  {
    obj->predict(DT);
  }

  // ── Associate: class-agnostic, geometry decides, labels are voted per track ─
  std::vector<int> det_to_track(detections.size(), -1);
  std::set<size_t> matched;
  const auto ASSOCIATE = [&](bool confirmed_stage, double gate) {
    for (size_t d = 0; d < detections.size(); ++d)
    {
      if (det_to_track[d] >= 0)
      {
        continue;
      }
      const auto& det  = detections[d];
      double best_dist = std::numeric_limits<double>::max();
      int best_idx     = -1;
      for (size_t o = 0; o < m_tracks.size(); ++o)
      {
        const auto& obj = *m_tracks[o];
        if (obj.confirmed != confirmed_stage || matched.contains(o) ||
            (center(det) - obj.getPosition()).norm() > params.max_association_distance ||
            !TrackedObject::sizesCompatible(
              extent(det), obj.bbox3d_size, params.association_max_size_ratio))
        {
          continue;
        }
        const double DIST = m_tracks[o]->mahalanobisDistance(det);
        if (DIST < gate && DIST < best_dist)
        {
          best_dist = DIST;
          best_idx  = static_cast<int>(o);
        }
      }
      if (best_idx >= 0)
      {
        m_tracks[best_idx]->update(det, CURRENT_TIME);
        det_to_track[d] = best_idx;
        matched.insert(best_idx);
      }
    }
  };
  ASSOCIATE(true, params.gate_threshold_confirmed); // Stage 1: confirmed tracks (relaxed gate)
  ASSOCIATE(false, params.gate_threshold_strict);   // Stage 2: tentative tracks (strict gate)

  for (size_t d = 0; d < detections.size(); ++d)
  {
    if (det_to_track[d] >= 0)
    {
      continue;
    }
    if (std::cmp_greater_equal(m_tracks.size(), params.max_trackable_objects))
    {
      break;
    }
    m_tracks.push_back(
      std::make_unique<TrackedObject>(detections[d],
                                      CURRENT_TIME,
                                      m_next_id++,
                                      params.measurement_noise_std,
                                      params.process_noise_std,
                                      params.confirmation_threshold,
                                      static_cast<size_t>(params.max_crops_per_object),
                                      params.bbox_size_alpha));
    det_to_track[d] = static_cast<int>(m_tracks.size() - 1);
  }

  // ── Upright crops for observed confirmed tracks ──────────────────────────
  const cv::Mat UPRIGHT = detail::rotateImage(image, m_rotation);
  for (size_t d = 0; d < detections.size(); ++d)
  {
    if (det_to_track[d] < 0 || !m_tracks[det_to_track[d]]->confirmed)
    {
      continue;
    }
    const auto BOX = detail::rotateBox(detections[d].bbox2d, m_rotation, image.cols, image.rows);
    auto crop      = cropImagePatch(UPRIGHT, BOX, static_cast<int>(params.crop_padding));
    if (!crop)
    {
      ++m_crop_errors;
      continue;
    }
    crop->header = image_msg.header;
    m_tracks[det_to_track[d]]->addCrop(*crop);
  }
  return FrameResult::ACCEPTED;
}

std::vector<Tracklet> CameraTracker::tracklets() const
{
  std::vector<Tracklet> result;
  for (const auto& obj : m_tracks)
  {
    if (!obj->confirmed)
    {
      continue;
    }
    result.push_back(Tracklet{
      .camera     = m_camera,
      .local_id   = obj->track_id,
      .last_seen  = obj->last_seen,
      .position   = obj->getPosition(),
      .size       = obj->bbox3d_size,
      .class_name = obj->class_name,
      .score      = obj->score,
      .crops      = std::make_shared<const std::vector<sensor_msgs::msg::Image>>(obj->getCrops())});
  }
  return result;
}

} // namespace semantic_scene_forests_object_tracker
