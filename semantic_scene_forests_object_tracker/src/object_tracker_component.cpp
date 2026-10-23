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

#include "semantic_scene_forests_object_tracker/object_tracker_component.hpp"

#include "tracker_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace semantic_scene_forests_object_tracker {

namespace {

// message_filters >= 7 (Kilted onwards) takes an rclcpp::QoS; message_filters 6 (Jazzy) only
// accepts an rmw_qos_profile_t. Select the available overload at compile time so the package
// builds against either distribution.
template <typename TSubscriber, typename TNode>
void subscribeWithQoS(TSubscriber& subscriber,
                      TNode* node,
                      const std::string& topic,
                      const rclcpp::QoS& qos)
{
  if constexpr (requires { subscriber.subscribe(node, topic, qos); })
  {
    subscriber.subscribe(node, topic, qos);
  }
  else
  {
    subscriber.subscribe(node, topic, qos.get_rmw_qos_profile());
  }
}

} // namespace

ObjectTrackerComponent::CameraInput::CameraInput(ObjectTrackerComponent& owner, std::string name)
  : owner(owner)
  , name(std::move(name))
{
}

void ObjectTrackerComponent::CameraInput::onSynchronized(
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr& det2d_msg,
  const vision_msgs::msg::Detection3DArray::ConstSharedPtr& det3d_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr& img_msg)
{
  owner.syncedCallbackVision(*this, det2d_msg, det3d_msg, img_msg);
}

ObjectTrackerComponent::ObjectTrackerComponent(const rclcpp::NodeOptions& options)
  : Node("object_tracker", options)
  , m_tf_buffer(this->get_clock())
  , m_tf_listener(m_tf_buffer)
  , m_diagnostic_updater(this)
{
  m_param_listener = std::make_shared<object_tracker::ParamListener>(
    this->get_node_parameters_interface(), this->get_logger());
  m_params = m_param_listener->get_params();

  m_diagnostic_updater.setHardwareID("object_tracker");
  m_diagnostic_updater.add("Health", this, &ObjectTrackerComponent::onHealthDiagnostic);

  m_world_frame = m_params.world_frame;

  auto sensor_qos = rclcpp::SensorDataQoS();

  std::set<std::string> input_topics;
  for (const auto& name : m_params.camera_names)
  {
    if (name.empty())
    {
      throw std::invalid_argument("Camera names must not be empty");
    }
    const auto& topics = m_params.cameras.camera_names_map.at(name);
    for (const auto& topic :
         {topics.detections_2d_topic, topics.detections_3d_topic, topics.image_topic})
    {
      const auto RESOLVED = this->get_node_topics_interface()->resolve_topic_name(topic);
      if (!input_topics.insert(RESOLVED).second)
      {
        throw std::invalid_argument("Repeated camera input topic: " + RESOLVED);
      }
    }

    auto camera = std::make_unique<CameraInput>(*this, name);
    // Default subscriptions share the node's mutually exclusive callback group.
    subscribeWithQoS(camera->det2d_sub, this, topics.detections_2d_topic, sensor_qos);
    subscribeWithQoS(camera->det3d_sub, this, topics.detections_3d_topic, sensor_qos);
    subscribeWithQoS(camera->img_sub, this, topics.image_topic, sensor_qos);
    camera->synchronizer = std::make_unique<SynchronizerVision>(
      SyncPolicyVision(20), camera->det2d_sub, camera->det3d_sub, camera->img_sub);
    camera->synchronizer->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.05));
    // Concrete member callbacks also work with the Jazzy message_filters Signal9 adapter.
    camera->synchronizer->registerCallback(&CameraInput::onSynchronized, camera.get());
    camera->frequency = std::make_unique<diagnostic_updater::HeaderlessTopicDiagnostic>(
      "Vision pipeline (" + name + ")",
      m_diagnostic_updater,
      diagnostic_updater::FrequencyStatusParam(&m_min_freq, &m_max_freq, 0.1, 5));
    m_cameras.push_back(std::move(camera));
  }

  auto pub_qos = rclcpp::QoS(1000).reliable().transient_local();
  m_candidates_pub =
    this->create_publisher<semantic_scene_forests_object_msgs::msg::ObjectCandidate>(
      "object_candidates", pub_qos);

  RCLCPP_INFO(this->get_logger(),
              "ObjectTrackerComponent started with %zu cameras. World frame: %s",
              m_cameras.size(),
              m_world_frame.c_str());
}

bool ObjectTrackerComponent::has3dBbox(const vision_msgs::msg::Detection3D& det)
{
  const auto& b = det.bbox;
  return b.center.position.x != 0.0 || b.center.position.y != 0.0 || b.center.position.z != 0.0 ||
         b.size.x != 0.0 || b.size.y != 0.0 || b.size.z != 0.0;
}

std::map<std::string, CombinedDetection>
ObjectTrackerComponent::associateById(const vision_msgs::msg::Detection2DArray& det2d_msg,
                                      const vision_msgs::msg::Detection3DArray& det3d_msg)
{
  std::map<std::string, CombinedDetection> combined;

  for (const auto& det2d : det2d_msg.detections)
  {
    if (!det2d.id.empty())
    {
      combined[det2d.id].det2d = &det2d;
    }
  }

  for (const auto& det3d : det3d_msg.detections)
  {
    if (!det3d.id.empty() && combined.contains(det3d.id))
    {
      combined[det3d.id].det3d = &det3d;
    }
  }

  return combined;
}

void ObjectTrackerComponent::mergeOverlappingTracks()
{
  double threshold = m_params.merge_iou_threshold;
  std::set<size_t> removed;

  for (size_t i = 0; i < m_tracked_objects.size(); ++i)
  {
    auto& tracked_object = m_tracked_objects[i];

    if (removed.contains(i) || !tracked_object->confirmed)
    {
      continue;
    }

    for (size_t j = i + 1; j < m_tracked_objects.size(); ++j)
    {
      auto& other_tracked_object = m_tracked_objects[j];
      if (removed.contains(j) || !other_tracked_object->confirmed)
      {
        continue;
      }

      // TODO: Implement better name matching for similar but not equal names.
      if (tracked_object->class_name != other_tracked_object->class_name)
      {
        continue;
      }

      // The manager cannot retire an ID that has already been published.
      if (tracked_object->published && other_tracked_object->published)
      {
        continue;
      }

      if (TrackedObject::bbox3dOverlap(*tracked_object, *other_tracked_object) > threshold)
      {
        if (tracked_object->published ||
            (!other_tracked_object->published &&
             tracked_object->track_id < other_tracked_object->track_id))
        {
          tracked_object->mergeFrom(*other_tracked_object);
          removed.insert(j);
        }
        else
        {
          other_tracked_object->mergeFrom(*tracked_object);
          removed.insert(i);
          break;
        }
      }
    }
  }

  if (!removed.empty())
  {
    std::vector<std::unique_ptr<TrackedObject>> kept;
    kept.reserve(m_tracked_objects.size() - removed.size());
    for (size_t i = 0; i < m_tracked_objects.size(); ++i)
    {
      if (!removed.contains(i))
      {
        kept.push_back(std::move(m_tracked_objects[i]));
      }
    }
    m_tracked_objects = std::move(kept);
  }
}

std::optional<sensor_msgs::msg::Image> ObjectTrackerComponent::cropImagePatch(
  const cv::Mat& image, const vision_msgs::msg::BoundingBox2D& bbox, int padding)
{
  double cx = bbox.center.position.x;
  double cy = bbox.center.position.y;
  double w  = bbox.size_x;
  double h  = bbox.size_y;

  int x1 = std::max(0, static_cast<int>(std::round(cx - (w / 2.0))) - padding);
  int y1 = std::max(0, static_cast<int>(std::round(cy - (h / 2.0))) - padding);
  int x2 = std::min(image.cols, static_cast<int>(std::round(cx + (w / 2.0))) + padding);
  int y2 = std::min(image.rows, static_cast<int>(std::round(cy + (h / 2.0))) + padding);

  if (x2 <= x1 || y2 <= y1)
  {
    RCLCPP_WARN(this->get_logger(), "Empty crop region for bbox at (%.1f, %.1f)", cx, cy);
    m_crop_errors.increment("Empty crop region");
    return std::nullopt;
  }

  cv::Mat roi      = image(cv::Rect(x1, y1, x2 - x1, y2 - y1)).clone();
  auto img_msg_ptr = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", roi).toImageMsg();
  return *img_msg_ptr;
}

void ObjectTrackerComponent::syncedCallbackVision(
  CameraInput& camera,
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr& det2d_msg,
  const vision_msgs::msg::Detection3DArray::ConstSharedPtr& det3d_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr& img_msg)
{
  camera.frequency->tick();

  m_params = m_param_listener->get_params();

  double current_time = static_cast<double>(det2d_msg->header.stamp.sec) +
                        (static_cast<double>(det2d_msg->header.stamp.nanosec) / 1e9);

  // shortcut: source clocks must be monotonic; restart with the manager after replay time resets.
  if (camera.last_update_time && current_time <= *camera.last_update_time)
  {
    ++camera.rejected_timestamps;
    return;
  }
  camera.last_update_time = current_time;

  const double TRACKING_TIME = std::max(m_last_update_time.value_or(current_time), current_time);
  const double DT            = TRACKING_TIME - m_last_update_time.value_or(TRACKING_TIME);

  const auto& whitelist = m_params.class_whitelist;
  bool use_whitelist    = !whitelist.empty();

  auto combined_detections = associateById(*det2d_msg, *det3d_msg);

  struct TransformedDetection
  {
    const vision_msgs::msg::Detection2D* det2d;
    vision_msgs::msg::Detection3D det3d;
  };

  std::vector<TransformedDetection> transformed_detections;
  transformed_detections.reserve(combined_detections.size());
  std::optional<geometry_msgs::msg::TransformStamped> world_transform;

  for (auto& [id, combined] : combined_detections)
  {
    if (!combined.det2d || !combined.det3d)
    {
      continue;
    }

    if (combined.det2d->results.empty() || combined.det3d->results.empty())
    {
      continue;
    }

    double score = combined.det2d->results[0].hypothesis.score;
    if (score < m_params.confidence_threshold)
    {
      continue;
    }

    if (use_whitelist && std::ranges::find(whitelist, combined.className()) == whitelist.end())
    {
      continue;
    }

    if (!has3dBbox(*combined.det3d))
    {
      continue;
    }

    TransformedDetection trans_det;
    trans_det.det2d = combined.det2d;
    trans_det.det3d = *combined.det3d;

    try
    {
      if (!world_transform)
      {
        world_transform = m_tf_buffer.lookupTransform(m_world_frame,
                                                      det3d_msg->header.frame_id,
                                                      det3d_msg->header.stamp,
                                                      rclcpp::Duration::from_seconds(0.1));
      }

      // Whole-box world transform; invalid geometry is skipped with a warning.
      auto world_bbox = detail::transformBoundingBoxToFrame(combined.det3d->bbox, *world_transform);
      if (!world_bbox)
      {
        RCLCPP_WARN(this->get_logger(),
                    "Non-finite or degenerate bbox geometry - skipping detection ID: %s",
                    combined.id().c_str());
        continue;
      }

      trans_det.det3d.bbox            = *world_bbox;
      trans_det.det3d.header.frame_id = m_world_frame;
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Camera '%s': cannot transform from '%s' to '%s': %s - skipping batch",
                  camera.name.c_str(),
                  det3d_msg->header.frame_id.c_str(),
                  m_world_frame.c_str(),
                  ex.what());
      camera.tf_errors.increment(ex.what());
      return;
    }

    transformed_detections.push_back(std::move(trans_det));
  }

  std::vector<std::unique_ptr<CombinedDetection>> detections;
  detections.reserve(transformed_detections.size());
  for (auto& trans_det : transformed_detections)
  {
    auto combined   = std::make_unique<CombinedDetection>();
    combined->det2d = trans_det.det2d;
    combined->det3d = &trans_det.det3d;
    detections.push_back(std::move(combined));
  }

  cv_bridge::CvImageConstPtr cv_image;
  try
  {
    cv_image = cv_bridge::toCvCopy(*img_msg, "bgr8");
  }
  catch (const cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  for (auto& obj : m_tracked_objects)
  {
    obj->predict(DT);
  }

  std::vector<int> det_to_track(detections.size(), -1);
  std::set<size_t> obj_matched;

  // Stage 1: Confirmed tracks (relaxed gate)
  for (size_t d = 0; d < detections.size(); ++d)
  {
    if (det_to_track[d] >= 0)
    {
      continue;
    }
    const auto& det = *detections[d];

    double best_dist = std::numeric_limits<double>::max();
    int best_idx     = -1;

    for (size_t o = 0; o < m_tracked_objects.size(); ++o)
    {
      auto& obj = m_tracked_objects[o];
      if (!obj->confirmed || obj_matched.contains(o))
      {
        continue;
      }
      if (det.className() != obj->class_name)
      {
        continue;
      }

      DetectionAdapter adapter;
      adapter.class_name = det.className();
      adapter.class_id   = 0;
      adapter.score      = det.score();
      adapter.bbox3d     = det.bbox3D();
      adapter.bbox2d     = det.bbox2D();
      adapter.frame_id   = m_world_frame;

      double dist = obj->mahalanobisDistance(adapter);
      if (dist < m_params.gate_threshold_confirmed && dist < best_dist)
      {
        best_dist = dist;
        best_idx  = static_cast<int>(o);
      }
    }

    if (best_idx >= 0)
    {
      DetectionAdapter adapter;
      adapter.class_name = detections[d]->className();
      adapter.class_id   = 0;
      adapter.score      = detections[d]->score();
      adapter.bbox3d     = detections[d]->bbox3D();
      adapter.bbox2d     = detections[d]->bbox2D();
      adapter.frame_id   = m_world_frame;

      m_tracked_objects[best_idx]->update(adapter, current_time);
      det_to_track[d] = best_idx;
      obj_matched.insert(best_idx);
    }
  }

  // Stage 2: Unconfirmed tracks (strict gate)
  for (size_t d = 0; d < detections.size(); ++d)
  {
    if (det_to_track[d] >= 0)
    {
      continue;
    }
    const auto& det = *detections[d];

    double best_dist = std::numeric_limits<double>::max();
    int best_idx     = -1;

    for (size_t o = 0; o < m_tracked_objects.size(); ++o)
    {
      auto& obj = m_tracked_objects[o];
      if (obj->confirmed || obj_matched.contains(o))
      {
        continue;
      }
      if (det.className() != obj->class_name)
      {
        continue;
      }

      DetectionAdapter adapter;
      adapter.class_name = det.className();
      adapter.class_id   = 0;
      adapter.score      = det.score();
      adapter.bbox3d     = det.bbox3D();
      adapter.bbox2d     = det.bbox2D();
      adapter.frame_id   = m_world_frame;

      double dist = obj->mahalanobisDistance(adapter);
      if (dist < m_params.gate_threshold_strict && dist < best_dist)
      {
        best_dist = dist;
        best_idx  = static_cast<int>(o);
      }
    }

    if (best_idx >= 0)
    {
      DetectionAdapter adapter;
      adapter.class_name = detections[d]->className();
      adapter.class_id   = 0;
      adapter.score      = detections[d]->score();
      adapter.bbox3d     = detections[d]->bbox3D();
      adapter.bbox2d     = detections[d]->bbox2D();
      adapter.frame_id   = m_world_frame;

      m_tracked_objects[best_idx]->update(adapter, current_time);
      det_to_track[d] = best_idx;
      obj_matched.insert(best_idx);
    }
  }

  // Create new tracks for unmatched detections
  for (size_t d = 0; d < detections.size(); ++d)
  {
    if (det_to_track[d] >= 0)
    {
      continue;
    }
    if (static_cast<int>(m_tracked_objects.size()) >= m_params.max_trackable_objects)
    {
      break;
    }

    DetectionAdapter adapter;
    adapter.class_name = detections[d]->className();
    adapter.class_id   = 0;
    adapter.score      = detections[d]->score();
    adapter.bbox3d     = detections[d]->bbox3D();
    adapter.bbox2d     = detections[d]->bbox2D();
    adapter.frame_id   = m_world_frame;

    m_tracked_objects.push_back(
      std::make_unique<TrackedObject>(adapter,
                                      current_time,
                                      m_next_track_id,
                                      m_params.measurement_noise_std,
                                      m_params.process_noise_std,
                                      m_params.confirmation_threshold,
                                      static_cast<size_t>(m_params.max_crops_per_object)));
    m_next_track_id++;
  }

  // Crop images for matched confirmed tracks. This must run before
  // mergeOverlappingTracks(): det_to_track stores indices into the
  // pre-compaction m_tracked_objects vector, and TrackedObject::mergeFrom()
  // carries the crops into the surviving track.
  for (size_t d = 0; d < detections.size(); ++d)
  {
    int obj_idx = det_to_track[d];
    if (obj_idx < 0)
    {
      continue;
    }

    auto& obj = m_tracked_objects[obj_idx];
    if (!obj->confirmed)
    {
      continue;
    }

    auto crop = cropImagePatch(cv_image->image, detections[d]->bbox2D(), m_params.crop_padding);
    if (crop)
    {
      crop->header = img_msg->header;
      obj->addCrop(*crop);
    }
  }

  mergeOverlappingTracks();

  {
    size_t confirmed_count   = 0;
    size_t unconfirmed_count = 0;
    for (const auto& obj : m_tracked_objects)
    {
      if (obj->confirmed)
      {
        ++confirmed_count;
      }
      else
      {
        ++unconfirmed_count;
      }
    }
    RCLCPP_DEBUG(this->get_logger(),
                 "Tracks: %zu confirmed, %zu unconfirmed (%zu total) | "
                 "detections this frame: %zu",
                 confirmed_count,
                 unconfirmed_count,
                 m_tracked_objects.size(),
                 detections.size());
  }

  // Publish all confirmed tracks
  for (const auto& obj : m_tracked_objects)
  {
    if (!obj->confirmed)
    {
      continue;
    }

    semantic_scene_forests_object_msgs::msg::ObjectCandidate candidate;
    candidate.header       = det2d_msg->header;
    candidate.header.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(obj->last_seen * 1e9)));
    candidate.header.frame_id           = m_world_frame;
    candidate.id                        = obj->track_id;
    candidate.class_name                = obj->class_name;
    candidate.detection                 = obj->toDetection3D();
    candidate.detection.header          = candidate.header;
    candidate.detection.header.frame_id = m_world_frame;
    candidate.image_crops               = obj->getCrops();
    m_candidates_pub->publish(candidate);
    obj->published = true;
  }

  // Prune unconfirmed tracks not observed within the timeout window.
  double timeout = m_params.unconfirmed_track_timeout;
  m_tracked_objects.erase(std::remove_if(m_tracked_objects.begin(),
                                         m_tracked_objects.end(),
                                         [timeout](const std::unique_ptr<TrackedObject>& obj) {
                                           return !obj->confirmed &&
                                                  obj->time_since_update > timeout;
                                         }),
                          m_tracked_objects.end());

  m_last_update_time = TRACKING_TIME;
}

void ObjectTrackerComponent::onHealthDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  size_t confirmed = 0;
  for (const auto& obj : m_tracked_objects)
  {
    if (obj->confirmed)
    {
      ++confirmed;
    }
  }

  stat.add("Total tracks", m_tracked_objects.size());
  stat.add("Confirmed tracks", confirmed);
  size_t tf_errors = 0;
  for (const auto& camera : m_cameras)
  {
    camera->tf_errors.report(stat, "TF (" + camera->name + ")");
    stat.add("Rejected timestamps (" + camera->name + ")", camera->rejected_timestamps);
    tf_errors += camera->tf_errors.count();
  }
  m_crop_errors.report(stat, "Crop");

  if (tf_errors > 0 || m_crop_errors.count() > 0)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "TF/crop errors encountered");
  }
  else
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                 "Tracking " + std::to_string(confirmed) + " confirmed objects");
  }
}

} // namespace semantic_scene_forests_object_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(semantic_scene_forests_object_tracker::ObjectTrackerComponent)
