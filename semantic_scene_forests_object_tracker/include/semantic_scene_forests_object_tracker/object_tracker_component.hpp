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
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <semantic_scene_forests_object_msgs/msg/object_candidate.hpp>
#include <semantic_scene_forests_utils/diagnostic_utils.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "semantic_scene_forests_object_tracker/tracked_object.hpp"
#include <semantic_scene_forests_object_tracker/object_tracker_parameters.hpp>

namespace semantic_scene_forests_object_tracker {

struct CombinedDetection
{
  const vision_msgs::msg::Detection2D* det2d = nullptr;
  const vision_msgs::msg::Detection3D* det3d = nullptr;

  [[nodiscard]] std::string className() const
  {
    if (det2d && !det2d->results.empty())
    {
      return det2d->results[0].hypothesis.class_id;
    }
    if (det3d && !det3d->results.empty())
    {
      return det3d->results[0].hypothesis.class_id;
    }
    return "";
  }

  [[nodiscard]] double score() const
  {
    if (det2d && !det2d->results.empty())
    {
      return det2d->results[0].hypothesis.score;
    }
    if (det3d && !det3d->results.empty())
    {
      return det3d->results[0].hypothesis.score;
    }
    return 0.0;
  }

  [[nodiscard]] const vision_msgs::msg::BoundingBox2D& bbox2D() const { return det2d->bbox; }

  [[nodiscard]] const vision_msgs::msg::BoundingBox3D& bbox3D() const { return det3d->bbox; }

  [[nodiscard]] std::string id() const
  {
    if (det2d && !det2d->id.empty())
    {
      return det2d->id;
    }
    if (det3d && !det3d->id.empty())
    {
      return det3d->id;
    }
    return "";
  }
};

class ObjectTrackerComponent : public rclcpp::Node
{
public:
  explicit ObjectTrackerComponent(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  using SyncPolicyVision =
    message_filters::sync_policies::ApproximateTime<vision_msgs::msg::Detection2DArray,
                                                    vision_msgs::msg::Detection3DArray,
                                                    sensor_msgs::msg::Image>;
  using SynchronizerVision = message_filters::Synchronizer<SyncPolicyVision>;

  void syncedCallbackVision(const vision_msgs::msg::Detection2DArray::ConstSharedPtr& det2d_msg,
                            const vision_msgs::msg::Detection3DArray::ConstSharedPtr& det3d_msg,
                            const sensor_msgs::msg::Image::ConstSharedPtr& img_msg);

  static std::map<std::string, CombinedDetection>
  associateById(const vision_msgs::msg::Detection2DArray& det2d_msg,
                const vision_msgs::msg::Detection3DArray& det3d_msg);

  static bool has3dBbox(const vision_msgs::msg::Detection3D& det);
  void mergeOverlappingTracks();
  std::optional<sensor_msgs::msg::Image>
  cropImagePatch(const cv::Mat& image, const vision_msgs::msg::BoundingBox2D& bbox, int padding);

  void onHealthDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);

  message_filters::Subscriber<vision_msgs::msg::Detection2DArray> m_det2d_sub;
  message_filters::Subscriber<vision_msgs::msg::Detection3DArray> m_det3d_sub;
  message_filters::Subscriber<sensor_msgs::msg::Image> m_img_sub;

  std::shared_ptr<SynchronizerVision> m_sync_vision;

  rclcpp::Publisher<semantic_scene_forests_object_msgs::msg::ObjectCandidate>::SharedPtr
    m_candidates_pub;

  std::vector<std::unique_ptr<TrackedObject>> m_tracked_objects;
  std::optional<double> m_last_update_time;
  uint64_t m_next_track_id{1};

  std::shared_ptr<object_tracker::ParamListener> m_param_listener;
  object_tracker::Params m_params;

  tf2_ros::Buffer m_tf_buffer;
  tf2_ros::TransformListener m_tf_listener;
  std::string m_world_frame;

  // ── Diagnostics ────────────────────────────────────────────────────────
  // Topic freshness uses the built-in frequency monitor; only failure counters
  // are tracked by hand.
  diagnostic_updater::Updater m_diagnostic_updater;
  double m_min_freq{0.5};
  double m_max_freq{1000.0};
  std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> m_vision_freq;
  semantic_scene_forests_utils::ErrorCounter m_tf_errors;
  semantic_scene_forests_utils::ErrorCounter m_crop_errors;
};

} // namespace semantic_scene_forests_object_tracker
