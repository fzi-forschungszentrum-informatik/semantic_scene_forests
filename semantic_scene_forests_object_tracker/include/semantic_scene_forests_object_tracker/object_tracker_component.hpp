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

#include <atomic>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <semantic_scene_forests_object_msgs/msg/object_candidate_array.hpp>
#include <semantic_scene_forests_utils/diagnostic_utils.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <semantic_scene_forests_object_tracker/object_tracker_parameters.hpp>

namespace semantic_scene_forests_object_tracker {

class ArbiterWorker;
class CameraTracker;
class GlobalTracker;
struct Tracklet;

// Two-level tracker. Each camera runs in its own callback group: exactly synchronized
// (2D, 3D, image) frames feed a CameraTracker whose confirmed tracks are posted to the
// camera's mailbox. A global timer (own callback group) reconciles all mailboxes into
// global tracks, talks to the arbiter worker and publishes the snapshot.
class ObjectTrackerComponent : public rclcpp::Node
{
public:
  explicit ObjectTrackerComponent(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ObjectTrackerComponent() override;

private:
  using SyncPolicyVision =
    message_filters::sync_policies::ExactTime<vision_msgs::msg::Detection2DArray,
                                              vision_msgs::msg::Detection3DArray,
                                              sensor_msgs::msg::Image>;
  using SynchronizerVision = message_filters::Synchronizer<SyncPolicyVision>;

  struct CameraInput
  {
    CameraInput(ObjectTrackerComponent& owner, std::string name, int rotation);
    ~CameraInput();

    void onSynchronized(const vision_msgs::msg::Detection2DArray::ConstSharedPtr& det2d_msg,
                        const vision_msgs::msg::Detection3DArray::ConstSharedPtr& det3d_msg,
                        const sensor_msgs::msg::Image::ConstSharedPtr& img_msg);

    ObjectTrackerComponent& owner;
    std::string name;
    rclcpp::CallbackGroup::SharedPtr group;
    std::unique_ptr<CameraTracker> tracker; // only touched from `group`
    message_filters::Subscriber<vision_msgs::msg::Detection2DArray> det2d_sub;
    message_filters::Subscriber<vision_msgs::msg::Detection3DArray> det3d_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> img_sub;
    std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> frequency;
    std::unique_ptr<SynchronizerVision> synchronizer;

    // Mailbox and stats, shared with the global cycle and diagnostics.
    std::mutex mutex;
    std::vector<Tracklet> tracklets; // latest confirmed tracks
    size_t local_tracks{0};
    size_t rejected_timestamps{0};
    size_t bad_images{0};
    size_t crop_errors{0};
    semantic_scene_forests_utils::ErrorCounter tf_errors;
  };

  void onCameraFrame(CameraInput& camera,
                     const vision_msgs::msg::Detection2DArray& det2d_msg,
                     const vision_msgs::msg::Detection3DArray& det3d_msg,
                     const sensor_msgs::msg::Image& img_msg);
  void onGlobalCycle();
  void publishSnapshot();
  void onHealthDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);

  std::shared_ptr<object_tracker::ParamListener> m_param_listener;
  std::string m_world_frame;
  tf2_ros::Buffer m_tf_buffer;
  tf2_ros::TransformListener m_tf_listener;

  rclcpp::Publisher<semantic_scene_forests_object_msgs::msg::ObjectCandidateArray>::SharedPtr
    m_candidates_pub;
  rclcpp::CallbackGroup::SharedPtr m_global_group;
  rclcpp::TimerBase::SharedPtr m_global_timer;

  // Global state: only touched from m_global_group (and the constructor).
  std::unique_ptr<GlobalTracker> m_global;
  std::unique_ptr<ArbiterWorker> m_arbiter;
  // Labels rejected by the arbiter, published by the global cycle to the camera callbacks.
  std::atomic<std::shared_ptr<const std::set<std::string>>> m_rejected_labels{
    std::make_shared<const std::set<std::string>>()};

  // Snapshot of global stats for diagnostics.
  std::mutex m_stats_mutex;
  size_t m_global_tracks{0};
  size_t m_stale_tracks{0};
  size_t m_links{0};
  size_t m_decisions{0};
  size_t m_merges{0};
  size_t m_replacements{0};
  size_t m_held_tracks{0};
  size_t m_label_rejections{0};
  size_t m_decision_errors{0};
  size_t m_arbiter_pending{0};
  std::string m_last_decision_error;

  diagnostic_updater::Updater m_diagnostic_updater;
  double m_min_freq{0.1};
  double m_max_freq{1000.0};
  // Inputs and their diagnostics must be destroyed before the updater they reference.
  std::vector<std::unique_ptr<CameraInput>> m_cameras;
};

} // namespace semantic_scene_forests_object_tracker
