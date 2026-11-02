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

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stop_token>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <oai_api/client.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <semantic_scene_forests_object_msgs/msg/object_candidate_array.hpp>
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

  struct CameraInput
  {
    CameraInput(ObjectTrackerComponent& owner, std::string name);

    void onSynchronized(const vision_msgs::msg::Detection2DArray::ConstSharedPtr& det2d_msg,
                        const vision_msgs::msg::Detection3DArray::ConstSharedPtr& det3d_msg,
                        const sensor_msgs::msg::Image::ConstSharedPtr& img_msg);

    ObjectTrackerComponent& owner;
    std::string name;
    message_filters::Subscriber<vision_msgs::msg::Detection2DArray> det2d_sub;
    message_filters::Subscriber<vision_msgs::msg::Detection3DArray> det3d_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> img_sub;
    std::optional<rclcpp::Time> last_update_time;
    size_t rejected_timestamps{0};
    semantic_scene_forests_utils::ErrorCounter tf_errors;
    std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> frequency;
    std::unique_ptr<SynchronizerVision> synchronizer;
  };

  void syncedCallbackVision(CameraInput& camera,
                            const vision_msgs::msg::Detection2DArray::ConstSharedPtr& det2d_msg,
                            const vision_msgs::msg::Detection3DArray::ConstSharedPtr& det3d_msg,
                            const sensor_msgs::msg::Image::ConstSharedPtr& img_msg);

  static std::map<std::string, CombinedDetection>
  associateById(const vision_msgs::msg::Detection2DArray& det2d_msg,
                const vision_msgs::msg::Detection3DArray& det3d_msg);

  static bool has3dBbox(const vision_msgs::msg::Detection3D& det);
  bool isRecent(const TrackedObject& object, const rclcpp::Time& current_time) const;
  // Returns a stale/recent replacement pair for the arbiter when one is pending.
  std::optional<std::pair<uint64_t, uint64_t>>
  reconcileStaleTracks(const rclcpp::Time& current_time);
  void mergeOverlappingTracks(const rclcpp::Time& current_time);
  void mergeTrackPair(size_t i, size_t j);

  // ── Identity arbiter (Decisions API, last resort) ──────────────────────
  struct ArbiterRequest
  {
    bool stale_replacement; // a = stale track, b = recent track at its pose
    uint64_t a_id;
    uint64_t b_id;
    std::string a_label;
    std::string b_label;
    std::vector<sensor_msgs::msg::Image> a_crops;
    std::vector<sensor_msgs::msg::Image> b_crops;
    Eigen::Vector3d a_size;
    Eigen::Vector3d b_size;
    double center_distance;
  };
  struct ArbiterResult
  {
    bool stale_replacement;
    uint64_t a_id;
    uint64_t b_id;
    std::optional<double> same_probability; // nullopt on error or refusal
    std::string label;
    std::string error;
  };
  void applyArbiterResult(const rclcpp::Time& current_time);
  void startArbiter(const rclcpp::Time& current_time,
                    const std::optional<std::pair<uint64_t, uint64_t>>& replacement);
  static ArbiterResult runArbiter(const oai::OpenAIClient& client,
                                  const std::string& model,
                                  const ArbiterRequest& request);
  void publishSnapshot();
  std::optional<sensor_msgs::msg::Image>
  cropImagePatch(const cv::Mat& image, const vision_msgs::msg::BoundingBox2D& bbox, int padding);

  void onHealthDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);

  rclcpp::Publisher<semantic_scene_forests_object_msgs::msg::ObjectCandidateArray>::SharedPtr
    m_candidates_pub;
  rclcpp::TimerBase::SharedPtr m_snapshot_timer;

  std::vector<std::unique_ptr<TrackedObject>> m_tracked_objects;
  std::optional<rclcpp::Time> m_last_update_time;
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
  semantic_scene_forests_utils::ErrorCounter m_crop_errors;

  // Arbiter state. The worker only touches the client and the handoff below;
  // results are applied on the (mutually exclusive) camera callback thread.
  std::optional<oai::OpenAIClient> m_arbiter_client;
  // ponytail: decided pairs are kept for the session; prune with retired IDs if it grows.
  std::set<std::pair<uint64_t, uint64_t>> m_arbitrated_pairs;
  std::optional<std::chrono::steady_clock::time_point> m_last_arbiter_start;
  size_t m_identity_decisions{0};
  size_t m_identity_merges{0};
  size_t m_identity_replacements{0};
  semantic_scene_forests_utils::ErrorCounter m_arbiter_errors;
  std::mutex m_arbiter_mutex;
  bool m_arbiter_active{false};                  // guarded by m_arbiter_mutex
  std::optional<ArbiterResult> m_arbiter_result; // guarded by m_arbiter_mutex
  // Declared after everything it uses: destroyed (stop + join) first.
  std::jthread m_arbiter_worker;
  // Inputs and their diagnostics must be destroyed before the updater they reference.
  std::vector<std::unique_ptr<CameraInput>> m_cameras;
};

} // namespace semantic_scene_forests_object_tracker
