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

#include <chrono>
#include <set>
#include <stdexcept>
#include <utility>

#include <tf2/exceptions.hpp>

#include "arbiter_worker.hpp"
#include "camera_tracker.hpp"
#include "global_tracker.hpp"

namespace semantic_scene_forests_object_tracker {

namespace {

// message_filters >= 7 (Kilted onwards) takes an rclcpp::QoS; message_filters 6 (Jazzy) only
// accepts an rmw_qos_profile_t. Select the available overload at compile time so the package
// builds against either distribution.
template <typename TSubscriber, typename TNode>
void subscribeWithQoS(TSubscriber& subscriber,
                      TNode* node,
                      const std::string& topic,
                      const rclcpp::QoS& qos,
                      const rclcpp::SubscriptionOptions& options)
{
  if constexpr (requires { subscriber.subscribe(node, topic, qos, options); })
  {
    subscriber.subscribe(node, topic, qos, options);
  }
  else
  {
    subscriber.subscribe(node, topic, qos.get_rmw_qos_profile(), options);
  }
}

} // namespace

ObjectTrackerComponent::CameraInput::CameraInput(ObjectTrackerComponent& owner,
                                                 std::string name,
                                                 int rotation)
  : owner(owner)
  , name(std::move(name))
  , group(owner.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
  , tracker(std::make_unique<CameraTracker>(this->name, rotation, owner.get_logger()))
{
}

ObjectTrackerComponent::CameraInput::~CameraInput() = default;

void ObjectTrackerComponent::CameraInput::onSynchronized(
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr& det2d_msg,
  const vision_msgs::msg::Detection3DArray::ConstSharedPtr& det3d_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr& img_msg)
{
  owner.onCameraFrame(*this, *det2d_msg, *det3d_msg, *img_msg);
}

ObjectTrackerComponent::ObjectTrackerComponent(const rclcpp::NodeOptions& options)
  : Node("object_tracker", options)
  , m_tf_buffer(this->get_clock())
  , m_tf_listener(m_tf_buffer)
  , m_diagnostic_updater(this)
{
  m_param_listener = std::make_shared<object_tracker::ParamListener>(
    this->get_node_parameters_interface(), this->get_logger());
  const auto PARAMS = m_param_listener->get_params();
  m_world_frame     = PARAMS.world_frame;
  if (PARAMS.snapshot_rate_hz <= 0.0)
  {
    throw std::invalid_argument("snapshot_rate_hz must be > 0 (it drives the global cycle)");
  }

  m_diagnostic_updater.setHardwareID("object_tracker");
  m_diagnostic_updater.add("Health", this, &ObjectTrackerComponent::onHealthDiagnostic);

  m_global = std::make_unique<GlobalTracker>(this->get_logger().get_child("global"));
  if (PARAMS.arbiter.enabled)
  {
    // Throws for an unknown provider, an invalid extra_body or a missing key.
    auto decide       = makeDecisionClient(PARAMS.arbiter);
    const auto FORMAT = parseDecisionProvider(PARAMS.arbiter.provider);
    if (!FORMAT)
    {
      throw std::invalid_argument("arbiter.provider: " + PARAMS.arbiter.provider);
    }
    m_arbiter = std::make_unique<ArbiterWorker>(std::move(decide), *FORMAT, PARAMS.arbiter.model);
    RCLCPP_INFO(this->get_logger(),
                "Identity arbiter on: provider %s, model %s",
                PARAMS.arbiter.provider.c_str(),
                PARAMS.arbiter.model.c_str());
  }

  const auto SENSOR_QOS = rclcpp::SensorDataQoS();
  std::set<std::string> input_topics;
  for (const auto& name : PARAMS.camera_names)
  {
    if (name.empty())
    {
      throw std::invalid_argument("Camera names must not be empty");
    }
    const auto& config = PARAMS.cameras.camera_names_map.at(name);
    for (const auto& topic :
         {config.detections_2d_topic, config.detections_3d_topic, config.image_topic})
    {
      const auto RESOLVED = this->get_node_topics_interface()->resolve_topic_name(topic);
      if (!input_topics.insert(RESOLVED).second)
      {
        throw std::invalid_argument("Repeated camera input topic: " + RESOLVED);
      }
    }

    auto camera = std::make_unique<CameraInput>(*this, name, config.image_rotation);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = camera->group; // One thread at a time per camera.
    subscribeWithQoS(camera->det2d_sub, this, config.detections_2d_topic, SENSOR_QOS, sub_options);
    subscribeWithQoS(camera->det3d_sub, this, config.detections_3d_topic, SENSOR_QOS, sub_options);
    subscribeWithQoS(camera->img_sub, this, config.image_topic, SENSOR_QOS, sub_options);
    // Exact stamps only: detections carry their image's stamp. The queue must hold the images
    // that arrive while the detector is still working on a frame.
    camera->synchronizer = std::make_unique<SynchronizerVision>(
      SyncPolicyVision(static_cast<uint32_t>(PARAMS.sync_queue_size)),
      camera->det2d_sub,
      camera->det3d_sub,
      camera->img_sub);
    // Concrete member callbacks also work with the Jazzy message_filters Signal9 adapter.
    camera->synchronizer->registerCallback(&CameraInput::onSynchronized, camera.get());
    camera->frequency = std::make_unique<diagnostic_updater::HeaderlessTopicDiagnostic>(
      "Vision pipeline (" + name + ")",
      m_diagnostic_updater,
      diagnostic_updater::FrequencyStatusParam(&m_min_freq, &m_max_freq, 0.1, 5));
    m_cameras.push_back(std::move(camera));
  }

  auto pub_qos = rclcpp::QoS(1).reliable().transient_local();
  m_candidates_pub =
    this->create_publisher<semantic_scene_forests_object_msgs::msg::ObjectCandidateArray>(
      "object_candidate_snapshots", pub_qos);
  publishSnapshot();
  m_global_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  m_global_timer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / PARAMS.snapshot_rate_hz),
    [this] { onGlobalCycle(); },
    m_global_group);

  RCLCPP_INFO(this->get_logger(),
              "ObjectTrackerComponent started with %zu cameras. World frame: %s",
              m_cameras.size(),
              m_world_frame.c_str());
}

ObjectTrackerComponent::~ObjectTrackerComponent() = default;

void ObjectTrackerComponent::onCameraFrame(CameraInput& camera,
                                           const vision_msgs::msg::Detection2DArray& det2d_msg,
                                           const vision_msgs::msg::Detection3DArray& det3d_msg,
                                           const sensor_msgs::msg::Image& img_msg)
{
  camera.frequency->tick();
  const auto PARAMS = m_param_listener->get_params();

  std::string tf_error;
  const auto LOOKUP = [&]() -> std::optional<geometry_msgs::msg::TransformStamped> {
    try
    {
      return m_tf_buffer.lookupTransform(m_world_frame,
                                         det3d_msg.header.frame_id,
                                         det3d_msg.header.stamp,
                                         rclcpp::Duration::from_seconds(0.1));
    }
    catch (const tf2::TransformException& ex)
    {
      tf_error = ex.what();
      RCLCPP_WARN(this->get_logger(),
                  "Camera '%s': cannot transform from '%s' to '%s': %s - skipping frame",
                  camera.name.c_str(),
                  det3d_msg.header.frame_id.c_str(),
                  m_world_frame.c_str(),
                  ex.what());
      return std::nullopt;
    }
  };
  const auto RESULT = camera.tracker->process(det2d_msg, det3d_msg, img_msg, LOOKUP, PARAMS);
  auto tracklets    = camera.tracker->tracklets();

  const std::scoped_lock LOCK{camera.mutex};
  switch (RESULT)
  {
    case CameraTracker::FrameResult::OUT_OF_ORDER:
      ++camera.rejected_timestamps;
      return;
    case CameraTracker::FrameResult::BAD_IMAGE:
      ++camera.bad_images;
      return;
    case CameraTracker::FrameResult::NO_TRANSFORM:
      camera.tf_errors.increment(tf_error);
      return;
    case CameraTracker::FrameResult::ACCEPTED:
      break;
  }
  camera.tracklets    = std::move(tracklets);
  camera.local_tracks = camera.tracker->trackCount();
  camera.crop_errors  = camera.tracker->cropErrors();
}

void ObjectTrackerComponent::onGlobalCycle()
{
  const auto PARAMS = m_param_listener->get_params();
  std::vector<Tracklet> tracklets;
  for (const auto& camera : m_cameras)
  {
    const std::scoped_lock LOCK{camera->mutex};
    tracklets.insert(tracklets.end(), camera->tracklets.begin(), camera->tracklets.end());
  }
  const auto RESULTS = m_arbiter ? m_arbiter->drainResults() : std::vector<ArbiterResult>{};
  auto requests      = m_global->cycle(tracklets, RESULTS, PARAMS, m_arbiter != nullptr);
  for (auto& request : requests)
  {
    m_arbiter->enqueue(std::move(request));
  }
  publishSnapshot();

  const auto STATS = m_global->stats(PARAMS);
  const std::scoped_lock LOCK{m_stats_mutex};
  m_global_tracks       = STATS.tracks;
  m_stale_tracks        = STATS.stale;
  m_links               = STATS.links;
  m_decisions           = STATS.decisions;
  m_merges              = STATS.merges;
  m_replacements        = STATS.replacements;
  m_held_tracks         = STATS.held;
  m_label_rejections    = STATS.label_rejections;
  m_decision_errors     = STATS.decision_errors;
  m_last_decision_error = STATS.last_error;
  m_arbiter_pending     = m_arbiter ? m_arbiter->pending() : 0;
}

void ObjectTrackerComponent::publishSnapshot()
{
  // unique_ptr lets intra-process delivery move the snapshot (crops included) without a copy.
  auto snapshot_ptr =
    std::make_unique<semantic_scene_forests_object_msgs::msg::ObjectCandidateArray>();
  auto& snapshot           = *snapshot_ptr;
  snapshot.header.stamp    = this->now();
  snapshot.header.frame_id = m_world_frame;
  for (const auto& track : m_global->tracks())
  {
    if (m_global->isHeld(*track))
    {
      continue; // Waiting for its label check.
    }
    semantic_scene_forests_object_msgs::msg::ObjectCandidate candidate;
    candidate.header      = snapshot.header;
    candidate.id          = static_cast<int64_t>(track->track_id);
    candidate.class_name  = track->class_name;
    candidate.detection   = track->toDetection3D();
    candidate.last_seen   = candidate.detection.header.stamp;
    candidate.image_crops = track->getCrops();
    snapshot.objects.push_back(std::move(candidate));
  }
  m_candidates_pub->publish(std::move(snapshot_ptr));
}

void ObjectTrackerComponent::onHealthDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  size_t tf_errors = 0;
  for (const auto& camera : m_cameras)
  {
    const std::scoped_lock LOCK{camera->mutex};
    stat.add("Local tracks (" + camera->name + ")", camera->local_tracks);
    stat.add("Rejected timestamps (" + camera->name + ")", camera->rejected_timestamps);
    stat.add("Bad images (" + camera->name + ")", camera->bad_images);
    stat.add("Crop errors (" + camera->name + ")", camera->crop_errors);
    camera->tf_errors.report(stat, "TF (" + camera->name + ")");
    tf_errors += camera->tf_errors.count();
  }
  const std::scoped_lock LOCK{m_stats_mutex};
  stat.add("Global tracks", m_global_tracks);
  stat.add("Stale tracks", m_stale_tracks);
  stat.add("Tracklet links", m_links);
  stat.add("Identity merges", m_merges);
  stat.add("Identity replacements", m_replacements);
  if (m_arbiter)
  {
    stat.add("Identity decisions", m_decisions);
    stat.add("Tracks held for label check", m_held_tracks);
    stat.add("Label rejections", m_label_rejections);
    stat.add("Identity decision errors", m_decision_errors);
    stat.add("Identity decision last error", m_last_decision_error);
    stat.add("Arbiter queue", m_arbiter_pending);
  }
  if (tf_errors > 0 || m_decision_errors > 0)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "TF or decision errors encountered");
  }
  else
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                 "Tracking " + std::to_string(m_global_tracks) + " global objects");
  }
}

} // namespace semantic_scene_forests_object_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(semantic_scene_forests_object_tracker::ObjectTrackerComponent)
