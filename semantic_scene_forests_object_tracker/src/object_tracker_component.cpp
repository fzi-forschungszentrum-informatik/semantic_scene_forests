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
#include "tracker_geometry.hpp"

namespace semantic_scene_forests_object_tracker {

namespace {

// Tracklet camera of add_observations sightings; camera names are user-chosen identifiers.
const std::string MANUAL_SOURCE = "<manual>";

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
  // Same group as the cycle: requests and the cycle never run at the same time.
  m_add_observations_srv = this->create_service<AddObservations>(
    "add_observations",
    [this](const std::shared_ptr<rmw_request_id_t> header,
           const std::shared_ptr<AddObservations::Request> request) {
      onAddObservations(header, request);
    },
    rclcpp::ServicesQoS(),
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
  const auto REJECTED = m_rejected_labels.load();
  const auto RESULT =
    camera.tracker->process(det2d_msg, det3d_msg, img_msg, LOOKUP, PARAMS, *REJECTED);
  auto tracklets = camera.tracker->tracklets();

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

void ObjectTrackerComponent::onAddObservations(
  const std::shared_ptr<rmw_request_id_t>& header,
  const std::shared_ptr<AddObservations::Request>& request)
{
  PendingObservations pending;
  pending.header = header;
  for (const auto& observation : request->observations)
  {
    const auto& detection = observation.detection;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    std::string error;
    std::optional<vision_msgs::msg::BoundingBox3D> box;
    if (observation.class_name.empty())
    {
      error = "empty class_name";
    }
    else if (detection.header.frame_id.empty())
    {
      error = "empty detection frame";
    }
    else if (detection.header.stamp.sec < 0)
    {
      error = "negative detection timestamp";
    }
    else if (m_global->rejectedLabels().contains(observation.class_name))
    {
      error = "label '" + observation.class_name + "' was rejected for this session";
    }
    else
    {
      stamp = detection.header.stamp.sec == 0 && detection.header.stamp.nanosec == 0
                ? this->now()
                : rclcpp::Time(detection.header.stamp, RCL_ROS_TIME);
      try
      {
        const auto TRANSFORM = m_tf_buffer.lookupTransform(
          m_world_frame, detection.header.frame_id, stamp, rclcpp::Duration::from_seconds(0.1));
        box = detail::transformBoundingBoxToFrame(detection.bbox, TRANSFORM);
        if (!box)
        {
          error = "non-finite geometry, nonpositive size or invalid orientation";
        }
      }
      catch (const tf2::TransformException& ex)
      {
        error = std::string("no transform to ") + m_world_frame + ": " + ex.what();
      }
    }
    if (!error.empty())
    {
      RCLCPP_WARN(this->get_logger(),
                  "add_observations: rejecting '%s': %s",
                  observation.class_name.c_str(),
                  error.c_str());
      pending.local_ids.emplace_back(std::nullopt);
      pending.errors.push_back(std::move(error));
      continue;
    }
    const auto& center = box->center.position;
    pending.tracklets.push_back(Tracklet{
      .camera     = MANUAL_SOURCE,
      .local_id   = m_manual_next_id,
      .last_seen  = stamp,
      .position   = {center.x, center.y, center.z},
      .size       = {box->size.x, box->size.y, box->size.z},
      .class_name = observation.class_name,
      .score      = 1.0,
      .crops =
        std::make_shared<const std::vector<sensor_msgs::msg::Image>>(observation.image_crops)});
    pending.local_ids.emplace_back(m_manual_next_id++);
    pending.errors.emplace_back();
  }
  m_manual_pending.push_back(std::move(pending));
}

void ObjectTrackerComponent::answerObservations(uint64_t first_new_id)
{
  std::erase_if(m_manual_pending, [&](PendingObservations& pending) {
    if (!pending.first_new_id)
    {
      pending.first_new_id = first_new_id; // this cycle fused them
    }
    std::vector<std::optional<uint64_t>> ids;
    for (const auto& local_id : pending.local_ids)
    {
      ids.push_back(local_id ? m_global->linkedTrack(MANUAL_SOURCE, *local_id) : std::nullopt);
      if (ids.back() && m_global->isHeld(*ids.back()))
      {
        return false; // label check still running; ask again next cycle
      }
    }
    AddObservations::Response response;
    response.success = true;
    size_t fused     = 0;
    for (size_t i = 0; i < ids.size(); ++i)
    {
      const auto& tracks = m_global->tracks();
      const auto TRACK =
        ids[i] ? std::ranges::find_if(
                   tracks, [id = *ids[i]](const auto& track) { return track->track_id == id; })
               : tracks.end();
      if (TRACK == tracks.end())
      {
        const auto& error = !pending.errors[i].empty() ? pending.errors[i]
                            : m_arbiter                ? "rejected by the tracker's label check"
                                                       : "rejected by the global tracker";
        response.success  = false;
        response.message += "observation " + std::to_string(i) + ": " + error + "; ";
        response.object_ids.push_back(0);
        response.class_names.emplace_back();
        response.created.push_back(false);
        continue;
      }
      ++fused;
      response.object_ids.push_back(static_cast<int64_t>(*ids[i]));
      response.class_names.push_back((*TRACK)->class_name);
      response.created.push_back(*ids[i] >= *pending.first_new_id);
    }
    response.message += "fused " + std::to_string(fused) + " of " +
                        std::to_string(pending.local_ids.size()) + " observations";
    m_add_observations_srv->send_response(*pending.header, response);
    return true;
  });
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
  for (const auto& pending : m_manual_pending)
  {
    // Reposted until answered; an unchanged stamp is not applied twice.
    tracklets.insert(tracklets.end(), pending.tracklets.begin(), pending.tracklets.end());
  }
  const auto FIRST_NEW_ID = m_global->nextId();
  const auto RESULTS      = m_arbiter ? m_arbiter->drainResults() : std::vector<ArbiterResult>{};
  auto requests           = m_global->cycle(tracklets, RESULTS, PARAMS, m_arbiter != nullptr);
  if (m_global->rejectedLabels().size() != m_rejected_labels.load()->size()) // only grows
  {
    m_rejected_labels.store(
      std::make_shared<const std::set<std::string>>(m_global->rejectedLabels()));
  }
  for (auto& request : requests)
  {
    m_arbiter->enqueue(std::move(request));
  }
  publishSnapshot();
  answerObservations(FIRST_NEW_ID); // after the snapshot, so answered objects are already out

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
