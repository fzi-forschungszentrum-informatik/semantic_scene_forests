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

#include "semantic_scene_forests_object_tracker/tracker_worker.hpp"

#include <chrono>
#include <format>
#include <set>
#include <stdexcept>
#include <utility>

#include <pluginlib/class_list_macros.hpp>
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

TrackerWorker::CameraInput::CameraInput(TrackerWorker& owner, std::string name, int rotation)
  : owner(owner)
  , name(std::move(name))
  , group(owner.m_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
  , tracker(std::make_unique<CameraTracker>(this->name, rotation, owner.m_logger))
{
}

TrackerWorker::CameraInput::~CameraInput() = default;

void TrackerWorker::CameraInput::onSynchronized(
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr& det2d_msg,
  const vision_msgs::msg::Detection3DArray::ConstSharedPtr& det3d_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr& img_msg)
{
  owner.onCameraFrame(*this, *det2d_msg, *det3d_msg, *img_msg);
}

TrackerWorker::TrackerWorker()
  : m_logger(rclcpp::get_logger("tracker"))
{
}

void TrackerWorker::initialize(rclcpp::Node& node,
                               std::shared_ptr<fzi::semantic_scene_forests::SceneGraph> graph,
                               const std::string& name,
                               diagnostic_updater::Updater& diagnostics)
{
  m_node           = &node;
  m_graph          = std::move(graph);
  m_logger         = node.get_logger().get_child(name);
  m_tf_buffer      = std::make_unique<tf2_ros::Buffer>(node.get_clock());
  m_tf_listener    = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer);
  m_param_listener = std::make_shared<object_tracker::ParamListener>(
    node.get_node_parameters_interface(), m_logger, name + ".");
  const auto PARAMS = m_param_listener->get_params();
  m_world_frame     = PARAMS.world_frame;
  if (PARAMS.snapshot_rate_hz <= 0.0)
  {
    throw std::invalid_argument(name +
                                ".snapshot_rate_hz must be > 0 (it drives the global cycle)");
  }

  diagnostics.add(name + " health", this, &TrackerWorker::onHealthDiagnostic);

  // The tracker is the only component that writes crops.
  m_graph->setCropPolicy(
    static_cast<size_t>(PARAMS.max_crops_per_object),
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(PARAMS.crop_replace_min_interval_seconds)));
  m_global = std::make_unique<GlobalTracker>(m_logger.get_child("global"), m_graph);
  if (PARAMS.arbiter.enabled)
  {
    // Throws for an unknown provider, an invalid extra_body or a missing key.
    auto decide       = makeDecisionClient(PARAMS.arbiter);
    const auto FORMAT = parseDecisionProvider(PARAMS.arbiter.provider);
    if (!FORMAT)
    {
      throw std::invalid_argument("tracker.arbiter.provider: " + PARAMS.arbiter.provider);
    }
    m_arbiter = std::make_unique<ArbiterWorker>(std::move(decide), *FORMAT, PARAMS.arbiter.model);
    RCLCPP_INFO(m_logger,
                "Identity arbiter on: provider %s, model %s",
                PARAMS.arbiter.provider.c_str(),
                PARAMS.arbiter.model.c_str());
  }

  const auto SENSOR_QOS = rclcpp::SensorDataQoS();
  std::set<std::string> input_topics;
  for (const auto& camera_name : PARAMS.camera_names)
  {
    if (camera_name.empty())
    {
      throw std::invalid_argument("Camera names must not be empty");
    }
    const auto& config = PARAMS.cameras.camera_names_map.at(camera_name);
    for (const auto& topic :
         {config.detections_2d_topic, config.detections_3d_topic, config.image_topic})
    {
      const auto RESOLVED = node.get_node_topics_interface()->resolve_topic_name(topic);
      if (!input_topics.insert(RESOLVED).second)
      {
        throw std::invalid_argument("Repeated camera input topic: " + RESOLVED);
      }
    }

    auto camera = std::make_unique<CameraInput>(*this, camera_name, config.image_rotation);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = camera->group; // One thread at a time per camera.
    subscribeWithQoS(camera->det2d_sub, &node, config.detections_2d_topic, SENSOR_QOS, sub_options);
    subscribeWithQoS(camera->det3d_sub, &node, config.detections_3d_topic, SENSOR_QOS, sub_options);
    subscribeWithQoS(camera->img_sub, &node, config.image_topic, SENSOR_QOS, sub_options);
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
      std::format("{} vision pipeline ({})", name, camera_name),
      diagnostics,
      diagnostic_updater::FrequencyStatusParam(&m_min_freq, &m_max_freq, 0.1, 5));
    m_cameras.push_back(std::move(camera));
  }

  m_global_group = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // Same group as the cycle: requests and the cycle never run at the same time.
  m_add_observations_srv = node.create_service<AddObservations>(
    "add_observations",
    [this](const std::shared_ptr<rmw_request_id_t> header,
           const std::shared_ptr<AddObservations::Request> request) {
      onAddObservations(header, request);
    },
    rclcpp::ServicesQoS(),
    m_global_group);
  m_add_object_srv = node.create_service<AddObject>(
    "add_object",
    [this](const std::shared_ptr<AddObject::Request> request,
           std::shared_ptr<AddObject::Response> response) { onAddObject(request, response); },
    rclcpp::ServicesQoS(),
    m_global_group);

  RCLCPP_INFO(m_logger,
              "Tracker initialized with %zu cameras. World frame: %s",
              m_cameras.size(),
              m_world_frame.c_str());
}

TrackerWorker::~TrackerWorker()
{
  TrackerWorker::stop();
}

void TrackerWorker::start()
{
  const auto PARAMS = m_param_listener->get_params();
  m_global_timer    = m_node->create_wall_timer(
    std::chrono::duration<double>(1.0 / PARAMS.snapshot_rate_hz),
    [this] { onGlobalCycle(); },
    m_global_group);
}

void TrackerWorker::stop()
{
  if (m_global_timer)
  {
    m_global_timer->cancel();
  }
  const std::scoped_lock LOCK{m_cycle_mutex};
  m_arbiter.reset(); // Joins its thread; a running HTTP call finishes first.
}

void TrackerWorker::onCameraFrame(CameraInput& camera,
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
      return m_tf_buffer->lookupTransform(m_world_frame,
                                          det3d_msg.header.frame_id,
                                          det3d_msg.header.stamp,
                                          rclcpp::Duration::from_seconds(0.1));
    }
    catch (const tf2::TransformException& ex)
    {
      tf_error = ex.what();
      RCLCPP_WARN(m_logger,
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

void TrackerWorker::onAddObservations(const std::shared_ptr<rmw_request_id_t>& header,
                                      const std::shared_ptr<AddObservations::Request>& request)
{
  const auto PARAMS = m_param_listener->get_params();
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
                ? m_node->now()
                : rclcpp::Time(detection.header.stamp, RCL_ROS_TIME);
      try
      {
        const auto TRANSFORM = m_tf_buffer->lookupTransform(
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
      RCLCPP_WARN(m_logger,
                  "add_observations: rejecting '%s': %s",
                  observation.class_name.c_str(),
                  error.c_str());
      pending.local_ids.emplace_back(std::nullopt);
      pending.errors.push_back(std::move(error));
      continue;
    }
    auto crops = std::make_shared<std::vector<CropPtr>>();
    for (const auto& image : observation.image_crops)
    {
      if (auto jpeg = detail::encodeCrop(image, PARAMS.crop_jpeg_quality))
      {
        crops->push_back(std::move(jpeg));
      }
      else
      {
        RCLCPP_WARN(m_logger, "add_observations: skipping a crop that cannot be encoded");
      }
    }
    const auto& center = box->center.position;
    pending.tracklets.push_back(Tracklet{.camera     = MANUAL_SOURCE,
                                         .local_id   = m_manual_next_id,
                                         .last_seen  = stamp,
                                         .position   = {center.x, center.y, center.z},
                                         .size       = {box->size.x, box->size.y, box->size.z},
                                         .class_name = observation.class_name,
                                         .score      = 1.0,
                                         .crops      = std::move(crops)});
    pending.local_ids.emplace_back(m_manual_next_id++);
    pending.errors.emplace_back();
  }
  m_manual_pending.push_back(std::move(pending));
}

void TrackerWorker::onAddObject(const std::shared_ptr<AddObject::Request>& request,
                                const std::shared_ptr<AddObject::Response>& response)
{
  const auto PARAMS = m_param_listener->get_params();
  const auto FAIL   = [&](const std::string& message) {
    RCLCPP_WARN(
      m_logger, "add_object: rejecting '%s': %s", request->class_name.c_str(), message.c_str());
    response->success = false;
    response->message = message;
  };
  if (request->class_name.empty())
  {
    FAIL("empty class_name");
    return;
  }
  if (request->header.frame_id.empty())
  {
    FAIL("empty header frame");
    return;
  }
  if (request->header.stamp.sec < 0)
  {
    FAIL("negative header timestamp");
    return;
  }
  const bool NO_STAMP = request->header.stamp.sec == 0 && request->header.stamp.nanosec == 0;
  const rclcpp::Time STAMP =
    NO_STAMP ? m_node->now() : rclcpp::Time(request->header.stamp, RCL_ROS_TIME);
  std::optional<vision_msgs::msg::BoundingBox3D> box;
  try
  {
    const auto TRANSFORM = m_tf_buffer->lookupTransform(
      m_world_frame, request->header.frame_id, STAMP, rclcpp::Duration::from_seconds(0.1));
    box = detail::transformBoundingBoxToFrame(request->bbox, TRANSFORM);
  }
  catch (const tf2::TransformException& ex)
  {
    FAIL(std::string("no transform to ") + m_world_frame + ": " + ex.what());
    return;
  }
  if (!box)
  {
    FAIL("non-finite geometry, nonpositive size or invalid orientation");
    return;
  }
  std::vector<CropPtr> crops;
  for (const auto& image : request->image_crops)
  {
    if (auto jpeg = detail::encodeCrop(image, PARAMS.crop_jpeg_quality))
    {
      crops.push_back(std::move(jpeg));
    }
    else
    {
      RCLCPP_WARN(m_logger, "add_object: skipping a crop that cannot be encoded");
    }
  }
  const auto& center = box->center.position;
  const auto ID      = m_global->addConfirmed(request->class_name,
                                              {center.x, center.y, center.z},
                                              {box->size.x, box->size.y, box->size.z},
                                              STAMP,
                                              crops,
                                              PARAMS);
  m_manual_tracks.insert(ID);
  response->success   = true;
  response->object_id = static_cast<int64_t>(ID);
  response->message   = "added track " + std::to_string(ID);
}

void TrackerWorker::answerObservations()
{
  std::erase_if(m_manual_pending, [&](PendingObservations& pending) {
    pending.created.insert(m_global->createdLastCycle().begin(),
                           m_global->createdLastCycle().end());
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
      response.created.push_back(pending.created.contains(*ids[i]));
    }
    response.message += "fused " + std::to_string(fused) + " of " +
                        std::to_string(pending.local_ids.size()) + " observations";
    m_add_observations_srv->send_response(*pending.header, response);
    return true;
  });
}

void TrackerWorker::onGlobalCycle()
{
  const std::scoped_lock CYCLE_LOCK{m_cycle_mutex};
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
  const auto RESULTS = m_arbiter ? m_arbiter->drainResults() : std::vector<ArbiterResult>{};
  auto requests      = m_global->cycle(tracklets, RESULTS, PARAMS, m_arbiter != nullptr);
  if (m_global->rejectedLabels().size() != m_rejected_labels.load()->size()) // only grows
  {
    m_rejected_labels.store(
      std::make_shared<const std::set<std::string>>(m_global->rejectedLabels()));
  }
  for (auto& request : requests)
  {
    m_arbiter->enqueue(std::move(request));
  }
  writeToGraph();
  answerObservations(); // after the write, so answered objects are already in the graph

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

void TrackerWorker::writeToGraph()
{
  using fzi::semantic_scene_forests::ObjectNode;
  std::set<uint64_t> alive;
  for (const auto& track : m_global->tracks())
  {
    alive.insert(track->track_id);
    if (m_global->isHeld(*track))
    {
      continue; // Waiting for its label check.
    }
    const auto ID  = static_cast<int64_t>(track->track_id);
    const auto DET = track->toDetection3D();
    const auto IT  = m_written.find(track->track_id);
    if (IT == m_written.end())
    {
      // Track confirmed (after the label check, if held).
      const auto RESULT = m_graph->addObject(
        ObjectNode{.id         = ID,
                   .class_name = track->class_name,
                   .bbox       = DET.bbox,
                   .frame_id   = DET.header.frame_id,
                   .source = m_manual_tracks.contains(track->track_id) ? ObjectNode::SOURCE_MANUAL
                                                                       : ObjectNode::SOURCE_CAMERA,
                   .last_seen    = DET.header.stamp,
                   .last_updated = DET.header.stamp,
                   .crops        = {track->getCrops().begin(), track->getCrops().end()}});
      if (!RESULT)
      {
        RCLCPP_WARN(
          m_logger, "Cannot add track %ld to the graph: %s", ID, RESULT.error().message.c_str());
      }
      m_written[track->track_id] = Written{.last_seen     = track->last_seen,
                                           .crop_revision = track->cropRevision(),
                                           .suppressed    = !RESULT};
      continue;
    }
    auto& written = IT->second;
    if (written.suppressed ||
        (written.last_seen == track->last_seen && written.crop_revision == track->cropRevision()))
    {
      continue; // Class votes after creation never touch the graph.
    }
    // An unknown id means delete_object removed the node: stop writing it until the track retires.
    const auto CHECK = [&](const auto& result) {
      if (result)
      {
        return;
      }
      if (result.error().kind == SceneGraphErrorKind::NODE_DOES_NOT_EXIST)
      {
        RCLCPP_INFO(m_logger, "Node %ld was deleted: no longer writing track %ld", ID, ID);
        written.suppressed = true;
      }
      else
      {
        RCLCPP_WARN(
          m_logger, "Graph write for track %ld failed: %s", ID, result.error().message.c_str());
      }
    };
    if (written.last_seen != track->last_seen)
    {
      CHECK(m_graph->updateObjectPose(ID, DET.bbox, DET.header.stamp));
    }
    // The newest crops are the ones added since the last write; the graph decides which it keeps.
    const auto& crops = track->getCrops();
    const auto NEW_CROPS =
      std::min<size_t>(track->cropRevision() - written.crop_revision, crops.size());
    for (auto crop = crops.end() - static_cast<std::ptrdiff_t>(NEW_CROPS);
         !written.suppressed && crop != crops.end();
         ++crop)
    {
      CHECK(m_graph->offerCrop(ID, *crop));
    }
    written.last_seen     = track->last_seen;
    written.crop_revision = track->cropRevision();
  }
  // Merged, retired or rejected tracks: drop their nodes (held ones were never written).
  for (auto it = m_written.begin(); it != m_written.end();)
  {
    if (alive.contains(it->first))
    {
      ++it;
      continue;
    }
    if (!it->second.suppressed)
    {
      (void)m_graph->removeNode(static_cast<int64_t>(it->first)); // already gone is fine
    }
    m_manual_tracks.erase(it->first);
    it = m_written.erase(it);
  }
}

void TrackerWorker::onHealthDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
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

PLUGINLIB_EXPORT_CLASS(semantic_scene_forests_object_tracker::TrackerWorker,
                       semantic_scene_forests_manager::IWorker)
