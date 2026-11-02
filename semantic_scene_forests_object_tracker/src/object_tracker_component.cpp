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
#include <cstddef>
#include <functional>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>
#include <oai_api/builder.hpp>

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

  auto pub_qos = rclcpp::QoS(1).reliable().transient_local();
  m_candidates_pub =
    this->create_publisher<semantic_scene_forests_object_msgs::msg::ObjectCandidateArray>(
      "object_candidate_snapshots", pub_qos);
  publishSnapshot();
  if (m_params.snapshot_rate_hz > 0.0)
  {
    // Default callback group: mutually exclusive with the camera callbacks, so no extra lock.
    m_snapshot_timer =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / m_params.snapshot_rate_hz),
                              [this] { publishSnapshot(); });
  }

  if (m_params.arbiter.enabled && m_params.arbiter.api_key.empty())
  {
    RCLCPP_WARN(this->get_logger(), "arbiter.enabled but arbiter.api_key is empty; arbiter off");
  }
  else if (m_params.arbiter.enabled)
  {
    nlohmann::json extra_body;
    if (!m_params.arbiter.extra_body.empty())
    {
      extra_body = nlohmann::json::parse(m_params.arbiter.extra_body, nullptr, false);
      if (!extra_body.is_object())
      {
        throw std::invalid_argument("arbiter.extra_body must be a JSON object: " +
                                    m_params.arbiter.extra_body);
      }
    }
    m_arbiter_client.emplace(
      oai::ClientBuilder{}
        .baseUrl(m_params.arbiter.server_url)
        .apiKey(m_params.arbiter.api_key)
        .model(m_params.arbiter.model)
        // OpenRouter app attribution: the referer URL is the app identity, the title its name.
        .header("HTTP-Referer",
                "https://github.com/fzi-forschungszentrum-informatik/semantic_scene_forests/tree/"
                "main/semantic_scene_forests_object_tracker")
        .header("X-OpenRouter-Title", "semantic_scene_forests_object_tracker")
        .extraBody(extra_body)
        .connectTimeout(std::chrono::seconds{5})
        .requestTimeout(std::chrono::seconds{m_params.arbiter.request_timeout_seconds})
        .build());
  }

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

bool ObjectTrackerComponent::isRecent(const TrackedObject& object,
                                      const rclcpp::Time& current_time) const
{
  return current_time - object.last_seen <=
         rclcpp::Duration::from_seconds(m_params.reacquisition_timeout);
}

std::optional<detail::TrackPair>
ObjectTrackerComponent::reconcileStaleTracks(const rclcpp::Time& current_time)
{
  // A stale track (confirmed, not seen for reacquisition_timeout) meets a confirmed recent
  // track at its pose: a similar object reactivates the stale identity, anything else
  // replaces it. With the arbiter, replacement is decided from the crops instead.
  std::optional<detail::TrackPair> to_arbitrate;
  std::vector<detail::TrackPair> reactivate; // {stale id, recent id}
  std::set<uint64_t> retired;
  std::set<uint64_t> used;
  for (const auto& stale : m_tracked_objects)
  {
    if (!stale->confirmed || isRecent(*stale, current_time))
    {
      continue;
    }
    const TrackedObject* best = nullptr;
    double best_cover         = 0.0;
    for (const auto& recent : m_tracked_objects)
    {
      if (!recent->confirmed || !isRecent(*recent, current_time) ||
          used.contains(recent->track_id) || recent->last_seen <= stale->last_seen)
      {
        continue;
      }
      // Fraction of the stale box occupied by the recent one.
      const double COVER = TrackedObject::coveredFraction(recent->getPosition(),
                                                          recent->bbox3d_size,
                                                          stale->getPosition(),
                                                          stale->bbox3d_size,
                                                          m_params.overlap_min_extent);
      if (COVER > best_cover)
      {
        best_cover = COVER;
        best       = recent.get();
      }
    }
    if (!best)
    {
      continue;
    }
    const double OVERLAP =
      TrackedObject::overlapOfSmaller(*stale, *best, m_params.overlap_min_extent);
    const auto PAIR = std::minmax(stale->track_id, best->track_id);
    if (TrackedObject::sizesCompatible(
          stale->bbox3d_size, best->bbox3d_size, m_params.association_max_size_ratio) &&
        OVERLAP >= m_params.merge_overlap_threshold)
    {
      reactivate.emplace_back(stale->track_id, best->track_id);
      used.insert(best->track_id);
    }
    else if (m_arbiter_client && !m_arbitrated_pairs.contains(PAIR) &&
             OVERLAP > m_params.arbiter.min_overlap)
    {
      // Something is at the stale pose but geometry can't say what: the arbiter decides.
      if (!to_arbitrate)
      {
        to_arbitrate = detail::TrackPair{stale->track_id, best->track_id};
      }
    }
    else if (best_cover >= m_params.replacement_overlap_threshold)
    {
      // Without an arbiter answer (disabled, or the request failed) replace on coverage.
      retired.insert(stale->track_id);
      RCLCPP_INFO(this->get_logger(),
                  "Replacing stale track %lu with track %lu",
                  stale->track_id,
                  best->track_id);
    }
  }

  for (const auto& [stale_id, recent_id] : reactivate)
  {
    const auto INDEX = [this](uint64_t id) {
      return static_cast<size_t>(
        std::ranges::find_if(m_tracked_objects,
                             [id](const auto& obj) { return obj->track_id == id; }) -
        m_tracked_objects.begin());
    };
    RCLCPP_DEBUG(this->get_logger(), "Reactivating stale track %lu with %lu", stale_id, recent_id);
    mergeTrackPair(INDEX(stale_id), INDEX(recent_id));
  }
  std::erase_if(m_tracked_objects,
                [&retired](const auto& object) { return retired.contains(object->track_id); });
  return to_arbitrate;
}

void ObjectTrackerComponent::publishSnapshot()
{
  // unique_ptr lets intra-process delivery move the snapshot (crops included) without a copy.
  auto snapshot_ptr =
    std::make_unique<semantic_scene_forests_object_msgs::msg::ObjectCandidateArray>();
  auto& snapshot           = *snapshot_ptr;
  snapshot.header.stamp    = this->now();
  snapshot.header.frame_id = m_world_frame;
  for (const auto& obj : m_tracked_objects)
  {
    if (!obj->confirmed)
    {
      continue;
    }
    semantic_scene_forests_object_msgs::msg::ObjectCandidate candidate;
    candidate.header      = snapshot.header;
    candidate.id          = obj->track_id;
    candidate.class_name  = obj->class_name;
    candidate.detection   = obj->toDetection3D();
    candidate.last_seen   = candidate.detection.header.stamp;
    candidate.image_crops = obj->getCrops();
    snapshot.objects.push_back(std::move(candidate));
  }
  m_candidates_pub->publish(std::move(snapshot_ptr));
}

void ObjectTrackerComponent::mergeOverlappingTracks(const rclcpp::Time& current_time)
{
  const double THRESHOLD = m_params.merge_overlap_threshold;
  std::set<size_t> removed;

  for (size_t i = 0; i < m_tracked_objects.size(); ++i)
  {
    auto& tracked_object = m_tracked_objects[i];

    if (removed.contains(i) || !tracked_object->confirmed ||
        !isRecent(*tracked_object, current_time))
    {
      continue;
    }

    for (size_t j = i + 1; j < m_tracked_objects.size(); ++j)
    {
      auto& other_tracked_object = m_tracked_objects[j];
      if (removed.contains(j) || !other_tracked_object->confirmed ||
          !isRecent(*other_tracked_object, current_time))
      {
        continue;
      }

      // Class-agnostic: clearly overlapping boxes are one object; mergeFrom joins
      // the label histories and the vote picks the label.
      if (TrackedObject::sizesCompatible(tracked_object->bbox3d_size,
                                         other_tracked_object->bbox3d_size,
                                         m_params.association_max_size_ratio) &&
          TrackedObject::overlapOfSmaller(
            *tracked_object, *other_tracked_object, m_params.overlap_min_extent) >= THRESHOLD)
      {
        if (tracked_object->track_id < other_tracked_object->track_id)
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

void ObjectTrackerComponent::mergeTrackPair(size_t i, size_t j)
{
  // Same survivor rule as mergeOverlappingTracks: the older identity is kept.
  if (m_tracked_objects[i]->track_id > m_tracked_objects[j]->track_id)
  {
    std::swap(i, j);
  }
  m_tracked_objects[i]->mergeFrom(*m_tracked_objects[j]);
  m_tracked_objects.erase(m_tracked_objects.begin() + static_cast<std::ptrdiff_t>(j));
}

void ObjectTrackerComponent::applyArbiterResult(const rclcpp::Time& current_time)
{
  std::optional<ArbiterResult> result;
  {
    const std::scoped_lock LOCK{m_arbiter_mutex};
    result = std::exchange(m_arbiter_result, std::nullopt);
  }
  if (!result)
  {
    return;
  }
  // Every asked pair is memoized, whatever the outcome: arbitration is a last resort.
  m_arbitrated_pairs.insert(std::minmax(result->a_id, result->b_id));
  if (!result->same_probability)
  {
    m_arbiter_errors.increment(result->error);
    return;
  }
  ++m_identity_decisions;

  const auto FIND = [this](uint64_t id) {
    return std::ranges::find_if(m_tracked_objects,
                                [id](const auto& obj) { return obj->track_id == id; });
  };
  const auto A    = FIND(result->a_id);
  const auto B    = FIND(result->b_id);
  const bool SAME = *result->same_probability >= m_params.arbiter.same_object_probability;
  RCLCPP_INFO(this->get_logger(),
              "Identity decision for %s tracks %lu/%lu: p(same)=%.2f label='%s' -> %s",
              result->stale_replacement ? "stale/recent" : "overlapping",
              result->a_id,
              result->b_id,
              *result->same_probability,
              result->label.c_str(),
              SAME ? "merge" : (result->stale_replacement ? "replace" : "keep"));
  if (!SAME && result->stale_replacement && A != m_tracked_objects.end() &&
      !isRecent(**A, current_time))
  {
    m_tracked_objects.erase(A); // Something else now occupies the stale track's pose.
    ++m_identity_replacements;
    return;
  }
  if (!SAME || A == m_tracked_objects.end() || B == m_tracked_objects.end())
  {
    return;
  }
  const auto I           = static_cast<size_t>(A - m_tracked_objects.begin());
  const auto J           = static_cast<size_t>(B - m_tracked_objects.begin());
  const auto SURVIVOR_ID = std::min(result->a_id, result->b_id);
  mergeTrackPair(I, J);
  if (!result->label.empty())
  {
    (*FIND(SURVIVOR_ID))->setDecidedLabel(result->label);
  }
  ++m_identity_merges;
}

void ObjectTrackerComponent::startArbiter(const rclcpp::Time& current_time,
                                          const std::optional<detail::TrackPair>& replacement)
{
  const auto NOW = std::chrono::steady_clock::now();
  if (m_last_arbiter_start &&
      NOW - *m_last_arbiter_start <
        std::chrono::duration<double>(m_params.arbiter.min_interval_seconds))
  {
    return;
  }
  {
    const std::scoped_lock LOCK{m_arbiter_mutex};
    if (m_arbiter_active || m_arbiter_result)
    {
      return; // Reject while busy; no queue.
    }
  }

  // Stale replacements first; otherwise an ambiguous partial overlap of recent tracks.
  std::optional<std::pair<size_t, size_t>> pair;
  if (replacement)
  {
    const auto INDEX = [this](uint64_t id) {
      return static_cast<size_t>(
        std::ranges::find_if(m_tracked_objects,
                             [id](const auto& obj) { return obj->track_id == id; }) -
        m_tracked_objects.begin());
    };
    pair = std::pair{INDEX(replacement->first), INDEX(replacement->second)};
  }
  else
  {
    pair = detail::findAmbiguousPair(
      m_tracked_objects,
      [this, &current_time](const TrackedObject& obj) {
        return obj.confirmed && isRecent(obj, current_time);
      },
      m_params.arbiter.min_overlap,
      m_params.merge_overlap_threshold,
      m_params.association_max_size_ratio,
      m_params.overlap_min_extent,
      m_arbitrated_pairs);
  }
  if (!pair)
  {
    return;
  }
  const auto& first  = *m_tracked_objects[pair->first];
  const auto& second = *m_tracked_objects[pair->second];
  ArbiterRequest request{.stale_replacement = replacement.has_value(),
                         .a_id              = first.track_id,
                         .b_id              = second.track_id,
                         .a_label           = first.class_name,
                         .b_label           = second.class_name,
                         .a_crops           = first.getCrops(),
                         .b_crops           = second.getCrops(),
                         .a_size            = first.bbox3d_size,
                         .b_size            = second.bbox3d_size,
                         .center_distance   = (first.getPosition() - second.getPosition()).norm()};

  {
    const std::scoped_lock LOCK{m_arbiter_mutex};
    m_arbiter_active = true;
  }
  m_last_arbiter_start = NOW;
  // The previous worker has published its result and is finishing; join it before reuse.
  m_arbiter_worker =
    std::jthread([this, request = std::move(request), model = m_params.arbiter.model](
                   const std::stop_token& stop) {
      std::optional<ArbiterResult> result;
      if (!stop.stop_requested())
      {
        result = runArbiter(*m_arbiter_client, model, request);
      }
      const std::scoped_lock LOCK{m_arbiter_mutex};
      m_arbiter_active = false;
      if (!stop.stop_requested())
      {
        m_arbiter_result = std::move(result);
      }
    });
}

ObjectTrackerComponent::ArbiterResult ObjectTrackerComponent::runArbiter(
  const oai::OpenAIClient& client, const std::string& model, const ArbiterRequest& request)
{
  ArbiterResult result{
    .stale_replacement = request.stale_replacement, .a_id = request.a_id, .b_id = request.b_id};

  auto builder      = oai::MessageBuilder::user();
  const auto IMAGES = [&builder](const std::string& title,
                                 const std::vector<sensor_msgs::msg::Image>& crops) {
    builder.text(title);
    for (const auto& crop : crops)
    {
      builder.image(cv_bridge::toCvCopy(crop, "bgr8")->image);
    }
  };
  const auto SIZE = [](const Eigen::Vector3d& v) { return nlohmann::json{v.x(), v.y(), v.z()}; };
  try
  {
    IMAGES("Track A image crops:", request.a_crops);
    IMAGES("Track B image crops:", request.b_crops);
  }
  catch (const cv_bridge::Exception& e)
  {
    result.error = std::string("crop conversion: ") + e.what();
    return result;
  }
  builder.json(
    {{"track_a", {{"detector_label", request.a_label}, {"size_m", SIZE(request.a_size)}}},
     {"track_b", {{"detector_label", request.b_label}, {"size_m", SIZE(request.b_size)}}},
     {"center_distance_m", request.center_distance}});
  auto message = builder.build();
  if (!message)
  {
    result.error = "message build: " + message.error();
    return result;
  }

  oai::DecisionRequest decision;
  decision.format = oai::DecisionFormat::OPENROUTER;
  decision.model  = model;
  decision.input  = std::vector<oai::Message>{std::move(*message)};
  decision.questions.emplace_back(oai::PredicateQuestion{
    .name           = "same_object",
    .instructions   = request.stale_replacement
                        ? "Track A was seen earlier at a place where track B is seen now. Do the "
                          "image crops of track A and track B show the same physical object, or "
                          "was A replaced by something else? Detector labels are often wrong; "
                          "judge by appearance."
                        : "Two tracks from a robot's object tracker have partially overlapping 3D "
                          "boxes. Do the image crops of track A and track B show the same physical "
                          "object? Detector labels are often wrong; judge by appearance.",
    .true_criteria  = "A and B show the same physical object instance.",
    .false_criteria = "A and B show different physical objects."});
  const bool ASK_LABEL = request.a_label != request.b_label;
  if (ASK_LABEL)
  {
    decision.questions.emplace_back(oai::ChoiceQuestion{
      .name         = "label",
      .instructions = "If they are the same object, which label names it better?",
      .choices      = {{.value = request.a_label, .description = "label of track A"},
                       {.value = request.b_label, .description = "label of track B"}}});
  }

  auto response = client.decide(decision);
  if (!response)
  {
    result.error = response.error().message;
    return result;
  }
  const auto& answers = response->answers;
  if (answers.empty() || answers[0].type != "predicate" || !answers[0].probability)
  {
    result.error = answers.empty() ? "no answers" : "unusable answer: " + answers[0].type;
    return result;
  }
  result.same_probability = answers[0].probability;
  // An invalid or missing choice leaves the label empty: the vote stays in charge.
  const auto CHOICE =
    ASK_LABEL && answers.size() > 1 ? answers[1].choice.value_or("") : request.a_label;
  if (CHOICE == request.a_label || CHOICE == request.b_label)
  {
    result.label = CHOICE;
  }
  return result;
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

  const rclcpp::Time current_time(det3d_msg->header.stamp);

  // shortcut: source clocks must be monotonic; restart with the manager after replay time resets.
  if (camera.last_update_time && current_time <= *camera.last_update_time)
  {
    ++camera.rejected_timestamps;
    return;
  }

  // Validate the synchronized image before prediction or any observation-state mutation.
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

  std::vector<DetectionAdapter> detections;
  detections.reserve(transformed_detections.size());
  for (const auto& trans_det : transformed_detections)
  {
    const CombinedDetection COMBINED{.det2d = trans_det.det2d, .det3d = &trans_det.det3d};
    detections.push_back(DetectionAdapter{.class_name = COMBINED.className(),
                                          .class_id   = 0,
                                          .score      = COMBINED.score(),
                                          .bbox3d     = COMBINED.bbox3D(),
                                          .bbox2d     = COMBINED.bbox2D(),
                                          .frame_id   = m_world_frame});
  }

  // YOLOE often reports one object several times per frame under different labels. Keep the
  // highest-scoring detection of each similar-sized, clearly overlapping group.
  std::ranges::sort(detections, std::greater{}, &DetectionAdapter::score);
  const auto BOX = [](const DetectionAdapter& det) {
    const auto& b = det.bbox3d;
    return std::pair{Eigen::Vector3d(b.center.position.x, b.center.position.y, b.center.position.z),
                     Eigen::Vector3d(b.size.x, b.size.y, b.size.z)};
  };
  std::vector<DetectionAdapter> kept_detections;
  for (const auto& det : detections)
  {
    const auto [center, size] = BOX(det);
    const bool DUPLICATE      = std::ranges::any_of(kept_detections, [&](const auto& other) {
      const auto [other_center, other_size] = BOX(other);
      const double OVERLAP =
        std::max(TrackedObject::coveredFraction(
                   center, size, other_center, other_size, m_params.overlap_min_extent),
                 TrackedObject::coveredFraction(
                   other_center, other_size, center, size, m_params.overlap_min_extent));
      return TrackedObject::sizesCompatible(
               size, other_size, m_params.association_max_size_ratio) &&
             OVERLAP >= m_params.merge_overlap_threshold;
    });
    if (!DUPLICATE)
    {
      kept_detections.push_back(det);
    }
  }
  detections = std::move(kept_detections);

  camera.last_update_time  = current_time;
  const auto TRACKING_TIME = std::max(m_last_update_time.value_or(current_time), current_time);

  const double dt = m_last_update_time ? (TRACKING_TIME - *m_last_update_time).seconds() : 0.0;
  // Tentative inactivity follows tracking time so delayed cameras can still confirm tracks.
  std::erase_if(m_tracked_objects, [this, dt](const auto& obj) {
    return !obj->confirmed && obj->time_since_update + dt > m_params.unconfirmed_track_timeout;
  });
  for (auto& obj : m_tracked_objects)
  {
    // Staleness is decided BEFORE matching. Remembered objects do not grow uncertainty.
    if (!obj->confirmed || isRecent(*obj, TRACKING_TIME))
    {
      obj->predict(dt);
    }
  }

  std::vector<int> det_to_track(detections.size(), -1);
  std::set<size_t> obj_matched;

  // Labels flicker, so association is class-agnostic: geometry (distance, Mahalanobis
  // gate, comparable size) decides identity and the label is voted per track.
  const auto ASSOCIATE = [&](bool confirmed_stage, double gate) {
    for (size_t d = 0; d < detections.size(); ++d)
    {
      if (det_to_track[d] >= 0)
      {
        continue;
      }
      const auto& det    = detections[d];
      const auto& center = det.bbox3d.center.position;
      const Eigen::Vector3d DET_SIZE(det.bbox3d.size.x, det.bbox3d.size.y, det.bbox3d.size.z);

      double best_dist = std::numeric_limits<double>::max();
      int best_idx     = -1;
      for (size_t o = 0; o < m_tracked_objects.size(); ++o)
      {
        auto& obj = m_tracked_objects[o];
        if (obj->confirmed != confirmed_stage || obj_matched.contains(o) ||
            (confirmed_stage && !isRecent(*obj, current_time)))
        {
          continue;
        }
        if ((Eigen::Vector3d(center.x, center.y, center.z) - obj->getPosition()).norm() >
              m_params.max_association_distance ||
            !TrackedObject::sizesCompatible(
              DET_SIZE, obj->bbox3d_size, m_params.association_max_size_ratio))
        {
          continue;
        }
        const double DIST = obj->mahalanobisDistance(det);
        if (DIST < gate && DIST < best_dist)
        {
          best_dist = DIST;
          best_idx  = static_cast<int>(o);
        }
      }

      if (best_idx >= 0)
      {
        auto& object = *m_tracked_objects[best_idx];
        object.update(det, current_time);
        det_to_track[d] = best_idx;
        obj_matched.insert(best_idx);
      }
    }
  };
  ASSOCIATE(true, m_params.gate_threshold_confirmed); // Stage 1: confirmed tracks (relaxed gate)
  ASSOCIATE(false, m_params.gate_threshold_strict);   // Stage 2: unconfirmed tracks (strict gate)

  // Remembered storage lasts for this session and may grow without positive replacement
  // evidence. Never evict remembered identities merely to admit an active track.
  auto active_count = std::count_if(
    m_tracked_objects.begin(), m_tracked_objects.end(), [this, TRACKING_TIME](const auto& obj) {
      return !obj->confirmed || isRecent(*obj, TRACKING_TIME);
    });
  // Create new tracks for unmatched detections
  for (size_t d = 0; d < detections.size(); ++d)
  {
    if (det_to_track[d] >= 0)
    {
      continue;
    }
    if (active_count >= m_params.max_trackable_objects)
    {
      break;
    }

    m_tracked_objects.push_back(
      std::make_unique<TrackedObject>(detections[d],
                                      current_time,
                                      m_next_track_id,
                                      m_params.measurement_noise_std,
                                      m_params.process_noise_std,
                                      m_params.confirmation_threshold,
                                      static_cast<size_t>(m_params.max_crops_per_object),
                                      m_params.bbox_size_alpha));
    det_to_track[d] = static_cast<int>(m_tracked_objects.size() - 1);
    ++active_count;
    m_next_track_id++;
  }

  // Crop images for observed confirmed tracks. This must run before replacement
  // and merging: det_to_track stores indices into the
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

    auto crop = cropImagePatch(cv_image->image, detections[d].bbox2d, m_params.crop_padding);
    if (crop)
    {
      crop->header = img_msg->header;
      obj->addCrop(*crop);
    }
  }

  if (m_arbiter_client)
  {
    applyArbiterResult(TRACKING_TIME);
  }
  const auto REPLACEMENT = reconcileStaleTracks(TRACKING_TIME);
  mergeOverlappingTracks(TRACKING_TIME);
  if (m_arbiter_client)
  {
    startArbiter(TRACKING_TIME, REPLACEMENT);
  }
  if (!m_snapshot_timer)
  {
    publishSnapshot();
  }

  m_last_update_time = TRACKING_TIME;
}

void ObjectTrackerComponent::onHealthDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  size_t confirmed = 0;
  size_t recent    = 0;
  for (const auto& obj : m_tracked_objects)
  {
    if (obj->confirmed)
    {
      ++confirmed;
      if (m_last_update_time && isRecent(*obj, *m_last_update_time))
      {
        ++recent;
      }
    }
  }

  stat.add("Total tracks", m_tracked_objects.size());
  stat.add("Confirmed tracks", confirmed);
  stat.add("Recent tracks", recent);
  stat.add("Remembered tracks", confirmed - recent);
  size_t tf_errors = 0;
  for (const auto& camera : m_cameras)
  {
    camera->tf_errors.report(stat, "TF (" + camera->name + ")");
    stat.add("Rejected timestamps (" + camera->name + ")", camera->rejected_timestamps);
    tf_errors += camera->tf_errors.count();
  }
  m_crop_errors.report(stat, "Crop");
  if (m_arbiter_client)
  {
    stat.add("Identity decisions", m_identity_decisions);
    stat.add("Identity merges", m_identity_merges);
    stat.add("Identity replacements", m_identity_replacements);
    m_arbiter_errors.report(stat, "Identity decision");
  }

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
