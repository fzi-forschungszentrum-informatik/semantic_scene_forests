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

#include "arbiter_worker.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <stdexcept>
#include <utility>

#include <cv_bridge/cv_bridge.hpp>
#include <nlohmann/json.hpp>
#include <oai_api/builder.hpp>

namespace semantic_scene_forests_object_tracker {

std::optional<oai::DecisionFormat> parseDecisionProvider(std::string_view provider)
{
  if (provider == "openai")
  {
    return oai::DecisionFormat::OPENAI;
  }
  if (provider == "openrouter")
  {
    return oai::DecisionFormat::OPENROUTER;
  }
  if (provider == "ollama")
  {
    return oai::DecisionFormat::OLLAMA;
  }
  return std::nullopt;
}

DecideFn makeDecisionClient(const object_tracker::Params::Arbiter& params)
{
  const auto FORMAT = parseDecisionProvider(params.provider);
  if (!FORMAT)
  {
    throw std::invalid_argument("arbiter.provider must be openai, openrouter or ollama: " +
                                params.provider);
  }
  // Keys stay out of committed configs: an empty key falls back to the provider's env variable.
  std::string api_key = params.api_key;
  if (api_key.empty() && FORMAT != oai::DecisionFormat::OLLAMA)
  {
    const char* env =
      std::getenv(FORMAT == oai::DecisionFormat::OPENAI ? "OPENAI_API_KEY" : "OPENROUTER_API_KEY");
    api_key = env ? env : "";
  }
  if (api_key.empty() && FORMAT != oai::DecisionFormat::OLLAMA)
  {
    throw std::invalid_argument("arbiter.api_key (or OPENAI_API_KEY / OPENROUTER_API_KEY) is "
                                "required for provider " +
                                params.provider);
  }
  nlohmann::json extra_body;
  if (!params.extra_body.empty())
  {
    extra_body = nlohmann::json::parse(params.extra_body, nullptr, false);
    if (!extra_body.is_object())
    {
      throw std::invalid_argument("arbiter.extra_body must be a JSON object: " + params.extra_body);
    }
  }
  oai::ClientBuilder builder;
  builder.baseUrl(params.server_url)
    .apiKey(api_key)
    .model(params.model)
    .extraBody(extra_body)
    .connectTimeout(std::chrono::seconds{5})
    .requestTimeout(std::chrono::seconds{params.request_timeout_seconds});
  if (FORMAT == oai::DecisionFormat::OPENROUTER)
  {
    // OpenRouter app attribution: the referer URL is the app identity, the title its name.
    builder
      .header("HTTP-Referer",
              "https://github.com/fzi-forschungszentrum-informatik/semantic_scene_forests/tree/"
              "main/semantic_scene_forests_object_tracker")
      .header("X-OpenRouter-Title", "semantic_scene_forests_object_tracker");
  }
  auto client = std::make_shared<const oai::OpenAIClient>(builder.build());
  return [client](const oai::DecisionRequest& request) { return client->decide(request); };
}

ArbiterWorker::ArbiterWorker(DecideFn decide, oai::DecisionFormat format, std::string model)
  : m_decide(std::move(decide))
  , m_format(format)
  , m_model(std::move(model))
  , m_thread([this](const std::stop_token& stop) { run(stop); })
{
}

ArbiterWorker::~ArbiterWorker()
{
  m_thread.request_stop();
  m_wake.notify_all();
}

void ArbiterWorker::enqueue(ArbiterRequest request)
{
  {
    const std::scoped_lock LOCK{m_mutex};
    const bool PRIORITY = request.label_check || request.stale_replacement;
    (PRIORITY ? m_priority_queue : m_queue).push_back(std::move(request));
  }
  m_wake.notify_one();
}

std::vector<ArbiterResult> ArbiterWorker::drainResults()
{
  const std::scoped_lock LOCK{m_mutex};
  return std::exchange(m_results, {});
}

size_t ArbiterWorker::pending() const
{
  const std::scoped_lock LOCK{m_mutex};
  return m_priority_queue.size() + m_queue.size() + (m_busy ? 1 : 0);
}

void ArbiterWorker::run(const std::stop_token& stop)
{
  while (true)
  {
    ArbiterRequest request;
    {
      std::unique_lock lock{m_mutex};
      if (!m_wake.wait(
            lock, stop, [this] { return !m_priority_queue.empty() || !m_queue.empty(); }))
      {
        return; // Stop requested.
      }
      auto& queue = m_priority_queue.empty() ? m_queue : m_priority_queue;
      request     = std::move(queue.front());
      queue.pop_front();
      m_busy = true;
    }
    auto result = decide(m_decide, m_format, m_model, request);
    const std::scoped_lock LOCK{m_mutex};
    m_busy = false;
    if (stop.stop_requested())
    {
      return;
    }
    m_results.push_back(std::move(result));
  }
}

ArbiterResult ArbiterWorker::decide(const DecideFn& decide,
                                    oai::DecisionFormat format,
                                    const std::string& model,
                                    const ArbiterRequest& request)
{
  ArbiterResult result{.label_check       = request.label_check,
                       .stale_replacement = request.stale_replacement,
                       .a_id              = request.a_id,
                       .b_id              = request.b_id};

  // Crops go into the shared input as base64 images (required by Ollama).
  auto builder      = oai::MessageBuilder::user();
  const auto IMAGES = [&builder](const std::string& title, const auto& crops) {
    builder.text(title);
    if (!crops)
    {
      return;
    }
    for (const auto& crop : *crops)
    {
      builder.image(cv_bridge::toCvCopy(crop, "bgr8")->image);
    }
  };
  const auto SIZE = [](const Eigen::Vector3d& v) { return nlohmann::json{v.x(), v.y(), v.z()}; };
  try
  {
    IMAGES(request.label_check ? "Image crops of the object:" : "Track A image crops:",
           request.a_crops);
    if (!request.label_check)
    {
      IMAGES("Track B image crops:", request.b_crops);
    }
  }
  catch (const cv_bridge::Exception& e)
  {
    result.error = std::string("crop conversion: ") + e.what();
    return result;
  }
  if (request.label_check)
  {
    builder.json({{"detector_label", request.a_label}, {"size_m", SIZE(request.a_size)}});
  }
  else
  {
    builder.json(
      {{"track_a", {{"detector_label", request.a_label}, {"size_m", SIZE(request.a_size)}}},
       {"track_b", {{"detector_label", request.b_label}, {"size_m", SIZE(request.b_size)}}},
       {"center_distance_m", request.center_distance}});
  }
  auto message = builder.build();
  if (!message)
  {
    result.error = "message build: " + message.error();
    return result;
  }

  oai::DecisionRequest decision;
  decision.format = format;
  decision.model  = model;
  decision.input  = std::vector<oai::Message>{std::move(*message)};
  if (request.label_check)
  {
    decision.questions.emplace_back(oai::PredicateQuestion{
      .name         = "label_correct",
      .instructions = "An object detector labeled this object '" + request.a_label +
                      "'. Do the image crops show a real " + request.a_label +
                      "? Detector labels are often wrong or describe the scene instead of an "
                      "object.",
      .true_criteria  = "The crops show a " + request.a_label + ".",
      .false_criteria = "The crops show something else, or no real object."});
    // YOLOE often labels whole scenes with room categories ("kitchen", "office").
    decision.questions.emplace_back(oai::PredicateQuestion{
      .name           = "physical_object",
      .instructions   = "Is '" + request.a_label +
                      "' a single physical object (furniture, appliance, device, item), as "
                      "opposed to a room, area or scene category such as kitchen, office, "
                      "hallway, corridor, floor or wall?",
      .true_criteria  = "It is a discrete physical object that could be picked up, moved or "
                        "used.",
      .false_criteria = "It is a room, area, scene or building part."});
  }
  else
  {
    decision.questions.emplace_back(oai::PredicateQuestion{
      .name           = "same_object",
      .instructions   = request.stale_replacement
                          ? "Track A was seen earlier at a place where track B is seen now. Do the "
                            "image crops of track A and track B show the same physical object, or "
                            "was A replaced by something else? Detector labels are often wrong; "
                            "judge by appearance."
                          : "Two tracks from a robot's object tracker overlap in 3D. Do the image "
                            "crops of track A and track B show the same physical object? Detector "
                            "labels are often wrong; judge by appearance.",
      .true_criteria  = "A and B show the same physical object instance.",
      .false_criteria = "A and B show different physical objects."});
  }
  const bool ASK_LABEL = !request.label_check && request.a_label != request.b_label;
  if (ASK_LABEL)
  {
    decision.questions.emplace_back(oai::ChoiceQuestion{
      .name         = "label",
      .instructions = "If they are the same object, which label names it better?",
      .choices      = {{.value = request.a_label, .description = "label of track A"},
                       {.value = request.b_label, .description = "label of track B"}}});
  }

  auto response = decide(decision);
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
  if (request.label_check)
  {
    if (answers.size() < 2 || answers[1].type != "predicate" || !answers[1].probability)
    {
      result.same_probability.reset();
      result.error = "unusable physical_object answer";
      return result;
    }
    result.object_probability = answers[1].probability;
    result.same_probability   = std::min(*answers[0].probability, *answers[1].probability);
  }
  // An invalid or missing choice leaves the label empty: the vote stays in charge.
  const auto CHOICE =
    ASK_LABEL && answers.size() > 1 ? answers[1].choice.value_or("") : request.a_label;
  if (CHOICE == request.a_label || CHOICE == request.b_label)
  {
    result.label = CHOICE;
  }
  return result;
}

} // namespace semantic_scene_forests_object_tracker
