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

#include "scene_graph_bundle.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <ctime>
#include <system_error>
#include <utility>

#include <fmt/format.h>
#include <nlohmann/json.hpp>
#include <zip.h>

namespace semantic_scene_forests_manager {

namespace {

using json = nlohmann::ordered_json;

json toJson(const builtin_interfaces::msg::Time& time)
{
  return {{"sec", time.sec}, {"nanosec", time.nanosec}};
}

json toJson(const vision_msgs::msg::BoundingBox3D& bbox)
{
  const auto& p = bbox.center.position;
  const auto& q = bbox.center.orientation;
  const auto& s = bbox.size;
  return {{"center",
           {{"position", {{"x", p.x}, {"y", p.y}, {"z", p.z}}},
            {"orientation", {{"x", q.x}, {"y", q.y}, {"z", q.z}, {"w", q.w}}}}},
          {"size", {{"x", s.x}, {"y", s.y}, {"z", s.z}}}};
}

std::string bundleName()
{
  const auto NOW            = std::chrono::system_clock::now();
  const std::time_t SECONDS = std::chrono::system_clock::to_time_t(NOW);
  std::tm local{};
  localtime_r(&SECONDS, &local);
  std::array<char, 32> stamp{};
  std::strftime(stamp.data(), stamp.size(), "%Y%m%d-%H%M%S", &local);
  const auto MILLIS =
    std::chrono::duration_cast<std::chrono::milliseconds>(NOW.time_since_epoch()).count() % 1000;
  return fmt::format("scene_graph_{}-{:03d}.sg", stamp.data(), MILLIS);
}

} // namespace

std::expected<std::filesystem::path, std::string>
saveSceneGraphBundle(const fzi::semantic_scene_forests::SceneGraph& graph,
                     const std::string& frame_id,
                     const std::filesystem::path& dir)
{
  // One locked copy, so objects, concepts and edges are mutually consistent.
  auto snapshot = graph.clone();
  if (!snapshot)
  {
    return std::unexpected("Failed to copy the graph: " + snapshot.error().message);
  }
  // Holds the crop buffers the archive references until zip_close.
  const auto OBJECTS = (*snapshot)->getObjectNodes();

  json doc{{"format_version", 1},
           {"frame_id", frame_id},
           {"objects", json::array()},
           {"concepts", json::array()},
           {"edges", json::array()}};
  // (archive name, crop) pairs, added after the JSON.
  std::vector<std::pair<std::string, const fzi::semantic_scene_forests::CropPtr*>> crop_files;
  for (const auto& object : OBJECTS)
  {
    json crops = json::array();
    for (std::size_t i = 0; i < object.crops.size(); ++i)
    {
      const auto& crop = object.crops[i];
      auto name        = fmt::format("crops/{}/{}.jpg", object.id, i);
      crops.push_back({{"file", name},
                       {"frame_id", crop->header.frame_id},
                       {"stamp", toJson(crop->header.stamp)}});
      crop_files.emplace_back(std::move(name), &crop);
    }
    doc["objects"].push_back({{"id", object.id},
                              {"class_name", object.class_name},
                              {"frame_id", object.frame_id},
                              {"source", object.source},
                              {"bbox", toJson(object.bbox)},
                              {"last_seen", toJson(object.last_seen)},
                              {"last_updated", toJson(object.last_updated)},
                              {"crops", std::move(crops)}});
  }
  for (const auto& concept_node : (*snapshot)->getConceptNodes())
  {
    doc["concepts"].push_back({{"id", concept_node.id}, {"name", concept_node.concept_name}});
  }
  for (const auto& [source, target, edge] : (*snapshot)->getEdges())
  {
    doc["edges"].push_back({{"source", source}, {"target", target}, {"label", edge.label}});
  }
  const std::string JSON_TEXT = doc.dump(2);

  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec)
  {
    return std::unexpected("Cannot create " + dir.string() + ": " + ec.message());
  }
  auto path = dir / bundleName();

  // libzip writes to a temporary file and renames it in zip_close, so a failed save never
  // leaves a partial bundle. ZIP_EXCL: never overwrite an existing file.
  int open_error = 0;
  zip_t* archive = zip_open(path.c_str(), ZIP_CREATE | ZIP_EXCL, &open_error);
  if (archive == nullptr)
  {
    zip_error_t error;
    zip_error_init_with_code(&error, open_error);
    std::string message = "Cannot create " + path.string() + ": " + zip_error_strerror(&error);
    zip_error_fini(&error);
    return std::unexpected(std::move(message));
  }
  const auto ADD =
    [archive](const std::string& name, const void* data, std::size_t size, bool store) {
      zip_source_t* source = zip_source_buffer(archive, data, size, 0);
      if (source == nullptr)
      {
        return false;
      }
      const zip_int64_t INDEX = zip_file_add(archive, name.c_str(), source, ZIP_FL_ENC_UTF_8);
      if (INDEX < 0)
      {
        zip_source_free(source);
        return false;
      }
      // JPEGs are already compressed.
      return !store || zip_set_file_compression(archive, INDEX, ZIP_CM_STORE, 0) == 0;
    };

  bool ok = ADD("scene_graph.json", JSON_TEXT.data(), JSON_TEXT.size(), false);
  for (const auto& [name, crop] : crop_files)
  {
    ok = ok && ADD(name, (*crop)->data.data(), (*crop)->data.size(), true);
  }
  if (!ok || zip_close(archive) != 0)
  {
    std::string message = "Failed to write " + path.string() + ": " + zip_strerror(archive);
    zip_discard(archive);
    return std::unexpected(std::move(message));
  }
  return path;
}

} // namespace semantic_scene_forests_manager
