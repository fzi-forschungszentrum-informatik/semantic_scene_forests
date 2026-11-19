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

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <unistd.h>
#include <zip.h>

#include "scene_graph_bundle.hpp"

namespace {

using fzi::semantic_scene_forests::ObjectNode;
using fzi::semantic_scene_forests::SceneGraph;
using semantic_scene_forests_manager::saveSceneGraphBundle;

std::string readEntry(zip_t* archive, const std::string& name)
{
  zip_stat_t stat;
  if (zip_stat(archive, name.c_str(), 0, &stat) != 0)
  {
    return {};
  }
  std::string data(stat.size, '\0');
  zip_file_t* file = zip_fopen(archive, name.c_str(), 0);
  EXPECT_NE(file, nullptr);
  EXPECT_EQ(zip_fread(file, data.data(), stat.size), static_cast<zip_int64_t>(stat.size));
  zip_fclose(file);
  return data;
}

TEST(SceneGraphBundle, SavesGraphAndCrops)
{
  const auto DIR = std::filesystem::temp_directory_path() /
                   ("sg_bundle_test_" + std::to_string(::getpid())) / "nested";

  SceneGraph graph;
  auto crop             = std::make_shared<sensor_msgs::msg::CompressedImage>();
  crop->format          = "jpeg";
  crop->header.frame_id = "cam0";
  crop->data            = {0xFF, 0xD8, 0x01, 0x02, 0xFF, 0xD9};
  auto second           = std::make_shared<sensor_msgs::msg::CompressedImage>(*crop);
  second->data          = {0xFF, 0xD8, 0x03, 0xFF, 0xD9};
  const int64_t OBJECT  = graph.allocateId();
  ASSERT_TRUE(graph.addObject(
    ObjectNode{.id = OBJECT, .class_name = "cup", .frame_id = "map", .crops = {crop, second}}));
  const auto CONCEPT = graph.addConceptEdge(OBJECT, "has_color", "red");
  ASSERT_TRUE(CONCEPT);
  ASSERT_TRUE(graph.markProcessed(OBJECT, "object_attribution"));

  const auto SAVED = saveSceneGraphBundle(graph, "map", DIR);
  ASSERT_TRUE(SAVED) << SAVED.error();
  EXPECT_EQ(SAVED->extension(), ".sg");
  ASSERT_TRUE(std::filesystem::exists(*SAVED));

  int error      = 0;
  zip_t* archive = zip_open(SAVED->c_str(), ZIP_RDONLY, &error);
  ASSERT_NE(archive, nullptr);
  const auto DOC = nlohmann::json::parse(readEntry(archive, "scene_graph.json"));
  EXPECT_EQ(DOC["frame_id"], "map");
  ASSERT_EQ(DOC["objects"].size(), 1U);
  const auto& obj = DOC["objects"][0];
  EXPECT_EQ(obj["id"], OBJECT);
  EXPECT_EQ(obj["class_name"], "cup");
  EXPECT_FALSE(obj.contains("workers"));
  ASSERT_EQ(obj["crops"].size(), 2U);
  EXPECT_EQ(obj["crops"][0]["frame_id"], "cam0");
  ASSERT_EQ(DOC["concepts"].size(), 1U);
  EXPECT_EQ(DOC["concepts"][0]["id"], CONCEPT->id);
  EXPECT_EQ(DOC["concepts"][0]["name"], "red");
  ASSERT_EQ(DOC["edges"].size(), 1U);
  EXPECT_EQ(DOC["edges"][0]["source"], OBJECT);
  EXPECT_EQ(DOC["edges"][0]["target"], CONCEPT->id);
  EXPECT_EQ(DOC["edges"][0]["label"], "has_color");

  const std::string CROP0 = readEntry(archive, obj["crops"][0]["file"].get<std::string>());
  EXPECT_EQ(std::vector<uint8_t>(CROP0.begin(), CROP0.end()), crop->data);
  zip_close(archive);

  std::filesystem::remove_all(DIR.parent_path());
}

} // namespace
