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
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "semantic_scene_forests_manager/database.hpp"

namespace {

using semantic_scene_forests_manager::Database;
using semantic_scene_forests_manager::EditFailure;
using semantic_scene_forests_manager::ObjectRecord;
using semantic_scene_forests_manager::TrackUpdate;
using semantic_scene_forests_manager::Tx;
using semantic_scene_forests_object_msgs::srv::EditObject;
using Time = builtin_interfaces::msg::Time;

sensor_msgs::msg::Image makeCrop(const std::string& frame_id, uint8_t tag)
{
  sensor_msgs::msg::Image crop;
  crop.header.frame_id = frame_id;
  crop.header.stamp    = rclcpp::Time(80, 5);
  crop.height          = 1;
  crop.width           = 1;
  crop.encoding        = "mono8";
  crop.step            = 1;
  crop.data            = {tag};
  return crop;
}

std::vector<sensor_msgs::msg::Image> makeCrops(size_t count)
{
  std::vector<sensor_msgs::msg::Image> crops;
  crops.reserve(count);
  for (size_t i = 0; i < count; ++i)
  {
    crops.push_back(makeCrop("cam_" + std::to_string(i), static_cast<uint8_t>(i)));
  }
  return crops;
}

TrackUpdate makeTrack(int64_t id, const std::string& class_name, std::optional<size_t> crop_count)
{
  TrackUpdate update;
  update.id                                  = id;
  update.class_name                          = class_name;
  update.last_seen                           = rclcpp::Time(90, 123456789);
  update.detection.header.frame_id           = "world";
  update.detection.header.stamp              = update.last_seen;
  update.detection.bbox.center.position.x    = 1.5;
  update.detection.bbox.center.orientation.w = 1.0;
  update.detection.bbox.size.x               = 0.4;
  update.detection.bbox.size.y               = 0.5;
  update.detection.bbox.size.z               = 0.6;
  vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
  hypothesis.hypothesis.class_id = class_name;
  hypothesis.hypothesis.score    = 0.8;
  update.detection.results.push_back(hypothesis);
  if (crop_count)
  {
    update.crops = makeCrops(*crop_count);
  }
  return update;
}

class DatabaseObjectsTest : public ::testing::Test
{
protected:
  // Allocates an id and upserts a "cup" track with the given crops at m_now.
  int64_t addObject(size_t crop_count)
  {
    return m_db.write([&](Tx& tx) {
      const int64_t ID = tx.allocateObjectId();
      tx.upsertTrack(makeTrack(ID, "cup", crop_count), m_now);
      return ID;
    });
  }

  ObjectRecord object(int64_t id)
  {
    auto record = m_db.read([id](const Tx& tx) { return tx.object(id); });
    EXPECT_TRUE(record.has_value());
    return record.value_or(ObjectRecord());
  }

  std::vector<sensor_msgs::msg::Image> crops(int64_t id)
  {
    return m_db.read([id](const Tx& tx) { return tx.crops(id); });
  }

  std::vector<ObjectRecord> visible()
  {
    return m_db.read([](const Tx& tx) { return tx.visibleObjects(); });
  }

  void upsert(const TrackUpdate& update, const Time& now)
  {
    m_db.write([&](Tx& tx) { tx.upsertTrack(update, now); });
  }

  EditFailure edit(const EditObject::Request& request, const Time& now)
  {
    return m_db.write([&](Tx& tx) { return tx.edit(request, now); });
  }

  Database m_db;
  Time m_now   = rclcpp::Time(100, 0);
  Time m_later = rclcpp::Time(110, 0);
};

TEST_F(DatabaseObjectsTest, UpsertCreatesVisibleObject)
{
  const int64_t ID = m_db.write([](Tx& tx) { return tx.allocateObjectId(); });
  EXPECT_GT(ID, 0);
  EXPECT_TRUE(visible().empty());
  EXPECT_FALSE(m_db.read([ID](const Tx& tx) { return tx.object(ID); }).has_value());

  const auto UPDATE = makeTrack(ID, "cup", 2);
  upsert(UPDATE, m_now);
  const auto OBJECTS = visible();
  ASSERT_EQ(OBJECTS.size(), 1U);
  EXPECT_EQ(OBJECTS[0].id, ID);
  EXPECT_EQ(OBJECTS[0].class_name, "cup");
  EXPECT_EQ(OBJECTS[0].detection, UPDATE.detection);
  EXPECT_EQ(OBJECTS[0].last_seen, UPDATE.last_seen);
  EXPECT_EQ(OBJECTS[0].last_updated, m_now);
  EXPECT_EQ(OBJECTS[0].source, 0);
  EXPECT_FALSE(OBJECTS[0].blocked);
  EXPECT_EQ(crops(ID), makeCrops(2));
}

TEST_F(DatabaseObjectsTest, DeletedIdsAreNotReused)
{
  const int64_t FIRST = addObject(0);
  m_db.write([FIRST](Tx& tx) { tx.deleteObject(FIRST); });
  const int64_t SECOND = m_db.write([](Tx& tx) { return tx.allocateObjectId(); });
  EXPECT_GT(SECOND, FIRST);
}

TEST_F(DatabaseObjectsTest, UpsertWithoutChangeKeepsLastUpdated)
{
  const int64_t ID   = addObject(1);
  const auto FIRST   = object(ID);
  const auto CHANGES = m_db.changeCount();

  upsert(makeTrack(ID, "cup", std::nullopt), m_later);
  EXPECT_EQ(object(ID), FIRST);
  EXPECT_EQ(m_db.changeCount(), CHANGES);

  auto observed                   = makeTrack(ID, "cup", std::nullopt);
  observed.last_seen              = rclcpp::Time(105, 987654321);
  observed.detection.header.stamp = observed.last_seen;
  upsert(observed, m_later);
  const auto OBSERVED = object(ID);
  EXPECT_EQ(OBSERVED.last_seen, observed.last_seen);
  EXPECT_EQ(OBSERVED.last_updated, m_later);
  EXPECT_EQ(crops(ID).size(), 1U);
}

TEST_F(DatabaseObjectsTest, ClassOverrideSurvivesUpsert)
{
  const int64_t ID = addObject(1);
  EditObject::Request request;
  request.object_id         = ID;
  request.update_class_name = true;
  request.class_name        = "curated_cup";
  const Time EDIT_TIME      = rclcpp::Time(101, 0);
  ASSERT_EQ(edit(request, EDIT_TIME), EditFailure::SUCCESS);
  auto record = object(ID);
  EXPECT_EQ(record.detection.results[0].hypothesis.class_id, "curated_cup");
  EXPECT_EQ(record.last_updated, EDIT_TIME);

  auto update                                     = makeTrack(ID, "tracker_class", 2);
  update.detection.results[0].hypothesis.class_id = "tracker_class";
  update.detection.results[0].hypothesis.score    = 0.9;
  update.detection.bbox.center.position.x         = 2.5;
  upsert(update, m_later);
  record = object(ID);
  EXPECT_EQ(record.class_name, "curated_cup");
  ASSERT_EQ(record.detection.results.size(), 1U);
  EXPECT_EQ(record.detection.results[0].hypothesis.class_id, "curated_cup");
  EXPECT_DOUBLE_EQ(record.detection.results[0].hypothesis.score, 0.9);
  EXPECT_DOUBLE_EQ(record.detection.bbox.center.position.x, 2.5);
  EXPECT_EQ(record.source, 1);
  EXPECT_EQ(record.last_updated, m_later);
  EXPECT_EQ(crops(ID).size(), 2U);
}

TEST_F(DatabaseObjectsTest, PoseOverrideSurvivesUpsert)
{
  const int64_t ID = addObject(1);
  EditObject::Request request;
  request.object_id          = ID;
  request.update_pose        = true;
  request.pose.position.x    = 9.0;
  request.pose.orientation.z = 0.6;
  request.pose.orientation.w = 0.8;
  ASSERT_EQ(edit(request, rclcpp::Time(101, 0)), EditFailure::SUCCESS);

  auto update                             = makeTrack(ID, "bowl", 2);
  update.detection.bbox.center.position.x = 2.0;
  update.detection.bbox.size.x            = 0.7;
  update.last_seen                        = rclcpp::Time(102, 234567891);
  update.detection.header.stamp           = update.last_seen;
  upsert(update, m_later);
  const auto RECORD = object(ID);
  EXPECT_EQ(RECORD.detection.bbox.center, request.pose);
  EXPECT_DOUBLE_EQ(RECORD.detection.bbox.size.x, 0.7);
  EXPECT_EQ(RECORD.class_name, "bowl");
  EXPECT_EQ(RECORD.last_seen, update.last_seen);
  EXPECT_EQ(RECORD.detection.header, update.detection.header);
  EXPECT_EQ(RECORD.source, 1);
  EXPECT_EQ(RECORD.last_updated, m_later);
}

TEST_F(DatabaseObjectsTest, EditRejectsInvalidCropIndex)
{
  const int64_t ID = addObject(3);

  // Duplicate indices collapse: {1, 1} removes a single crop and keeps the rest in order.
  EditObject::Request request;
  request.object_id           = ID;
  request.remove_crop_indices = {1, 1};
  ASSERT_EQ(edit(request, m_now), EditFailure::SUCCESS);
  const auto CURATED = crops(ID);
  ASSERT_EQ(CURATED.size(), 2U);
  EXPECT_EQ(CURATED[0].header.frame_id, "cam_0");
  EXPECT_EQ(CURATED[1].header.frame_id, "cam_2");

  // An out-of-range index rejects the whole edit: nothing changes.
  const auto BEFORE           = object(ID);
  request.update_class_name   = true;
  request.class_name          = "rejected_class";
  request.update_pose         = true;
  request.pose.position.x     = -1.0;
  request.remove_crop_indices = {0, 99};
  EXPECT_EQ(edit(request, m_later), EditFailure::INVALID_CROP_INDEX);
  EXPECT_EQ(object(ID), BEFORE);
  EXPECT_EQ(crops(ID), CURATED);

  request.object_id = ID + 100;
  EXPECT_EQ(edit(request, m_later), EditFailure::UNKNOWN_OBJECT);
}

TEST_F(DatabaseObjectsTest, CuratedCropsSurviveTrackerCrops)
{
  const int64_t ID = addObject(3);
  EditObject::Request request;
  request.object_id           = ID;
  request.remove_crop_indices = {0, 1, 2};
  ASSERT_EQ(edit(request, m_now), EditFailure::SUCCESS);

  auto update                             = makeTrack(ID, "cup", 2);
  update.detection.bbox.center.position.x = 3.0;
  upsert(update, m_later);
  EXPECT_TRUE(crops(ID).empty());
  EXPECT_DOUBLE_EQ(object(ID).detection.bbox.center.position.x, 3.0);
}

TEST_F(DatabaseObjectsTest, UpsertIgnoredWhileBlocked)
{
  const int64_t ID = addObject(1);
  ASSERT_TRUE(m_db.write([ID](Tx& tx) { return tx.block(ID); }));

  auto update      = makeTrack(ID, "bowl", 2);
  update.last_seen = rclcpp::Time(105, 0);
  upsert(update, m_later);
  const auto RECORD = object(ID);
  EXPECT_TRUE(RECORD.blocked);
  EXPECT_EQ(RECORD.class_name, "cup");
  EXPECT_TRUE(visible().empty());
  EXPECT_TRUE(crops(ID).empty());
}

TEST_F(DatabaseObjectsTest, BlockKeepsRowDropsCrops)
{
  const int64_t ID  = addObject(2);
  const auto BEFORE = object(ID);
  ASSERT_TRUE(m_db.write([ID](Tx& tx) { return tx.block(ID); }));
  const auto RECORD = object(ID);
  EXPECT_TRUE(RECORD.blocked);
  EXPECT_EQ(RECORD.detection, BEFORE.detection);
  EXPECT_EQ(RECORD.class_name, BEFORE.class_name);
  EXPECT_TRUE(crops(ID).empty());
  EXPECT_FALSE(m_db.write([ID](Tx& tx) { return tx.block(ID + 100); }));
  const auto COUNTS = m_db.read([](const Tx& tx) { return tx.counts(); });
  EXPECT_EQ(COUNTS.objects, 1);
  EXPECT_EQ(COUNTS.blocked_objects, 1);
}

TEST_F(DatabaseObjectsTest, UnblockRestoresVisibility)
{
  const int64_t ID = addObject(1);
  EditObject::Request request;
  request.object_id         = ID;
  request.update_class_name = true;
  request.class_name        = "curated_cup";
  ASSERT_EQ(edit(request, m_now), EditFailure::SUCCESS);
  ASSERT_TRUE(m_db.write([ID](Tx& tx) { return tx.block(ID); }));
  EXPECT_EQ(edit(request, m_now), EditFailure::UNKNOWN_OBJECT);

  EXPECT_TRUE(m_db.write([ID](Tx& tx) { return tx.unblock(ID); }));
  EXPECT_FALSE(m_db.write([ID](Tx& tx) { return tx.unblock(ID); }));
  const auto OBJECTS = visible();
  ASSERT_EQ(OBJECTS.size(), 1U);
  EXPECT_EQ(OBJECTS[0].class_name, "curated_cup");

  // Crops come back as the tracker captures new ones.
  upsert(makeTrack(ID, "cup", 2), m_later);
  EXPECT_EQ(crops(ID).size(), 2U);
}

TEST_F(DatabaseObjectsTest, DeleteObjectCascadesCrops)
{
  const int64_t ID = addObject(2);
  m_db.write([ID](Tx& tx) {
    tx.deleteObject(ID);
    tx.deleteObject(ID + 100);
  });
  EXPECT_FALSE(m_db.read([ID](const Tx& tx) { return tx.object(ID); }).has_value());
  EXPECT_TRUE(crops(ID).empty());
  EXPECT_TRUE(visible().empty());
}

TEST_F(DatabaseObjectsTest, TransactionRollsBackOnException)
{
  EXPECT_THROW(m_db.write([this](Tx& tx) {
    const int64_t ID = tx.allocateObjectId();
    tx.upsertTrack(makeTrack(ID, "cup", 1), m_now);
    throw std::runtime_error("abort");
  }),
               std::runtime_error);
  EXPECT_TRUE(visible().empty());
  EXPECT_FALSE(m_db.read([](const Tx& tx) { return tx.object(1); }).has_value());
}

} // namespace
