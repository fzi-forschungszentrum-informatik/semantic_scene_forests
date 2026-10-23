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

#include "semantic_scene_forests_manager/database.hpp"

#include <algorithm>
#include <cstring>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_set>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include <sqlite3.h>

namespace semantic_scene_forests_manager {
namespace {

using Blob = std::vector<uint8_t>;

constexpr const char* SCHEMA = R"sql(
CREATE TABLE nodes (
  id   INTEGER PRIMARY KEY AUTOINCREMENT,
  kind TEXT NOT NULL CHECK (kind IN ('object', 'concept'))
);
CREATE TABLE objects (
  id               INTEGER PRIMARY KEY REFERENCES nodes (id) ON DELETE CASCADE,
  class_name       TEXT NOT NULL,
  detection        BLOB NOT NULL, -- vision_msgs/Detection3D, rclcpp CDR
  last_seen_ns     INTEGER NOT NULL,
  last_updated_ns  INTEGER NOT NULL,
  source           INTEGER NOT NULL DEFAULT 0,
  blocked          INTEGER NOT NULL DEFAULT 0,
  class_overridden INTEGER NOT NULL DEFAULT 0,
  pose_override    BLOB,          -- geometry_msgs/Pose, rclcpp CDR; NULL if not overridden
  crops_curated    INTEGER NOT NULL DEFAULT 0
);
-- idx only orders an object's crops; curation removes rows and leaves gaps.
CREATE TABLE crops (
  object_id INTEGER NOT NULL REFERENCES objects (id) ON DELETE CASCADE,
  idx       INTEGER NOT NULL,
  encoding  TEXT NOT NULL,
  width     INTEGER NOT NULL,
  height    INTEGER NOT NULL,
  step      INTEGER NOT NULL,
  frame_id  TEXT NOT NULL,
  stamp_ns  INTEGER NOT NULL,
  data      BLOB,
  PRIMARY KEY (object_id, idx)
);
CREATE TABLE concepts (
  id     INTEGER PRIMARY KEY REFERENCES nodes (id) ON DELETE CASCADE,
  label  TEXT NOT NULL UNIQUE
);
CREATE TABLE edges (
  source INTEGER NOT NULL REFERENCES nodes (id) ON DELETE CASCADE,
  target INTEGER NOT NULL REFERENCES nodes (id) ON DELETE CASCADE,
  label  TEXT NOT NULL,
  UNIQUE (source, target, label)
);
CREATE INDEX edges_target ON edges (target);
CREATE TABLE marks (
  worker  TEXT NOT NULL,
  node_id INTEGER NOT NULL REFERENCES nodes (id) ON DELETE CASCADE,
  PRIMARY KEY (worker, node_id)
);
CREATE INDEX marks_node ON marks (node_id);
)sql";

[[noreturn]] void fail(sqlite3* db)
{
  throw std::runtime_error(std::string("sqlite: ") + sqlite3_errmsg(db));
}

//! A prepared statement; binds parameters in call order.
// ponytail: prepared per call; cache statements on Tx if profiling shows prepare cost.
class Statement
{
public:
  Statement(sqlite3* db, const char* sql)
    : m_db(db)
  {
    if (sqlite3_prepare_v2(db, sql, -1, &m_stmt, nullptr) != SQLITE_OK)
    {
      fail(db);
    }
  }
  ~Statement() { sqlite3_finalize(m_stmt); }
  Statement(const Statement&)            = delete;
  Statement& operator=(const Statement&) = delete;
  Statement(Statement&&)                 = delete;
  Statement& operator=(Statement&&)      = delete;

  Statement& bind(int64_t value) { return check(sqlite3_bind_int64(m_stmt, ++m_param, value)); }
  Statement& bind(const std::string& value)
  {
    return check(sqlite3_bind_text(
      m_stmt, ++m_param, value.data(), static_cast<int>(value.size()), SQLITE_TRANSIENT));
  }
  Statement& bind(const Blob& value)
  {
    return check(sqlite3_bind_blob(
      m_stmt, ++m_param, value.data(), static_cast<int>(value.size()), SQLITE_TRANSIENT));
  }
  Statement& bindNull() { return check(sqlite3_bind_null(m_stmt, ++m_param)); }

  //! True if a row is available, false when done.
  bool step()
  {
    const int RC = sqlite3_step(m_stmt);
    if (RC == SQLITE_ROW)
    {
      return true;
    }
    if (RC != SQLITE_DONE)
    {
      fail(m_db);
    }
    return false;
  }

  [[nodiscard]] int64_t integer(int col) const { return sqlite3_column_int64(m_stmt, col); }
  [[nodiscard]] bool isNull(int col) const
  {
    return sqlite3_column_type(m_stmt, col) == SQLITE_NULL;
  }
  [[nodiscard]] std::string text(int col) const
  {
    const auto* data = sqlite3_column_text(m_stmt, col);
    return {reinterpret_cast<const char*>(data),
            static_cast<size_t>(sqlite3_column_bytes(m_stmt, col))};
  }
  [[nodiscard]] Blob blob(int col) const
  {
    const auto* data = static_cast<const uint8_t*>(sqlite3_column_blob(m_stmt, col));
    return {data, data + sqlite3_column_bytes(m_stmt, col)};
  }

private:
  Statement& check(int rc)
  {
    if (rc != SQLITE_OK)
    {
      fail(m_db);
    }
    return *this;
  }

  sqlite3* m_db;
  sqlite3_stmt* m_stmt{nullptr};
  int m_param{0};
};

template <class T> Blob serialize(const T& msg)
{
  rclcpp::SerializedMessage out;
  rclcpp::Serialization<T>().serialize_message(&msg, &out);
  const auto& raw = out.get_rcl_serialized_message();
  return {raw.buffer, raw.buffer + raw.buffer_length};
}

template <class T> T deserialize(const Blob& bytes)
{
  rclcpp::SerializedMessage in(bytes.size());
  auto& raw = in.get_rcl_serialized_message();
  std::memcpy(raw.buffer, bytes.data(), bytes.size());
  raw.buffer_length = bytes.size();
  T msg;
  rclcpp::Serialization<T>().deserialize_message(&in, &msg);
  return msg;
}

int64_t toNs(const builtin_interfaces::msg::Time& time)
{
  return rclcpp::Time(time).nanoseconds();
}

builtin_interfaces::msg::Time fromNs(int64_t ns)
{
  return rclcpp::Time(ns);
}

void setClass(vision_msgs::msg::Detection3D& detection, const std::string& class_name)
{
  for (auto& result : detection.results)
  {
    result.hypothesis.class_id = class_name;
  }
}

void insertCrops(sqlite3* db, int64_t id, const std::vector<sensor_msgs::msg::Image>& crops)
{
  for (size_t i = 0; i < crops.size(); ++i)
  {
    const auto& crop = crops[i];
    Statement(db,
              "INSERT INTO crops (object_id, idx, encoding, width, height, step, frame_id, "
              "stamp_ns, data) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)")
      .bind(id)
      .bind(static_cast<int64_t>(i))
      .bind(crop.encoding)
      .bind(static_cast<int64_t>(crop.width))
      .bind(static_cast<int64_t>(crop.height))
      .bind(static_cast<int64_t>(crop.step))
      .bind(crop.header.frame_id)
      .bind(toNs(crop.header.stamp))
      .bind(crop.data)
      .step();
  }
}

constexpr const char* OBJECT_COLUMNS =
  "SELECT id, class_name, detection, last_seen_ns, last_updated_ns, source, blocked FROM objects ";

ObjectRecord toRecord(const Statement& row)
{
  ObjectRecord record;
  record.id           = row.integer(0);
  record.class_name   = row.text(1);
  record.detection    = deserialize<vision_msgs::msg::Detection3D>(row.blob(2));
  record.last_seen    = fromNs(row.integer(3));
  record.last_updated = fromNs(row.integer(4));
  record.source       = static_cast<uint8_t>(row.integer(5));
  record.blocked      = row.integer(6) != 0;
  return record;
}

//! Label rules ported from validateConceptLabel: trim spaces/tabs, non-empty, at most 128
//! bytes, no control characters. nullopt if invalid.
std::optional<std::string> conceptLabel(const std::string& raw)
{
  const auto FIRST = raw.find_first_not_of(" \t");
  if (FIRST == std::string::npos)
  {
    return std::nullopt;
  }
  std::string label = raw.substr(FIRST, raw.find_last_not_of(" \t") - FIRST + 1);
  if (label.size() > 128U || std::ranges::any_of(label, [](unsigned char character) {
        return character < 0x20U || character == 0x7FU;
      }))
  {
    return std::nullopt;
  }
  return label;
}

std::vector<int64_t> ids(Statement& rows)
{
  std::vector<int64_t> result;
  while (rows.step())
  {
    result.push_back(rows.integer(0));
  }
  return result;
}

} // namespace

int64_t Tx::allocateObjectId()
{
  Statement(m_db, "INSERT INTO nodes (kind) VALUES ('object')").step();
  return sqlite3_last_insert_rowid(m_db);
}

void Tx::upsertTrack(const TrackUpdate& update, const builtin_interfaces::msg::Time& now)
{
  Statement row(m_db,
                "SELECT class_name, detection, last_seen_ns, blocked, class_overridden, "
                "pose_override, crops_curated FROM objects WHERE id = ?");
  row.bind(update.id);
  if (!row.step())
  {
    Statement(m_db,
              "INSERT INTO objects (id, class_name, detection, last_seen_ns, last_updated_ns) "
              "VALUES (?, ?, ?, ?, ?)")
      .bind(update.id)
      .bind(update.class_name)
      .bind(serialize(update.detection))
      .bind(toNs(update.last_seen))
      .bind(toNs(now))
      .step();
    if (update.crops)
    {
      insertCrops(m_db, update.id, *update.crops);
    }
    return;
  }
  if (row.integer(3) != 0)
  {
    return; // Blocked: tracker updates are ignored until unblocked.
  }

  // Overlay rules ported from ManagedObject::updateFromCandidate: class and pose overrides
  // survive, curated crops are never replaced, last_updated moves only on a real change.
  const bool CLASS_OVERRIDDEN  = row.integer(4) != 0;
  const std::string NEXT_CLASS = CLASS_OVERRIDDEN ? row.text(0) : update.class_name;
  auto next_detection          = update.detection;
  if (CLASS_OVERRIDDEN)
  {
    setClass(next_detection, NEXT_CLASS);
  }
  if (!row.isNull(5))
  {
    // Keep the operator pose, including orientation; dimensions stay the tracker's.
    next_detection.bbox.center = deserialize<geometry_msgs::msg::Pose>(row.blob(5));
  }
  const Blob NEXT_DETECTION = serialize(next_detection);
  const bool REPLACE_CROPS  = update.crops.has_value() && row.integer(6) == 0;
  if (NEXT_CLASS == row.text(0) && NEXT_DETECTION == row.blob(1) &&
      toNs(update.last_seen) == row.integer(2) && !REPLACE_CROPS)
  {
    return;
  }

  Statement(m_db,
            "UPDATE objects SET class_name = ?, detection = ?, last_seen_ns = ?, "
            "last_updated_ns = ? WHERE id = ?")
    .bind(NEXT_CLASS)
    .bind(NEXT_DETECTION)
    .bind(toNs(update.last_seen))
    .bind(toNs(now))
    .bind(update.id)
    .step();
  if (REPLACE_CROPS)
  {
    Statement(m_db, "DELETE FROM crops WHERE object_id = ?").bind(update.id).step();
    insertCrops(m_db, update.id, *update.crops);
  }
}

void Tx::deleteObject(int64_t id)
{
  Statement(m_db, "DELETE FROM nodes WHERE id = ?").bind(id).step();
}

std::optional<ObjectRecord> Tx::object(int64_t id) const
{
  Statement row(m_db, (std::string(OBJECT_COLUMNS) + "WHERE id = ?").c_str());
  row.bind(id);
  if (!row.step())
  {
    return std::nullopt;
  }
  return toRecord(row);
}

std::vector<ObjectRecord> Tx::visibleObjects() const
{
  Statement rows(m_db, (std::string(OBJECT_COLUMNS) + "WHERE blocked = 0 ORDER BY id").c_str());
  std::vector<ObjectRecord> records;
  while (rows.step())
  {
    records.push_back(toRecord(rows));
  }
  return records;
}

std::vector<sensor_msgs::msg::Image> Tx::crops(int64_t id) const
{
  Statement rows(m_db,
                 "SELECT encoding, width, height, step, frame_id, stamp_ns, data FROM crops "
                 "WHERE object_id = ? ORDER BY idx");
  rows.bind(id);
  std::vector<sensor_msgs::msg::Image> images;
  while (rows.step())
  {
    sensor_msgs::msg::Image image;
    image.encoding        = rows.text(0);
    image.width           = static_cast<uint32_t>(rows.integer(1));
    image.height          = static_cast<uint32_t>(rows.integer(2));
    image.step            = static_cast<uint32_t>(rows.integer(3));
    image.header.frame_id = rows.text(4);
    image.header.stamp    = fromNs(rows.integer(5));
    image.data            = rows.blob(6);
    images.push_back(std::move(image));
  }
  return images;
}

bool Tx::block(int64_t id)
{
  Statement(m_db, "UPDATE objects SET blocked = 1 WHERE id = ?").bind(id).step();
  if (sqlite3_changes(m_db) == 0)
  {
    return false;
  }
  Statement(m_db, "DELETE FROM crops WHERE object_id = ?").bind(id).step();
  return true;
}

bool Tx::unblock(int64_t id)
{
  Statement(m_db, "UPDATE objects SET blocked = 0 WHERE id = ? AND blocked = 1").bind(id).step();
  return sqlite3_changes(m_db) > 0;
}

EditFailure Tx::edit(const semantic_scene_forests_object_msgs::srv::EditObject::Request& request,
                     const builtin_interfaces::msg::Time& now)
{
  // Rules ported from ObjectManager::editObject. Blocked objects are hidden from clients and
  // therefore unknown to edits.
  Statement row(m_db,
                "SELECT class_name, detection, class_overridden, pose_override FROM objects "
                "WHERE id = ? AND blocked = 0");
  row.bind(request.object_id);
  if (!row.step())
  {
    return EditFailure::UNKNOWN_OBJECT;
  }

  std::vector<int64_t> crop_rowids;
  Statement crop_rows(m_db, "SELECT rowid FROM crops WHERE object_id = ? ORDER BY idx");
  crop_rows.bind(request.object_id);
  while (crop_rows.step())
  {
    crop_rowids.push_back(crop_rows.integer(0));
  }
  auto indices = request.remove_crop_indices;
  std::ranges::sort(indices);
  const auto DUPLICATES = std::ranges::unique(indices);
  indices.erase(DUPLICATES.begin(), DUPLICATES.end());
  if (!indices.empty() && indices.back() >= crop_rowids.size())
  {
    return EditFailure::INVALID_CROP_INDEX;
  }

  std::string class_name     = row.text(0);
  auto detection             = deserialize<vision_msgs::msg::Detection3D>(row.blob(1));
  bool class_overridden      = row.integer(2) != 0;
  const bool POSE_OVERRIDDEN = !row.isNull(3);
  bool changed               = false;
  if (request.update_class_name)
  {
    changed          = !class_overridden || class_name != request.class_name;
    class_name       = request.class_name;
    class_overridden = true;
    setClass(detection, class_name);
  }
  if (request.update_pose)
  {
    changed               = changed || !POSE_OVERRIDDEN || detection.bbox.center != request.pose;
    detection.bbox.center = request.pose;
  }
  for (const uint32_t IDX : indices)
  {
    Statement(m_db, "DELETE FROM crops WHERE rowid = ?").bind(crop_rowids[IDX]).step();
  }
  changed = changed || !indices.empty();
  if (!changed)
  {
    return EditFailure::SUCCESS;
  }

  Statement update(m_db,
                   "UPDATE objects SET class_name = ?, detection = ?, class_overridden = ?, "
                   "pose_override = COALESCE(?, pose_override), "
                   "crops_curated = crops_curated OR ?, source = 1, last_updated_ns = ? "
                   "WHERE id = ?");
  update.bind(class_name).bind(serialize(detection)).bind(static_cast<int64_t>(class_overridden));
  if (request.update_pose)
  {
    update.bind(serialize(request.pose));
  }
  else
  {
    update.bindNull();
  }
  update.bind(static_cast<int64_t>(!indices.empty())).bind(toNs(now)).bind(request.object_id);
  update.step();
  return EditFailure::SUCCESS;
}

int64_t Tx::findOrCreateConcept(const std::string& label)
{
  Statement row(m_db, "SELECT id FROM concepts WHERE label = ?");
  row.bind(label);
  if (row.step())
  {
    return row.integer(0);
  }
  Statement(m_db, "INSERT INTO nodes (kind) VALUES ('concept')").step();
  const int64_t ID = sqlite3_last_insert_rowid(m_db);
  Statement(m_db, "INSERT INTO concepts (id, label) VALUES (?, ?)").bind(ID).bind(label).step();
  return ID;
}

bool Tx::addEdge(int64_t source, int64_t target, const std::string& label)
{
  // OR IGNORE covers only the UNIQUE conflict; the WHERE keeps a missing endpoint from
  // tripping the foreign key, which would throw.
  Statement(m_db,
            "INSERT OR IGNORE INTO edges (source, target, label) SELECT ?, ?, ? "
            "WHERE EXISTS (SELECT 1 FROM nodes WHERE id = ?) "
            "AND EXISTS (SELECT 1 FROM nodes WHERE id = ?)")
    .bind(source)
    .bind(target)
    .bind(label)
    .bind(source)
    .bind(target)
    .step();
  return sqlite3_changes(m_db) > 0;
}

bool Tx::removeEdge(int64_t source, int64_t target, const std::string& label)
{
  Statement(m_db, "DELETE FROM edges WHERE source = ? AND target = ? AND label = ?")
    .bind(source)
    .bind(target)
    .bind(label)
    .step();
  return sqlite3_changes(m_db) > 0;
}

bool Tx::removeConcept(int64_t id)
{
  Statement(m_db, "DELETE FROM nodes WHERE id = (SELECT id FROM concepts WHERE id = ?)")
    .bind(id)
    .step();
  return sqlite3_changes(m_db) > 0;
}

std::pair<int64_t, bool> Tx::addContext(int64_t node_id, const std::string& context)
{
  const auto LABEL = conceptLabel(context);
  if (!LABEL)
  {
    return {0, false};
  }
  // Concepts, or objects that exist and are not blocked (an allocated id without an objects
  // row is not a node clients can see).
  Statement node(m_db,
                 "SELECT 1 FROM nodes n LEFT JOIN objects o ON o.id = n.id "
                 "WHERE n.id = ? AND (n.kind = 'concept' OR o.blocked = 0)");
  node.bind(node_id);
  if (!node.step())
  {
    return {0, false};
  }
  const int64_t CONCEPT = findOrCreateConcept(*LABEL);
  if (CONCEPT == node_id)
  {
    return {0, false}; // A concept as its own context would be a self-loop.
  }
  return {CONCEPT, addEdge(node_id, CONCEPT, "has_context")};
}

int Tx::pruneIsolatedConcepts()
{
  Statement(m_db,
            "DELETE FROM nodes WHERE kind = 'concept' AND id NOT IN (SELECT source FROM edges) "
            "AND id NOT IN (SELECT target FROM edges)")
    .step();
  return sqlite3_changes(m_db);
}

Counts Tx::counts() const
{
  Statement rows(m_db,
                 "SELECT (SELECT COUNT(*) FROM objects), "
                 "(SELECT COUNT(*) FROM objects WHERE blocked = 1), "
                 "(SELECT COUNT(*) FROM concepts), (SELECT COUNT(*) FROM edges)");
  rows.step();
  return {.objects         = rows.integer(0),
          .blocked_objects = rows.integer(1),
          .concepts        = rows.integer(2),
          .edges           = rows.integer(3)};
}

void Tx::mark(const std::string& worker, int64_t node_id)
{
  Statement(m_db,
            "INSERT OR IGNORE INTO marks (worker, node_id) SELECT ?, ? "
            "WHERE EXISTS (SELECT 1 FROM nodes WHERE id = ?)")
    .bind(worker)
    .bind(node_id)
    .bind(node_id)
    .step();
}

std::vector<int64_t> Tx::visibleObjectsUnmarkedBy(const std::string& worker) const
{
  Statement rows(m_db,
                 "SELECT id FROM objects WHERE blocked = 0 AND id NOT IN "
                 "(SELECT node_id FROM marks WHERE worker = ?) ORDER BY id");
  rows.bind(worker);
  return ids(rows);
}

std::vector<int64_t> Tx::conceptsUnmarkedBy(const std::string& worker) const
{
  Statement rows(m_db,
                 "SELECT id FROM concepts WHERE id NOT IN "
                 "(SELECT node_id FROM marks WHERE worker = ?) ORDER BY id");
  rows.bind(worker);
  return ids(rows);
}

fzi::semantic_scene_forests::SceneGraph Tx::visibleGraph() const
{
  fzi::semantic_scene_forests::SceneGraph graph;
  std::unordered_set<int64_t> visible_ids;
  for (const auto& record : visibleObjects())
  {
    visible_ids.insert(record.id);
    // Database ids are unique, so a failure here is a bug.
    if (auto added = graph.addObjectNodeWithId(
          record.id,
          record.class_name,
          std::make_shared<vision_msgs::msg::Detection3D>(record.detection),
          {});
        !added)
    {
      throw std::runtime_error("visibleGraph: " + added.error().message);
    }
  }

  // All concepts and edges load first; retainObjectNodes below keeps only what visible objects
  // reach, so concepts of blocked objects (and edgeless ones) drop out.
  Statement concepts(m_db, "SELECT id, label FROM concepts ORDER BY id");
  while (concepts.step())
  {
    if (auto added = graph.addConceptNodeWithId(concepts.integer(0), concepts.text(1)); !added)
    {
      throw std::runtime_error("visibleGraph: " + added.error().message);
    }
  }

  // SceneGraph rejects edges into objects and cycles; such rows (only writable through raw
  // addEdge) are left out of the view.
  Statement edges(m_db, "SELECT source, target, label FROM edges ORDER BY rowid");
  while (edges.step())
  {
    const int64_t SOURCE = edges.integer(0);
    const int64_t TARGET = edges.integer(1);
    if (graph.hasNode(SOURCE) && graph.hasNode(TARGET) &&
        !graph.addEdge(SOURCE, TARGET, edges.text(2)))
    {
      continue; // Rejected by SceneGraph; left out of the view.
    }
  }
  if (auto retained = graph.retainObjectNodes(visible_ids); !retained)
  {
    throw std::runtime_error("visibleGraph: " + retained.error().message);
  }
  return graph;
}

void Tx::commitScratch(const fzi::semantic_scene_forests::SceneGraph& before,
                       const fzi::semantic_scene_forests::SceneGraph& after)
{
  std::set<int64_t> before_concepts;
  for (const auto& node : before.getConceptNodes())
  {
    before_concepts.insert(node.id);
  }
  std::map<int64_t, int64_t> remap;
  std::set<int64_t> after_concepts;
  for (const auto& node : after.getConceptNodes())
  {
    after_concepts.insert(node.id);
    if (!before_concepts.contains(node.id))
    {
      remap[node.id] = findOrCreateConcept(node.concept_name);
    }
  }
  const auto TO_DB = [&remap](int64_t id) {
    const auto IT = remap.find(id);
    return IT == remap.end() ? id : IT->second;
  };

  using EdgeKey = std::tuple<int64_t, int64_t, std::string>;
  std::set<EdgeKey> before_edges;
  for (const auto& [source, target, edge] : before.getEdges())
  {
    before_edges.emplace(source, target, edge.label);
  }
  std::set<EdgeKey> after_edges;
  for (const auto& [source, target, edge] : after.getEdges())
  {
    after_edges.emplace(source, target, edge.label);
    if (!before_edges.contains({source, target, edge.label}))
    {
      // Returns false (skipped) if an endpoint was deleted meanwhile.
      addEdge(TO_DB(source), TO_DB(target), edge.label);
    }
  }
  for (const auto& [source, target, label] : before_edges)
  {
    if (!after_edges.contains({source, target, label}))
    {
      removeEdge(source, target, label);
    }
  }
  for (const int64_t ID : before_concepts)
  {
    if (!after_concepts.contains(ID))
    {
      removeConcept(ID);
    }
  }
}

Database::Database()
{
  if (sqlite3_open(":memory:", &m_db) != SQLITE_OK)
  {
    const std::string MESSAGE = sqlite3_errmsg(m_db);
    sqlite3_close(m_db);
    throw std::runtime_error("sqlite: " + MESSAGE);
  }
  m_tx = Tx(m_db);
  try
  {
    exec("PRAGMA foreign_keys = ON");
    exec(SCHEMA);
  }
  catch (...)
  {
    sqlite3_close(m_db);
    throw;
  }
}

Database::~Database()
{
  sqlite3_close(m_db);
}

int64_t Database::changeCount() const
{
  const std::scoped_lock LOCK{m_mutex};
  return sqlite3_total_changes64(m_db);
}

void Database::exec(const char* sql)
{
  if (sqlite3_exec(m_db, sql, nullptr, nullptr, nullptr) != SQLITE_OK)
  {
    fail(m_db);
  }
}

void Database::rollback() noexcept
{
  sqlite3_exec(m_db, "ROLLBACK", nullptr, nullptr, nullptr);
}

} // namespace semantic_scene_forests_manager
