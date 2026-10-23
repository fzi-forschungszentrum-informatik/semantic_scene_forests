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

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <semantic_scene_forests/scene_graph.hpp>
#include <semantic_scene_forests_object_msgs/srv/edit_object.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection3_d.hpp>

struct sqlite3;

namespace semantic_scene_forests_manager {

//! One tracker observation of a global track, written by upsertTrack().
struct TrackUpdate
{
  int64_t id{0}; //!< From Tx::allocateObjectId().
  std::string class_name;
  vision_msgs::msg::Detection3D detection;
  builtin_interfaces::msg::Time last_seen;
  //! Set only when the tracker's crop set changed.
  std::optional<std::vector<sensor_msgs::msg::Image>> crops;
};

//! An objects row as clients and workers see it (overrides already applied).
struct ObjectRecord
{
  int64_t id{0}; //!< Database id.
  std::string class_name;
  vision_msgs::msg::Detection3D detection;
  builtin_interfaces::msg::Time last_seen;
  builtin_interfaces::msg::Time last_updated;
  uint8_t source{0};
  bool blocked{false};

  bool operator==(const ObjectRecord&) const = default;
};

enum class EditFailure : uint8_t
{
  SUCCESS,
  UNKNOWN_OBJECT,
  INVALID_CROP_INDEX
};

//! Operations on the world model. Only valid inside Database::write/read, which hold the
//! connection mutex; not reentrant.
//! Row counts for diagnostics.
struct Counts
{
  int64_t objects{0};
  int64_t blocked_objects{0};
  int64_t concepts{0};
  int64_t edges{0};
};

class Tx
{
public:
  //! Inserts a `nodes` row (kind 'object') with no objects row yet; returns its id.
  int64_t allocateObjectId();
  //! Creates the objects row on the first call for an allocated id; afterwards applies the
  //! tracker update under the manager overlay (overrides, curated crops, blocked).
  void upsertTrack(const TrackUpdate& update, const builtin_interfaces::msg::Time& now);
  //! Deletes the node; cascades objects row, crops, edges and marks. No-op if unknown.
  void deleteObject(int64_t id);
  [[nodiscard]] std::optional<ObjectRecord> object(int64_t id) const;
  //! Objects that are not blocked, ordered by id.
  [[nodiscard]] std::vector<ObjectRecord> visibleObjects() const;
  [[nodiscard]] std::vector<sensor_msgs::msg::Image> crops(int64_t id) const;
  //! Hides the object and deletes its crop rows; false if the object is unknown.
  bool block(int64_t id);
  //! False if the object is not blocked.
  bool unblock(int64_t id);
  EditFailure edit(const semantic_scene_forests_object_msgs::srv::EditObject::Request& request,
                   const builtin_interfaces::msg::Time& now);

  //! Returns the concept with this label, creating it (new id from `nodes`) if absent.
  int64_t findOrCreateConcept(const std::string& label);
  //! True if the edge was inserted; false if an endpoint is missing or the edge exists.
  bool addEdge(int64_t source, int64_t target, const std::string& label);
  //! False if the edge does not exist.
  bool removeEdge(int64_t source, int64_t target, const std::string& label);
  //! Removes the concept and its edges; false if `id` is not a concept.
  bool removeConcept(int64_t id);
  //! Links node_id -has_context-> concept(context). Returns {concept id, whether the
  //! edge is new}; {0, false} for an invalid label, an unknown node or a blocked object.
  std::pair<int64_t, bool> addContext(int64_t node_id, const std::string& context);
  //! Deletes concepts with no edges; returns how many.
  int pruneIsolatedConcepts();
  [[nodiscard]] Counts counts() const;
  //! Records that `worker` processed node_id. No-op if the node is unknown.
  void mark(const std::string& worker, int64_t node_id);
  [[nodiscard]] std::vector<int64_t> visibleObjectsUnmarkedBy(const std::string& worker) const;
  [[nodiscard]] std::vector<int64_t> conceptsUnmarkedBy(const std::string& worker) const;
  //! Unblocked objects plus the concepts reachable from them through outgoing edges, all with
  //! database ids. Concepts reachable only from blocked objects, and edgeless ones, are hidden.
  [[nodiscard]] fzi::semantic_scene_forests::SceneGraph visibleGraph() const;
  //! Applies a worker's scratch edits (after vs. before) to the database. New concepts (ids not
  //! in `before`, negative for scratch-only ones) are merged by label; edges to nodes that
  //! vanished meanwhile are skipped; every `before` item missing from `after` is removed. The
  //! graph is a shared workspace: nobody owns concepts or edges, and no edit history is kept.
  void commitScratch(const fzi::semantic_scene_forests::SceneGraph& before,
                     const fzi::semantic_scene_forests::SceneGraph& after);

private:
  friend class Database;
  explicit Tx(sqlite3* db)
    : m_db(db)
  {
  }

  sqlite3* m_db;
};

//! The world model: one in-memory SQLite connection, serialized by a (non-recursive) mutex, so
//! write/read are not reentrant.
class Database
{
public:
  //! Opens ":memory:" with foreign keys on and creates the schema.
  Database();
  ~Database();
  Database(const Database&)            = delete;
  Database& operator=(const Database&) = delete;
  Database(Database&&)                 = delete;
  Database& operator=(Database&&)      = delete;

  //! Runs fn inside BEGIN IMMEDIATE..COMMIT; rolls back and rethrows on exception.
  template <class TFn> auto write(TFn&& fn) -> std::invoke_result_t<TFn, Tx&>
  {
    const std::scoped_lock LOCK{m_mutex};
    exec("BEGIN IMMEDIATE");
    try
    {
      if constexpr (std::is_void_v<std::invoke_result_t<TFn, Tx&>>)
      {
        std::forward<TFn>(fn)(m_tx);
        exec("COMMIT");
      }
      else
      {
        auto result = std::forward<TFn>(fn)(m_tx);
        exec("COMMIT");
        return result;
      }
    }
    catch (...)
    {
      rollback();
      throw;
    }
  }

  //! Runs fn under the connection mutex. No explicit transaction: the mutex already keeps
  //! writers out for the duration.
  template <class TFn> auto read(TFn&& fn) const -> std::invoke_result_t<TFn, const Tx&>
  {
    const std::scoped_lock LOCK{m_mutex};
    return std::forward<TFn>(fn)(m_tx);
  }

  //! sqlite3_total_changes64: SQLite's own count of changed rows; lets the publisher skip
  //! unchanged republishes. Counts rolled-back changes too.
  [[nodiscard]] int64_t changeCount() const;

private:
  void exec(const char* sql);
  void rollback() noexcept;

  mutable std::mutex m_mutex;
  sqlite3* m_db{nullptr};
  Tx m_tx{nullptr};
};

} // namespace semantic_scene_forests_manager
