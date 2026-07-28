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
#include <fmt/format.h>
#include <string>

enum class SceneGraphErrorKind : std::uint8_t
{
  NODE_ALREADY_EXISTS,
  NODE_DOES_NOT_EXIST,
  EDGE_DOES_NOT_EXIST,
  INVALID_EDGE,
  EDGE_ALREADY_EXISTS,
  CYCLE,
  GRAPH_EDITING_FAILED,
};

struct SceneGraphError
{
  SceneGraphErrorKind kind;
  std::string message;

  // Named constructors mirror your Rust variants
  static SceneGraphError nodeAlreadyExists(int64_t id)
  {
    return {.kind    = SceneGraphErrorKind::NODE_ALREADY_EXISTS,
            .message = fmt::format("Node with id {} already exists!", id)};
  }
  static SceneGraphError nodeDoesNotExist(int64_t id)
  {
    return {.kind    = SceneGraphErrorKind::NODE_DOES_NOT_EXIST,
            .message = fmt::format("Node with id {} does not exist!", id)};
  }
  static SceneGraphError edgeDoesNotExist(int64_t from, int64_t to, std::string_view label)
  {
    return {.kind = SceneGraphErrorKind::EDGE_DOES_NOT_EXIST,
            .message =
              fmt::format("Edge from {} to {} with label {} does not exist!", from, to, label)};
  }
  static SceneGraphError invalidEdge(int64_t from, int64_t to, std::string_view label)
  {
    return {.kind    = SceneGraphErrorKind::INVALID_EDGE,
            .message = fmt::format(
              "Edge target node {} cannot have incoming connections. Attempted to create "
              "(from: {}, to: {}, label: {})",
              to,
              from,
              to,
              label)};
  }
  static SceneGraphError edgeAlreadyExists(int64_t from, int64_t to)
  {
    return {.kind    = SceneGraphErrorKind::EDGE_ALREADY_EXISTS,
            .message = fmt::format("Edge from {} to {} already exists!", from, to)};
  }
  static SceneGraphError cycle(int64_t from, int64_t to)
  {
    return {.kind    = SceneGraphErrorKind::CYCLE,
            .message = fmt::format("Adding edge from {} to {} would introduce a cycle!", from, to)};
  }
  static SceneGraphError graphEditingFailed(std::string_view reason)
  {
    return {.kind    = SceneGraphErrorKind::GRAPH_EDITING_FAILED,
            .message = fmt::format("Graph editing failed: {}", reason)};
  }
};
