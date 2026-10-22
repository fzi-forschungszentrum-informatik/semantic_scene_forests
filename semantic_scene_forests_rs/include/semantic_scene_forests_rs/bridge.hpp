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

#include "rust/cxx.h"
#include <cstdint>
#include <memory>

namespace fzi::semantic_scene_forests {
class SceneGraph;
}

namespace ssf_rs {
struct Status;
struct IdResult;
struct EdgeList;
struct GraphRecord;
struct PayloadRecord;
struct ObjectNodeRecord;

class NativeGraph
{
public:
  NativeGraph();
  ~NativeGraph();
  IdResult addObjectNode(rust::Str name, const PayloadRecord& payload);
  rust::Vec<ObjectNodeRecord> objectNodes() const;
  Status copyFrom(const NativeGraph& other);
  IdResult addConceptNode(rust::Str name);
  bool hasNode(int64_t id) const;
  Status addEdge(int64_t source, int64_t target, rust::Str label);
  EdgeList outgoingEdges(int64_t id) const;
  Status loadMessage(const GraphRecord& message);
  GraphRecord toMessage();
  Status updateFromMessage(const GraphRecord& message);

private:
  std::unique_ptr<fzi::semantic_scene_forests::SceneGraph> m_graph;
};

std::unique_ptr<NativeGraph> newGraph();
} // namespace ssf_rs
