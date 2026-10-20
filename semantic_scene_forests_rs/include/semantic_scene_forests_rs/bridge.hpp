// Copyright (c) 2026 FZI Forschungszentrum Informatik
// SPDX-License-Identifier: BSD-3-Clause
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

class NativeGraph
{
public:
  NativeGraph();
  ~NativeGraph();
  IdResult addObjectNode(rust::Str name);
  IdResult addConceptNode(rust::Str name);
  bool hasNode(int64_t id) const;
  Status addEdge(int64_t source, int64_t target, rust::Str label);
  EdgeList outgoingEdges(int64_t id) const;

private:
  std::unique_ptr<fzi::semantic_scene_forests::SceneGraph> m_graph;
};

std::unique_ptr<NativeGraph> newGraph();
} // namespace ssf_rs
