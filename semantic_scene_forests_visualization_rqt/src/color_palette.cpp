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

#include "semantic_scene_forests_visualization_rqt/color_palette.hpp"

#include <array>
#include <cstdint>

namespace semantic_scene_forests_visualization_rqt {

namespace {

constexpr std::array<QColor, 32> PALETTE = {
  QColor{226, 54, 54},  // 0 red
  QColor{226, 118, 54}, // 1 orange
  QColor{226, 183, 54}, // 2 gold
  QColor{204, 226, 54}, // 3 yellow-green
  QColor{140, 226, 54}, // 4 lime
  QColor{75, 226, 54},  // 5 green
  QColor{54, 226, 97},  // 6 mint
  QColor{54, 226, 161}, // 7 teal-green
  QColor{54, 226, 226}, // 8 cyan
  QColor{54, 161, 226}, // 9 sky blue
  QColor{54, 97, 226},  // 10 blue
  QColor{75, 54, 226},  // 11 indigo
  QColor{140, 54, 226}, // 12 violet
  QColor{204, 54, 226}, // 13 magenta
  QColor{226, 54, 183}, // 14 hot pink
  QColor{226, 54, 118}, // 15 rose
  QColor{142, 35, 35},  // 16 dark red
  QColor{142, 75, 35},  // 17 dark orange
  QColor{142, 116, 35}, // 18 dark gold
  QColor{129, 142, 35}, // 19 dark yellow-green
  QColor{89, 142, 35},  // 20 dark lime
  QColor{49, 142, 35},  // 21 dark green
  QColor{35, 142, 62},  // 22 dark mint
  QColor{35, 142, 102}, // 23 dark teal
  QColor{35, 142, 142}, // 24 dark cyan
  QColor{35, 102, 142}, // 25 dark sky
  QColor{35, 62, 142},  // 26 dark blue
  QColor{49, 35, 142},  // 27 dark indigo
  QColor{89, 35, 142},  // 28 dark violet
  QColor{129, 35, 142}, // 29 dark magenta
  QColor{142, 35, 116}, // 30 dark pink
  QColor{142, 35, 75},  // 31 dark rose
};

} // namespace

QColor labelColor(std::string_view label)
{
  std::uint32_t hash = 0;
  for (unsigned char c : label)
  {
    hash = (hash * 31U) + static_cast<std::uint32_t>(c);
  }
  return PALETTE[hash % PALETTE.size()];
}

} // namespace semantic_scene_forests_visualization_rqt
