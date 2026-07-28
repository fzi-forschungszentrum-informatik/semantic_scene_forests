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

#include <cstddef>

#include <QString>
#include <QWidget>

#include "semantic_scene_forests_visualization_rqt/graph_view.hpp"

class QButtonGroup;
class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QProgressBar;
class QPushButton;
class QRadioButton;
class QVBoxLayout;
class QWidget;

namespace semantic_scene_forests_visualization_rqt {

class SidePanel : public QWidget
{
  Q_OBJECT

public:
  explicit SidePanel(QWidget* parent = nullptr);

  void setCounts(std::size_t nodes, std::size_t edges);
  void setMessageStatus(const QString& text);
  void setViewMode(ViewMode mode);
  void setActionProgress(unsigned current, unsigned total, const QString& status);
  void clearActionProgress(const QString& final_status);

  // Semantic-query controls.
  void setQueryRunning(bool running);
  void setQueryStatus(const QString& text);
  void setInspectedFilterAvailable(bool available);

signals:
  void viewModeChanged(ViewMode mode);
  void requestPhase1();
  void requestPhase2();

  // kind: 0 = PromptLLM, 1 = FindObject.
  void queryRequested(int kind, QString text);
  void queryCancelRequested();
  void showInspectedToggled(bool enabled);

private:
  QLabel* m_node_count;
  QLabel* m_edge_count;
  QLabel* m_message_status;

  QRadioButton* m_force_radio;
  QRadioButton* m_layered_radio;
  QButtonGroup* m_view_group;

  QComboBox* m_query_kind;
  QLineEdit* m_query_input;
  QPushButton* m_query_button;
  QProgressBar* m_query_progress;
  QLabel* m_query_status;
  QCheckBox* m_show_inspected;
  bool m_query_running{false};

  QPushButton* m_phase1_button;
  QPushButton* m_phase2_button;
  QProgressBar* m_progress_bar;
  QLabel* m_progress_status;

  void buildLayout();
};

} // namespace semantic_scene_forests_visualization_rqt
