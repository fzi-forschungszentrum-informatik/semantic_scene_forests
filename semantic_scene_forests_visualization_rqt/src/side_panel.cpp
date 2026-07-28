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

#include "semantic_scene_forests_visualization_rqt/side_panel.hpp"

#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QRadioButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

namespace semantic_scene_forests_visualization_rqt {

SidePanel::SidePanel(QWidget* parent)
  : QWidget{parent}
{
  buildLayout();
}

void SidePanel::buildLayout()
{
  auto* outer = new QVBoxLayout{this};
  outer->setContentsMargins(6, 6, 6, 6);
  outer->setSpacing(8);

  auto* stats_box = new QFrame{this};
  stats_box->setFrameShape(QFrame::StyledPanel);
  auto* stats_layout = new QVBoxLayout{stats_box};
  stats_layout->setContentsMargins(6, 6, 6, 6);
  m_node_count     = new QLabel{tr("Nodes: 0"), stats_box};
  m_edge_count     = new QLabel{tr("Edges: 0"), stats_box};
  m_message_status = new QLabel{tr("Waiting for scene graph…"), stats_box};
  m_message_status->setStyleSheet("color: #999999;");
  stats_layout->addWidget(m_node_count);
  stats_layout->addWidget(m_edge_count);
  stats_layout->addWidget(m_message_status);
  outer->addWidget(stats_box);

  auto* view_box = new QFrame{this};
  view_box->setFrameShape(QFrame::StyledPanel);
  auto* view_layout = new QVBoxLayout{view_box};
  view_layout->setContentsMargins(6, 6, 6, 6);
  view_layout->addWidget(new QLabel{tr("View mode"), view_box});
  m_force_radio   = new QRadioButton{tr("Force-directed"), view_box};
  m_layered_radio = new QRadioButton{tr("Layered"), view_box};
  m_force_radio->setChecked(true);
  m_view_group = new QButtonGroup{this};
  m_view_group->addButton(m_force_radio, 0);
  m_view_group->addButton(m_layered_radio, 1);
  view_layout->addWidget(m_force_radio);
  view_layout->addWidget(m_layered_radio);
  outer->addWidget(view_box);
  connect(m_view_group, &QButtonGroup::idClicked, this, [this](int id) {
    emit viewModeChanged((id == 1) ? ViewMode::LAYERED : ViewMode::FORCE_DIRECTED);
  });

  auto* query_box = new QFrame{this};
  query_box->setFrameShape(QFrame::StyledPanel);
  auto* query_layout = new QVBoxLayout{query_box};
  query_layout->setContentsMargins(6, 6, 6, 6);
  query_layout->addWidget(new QLabel{tr("Semantic query"), query_box});
  m_query_kind = new QComboBox{query_box};
  m_query_kind->addItem(tr("Prompt LLM"));
  m_query_kind->addItem(tr("Find Object"));
  query_layout->addWidget(m_query_kind);
  m_query_input = new QLineEdit{query_box};
  m_query_input->setPlaceholderText(tr("Enter a free-text query…"));
  query_layout->addWidget(m_query_input);
  m_query_button = new QPushButton{tr("Search"), query_box};
  query_layout->addWidget(m_query_button);
  m_query_progress = new QProgressBar{query_box};
  m_query_progress->setRange(0, 100);
  m_query_progress->setValue(0);
  m_query_progress->setVisible(false);
  query_layout->addWidget(m_query_progress);
  m_query_status = new QLabel{query_box};
  m_query_status->setStyleSheet("color: #aaaaaa;");
  m_query_status->setWordWrap(true);
  query_layout->addWidget(m_query_status);
  m_show_inspected = new QCheckBox{tr("Show only inspected nodes"), query_box};
  m_show_inspected->setEnabled(false);
  query_layout->addWidget(m_show_inspected);
  outer->addWidget(query_box);

  const auto emit_query = [this]() {
    if (m_query_running)
    {
      emit queryCancelRequested();
      return;
    }
    const QString text = m_query_input->text().trimmed();
    if (text.isEmpty())
    {
      return;
    }
    emit queryRequested(m_query_kind->currentIndex(), text);
  };
  connect(m_query_button, &QPushButton::clicked, this, emit_query);
  connect(m_query_input, &QLineEdit::returnPressed, this, emit_query);
  connect(m_show_inspected, &QCheckBox::toggled, this, &SidePanel::showInspectedToggled);

  auto* action_box = new QFrame{this};
  action_box->setFrameShape(QFrame::StyledPanel);
  auto* action_layout = new QVBoxLayout{action_box};
  action_layout->setContentsMargins(6, 6, 6, 6);
  action_layout->addWidget(new QLabel{tr("Update Scene Graph"), action_box});
  auto* button_row = new QHBoxLayout;
  m_phase1_button  = new QPushButton{tr("Instance (Phase 1)"), action_box};
  m_phase2_button  = new QPushButton{tr("Meta (Phase 2)"), action_box};
  button_row->addWidget(m_phase1_button);
  button_row->addWidget(m_phase2_button);
  action_layout->addLayout(button_row);
  m_progress_bar = new QProgressBar{action_box};
  m_progress_bar->setRange(0, 100);
  m_progress_bar->setValue(0);
  m_progress_bar->setVisible(false);
  m_progress_status = new QLabel{action_box};
  m_progress_status->setStyleSheet("color: #aaaaaa;");
  m_progress_status->setWordWrap(true);
  action_layout->addWidget(m_progress_bar);
  action_layout->addWidget(m_progress_status);
  outer->addWidget(action_box);
  connect(m_phase1_button, &QPushButton::clicked, this, &SidePanel::requestPhase1);
  connect(m_phase2_button, &QPushButton::clicked, this, &SidePanel::requestPhase2);

  outer->addStretch(1);
}

void SidePanel::setCounts(std::size_t nodes, std::size_t edges)
{
  m_node_count->setText(tr("Nodes: %1").arg(nodes));
  m_edge_count->setText(tr("Edges: %1").arg(edges));
}

void SidePanel::setMessageStatus(const QString& text)
{
  m_message_status->setText(text);
}

void SidePanel::setQueryRunning(bool running)
{
  m_query_running = running;
  m_query_button->setText(running ? tr("Cancel") : tr("Search"));
  m_query_kind->setEnabled(!running);
  m_query_input->setEnabled(!running);
  if (running)
  {
    m_query_progress->setVisible(true);
    m_query_progress->setRange(0, 0); // indeterminate
  }
  else
  {
    m_query_progress->setVisible(false);
    m_query_progress->setRange(0, 100);
    m_query_progress->setValue(0);
  }
}

void SidePanel::setQueryStatus(const QString& text)
{
  m_query_status->setText(text);
}

void SidePanel::setInspectedFilterAvailable(bool available)
{
  const QSignalBlocker block{m_show_inspected};
  m_show_inspected->setEnabled(available);
  m_show_inspected->setChecked(available);
}

void SidePanel::setViewMode(ViewMode mode)
{
  QSignalBlocker block_f{m_force_radio};
  QSignalBlocker block_l{m_layered_radio};
  if (mode == ViewMode::LAYERED)
  {
    m_layered_radio->setChecked(true);
  }
  else
  {
    m_force_radio->setChecked(true);
  }
}

void SidePanel::setActionProgress(unsigned current, unsigned total, const QString& status)
{
  m_progress_bar->setVisible(true);
  if (total == 0)
  {
    m_progress_bar->setRange(0, 0); // indeterminate
  }
  else
  {
    m_progress_bar->setRange(0, static_cast<int>(total));
    m_progress_bar->setValue(static_cast<int>(current));
  }
  m_progress_status->setText(status);
  m_phase1_button->setEnabled(false);
  m_phase2_button->setEnabled(false);
}

void SidePanel::clearActionProgress(const QString& final_status)
{
  m_progress_bar->setVisible(false);
  m_progress_bar->setRange(0, 100);
  m_progress_bar->setValue(0);
  m_progress_status->setText(final_status);
  m_phase1_button->setEnabled(true);
  m_phase2_button->setEnabled(true);
}

} // namespace semantic_scene_forests_visualization_rqt
