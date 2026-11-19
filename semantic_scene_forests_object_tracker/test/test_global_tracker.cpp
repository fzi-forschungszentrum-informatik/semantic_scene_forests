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

#include <string>
#include <vector>

#include <semantic_scene_forests/scene_graph.hpp>

#include "global_tracker.hpp"
#include "tracker_geometry.hpp"

namespace semantic_scene_forests_object_tracker {
namespace {

Tracklet tracklet(const std::string& camera,
                  uint64_t local_id,
                  int sec,
                  double x,
                  const std::string& label = "box",
                  double size              = 0.4)
{
  return Tracklet{.camera     = camera,
                  .local_id   = local_id,
                  .last_seen  = rclcpp::Time(sec, 0, RCL_ROS_TIME),
                  .position   = {x, 0.0, 1.0},
                  .size       = Eigen::Vector3d::Constant(size),
                  .class_name = label,
                  .score      = 0.9,
                  .crops      = nullptr};
}

ArbiterResult
labelCheck(uint64_t id, std::optional<double> probability, const std::string& error = "")
{
  ArbiterResult result;
  result.label_check      = true;
  result.a_id             = id;
  result.same_probability = probability;
  result.error            = error;
  return result;
}

std::vector<ArbiterRequest> pairRequests(const std::vector<ArbiterRequest>& requests)
{
  std::vector<ArbiterRequest> pairs;
  for (const auto& request : requests)
  {
    if (!request.label_check)
    {
      pairs.push_back(request);
    }
  }
  return pairs;
}

class GlobalTrackerTest : public testing::Test
{
protected:
  GlobalTrackerTest() { m_params.world_frame = "map"; }

  std::vector<ArbiterRequest> cycle(const std::vector<Tracklet>& tracklets,
                                    const std::vector<ArbiterResult>& results = {},
                                    bool arbiter                              = false)
  {
    return m_global.cycle(tracklets, results, m_params, arbiter);
  }

  object_tracker::Params m_params;
  // Real scene graph: track IDs are its allocated IDs (1, 2, ... in a fresh graph).
  std::shared_ptr<fzi::semantic_scene_forests::SceneGraph> m_graph{
    std::make_shared<fzi::semantic_scene_forests::SceneGraph>()};
  GlobalTracker m_global{rclcpp::get_logger("test"), m_graph};
};

TEST_F(GlobalTrackerTest, TwoCamerasSeeingOneObjectShareOneGlobalId)
{
  cycle({tracklet("front", 1, 10, 0.0), tracklet("rear", 1, 10, 0.05)});
  ASSERT_EQ(m_global.tracks().size(), 1U);
  EXPECT_EQ(m_global.stats(m_params).links, 2U);
}

TEST_F(GlobalTrackerTest, OneCycleTrackletReportsItsLinkAndLeavesAPersistentTrack)
{
  cycle({tracklet("front", 1, 10, 0.0)});
  EXPECT_EQ(m_global.createdLastCycle(), std::vector<uint64_t>{1});
  cycle({tracklet("<manual>", 1, 11, 0.05), tracklet("<manual>", 2, 11, 5.0)});
  EXPECT_EQ(m_global.linkedTrack("<manual>", 1), 1U); // joined the camera's track
  EXPECT_EQ(m_global.linkedTrack("<manual>", 2), 2U);
  EXPECT_EQ(m_global.createdLastCycle(), std::vector<uint64_t>{2});
  const uint64_t NEXT = 2;
  cycle({}); // The manual tracklets are not reposted: links end, tracks stay.
  EXPECT_FALSE(m_global.linkedTrack("<manual>", 2));
  EXPECT_EQ(m_global.tracks().size(), 2U);
  EXPECT_FALSE(m_global.isHeld(NEXT));
}

TEST_F(GlobalTrackerTest, AddConfirmedTrackIsNeverHeldAndMergesLikeAnyOther)
{
  const auto MANUAL = m_global.addConfirmed("cup",
                                            {0.0, 0.0, 1.0},
                                            Eigen::Vector3d::Constant(0.4),
                                            rclcpp::Time(10, 0, RCL_ROS_TIME),
                                            {},
                                            m_params);
  EXPECT_EQ(MANUAL, 1U);
  ASSERT_EQ(m_global.tracks().size(), 1U);
  EXPECT_TRUE(m_global.tracks()[0]->confirmed);
  EXPECT_FALSE(m_global.isHeld(MANUAL)); // no label check, even with the arbiter on
  // A camera sighting of the same cup joins it.
  cycle({tracklet("front", 1, 11, 0.05, "cup")}, {}, true);
  EXPECT_EQ(m_global.linkedTrack("front", 1), MANUAL);
  EXPECT_EQ(m_global.tracks().size(), 1U);
}

TEST(EncodeCrop, EmptyImageYieldsNoCropAndDoesNotThrow)
{
  sensor_msgs::msg::Image empty;
  empty.encoding = "bgr8";
  EXPECT_NO_THROW(EXPECT_EQ(detail::encodeCrop(empty, 90), nullptr));
}

TEST_F(GlobalTrackerTest, RepostedTrackletFollowsItsHeldTrackUntilTheLabelCheck)
{
  const std::vector<Tracklet> MANUAL{tracklet("<manual>", 1, 10, 0.0, "cup"),
                                     tracklet("<manual>", 2, 10, 5.0, "mug")};
  EXPECT_EQ(cycle(MANUAL, {}, true).size(), 2U);
  EXPECT_TRUE(cycle(MANUAL, {}, true).empty()); // Reposting asks nothing new.
  ASSERT_EQ(m_global.tracks().size(), 2U);
  EXPECT_TRUE(m_global.isHeld(1));
  EXPECT_EQ(m_global.linkedTrack("<manual>", 2), 2U);
  cycle(MANUAL, {labelCheck(1, 0.95), labelCheck(2, 0.05)}, true);
  EXPECT_EQ(m_global.linkedTrack("<manual>", 1), 1U);
  EXPECT_FALSE(m_global.isHeld(1));
  EXPECT_FALSE(m_global.linkedTrack("<manual>", 2)); // Rejected: the link ends ...
  cycle(MANUAL, {}, true);
  EXPECT_EQ(m_global.tracks().size(), 1U); // ... and the reposted tracklet starts no new track.
}

TEST_F(GlobalTrackerTest, RevisitWithin60sRelinksTheSameId)
{
  cycle({tracklet("front", 1, 10, 0.0)});
  cycle({}); // The camera track ended; the global track lives on.
  cycle({tracklet("front", 2, 40, 0.05)});
  ASSERT_EQ(m_global.tracks().size(), 1U);
  EXPECT_EQ(m_global.tracks()[0]->track_id, 1U);
  EXPECT_EQ(m_global.tracks()[0]->last_seen.seconds(), 40.0);
}

TEST_F(GlobalTrackerTest, StaleTrackIsReactivatedBySameObjectAndReplacedByDifferentOne)
{
  cycle({tracklet("front", 1, 10, 0.0), tracklet("front", 2, 10, 5.0)});
  cycle({tracklet("back", 7, 100, 0.0)});               // Similar, same label at the stale pose.
  cycle({tracklet("back", 8, 100, 5.0, "table", 1.0)}); // Larger, covers stale track 2.
  ASSERT_EQ(m_global.tracks().size(), 2U);
  EXPECT_EQ(m_global.tracks()[0]->track_id, 1U);
  EXPECT_EQ(m_global.tracks()[0]->last_seen.seconds(), 100.0);
  EXPECT_NE(m_global.tracks()[1]->track_id, 2U);
}

TEST_F(GlobalTrackerTest, LabelConflictIsAskedOnceAndFallsBackToOverlapWithoutArbiter)
{
  const std::vector<Tracklet> TRACKLETS{tracklet("front", 1, 10, 0.0, "box"),
                                        tracklet("rear", 1, 10, 0.05, "chair")};
  EXPECT_EQ(pairRequests(cycle(TRACKLETS, {}, true)).size(), 1U);
  EXPECT_TRUE(pairRequests(cycle(TRACKLETS, {}, true)).empty());
  EXPECT_EQ(m_global.tracks().size(), 2U);

  GlobalTracker without_arbiter{rclcpp::get_logger("test"), m_graph};
  without_arbiter.cycle(TRACKLETS, {}, m_params, false);
  EXPECT_EQ(without_arbiter.tracks().size(), 1U);
}

TEST_F(GlobalTrackerTest, SameAnswerMergesAndLinksTheOtherLabelWithoutAsking)
{
  const std::vector<Tracklet> TRACKLETS{tracklet("front", 1, 10, 0.0, "box"),
                                        tracklet("rear", 1, 10, 0.05, "chair")};
  const auto REQUESTS = pairRequests(cycle(TRACKLETS, {}, true));
  ASSERT_EQ(REQUESTS.size(), 1U);
  cycle(TRACKLETS,
        {ArbiterResult{.a_id               = REQUESTS[0].a_id,
                       .b_id               = REQUESTS[0].b_id,
                       .same_probability   = 0.95,
                       .object_probability = std::nullopt,
                       .label              = "box",
                       .error              = ""}},
        true);
  ASSERT_EQ(m_global.tracks().size(), 1U);
  EXPECT_EQ(m_global.tracks()[0]->class_name, "box");
  // A third camera reports the other label: linked by the recorded decision, no new request.
  auto more = TRACKLETS;
  more.push_back(tracklet("left", 7, 11, 0.0, "chair"));
  EXPECT_TRUE(pairRequests(cycle(more, {}, true)).empty());
  EXPECT_EQ(m_global.tracks().size(), 1U);
  EXPECT_EQ(m_global.stats(m_params).links, 3U);
}

TEST_F(GlobalTrackerTest, DifferentAnswerKeepsThePairApartForGood)
{
  const std::vector<Tracklet> TRACKLETS{tracklet("front", 1, 10, 0.0, "box"),
                                        tracklet("rear", 1, 10, 0.05, "chair")};
  const auto REQUESTS = pairRequests(cycle(TRACKLETS, {}, true));
  ASSERT_EQ(REQUESTS.size(), 1U);
  cycle(TRACKLETS,
        {ArbiterResult{.a_id               = REQUESTS[0].a_id,
                       .b_id               = REQUESTS[0].b_id,
                       .same_probability   = 0.1,
                       .object_probability = std::nullopt,
                       .label              = "",
                       .error              = ""}},
        true);
  cycle(TRACKLETS, {}, false); // Even the overlap fallback must not merge them now.
  EXPECT_EQ(m_global.tracks().size(), 2U);
}

TEST_F(GlobalTrackerTest, NewTrackIsHeldUntilTheArbiterConfirmsItsLabel)
{
  auto requests =
    cycle({tracklet("front", 1, 10, 0.0, "chair"), tracklet("rear", 1, 10, 5.0)}, {}, true);
  ASSERT_EQ(requests.size(), 2U);
  EXPECT_TRUE(requests[0].label_check);
  ASSERT_EQ(m_global.tracks().size(), 2U);
  EXPECT_TRUE(m_global.isHeld(*m_global.tracks()[0]));
  EXPECT_TRUE(m_global.isHeld(*m_global.tracks()[1]));

  cycle({},
        {labelCheck(1, 0.9),  // a chair
         labelCheck(2, 0.1)}, // no box
        true);
  ASSERT_EQ(m_global.tracks().size(), 1U);
  EXPECT_EQ(m_global.tracks()[0]->track_id, 1U);
  EXPECT_FALSE(m_global.isHeld(*m_global.tracks()[0]));
  EXPECT_EQ(m_global.stats(m_params).label_rejections, 1U);
}

TEST_F(GlobalTrackerTest, RejectedTrackletIsNotRecreatedWhileItLives)
{
  const std::vector<Tracklet> TRACKLETS{tracklet("front", 1, 10, 0.0, "assemble")};
  cycle(TRACKLETS, {}, true);
  EXPECT_TRUE(cycle(TRACKLETS, {labelCheck(1, 0.06)}, true).empty()); // Same local track.
  EXPECT_TRUE(m_global.tracks().empty());
  cycle({}, {}, true); // The local track ended; a new one is checked again.
  EXPECT_EQ(cycle({tracklet("front", 2, 20, 0.0, "assemble")}, {}, true).size(), 1U);
}

TEST_F(GlobalTrackerTest, LabelThatIsNoObjectIsIgnoredForTheSession)
{
  cycle(
    {tracklet("front", 1, 10, 0.0, "kitchen"), tracklet("rear", 1, 10, 5.0, "kitchen")}, {}, true);
  auto no_object               = labelCheck(1, 0.1);
  no_object.object_probability = 0.1;
  const auto REQUESTS =
    cycle({tracklet("left", 1, 11, 9.0, "kitchen")}, {no_object}, true); // Track 2 goes too.
  EXPECT_TRUE(REQUESTS.empty());
  EXPECT_TRUE(m_global.tracks().empty());
  EXPECT_TRUE(m_global.rejectedLabels().contains("kitchen"));
}

TEST_F(GlobalTrackerTest, FailedLabelCheckPublishesAndNoCheckWithoutArbiter)
{
  cycle({tracklet("front", 1, 10, 0.0)}, {}, true);
  cycle({}, {labelCheck(1, std::nullopt, "timeout")}, true);
  ASSERT_EQ(m_global.tracks().size(), 1U);
  EXPECT_FALSE(m_global.isHeld(*m_global.tracks()[0]));

  GlobalTracker without_arbiter{rclcpp::get_logger("test"), m_graph};
  EXPECT_TRUE(without_arbiter.cycle({tracklet("front", 1, 10, 0.0)}, {}, m_params, false).empty());
  EXPECT_FALSE(without_arbiter.isHeld(*without_arbiter.tracks()[0]));
}

} // namespace
} // namespace semantic_scene_forests_object_tracker
