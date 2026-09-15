#include "spmpc_local_planner/reference/motion_region.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

using namespace spmpc_local_planner;

static MotionRegionCell Box(const std::string& id, double a, double b, double x0, double x1) {
  MotionRegionCell c; c.id=id; c.s_begin=a; c.s_end=b;
  c.vertices={{x0,-1},{x1,-1},{x1,1},{x0,1}}; return c;
}
static MotionRegionConfig Config(std::vector<MotionRegionCell> cells) {
  MotionRegionConfig c; c.enabled=true; c.id="r"; c.cells=std::move(cells); return c;
}

TEST(MotionRegion, AcceptsClockwiseAndCounterClockwise) {
  auto ccw=Box("a",0,2,0,3); MotionRegion r(Config({ccw}));
  auto cw=ccw; std::reverse(cw.vertices.begin(),cw.vertices.end());
  EXPECT_TRUE(MotionRegion::validate(Config({cw})));
  EXPECT_GT(r.clearance(1,0,1),0.5);
}
TEST(MotionRegion, RejectsDegenerateNonfiniteAndNonconvex) {
  auto c=Box("a",0,1,0,2); c.vertices[2]=c.vertices[1]; EXPECT_FALSE(MotionRegion::validate(Config({c})));
  c=Box("a",0,1,0,2); c.vertices[2].x=std::numeric_limits<double>::quiet_NaN(); EXPECT_FALSE(MotionRegion::validate(Config({c})));
  c=Box("a",0,1,0,2); std::swap(c.vertices[2],c.vertices[3]); EXPECT_FALSE(MotionRegion::validate(Config({c})));
}
TEST(MotionRegion, RejectsProgressGapAndSpatialDisconnection) {
  EXPECT_FALSE(MotionRegion::validate(Config({Box("a",0,1,0,2),Box("b",2.1,3,2,4)})));
  EXPECT_FALSE(MotionRegion::validate(Config({Box("a",0,1,0,1),Box("b",1,2,3,4)})));
}
TEST(MotionRegion, RejectsSelfIntersectingPolygonAndDuplicateIds) {
  auto c=Box("a",0,1,0,2);
  std::swap(c.vertices[2],c.vertices[3]);
  EXPECT_FALSE(MotionRegion::validate(Config({c})));
  auto a=Box("same",0,1,0,2); auto b=Box("same",1,2,0,2);
  EXPECT_FALSE(MotionRegion::validate(Config({a,b})));
}
TEST(MotionRegion, RejectsReverseProgressEndAndClearsSuccessReason) {
  auto a=Box("a",0,3,0,2); auto b=Box("b",1,2,0,2);
  EXPECT_FALSE(MotionRegion::validate(Config({a,b})));
  std::string reason="stale";
  EXPECT_TRUE(MotionRegion::validate(Config({Box("a",0,1,0,2)}),&reason));
  EXPECT_TRUE(reason.empty());
}
TEST(MotionRegion, StageInflatesFootprintAndSweep) {
  MotionRegion r(Config({Box("a",0,2,0,4)}));
  auto s=r.stage(1,0.5); EXPECT_EQ(s.cell_id,"a");
  // Right boundary is x <= 4 - (0.45 + 0.02 + 0.5).
  EXPECT_NEAR(s.faces[1].offset,3.03,1e-9);
  EXPECT_THROW(r.stage(3,0),std::out_of_range);
  EXPECT_THROW(r.stage(1,2.0),std::out_of_range);
}
TEST(MotionRegion, ChoosesFirstOverlappingCellAndDisabledIsInfinite) {
  MotionRegion r(Config({Box("first",0,2,0,3),Box("second",1,3,0,3)}));
  EXPECT_EQ(r.stage(1.5,0).cell_id,"first");
  MotionRegionConfig d; MotionRegion disabled(d); EXPECT_FALSE(disabled.stage(0,0).enabled);
  EXPECT_TRUE(std::isinf(disabled.clearance(0,0,0)));
  EXPECT_THROW(r.clearance(0,0,3.1),std::out_of_range);
}

TEST(MotionRegion, ProgressRoundoffDoesNotExpandTheSpatialRegion) {
  MotionRegion r(Config({Box("a",0,2,0,4)}));
  EXPECT_NO_THROW(r.stage(-2e-10,0));
  EXPECT_NO_THROW(r.stage(2+2e-10,0));
  EXPECT_NEAR(r.clearance(1,0,2+2e-10),r.clearance(1,0,2),1e-12);
  EXPECT_LT(r.clearance(4,0,2+2e-10),0);  // Footprint still outside.
  EXPECT_THROW(r.stage(-1e-4,0),std::out_of_range);
  EXPECT_THROW(r.clearance(1,0,2+1e-4),std::out_of_range);
}
