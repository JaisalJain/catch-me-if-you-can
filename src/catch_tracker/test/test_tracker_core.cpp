// tracker_core: HSV match agrees with OpenCV's conversion on the cube colors, projection/ray round trip,
// triangulation, ballistic fit recovers p/v exactly from noise-free points, the track acquires a flight and ends it.
#include <gtest/gtest.h>

#include <cmath>

#include "catch_tracker/tracker_core.hpp"

using catch_tracker::Camera;
using catch_tracker::Vec3;

static Camera make_cam(const Vec3 & pos, const Vec3 & target)
{
  Camera c;
  c.width = 320; c.height = 240;
  c.fx = c.fy = 120.0 / std::tan(45.0 * M_PI / 360.0);
  c.cx = 160; c.cy = 120;
  Vec3 f{target[0] - pos[0], target[1] - pos[1], target[2] - pos[2]};
  double n = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
  for (auto & x : f) x /= n;
  Vec3 up{0, 0, 1};
  if (std::abs(f[2]) > 0.99) up = {0, 1, 0};
  Vec3 x{f[1] * up[2] - f[2] * up[1], f[2] * up[0] - f[0] * up[2], f[0] * up[1] - f[1] * up[0]};
  n = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
  for (auto & v : x) v /= n;
  Vec3 y{x[1] * f[2] - x[2] * f[1], x[2] * f[0] - x[0] * f[2], x[0] * f[1] - x[1] * f[0]};   // image up
  // optical frame: x right, y down (= -up), z forward
  for (int i = 0; i < 3; ++i) { c.R[i * 3 + 0] = x[i]; c.R[i * 3 + 1] = -y[i]; c.R[i * 3 + 2] = f[i]; }
  c.t = pos;
  return c;
}

TEST(TrackerCore, HsvMatchesCubeColors)
{
  catch_tracker::ColorSpec red{"red", {{0, 8}, {170, 179}}, 120, 60}, blue{"blue", {{100, 130}}, 120, 60};
  EXPECT_TRUE(red.match(217, 26, 26));
  EXPECT_FALSE(red.match(26, 64, 217));
  EXPECT_TRUE(blue.match(26, 64, 217));
  EXPECT_FALSE(red.match(120, 120, 120));      // grey: no saturation
  EXPECT_FALSE(red.match(40, 5, 5));           // too dark
}

TEST(TrackerCore, ProjectRayTriangulate)
{
  const Camera a = make_cam({0.15, -1.30, 1.30}, {0.15, 0.20, 1.25});
  const Camera b = make_cam({0.15, 0.20, 2.35}, {0.15, 0.20, 1.25});
  const Vec3 p{0.27, 0.31, 1.33};
  double ua, va, da, ub, vb, db;
  ASSERT_TRUE(a.project(p, ua, va, da));
  ASSERT_TRUE(b.project(p, ub, vb, db));
  double gap;
  const Vec3 q = catch_tracker::triangulate(a.t, a.ray(ua, va), b.t, b.ray(ub, vb), gap);
  for (int k = 0; k < 3; ++k) EXPECT_NEAR(q[k], p[k], 1e-9);
  EXPECT_LT(gap, 1e-9);
}

TEST(TrackerCore, BallisticFitAndTrack)
{
  const Vec3 p0{-0.05, 0.25, 1.25}, v0{0.905, 0.0, 1.941};
  const double g = 9.81;
  catch_tracker::Track tr;
  bool fly = false;
  int first_fly = -1;
  // 1 s at rest (A holding the cube), then ballistic flight sampled at 125 Hz
  for (int k = 0; k < 10; ++k) tr.add(-0.1 + 0.008 * k, p0);
  for (int k = 0; k < 50; ++k) {
    const double t = 0.03 + 0.008 * k;
    const Vec3 x{p0[0] + v0[0] * t, p0[1] + v0[1] * t, p0[2] + v0[2] * t - 0.5 * g * t * t};
    fly = tr.add(t, x);
    if (fly && first_fly < 0) first_fly = k;
  }
  ASSERT_TRUE(fly);
  EXPECT_EQ(first_fly, 4);                      // acquired after acquire_frames = 5 fixes
  const auto & f = tr.fit();
  const double t = 0.03 + 0.008 * 49;
  EXPECT_NEAR(f.v[0], v0[0], 1e-9);
  EXPECT_NEAR(f.v[2], v0[2] - g * t, 1e-9);
  EXPECT_NEAR(f.p[2], p0[2] + v0[2] * t - 0.5 * g * t * t, 1e-9);
  EXPECT_NEAR(tr.t_first(), 0.03, 1e-12);       // the rest samples are not part of the segment
  // the cube stops (caught): the segment ends
  tr.add(t + 0.008, f.p);
  EXPECT_FALSE(tr.in_flight());
  EXPECT_TRUE(tr.ended());
}
