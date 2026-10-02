// v2 flight tracking core (no ROS): pinhole camera, HSV cube detection, stereo triangulation, ballistic least
// squares with known gravity, and the per-cube track state machine. Units: m, s, pixels.
#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace catch_tracker
{

using Vec3 = std::array<double, 3>;

struct Camera
{
  int width = 0, height = 0;
  double fx = 0, fy = 0, cx = 0, cy = 0;
  std::array<double, 9> R{};  // world <- optical (ROS optical frame: x right, y down, z forward), row-major
  Vec3 t{};                   // optical frame origin in the world
  // pixel (u, v) and depth of a world point; depth <= 0 behind the camera
  bool project(const Vec3 & p, double & u, double & v, double & depth) const;
  // unit ray (world) through pixel (u, v)
  Vec3 ray(double u, double v) const;
};

struct ColorSpec
{
  std::string name;
  std::vector<std::pair<int, int>> h;  // OpenCV hue ranges (0..179), inclusive
  int s_min = 120, v_min = 60;
  bool match(uint8_t r, uint8_t g, uint8_t b) const;
};

struct Box
{
  Vec3 lo{}, hi{};
};

struct Blob
{
  double u = 0, v = 0;
  int count = 0;
};

// Pixel rectangle (clamped) covering the projection of a world box.
std::array<int, 4> roi_rect(const Camera & cam, const Box & box);
// Centroid of the pixels of one color inside rect (RGB8, row stride `step` bytes). nullopt if < min_px.
std::optional<Blob> detect(const uint8_t * rgb, int width, int height, int step, const ColorSpec & spec,
                           const std::array<int, 4> & rect, int min_px);
// Midpoint of the shortest segment between two rays; gap = its length.
Vec3 triangulate(const Vec3 & o1, const Vec3 & d1, const Vec3 & o2, const Vec3 & d2, double & gap);

struct Fit
{
  Vec3 p{}, v{};                    // at the reference time t_ref
  std::array<double, 36> cov{};     // (p, v) at t_ref
  double rms = 0;
  int n = 0;
};
// Least squares p(t) = p0 + v0 (t - t_ref) + g/2 (t - t_ref)^2 per axis (gravity along -z), sigma per axis.
bool ballistic_fit(const std::vector<std::pair<double, Vec3>> & pts, double t_ref, double g, const Vec3 & sigma, Fit & out);

struct TrackParams
{
  int acquire_frames = 5;       // ballistic fit of the last N fixes ...
  double acquire_rms = 0.003;   // ... with residual rms below this ...
  double acquire_speed = 0.8;   // ... and speed above this (m/s) starts a flight segment
  double lose_residual = 0.015; // a new fix this far from the prediction ends the segment (caught / landed)
  int max_frames = 120;         // fit window
  double gravity = 9.81;
  Vec3 sigma{0.0015, 0.0015, 0.0025};  // per-axis measurement noise for the covariance
};

class Track
{
public:
  explicit Track(TrackParams p = {}) : p_(p) {}
  // add a fix; returns true if the cube is in a ballistic flight segment after it
  bool add(double t, const Vec3 & x);
  bool in_flight() const { return in_flight_; }
  const Fit & fit() const { return fit_; }
  double t_first() const { return seg_.empty() ? 0.0 : seg_.front().first; }
  bool ended() const { return ended_; }   // the last segment ended (one-shot flag, cleared by the next add)
  void reset() { hist_.clear(); seg_.clear(); in_flight_ = false; ended_ = false; }

private:
  TrackParams p_;
  std::deque<std::pair<double, Vec3>> hist_;
  std::vector<std::pair<double, Vec3>> seg_;
  bool in_flight_ = false, ended_ = false;
  Fit fit_;
};

// Point on the predicted descending path closest to ref (tau from t_ref in [0, horizon], z >= z_min); cov 3x3.
bool predict_intercept(const Fit & f, double g, const Vec3 & ref, double z_min, double horizon, double & tau, Vec3 & p,
                       std::array<double, 9> & cov);

}  // namespace catch_tracker
