#include "catch_tracker/tracker_core.hpp"

#include <algorithm>
#include <cmath>

namespace catch_tracker
{
using blob_label_t = int;

bool Camera::project(const Vec3 & p, double & u, double & v, double & depth) const
{
  // optical = R^T (p - t)
  const double d[3] = {p[0] - t[0], p[1] - t[1], p[2] - t[2]};
  double c[3];
  for (int i = 0; i < 3; ++i) c[i] = R[0 * 3 + i] * d[0] + R[1 * 3 + i] * d[1] + R[2 * 3 + i] * d[2];
  depth = c[2];
  if (depth <= 1e-9) return false;
  u = fx * c[0] / c[2] + cx;
  v = fy * c[1] / c[2] + cy;
  return true;
}

Vec3 Camera::ray(double u, double v) const
{
  const double c[3] = {(u - cx) / fx, (v - cy) / fy, 1.0};
  Vec3 w;
  for (int i = 0; i < 3; ++i) w[i] = R[i * 3 + 0] * c[0] + R[i * 3 + 1] * c[1] + R[i * 3 + 2] * c[2];
  const double n = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
  for (auto & x : w) x /= n;
  return w;
}

bool ColorSpec::match(uint8_t r, uint8_t g, uint8_t b) const
{
  // OpenCV COLOR_RGB2HSV for 8-bit images: V = max, S = 255 (V - min) / V, H in [0, 180)
  const int mx = std::max({r, g, b}), mn = std::min({r, g, b});
  if (mx < v_min) return false;
  const int delta = mx - mn;
  if (mx == 0 || 255 * delta < s_min * mx) return false;
  double h;
  if (delta == 0) {
    h = 0;
  } else if (mx == r) {
    h = 60.0 * (g - b) / delta;
  } else if (mx == g) {
    h = 120.0 + 60.0 * (b - r) / delta;
  } else {
    h = 240.0 + 60.0 * (r - g) / delta;
  }
  if (h < 0) h += 360.0;
  const int hh = static_cast<int>(std::lround(h / 2.0)) % 180;
  for (const auto & range : this->h) {
    if (hh >= range.first && hh <= range.second) return true;
  }
  return false;
}

std::array<int, 4> roi_rect(const Camera & cam, const Box & box)
{
  double u0 = 1e9, v0 = 1e9, u1 = -1e9, v1 = -1e9;
  for (int k = 0; k < 8; ++k) {
    const Vec3 p{(k & 1) ? box.hi[0] : box.lo[0], (k & 2) ? box.hi[1] : box.lo[1], (k & 4) ? box.hi[2] : box.lo[2]};
    double u, v, d;
    if (!cam.project(p, u, v, d)) continue;
    u0 = std::min(u0, u); v0 = std::min(v0, v); u1 = std::max(u1, u); v1 = std::max(v1, v);
  }
  auto clamp = [](double x, int hi) { return std::max(0, std::min(hi, static_cast<int>(std::floor(x)))); };
  return {clamp(u0, cam.width), clamp(v0, cam.height), clamp(u1 + 1, cam.width), clamp(v1 + 1, cam.height)};
}

std::optional<Blob> detect(const uint8_t * rgb, int /*width*/, int /*height*/, int step, const ColorSpec & spec,
                           const std::array<int, 4> & rect, int min_px)
{
  double su = 0, sv = 0;
  int n = 0;
  for (int v = rect[1]; v < rect[3]; ++v) {
    const uint8_t * row = rgb + static_cast<size_t>(v) * step;
    for (int u = rect[0]; u < rect[2]; ++u) {
      const uint8_t * px = row + 3 * u;
      if (spec.match(px[0], px[1], px[2])) {
        su += u; sv += v; ++n;
      }
    }
  }
  if (n < min_px) return std::nullopt;
  // pixel centres: +0.5 (pixel u covers [u, u+1))
  return Blob{su / n + 0.5, sv / n + 0.5, n};
}

std::vector<Blob> components(const uint8_t * rgb, int /*width*/, int /*height*/, int step, const ColorSpec & spec,
                             const std::array<int, 4> & rect, int min_px)
{
  const int w = rect[2] - rect[0], h = rect[3] - rect[1];
  std::vector<blob_label_t> lab(static_cast<size_t>(std::max(w, 0)) * std::max(h, 0), 0);
  for (int v = 0; v < h; ++v) {
    const uint8_t * row = rgb + static_cast<size_t>(v + rect[1]) * step;
    for (int u = 0; u < w; ++u) {
      const uint8_t * px = row + 3 * (u + rect[0]);
      lab[v * w + u] = spec.match(px[0], px[1], px[2]) ? -1 : 0;
    }
  }
  std::vector<Blob> out;
  std::vector<int> stack;
  int next = 1;
  for (int i = 0; i < w * h; ++i) {
    if (lab[i] != -1) continue;
    Blob b;
    double su = 0, sv = 0;
    stack.assign(1, i);
    lab[i] = next;
    while (!stack.empty()) {
      const int k = stack.back();
      stack.pop_back();
      const int u = k % w, v = k / w;
      su += u; sv += v; ++b.count;
      for (int dv = -1; dv <= 1; ++dv) {
        for (int du = -1; du <= 1; ++du) {
          const int uu = u + du, vv = v + dv;
          if (uu < 0 || vv < 0 || uu >= w || vv >= h) continue;
          const int kk = vv * w + uu;
          if (lab[kk] == -1) { lab[kk] = next; stack.push_back(kk); }
        }
      }
    }
    ++next;
    if (b.count >= min_px) {
      b.u = su / b.count + rect[0] + 0.5;
      b.v = sv / b.count + rect[1] + 0.5;
      out.push_back(b);
    }
  }
  std::sort(out.begin(), out.end(), [](const Blob & a, const Blob & b) { return a.count > b.count; });
  return out;
}

bool inside(const Box & b, const Vec3 & p)
{
  for (int k = 0; k < 3; ++k) {
    if (p[k] < b.lo[k] || p[k] > b.hi[k]) return false;
  }
  return true;
}

Vec3 triangulate(const Vec3 & o1, const Vec3 & d1, const Vec3 & o2, const Vec3 & d2, double & gap)
{
  const Vec3 w{o1[0] - o2[0], o1[1] - o2[1], o1[2] - o2[2]};
  auto dot = [](const Vec3 & a, const Vec3 & b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; };
  const double a = dot(d1, d1), b = dot(d1, d2), c = dot(d2, d2), d = dot(d1, w), e = dot(d2, w);
  const double den = a * c - b * b;
  const double s = (b * e - c * d) / den, t = (a * e - b * d) / den;
  Vec3 p1, p2, m;
  for (int i = 0; i < 3; ++i) {
    p1[i] = o1[i] + s * d1[i];
    p2[i] = o2[i] + t * d2[i];
    m[i] = 0.5 * (p1[i] + p2[i]);
  }
  gap = std::sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]) + (p1[2] - p2[2]) * (p1[2] - p2[2]));
  return m;
}

bool ballistic_fit(const std::vector<std::pair<double, Vec3>> & pts, double t_ref, double g, const Vec3 & sigma, Fit & out)
{
  const int n = static_cast<int>(pts.size());
  if (n < 3) return false;
  double s0 = n, s1 = 0, s2 = 0;
  Vec3 sy{}, sty{};
  for (const auto & [t, x] : pts) {
    const double tau = t - t_ref;
    s1 += tau; s2 += tau * tau;
    for (int k = 0; k < 3; ++k) {
      const double y = x[k] + (k == 2 ? 0.5 * g * tau * tau : 0.0);   // remove gravity: z + g tau^2 / 2
      sy[k] += y; sty[k] += tau * y;
    }
  }
  const double det = s0 * s2 - s1 * s1;
  if (std::abs(det) < 1e-12) return false;
  // (A^T A)^-1 = [[s2, -s1], [-s1, s0]] / det
  const double i00 = s2 / det, i01 = -s1 / det, i11 = s0 / det;
  for (int k = 0; k < 3; ++k) {
    out.p[k] = i00 * sy[k] + i01 * sty[k];
    out.v[k] = i01 * sy[k] + i11 * sty[k];
  }
  double ss = 0;
  for (const auto & [t, x] : pts) {
    const double tau = t - t_ref;
    for (int k = 0; k < 3; ++k) {
      const double pred = out.p[k] + out.v[k] * tau - (k == 2 ? 0.5 * g * tau * tau : 0.0);
      ss += (x[k] - pred) * (x[k] - pred);
    }
  }
  out.rms = std::sqrt(ss / n);
  out.n = n;
  out.cov.fill(0.0);
  for (int k = 0; k < 3; ++k) {
    const double s = sigma[k] * sigma[k];
    out.cov[k * 6 + k] = s * i00;                      // var p_k
    out.cov[(k + 3) * 6 + (k + 3)] = s * i11;          // var v_k
    out.cov[k * 6 + (k + 3)] = out.cov[(k + 3) * 6 + k] = s * i01;
  }
  return true;
}

bool Track::add(double t, const Vec3 & x)
{
  ended_ = false;
  hist_.emplace_back(t, x);
  while (static_cast<int>(hist_.size()) > p_.max_frames) hist_.pop_front();
  if (in_flight_) {
    // residual of the new fix against the current prediction
    const double tau = t - (seg_.empty() ? t : seg_.back().first);
    const Fit & f = fit_;                      // referenced to the previous newest fix (seg_.back())
    double r2 = 0;
    for (int k = 0; k < 3; ++k) {
      const double pred = f.p[k] + f.v[k] * tau - (k == 2 ? 0.5 * p_.gravity * tau * tau : 0.0);
      r2 += (x[k] - pred) * (x[k] - pred);
    }
    if (std::sqrt(r2) > p_.lose_residual) {
      in_flight_ = false;
      ended_ = true;
      seg_.clear();
      hist_.clear();
      hist_.emplace_back(t, x);
      return false;
    }
    seg_.emplace_back(t, x);
    if (static_cast<int>(seg_.size()) > p_.max_frames) seg_.erase(seg_.begin());
    ballistic_fit(seg_, t, p_.gravity, p_.sigma, fit_);
    return true;
  }
  if (static_cast<int>(hist_.size()) < p_.acquire_frames) return false;
  std::vector<std::pair<double, Vec3>> last(hist_.end() - p_.acquire_frames, hist_.end());
  Fit f;
  if (!ballistic_fit(last, t, p_.gravity, p_.sigma, f)) return false;
  const double speed = std::sqrt(f.v[0] * f.v[0] + f.v[1] * f.v[1] + f.v[2] * f.v[2]);
  if (f.rms > p_.acquire_rms || speed < p_.acquire_speed) return false;
  // extend backwards while the earlier fixes stay on the same parabola
  int start = static_cast<int>(hist_.size()) - p_.acquire_frames;
  while (start > 0) {
    std::vector<std::pair<double, Vec3>> ext(hist_.begin() + (start - 1), hist_.end());
    Fit fe;
    if (!ballistic_fit(ext, t, p_.gravity, p_.sigma, fe) || fe.rms > p_.acquire_rms) break;
    --start;
  }
  seg_.assign(hist_.begin() + start, hist_.end());
  ballistic_fit(seg_, t, p_.gravity, p_.sigma, fit_);
  in_flight_ = true;
  return true;
}

bool predict_intercept(const Fit & f, double g, const Vec3 & ref, double z_min, double horizon, double & tau_best, Vec3 & p,
                       std::array<double, 9> & cov)
{
  double best = 1e18;
  bool found = false;
  for (double tau = 0.0; tau <= horizon; tau += 0.002) {
    const double vz = f.v[2] - g * tau;
    const Vec3 q{f.p[0] + f.v[0] * tau, f.p[1] + f.v[1] * tau, f.p[2] + f.v[2] * tau - 0.5 * g * tau * tau};
    if (q[2] < z_min) break;
    if (vz >= 0) continue;
    const double d = (q[0] - ref[0]) * (q[0] - ref[0]) + (q[1] - ref[1]) * (q[1] - ref[1]) + (q[2] - ref[2]) * (q[2] - ref[2]);
    if (d < best) {
      best = d; tau_best = tau; p = q; found = true;
    }
  }
  if (!found) return false;
  cov.fill(0.0);
  for (int k = 0; k < 3; ++k) {
    // var(p + v tau) = var p + 2 tau cov(p, v) + tau^2 var v
    cov[k * 3 + k] = f.cov[k * 6 + k] + 2 * tau_best * f.cov[k * 6 + k + 3] + tau_best * tau_best * f.cov[(k + 3) * 6 + k + 3];
  }
  return true;
}

}  // namespace catch_tracker
