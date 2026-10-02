// v2 flight tracking node (composable): two fixed cameras -> per-color HSV centroid inside the flight volume ->
// stereo triangulation -> bias correction -> ballistic least squares (known g) -> /tracker/cube (CubeTrack).
//
// Detection runs in this C++ node, not in a Python callback. The camera images come from the MuJoCo camera plugin
// in the ros2_control_node process, so they cross a process boundary (FastDDS, shared memory on one host) whatever
// this node is loaded into; intra-process transport would need the publisher in the same process. The latency
// this costs is measured (catch_tasks tracker_eval: render stamp -> published estimate).
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "catch_interfaces/msg/cube_track.hpp"
#include "catch_tracker/tracker_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace catch_tracker
{

class CubeTracker : public rclcpp::Node
{
public:
  explicit CubeTracker(const rclcpp::NodeOptions & options) : Node("cube_tracker", options)
  {
    const auto cams = declare_parameter<std::vector<std::string>>("cameras", {"track_side", "track_top"});
    for (const auto & name : cams) {
      Cam c;
      c.name = name;
      c.topic = declare_parameter<std::string>(name + ".topic", "/cameras/" + name + "/image_raw");
      c.cam.width = static_cast<int>(declare_parameter<int64_t>(name + ".width", 320));
      c.cam.height = static_cast<int>(declare_parameter<int64_t>(name + ".height", 240));
      c.cam.fx = declare_parameter<double>(name + ".fx", 0.0);
      c.cam.fy = declare_parameter<double>(name + ".fy", 0.0);
      c.cam.cx = declare_parameter<double>(name + ".cx", 0.0);
      c.cam.cy = declare_parameter<double>(name + ".cy", 0.0);
      const auto R = declare_parameter<std::vector<double>>(name + ".R", std::vector<double>(9, 0.0));
      const auto t = declare_parameter<std::vector<double>>(name + ".t", std::vector<double>(3, 0.0));
      for (int i = 0; i < 9; ++i) c.cam.R[i] = R.at(i);
      for (int i = 0; i < 3; ++i) c.cam.t[i] = t.at(i);
      cams_.push_back(c);
    }
    const auto colors = declare_parameter<std::vector<std::string>>("colors", {"red", "green", "blue"});
    for (const auto & name : colors) {
      ColorSpec s;
      s.name = name;
      const auto h = declare_parameter<std::vector<int64_t>>(name + ".h", std::vector<int64_t>{});
      for (size_t i = 0; i + 1 < h.size(); i += 2) s.h.emplace_back(static_cast<int>(h[i]), static_cast<int>(h[i + 1]));
      s.s_min = static_cast<int>(declare_parameter<int64_t>(name + ".s_min", 120));
      s.v_min = static_cast<int>(declare_parameter<int64_t>(name + ".v_min", 60));
      specs_.push_back(s);
    }
    const auto lo = declare_parameter<std::vector<double>>("roi.lo", {-0.15, 0.0, 0.95});
    const auto hi = declare_parameter<std::vector<double>>("roi.hi", {0.65, 0.45, 1.70});
    for (int i = 0; i < 3; ++i) { roi_.lo[i] = lo.at(i); roi_.hi[i] = hi.at(i); }
    min_px_ = static_cast<int>(declare_parameter<int64_t>("min_px", 8));
    max_gap_ = declare_parameter<double>("max_ray_gap", 0.010);
    const auto bias = declare_parameter<std::vector<double>>("bias", {0.0, 0.0, 0.0});
    for (int i = 0; i < 3; ++i) bias_[i] = bias.at(i);
    TrackParams tp;
    tp.acquire_frames = static_cast<int>(declare_parameter<int64_t>("acquire_frames", tp.acquire_frames));
    tp.acquire_rms = declare_parameter<double>("acquire_rms", tp.acquire_rms);
    tp.acquire_speed = declare_parameter<double>("acquire_speed", tp.acquire_speed);
    tp.lose_residual = declare_parameter<double>("lose_residual", tp.lose_residual);
    tp.max_frames = static_cast<int>(declare_parameter<int64_t>("max_frames", tp.max_frames));
    const auto sigma = declare_parameter<std::vector<double>>("sigma", {0.0015, 0.0015, 0.0025});
    for (int i = 0; i < 3; ++i) tp.sigma[i] = sigma.at(i);
    gravity_ = tp.gravity;
    for (const auto & s : specs_) tracks_.emplace(s.name, Track(tp));
    const auto ref = declare_parameter<std::vector<double>>("intercept_ref", {0.30, 0.25, 1.10});
    for (int i = 0; i < 3; ++i) ref_[i] = ref.at(i);
    z_min_ = declare_parameter<double>("intercept_z_min", 0.95);

    for (auto & c : cams_) c.rect = roi_rect(c.cam, roi_);
    pub_ = create_publisher<catch_interfaces::msg::CubeTrack>("/tracker/cube", rclcpp::QoS(50));
    for (size_t i = 0; i < cams_.size(); ++i) {
      subs_.push_back(create_subscription<sensor_msgs::msg::Image>(
          cams_[i].topic, rclcpp::QoS(2),
          [this, i](sensor_msgs::msg::Image::ConstSharedPtr m) { on_image(i, std::move(m)); }));
      RCLCPP_INFO(get_logger(), "camera %s (%s) %dx%d f=%.1f ROI px [%d %d %d %d]", cams_[i].name.c_str(),
                  cams_[i].topic.c_str(), cams_[i].cam.width, cams_[i].cam.height, cams_[i].cam.fx, cams_[i].rect[0],
                  cams_[i].rect[1], cams_[i].rect[2], cams_[i].rect[3]);
    }
    RCLCPP_INFO(get_logger(), "bias (%.4f, %.4f, %.4f) m, sigma (%.4f, %.4f, %.4f) m", bias_[0], bias_[1], bias_[2],
                tp.sigma[0], tp.sigma[1], tp.sigma[2]);
  }

private:
  struct Cam
  {
    std::string name, topic;
    Camera cam;
    std::array<int, 4> rect{};
    sensor_msgs::msg::Image::ConstSharedPtr last;
  };

  static int64_t ns(const builtin_interfaces::msg::Time & t) { return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec; }

  void on_image(size_t i, sensor_msgs::msg::Image::ConstSharedPtr m)
  {
    cams_[i].last = std::move(m);
    const int64_t stamp = ns(cams_[i].last->header.stamp);
    for (const auto & c : cams_) {
      if (!c.last || ns(c.last->header.stamp) != stamp) return;   // wait for the pair rendered from the same data
    }
    if (stamp == last_processed_) return;
    last_processed_ = stamp;
    process(stamp);
  }

  void process(int64_t stamp_ns)
  {
    const auto t_recv = now();
    const double t = stamp_ns * 1e-9;
    for (const auto & spec : specs_) {
      // components in each view; every cross-view pair whose rays meet (gap) inside the flight box is a candidate
      std::vector<std::vector<Blob>> blobs;
      for (const auto & c : cams_) {
        const auto & im = *c.last;
        if (im.encoding != "rgb8") break;
        blobs.push_back(components(im.data.data(), im.width, im.height, im.step, spec, c.rect, min_px_));
      }
      auto & tr = tracks_.at(spec.name);
      if (blobs.size() != 2 || blobs[0].empty() || blobs[1].empty()) continue;
      bool found = false;
      Vec3 x{};
      double best = 1e9;
      Vec3 pred{};
      const bool have_pred = tr.in_flight();
      if (have_pred) {
        const auto & f = tr.fit();
        const double tau = t - last_t_.at(spec.name);
        for (int k = 0; k < 3; ++k) pred[k] = f.p[k] + f.v[k] * tau - (k == 2 ? 0.5 * gravity_ * tau * tau : 0.0);
      }
      for (const auto & b0 : blobs[0]) {
        for (const auto & b1 : blobs[1]) {
          double gap;
          Vec3 p = triangulate(cams_[0].cam.t, cams_[0].cam.ray(b0.u, b0.v), cams_[1].cam.t, cams_[1].cam.ray(b1.u, b1.v), gap);
          if (gap > max_gap_ || !inside(roi_, p)) continue;
          double score = gap;
          if (have_pred) {
            score = std::sqrt((p[0] - pred[0]) * (p[0] - pred[0]) + (p[1] - pred[1]) * (p[1] - pred[1]) + (p[2] - pred[2]) * (p[2] - pred[2]));
          }
          if (score < best) { best = score; x = p; found = true; }
        }
      }
      if (!found) continue;
      for (int k = 0; k < 3; ++k) x[k] -= bias_[k];
      last_t_[spec.name] = t;
      const bool fly = tr.add(t, x);
      if (!fly && !tr.ended()) continue;
      catch_interfaces::msg::CubeTrack msg;
      msg.header.stamp = rclcpp::Time(stamp_ns, RCL_ROS_TIME);
      msg.header.frame_id = "world";
      msg.color = spec.name;
      msg.in_flight = fly;
      msg.measured.x = x[0]; msg.measured.y = x[1]; msg.measured.z = x[2];
      msg.t_received = t_recv;
      if (fly) {
        const auto & f = tr.fit();
        msg.n_frames = f.n;
        msg.t_first = rclcpp::Time(static_cast<int64_t>(tr.t_first() * 1e9), RCL_ROS_TIME);
        msg.position.x = f.p[0]; msg.position.y = f.p[1]; msg.position.z = f.p[2];
        msg.velocity.x = f.v[0]; msg.velocity.y = f.v[1]; msg.velocity.z = f.v[2];
        std::copy(f.cov.begin(), f.cov.end(), msg.covariance.begin());
        msg.rms = f.rms;
        double tau;
        Vec3 p;
        std::array<double, 9> cov;
        if (predict_intercept(f, gravity_, ref_, z_min_, 1.0, tau, p, cov)) {
          msg.intercept.x = p[0]; msg.intercept.y = p[1]; msg.intercept.z = p[2];
          msg.intercept_time = rclcpp::Time(stamp_ns + static_cast<int64_t>(tau * 1e9), RCL_ROS_TIME);
          std::copy(cov.begin(), cov.end(), msg.intercept_covariance.begin());
        }
      }
      msg.t_published = now();
      pub_->publish(msg);
    }
  }

  std::vector<Cam> cams_;
  std::vector<ColorSpec> specs_;
  std::map<std::string, Track> tracks_;
  std::map<std::string, double> last_t_;
  Box roi_;
  int min_px_ = 8;
  double max_gap_ = 0.01, gravity_ = 9.81, z_min_ = 0.95;
  Vec3 bias_{}, ref_{};
  int64_t last_processed_ = -1;
  rclcpp::Publisher<catch_interfaces::msg::CubeTrack>::SharedPtr pub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subs_;
};

}  // namespace catch_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(catch_tracker::CubeTracker)
