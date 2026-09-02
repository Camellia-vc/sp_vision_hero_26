#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <list>
#include <optional>

#include "tasks/auto_aim/target.hpp"
#include "tinympc/tiny_api.hpp"

namespace auto_aim
{
constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2;

using Trajectory = Eigen::Matrix<double, 4, HORIZON>;  // yaw, yaw_vel, pitch, pitch_vel

struct Plan
{
  bool control = false;
  bool fire = false;
  float target_yaw = 0.0f;
  float target_pitch = 0.0f;
  float yaw = 0.0f;
  float yaw_vel = 0.0f;
  float yaw_acc = 0.0f;
  float pitch = 0.0f;
  float pitch_vel = 0.0f;
  float pitch_acc = 0.0f;
  float distance = -1.0f;
};

class Planner
{
public:
  Eigen::Vector4d debug_xyza;
  Planner(const std::string & config_path);

  Plan plan(Target target, double bullet_speed);
  Plan plan(std::optional<Target> target, double bullet_speed);

private:
  double yaw_offset_;
  double pitch_offset_;
  double fire_thresh_;
  double low_speed_delay_time_, high_speed_delay_time_, decision_speed_;

  TinySolver * yaw_solver_;
  TinySolver * pitch_solver_;

  // Warm-start: store previous MPC solution to avoid cold-start jumps after shots.
  // Only applied when the reference trajectory has changed significantly;
  // on stationary targets this prevents ADMM residual drift from shifting
  // plan_pitch / plan_yaw frame-to-frame.
  bool has_warm_start_{false};
  bool has_last_traj_{false};
  Trajectory last_traj_;
  Eigen::MatrixXd last_yaw_x_, last_yaw_u_;
  Eigen::MatrixXd last_pitch_x_, last_pitch_u_;

  void setup_yaw_solver(const std::string & config_path);
  void setup_pitch_solver(const std::string & config_path);

  Eigen::Matrix<double, 2, 1> aim(const Target & target, double bullet_speed);
  Trajectory get_trajectory(Target & target, double yaw0, double bullet_speed);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP
