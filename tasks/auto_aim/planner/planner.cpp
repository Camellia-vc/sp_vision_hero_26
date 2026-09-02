#include "planner.hpp"

#include <vector>

#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace auto_aim
{
namespace
{
Eigen::Vector4d select_aim_xyza(const Target & target)
{
  const auto & xyza_list = target.armor_xyza_list();
  if (xyza_list.empty()) return Eigen::Vector4d::Zero();

  // Lock mechanism: prevent oscillation between two armors at rotation boundaries
  static int lock_id = -1;

  // Find the two closest armors
  int best_id = 0;
  int second_id = -1;
  double best_dist = std::numeric_limits<double>::max();
  double second_dist = std::numeric_limits<double>::max();

  for (int i = 0; i < static_cast<int>(xyza_list.size()); i++) {
    const auto dist = xyza_list[i].head<2>().norm();
    if (dist < best_dist) {
      second_dist = best_dist;
      second_id = best_id;
      best_dist = dist;
      best_id = i;
    } else if (dist < second_dist) {
      second_dist = dist;
      second_id = i;
    }
  }

  // Only enable lock for outpost with multiple armors
  if (target.name == ArmorName::outpost && xyza_list.size() >= 2 && second_id >= 0) {
    constexpr double LOCK_HYSTERESIS = 0.05;  // 5cm hysteresis
    if (std::abs(best_dist - second_dist) < LOCK_HYSTERESIS) {
      // Two armors are close in distance, keep the previously locked one
      if (lock_id == best_id || lock_id == second_id) {
        best_id = lock_id;  // Maintain lock
      } else {
        lock_id = best_id;  // First entry, pick the closest
      }
    } else {
      lock_id = best_id;  // Clear distance difference, normal switch
    }
  } else {
    lock_id = best_id;
  }

  return xyza_list[best_id];
}

Eigen::Vector3d target_center_xyz(const Target & target)
{
  const auto ekf_x = target.ekf_x();
  return {ekf_x[0], ekf_x[2], ekf_x[4]};
}
}  // namespace

Planner::Planner(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  yaw_offset_ = tools::read<double>(yaml, "yaw_offset") / 57.3;
  pitch_offset_ = tools::read<double>(yaml, "pitch_offset") / 57.3;
  fire_thresh_ = tools::read<double>(yaml, "fire_thresh");
  decision_speed_ = tools::read<double>(yaml, "decision_speed");
  high_speed_delay_time_ = tools::read<double>(yaml, "high_speed_delay_time");
  low_speed_delay_time_ = tools::read<double>(yaml, "low_speed_delay_time");

  setup_yaw_solver(config_path);
  setup_pitch_solver(config_path);
}

Plan Planner::plan(Target target, double bullet_speed)
{
  // 0. Check bullet speed
  if (bullet_speed < 10.0 || bullet_speed > 12.5) {
    bullet_speed = 11.5;
  }

  // 1. Predict fly_time with iterative convergence
  const auto aim_xyza = select_aim_xyza(target);
  const auto xyz = aim_xyza.head<3>();
  const auto min_dist = aim_xyza.head<2>().norm();
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  if (bullet_traj.unsolvable) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    return {};
  }

  double prev_fly_time = bullet_traj.fly_time;
  std::vector<Target> iteration_target(10, target);
  for (int iter = 0; iter < 10; ++iter) {
    iteration_target[iter].predict(prev_fly_time);
    const auto iter_aim_xyza = select_aim_xyza(iteration_target[iter]);
    const auto iter_xyz = iter_aim_xyza.head<3>();
    const auto iter_min_dist = iter_aim_xyza.head<2>().norm();
    auto iter_traj = tools::Trajectory(bullet_speed, iter_min_dist, iter_xyz.z());
    if (iter_traj.unsolvable) break;
    if (std::abs(iter_traj.fly_time - prev_fly_time) < 0.001) {
      bullet_traj = iter_traj;
      break;
    }
    prev_fly_time = iter_traj.fly_time;
    bullet_traj = iter_traj;
  }

  target.predict(bullet_traj.fly_time);

  // 2. Get trajectory
  double yaw0;
  Trajectory traj;
  try {
    yaw0 = aim(target, bullet_speed)(0);
    traj = get_trajectory(target, yaw0, bullet_speed);
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    return {};
  }

  // Determine whether the reference trajectory has shifted enough to
  // warrant a warm-start.  When the target is stationary, tiny EKF
  // perturbations should NOT trigger warm-start, otherwise the ADMM
  // residual (max_iter=100 still not machine-precision) accumulates
  // drift that changes plan_pitch / plan_yaw shot-to-shot.
  constexpr double kWarmStartRefThreshold = 1e-4;
  const bool ref_changed =
    !has_last_traj_ ||
    (traj - last_traj_).cwiseAbs().maxCoeff() > kWarmStartRefThreshold;
  last_traj_ = traj;
  has_last_traj_ = true;

  // 3. Solve yaw — x0 from reference trajectory, warm-start from previous solution.
  {
    Eigen::VectorXd x0(2);
    x0 << traj(0, 0), traj(1, 0);
    tiny_set_x0(yaw_solver_, x0);

    // Warm-start only when the reference has genuinely moved.
    if (has_warm_start_ && ref_changed) {
      yaw_solver_->work->x = last_yaw_x_;
      yaw_solver_->work->u = last_yaw_u_;
    }

    yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
    tiny_solve(yaw_solver_);

    // Save for next warm-start.
    last_yaw_x_ = yaw_solver_->work->x;
    last_yaw_u_ = yaw_solver_->work->u;
  }

  // 4. Solve pitch — same warm-start pattern.
  {
    Eigen::VectorXd x0(2);
    x0 << traj(2, 0), traj(3, 0);
    tiny_set_x0(pitch_solver_, x0);

    // Warm-start only when the reference has genuinely moved.
    if (has_warm_start_ && ref_changed) {
      pitch_solver_->work->x = last_pitch_x_;
      pitch_solver_->work->u = last_pitch_u_;
    }

    pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
    tiny_solve(pitch_solver_);

    // Save for next warm-start.
    last_pitch_x_ = pitch_solver_->work->x;
    last_pitch_u_ = pitch_solver_->work->u;

    has_warm_start_ = true;
  }

  Plan plan;
  plan.control = true;

  plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  plan.target_pitch = traj(2, HALF_HORIZON);

  plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
  plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
  plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

  plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);
  plan.distance = target_center_xyz(target).norm();

  auto shoot_offset_ = 2;
  plan.fire =
    std::hypot(
      traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
      traj(2, HALF_HORIZON + shoot_offset_) -
        pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < fire_thresh_;
  return plan;
}

Plan Planner::plan(std::optional<Target> target, double bullet_speed)
{
  if (!target.has_value()) return {};

  double delay_time =
    std::abs(target->ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  auto future = std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));

  target->predict(future);

  return plan(*target, bullet_speed);
}

void Planner::setup_yaw_solver(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto max_yaw_acc = tools::read<double>(yaml, "max_yaw_acc");
  auto Q_yaw = tools::read<std::vector<double>>(yaml, "Q_yaw");
  auto R_yaw = tools::read<std::vector<double>>(yaml, "R_yaw");

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_yaw.data());
  Eigen::Matrix<double, 1, 1> R(R_yaw.data());
  tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);
  tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

  yaw_solver_->settings->max_iter = 100;
}

void Planner::setup_pitch_solver(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto max_pitch_acc = tools::read<double>(yaml, "max_pitch_acc");
  auto Q_pitch = tools::read<std::vector<double>>(yaml, "Q_pitch");
  auto R_pitch = tools::read<std::vector<double>>(yaml, "R_pitch");

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_pitch.data());
  Eigen::Matrix<double, 1, 1> R(R_pitch.data());
  tiny_setup(&pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);
  tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);

  pitch_solver_->settings->max_iter = 100;
}

Eigen::Matrix<double, 2, 1> Planner::aim(const Target & target, double bullet_speed)
{
  const auto aim_xyza = select_aim_xyza(target);
  const auto xyz = aim_xyza.head<3>();
  const auto yaw = aim_xyza[3];
  const auto min_dist = aim_xyza.head<2>().norm();
  debug_xyza = aim_xyza;

  auto azim = std::atan2(xyz.y(), xyz.x());
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");

  return {tools::limit_rad(azim + yaw_offset_), -bullet_traj.pitch - pitch_offset_};
}

Trajectory Planner::get_trajectory(Target & target, double yaw0, double bullet_speed)
{
  Trajectory traj;

  target.predict(-DT * (HALF_HORIZON + 1));
  auto yaw_pitch_last = aim(target, bullet_speed);

  target.predict(DT);  // [0] = -HALF_HORIZON * DT -> [HHALF_HORIZON] = 0
  auto yaw_pitch = aim(target, bullet_speed);

  for (int i = 0; i < HORIZON; i++) {
    target.predict(DT);
    auto yaw_pitch_next = aim(target, bullet_speed);

    auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
    auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

    traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

    yaw_pitch_last = yaw_pitch;
    yaw_pitch = yaw_pitch_next;
  }

  return traj;
}

}  // namespace auto_aim
