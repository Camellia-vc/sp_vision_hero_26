#include "target.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <numeric>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
namespace
{
constexpr double OUTPOST_HEIGHT_STEP = 0.10;  // m
constexpr double OUTPOST_HEIGHT_PROCESS_NOISE = 1e-2;
constexpr double OUTPOST_DIRECTION_GATING_SPEED = 0.4;
constexpr double OUTPOST_HEIGHT_MIN = -0.08;
constexpr double OUTPOST_HEIGHT_MAX = 0.28;
constexpr double OUTPOST_HEIGHT_MIN_GAP = 0.03;
constexpr double OUTPOST_HEIGHT_MAX_GAP = 0.17;

// Outpost plate identification thresholds (per flowchart)
constexpr double OUTPOST_HIGH_GAP = 2.0 * OUTPOST_HEIGHT_STEP;            // 0.20 m
constexpr double OUTPOST_LOW_GAP = OUTPOST_HEIGHT_STEP;                   // 0.10 m
constexpr double OUTPOST_GAP_HIGH_THRESHOLD = 1.5 * OUTPOST_HEIGHT_STEP;  // 0.15 m
constexpr double OUTPOST_GAP_LOW_THRESHOLD = 0.5 * OUTPOST_HEIGHT_STEP;   // 0.05 m

constexpr int STATE_X = 0;
constexpr int STATE_VX = 1;
constexpr int STATE_Y = 2;
constexpr int STATE_VY = 3;
constexpr int STATE_Z = 4;
constexpr int STATE_VZ = 5;
constexpr int STATE_YAW = 6;
constexpr int STATE_VYAW = 7;
constexpr int STATE_R = 8;
constexpr int STATE_L = 9;
constexpr int STATE_H = 10;
}  // namespace

Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig)
: name(armor.name),
  armor_type(armor.type),
  priority(armor.priority),
  jumped(false),
  last_id(0),
  update_count_(0),
  armor_num_(armor_num),
  switch_count_(0),
  is_switch_(false),
  is_converged_(false),
  t_(t)
{
  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  auto center_x = xyz[0] + radius * std::cos(ypr[0]);
  auto center_y = xyz[1] + radius * std::sin(ypr[0]);
  auto center_z = xyz[2];

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(kStateSize);
  x0 << center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, radius, 0, 0, 0, 0;

  if (is_outpost()) {
    // The outpost keeps three cyclic height slots aligned with armor ids 0/1/2.
    x0[outpost_height_index(0)] = 0.0;
    x0[outpost_height_index(1)] = 2.0 * OUTPOST_HEIGHT_STEP;
    x0[outpost_height_index(2)] = OUTPOST_HEIGHT_STEP;
  }

  if (P0_dig.rows() != kStateSize) {
    Eigen::VectorXd expanded = Eigen::VectorXd::Zero(kStateSize);
    expanded.head(std::min<int>(P0_dig.rows(), kStateSize)) =
      P0_dig.head(std::min<int>(P0_dig.rows(), kStateSize));
    P0_dig = expanded;
  }
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[STATE_YAW] = tools::limit_rad(c[STATE_YAW]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);
}

Target::Target(double x, double vyaw, double radius, double h)
: name(ArmorName::not_armor),
  armor_type(ArmorType::small),
  priority(ArmorPriority::fifth),
  jumped(false),
  last_id(0),
  armor_num_(4),
  switch_count_(0),
  update_count_(0),
  is_switch_(false),
  is_converged_(false)
{
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(kStateSize);
  x0 << x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h, 0, 0;
  Eigen::VectorXd P0_dig = Eigen::VectorXd::Zero(kStateSize);
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[STATE_YAW] = tools::limit_rad(c[STATE_YAW]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  predict(dt);
  t_ = t;
}

void Target::predict(double dt)
{
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(kStateSize, kStateSize);

  // Outpost: center (X,Y,Z) is stationary — no velocity coupling.
  // YAW still rotates at constant speed.
  if (name == ArmorName::outpost) {
    F(STATE_YAW, STATE_VYAW) = dt;
  } else {
    F(STATE_X, STATE_VX) = dt;
    F(STATE_Y, STATE_VY) = dt;
    F(STATE_Z, STATE_VZ) = dt;
    F(STATE_YAW, STATE_VYAW) = dt;
  }

  // Process noise intensities.
  double v_pos, v_yaw;
  if (name == ArmorName::outpost) {
    v_pos = 5.0;    // small: center is fixed, leaves room for filter health
    v_yaw = 0.02;   // tiny: VYAW is known constant 0.8π, only tiny noise for filter health
  } else {
    v_pos = 100;
    v_yaw = 400;
  }

  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
  Q(STATE_X, STATE_X) = a * v_pos;
  Q(STATE_X, STATE_VX) = b * v_pos;
  Q(STATE_VX, STATE_X) = b * v_pos;
  Q(STATE_VX, STATE_VX) = c * v_pos;
  Q(STATE_Y, STATE_Y) = a * v_pos;
  Q(STATE_Y, STATE_VY) = b * v_pos;
  Q(STATE_VY, STATE_Y) = b * v_pos;
  Q(STATE_VY, STATE_VY) = c * v_pos;
  Q(STATE_Z, STATE_Z) = a * v_pos;
  Q(STATE_Z, STATE_VZ) = b * v_pos;
  Q(STATE_VZ, STATE_Z) = b * v_pos;
  Q(STATE_VZ, STATE_VZ) = c * v_pos;
  Q(STATE_YAW, STATE_YAW) = a * v_yaw;
  Q(STATE_YAW, STATE_VYAW) = b * v_yaw;
  Q(STATE_VYAW, STATE_YAW) = b * v_yaw;
  Q(STATE_VYAW, STATE_VYAW) = c * v_yaw;

  if (is_outpost()) {
    for (int i = 0; i < armor_num_; i++) {
      Q(outpost_height_index(i), outpost_height_index(i)) = OUTPOST_HEIGHT_PROCESS_NOISE;
    }
    Q(STATE_R, STATE_R) = 1e-4;
  }

  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[STATE_YAW] = tools::limit_rad(x_prior[STATE_YAW]);
    return x_prior;
  };

  ekf_.predict(F, Q, f);
}

void Target::update(const Armor & armor)
{
  if (is_outpost()) {
    // ---- Compute observation angle relative to center ----
    auto center_yaw = std::atan2(ekf_.x[STATE_Y], ekf_.x[STATE_X]);
    auto armor_delta = std::abs(tools::limit_rad(armor.ypr_in_world[0] - center_yaw));

    constexpr double SWITCH_ANGLE_THRESHOLD = 60.0 * CV_PI / 180.0;  // rad
    constexpr int MIN_SWITCH_INTERVAL = 5;  // frames

    int best_id;

    // ---- Matching strategy ----
    // At steep viewing angles (>60°) the PnP yaw solution is noisy and
    // angle-based matching becomes unreliable.  When direction has been
    // identified, use the expected next plate instead of noisy angle cost.
    bool use_direction_switch =
      armor_delta > SWITCH_ANGLE_THRESHOLD &&
      outpost_id_.state == OutpostPlateId::State::kIdentified &&
      frames_since_switch_ >= MIN_SWITCH_INTERVAL;

    if (use_direction_switch) {
      // Direction-predicted switch: CW → slot advances by 1, CCW → by 2.
      int shift = (outpost_id_.direction > 0) ? 1 : 2;
      best_id = (last_id + shift) % armor_num_;
    } else {
      // Normal angle-based min-cost matching.
      std::array<double, 3> costs{};
      for (int i = 0; i < armor_num_; i++) {
        costs[i] = outpost_match_cost(armor, i);
      }
      best_id = 0;
      auto best_cost = costs[0];
      for (int i = 1; i < armor_num_; i++) {
        if (costs[i] < best_cost) {
          best_cost = costs[i];
          best_id = i;
        }
      }
    }

    // ---- Hysteresis: prevent rapid flip-flopping ----
    if (best_id != last_id && frames_since_switch_ < MIN_SWITCH_INTERVAL) {
      best_id = last_id;  // stay locked
    }

    // ---- Handle slot transition ----
    if (best_id != last_id) {
      frames_since_switch_ = 0;
      switch_count_++;
      outpost_identify(armor.xyz_in_world[2]);

      // Fast-converge the height state to the measurement on the
      // transition frame.  Without this the EKF takes 3-5 frames to
      // converge and the predicted Z visibly jumps during that lag.
      double measured_h = armor.xyz_in_world[2] - ekf_.x[STATE_Z];
      measured_h = std::clamp(measured_h, OUTPOST_HEIGHT_MIN, OUTPOST_HEIGHT_MAX);
      ekf_.x[outpost_height_index(best_id)] = measured_h;
    } else {
      frames_since_switch_++;
    }

    // ---- Cross-validate identified direction against EKF VYAW ----
    if (
      outpost_id_.state == OutpostPlateId::State::kIdentified && convergened() &&
      std::abs(ekf_.x[STATE_VYAW]) > OUTPOST_DIRECTION_GATING_SPEED) {
      int vyaw_dir = (ekf_.x[STATE_VYAW] > 0) ? -1 : 1;
      if (vyaw_dir != outpost_id_.direction) {
        tools::logger()->debug(
          "[Target] Outpost id direction mismatch — resetting identification");
        outpost_id_ = {};
      }
    }

    // ---- Rotate reference: keep slot 0 tracking the visible plate ----
    // Only rotate when the match is reliable (not at steep angle), so the
    // permutation never operates on a wrong match.
    // if (best_id != 0 && !use_direction_switch) {
    if (best_id != 0) {
      jumped = true;
      rotate_outpost_reference(best_id);
      best_id = 0;
    }

    if (switch_count_ > 0) jumped = true;

    last_id = best_id;
    update_count_++;
    update_ypda(armor, best_id);
    return;
  }

  int id = 0;
  auto min_angle_error = 1e10;
  const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();

  std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
  for (int i = 0; i < armor_num_; i++) {
    xyza_i_list.push_back({xyza_list[i], i});
  }

  std::sort(
    xyza_i_list.begin(), xyza_i_list.end(),
    [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
      Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));
      Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
      return ypd1[2] < ypd2[2];
    });

  for (int i = 0; i < 3; i++) {
    const auto & xyza = xyza_i_list[i].first;
    Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
    auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
                       std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));

    if (std::abs(angle_error) < std::abs(min_angle_error)) {
      id = xyza_i_list[i].second;
      min_angle_error = angle_error;
    }
  }

  if (id != 0) jumped = true;

  is_switch_ = id != last_id;
  if (is_switch_) switch_count_++;

  last_id = id;
  update_count_++;

  update_ypda(armor, id);
}

void Target::update(const std::vector<Armor> & armors)
{
  if (armors.empty()) return;

  const auto armor_count = std::min(static_cast<int>(armors.size()), armor_num_);

  if (!is_outpost()) {
    for (int i = 0; i < armor_count; i++) update(armors[i]);
    return;
  }

  if (armor_count == 1) {
    update(armors.front());
    return;
  }

  struct MatchResult
  {
    double cost = std::numeric_limits<double>::max();
    double z_base = 0.0;
    std::vector<int> ids;
  } best_match;

  // DFS enumerates all assignments of detected armors to slots.
  // Uses angle-dominant cost (120° separation ensures correctness).
  {
    std::vector<int> current_ids(armor_count, -1);
    std::vector<bool> used_ids(armor_num_, false);

    auto dfs = [&](auto && self, int armor_index) -> void {
      if (armor_index == armor_count) {
        double z_base = 0.0;
        for (int i = 0; i < armor_count; i++) {
          z_base +=
            armors[i].xyz_in_world[2] - outpost_height_offset(ekf_.x, current_ids[i]);
        }
        z_base /= armor_count;

        double total_cost = 0.0;
        for (int i = 0; i < armor_count; i++) {
          total_cost += outpost_match_cost(armors[i], current_ids[i]);
        }

        if (total_cost < best_match.cost) {
          best_match = {total_cost, z_base, current_ids};
        }
        return;
      }

      for (int id = 0; id < armor_num_; id++) {
        if (used_ids[id]) continue;

        used_ids[id] = true;
        current_ids[armor_index] = id;
        self(self, armor_index + 1);
        used_ids[id] = false;
      }
    };

    dfs(dfs, 0);
  }

  // Consensus z_base to refine center height.
  ekf_.x[STATE_Z] = best_match.z_base;

  // Seed outpost identification from the first multi-armor observation.
  if (outpost_id_.state == OutpostPlateId::State::kInit) {
    int seed_idx = -1;
    for (int i = 0; i < armor_count; i++) {
      if (best_match.ids[i] == 0) { seed_idx = i; break; }
    }
    if (seed_idx < 0) seed_idx = 0;

    outpost_id_.prev_z = armors[seed_idx].xyz_in_world[2];
    outpost_id_.first_z = outpost_id_.prev_z;
    outpost_id_.state = OutpostPlateId::State::kFirstObserved;
    outpost_id_.transition_count = 1;
    tools::logger()->debug(
      "[Target] Outpost id seeded from multi-armor: ref_z={:.3f}", outpost_id_.prev_z);
  }

  // Fast-converge heights on each matched slot, then update via EKF.
  for (int i = 0; i < armor_count; i++) {
    int id = best_match.ids[i];

    // Trigger identification on non-zero slot matches.
    if (id != 0) outpost_identify(armors[i].xyz_in_world[2]);

    // Directly set height on the first observation for this slot.
    double measured_h = armors[i].xyz_in_world[2] - ekf_.x[STATE_Z];
    measured_h = std::clamp(measured_h, OUTPOST_HEIGHT_MIN, OUTPOST_HEIGHT_MAX);
    ekf_.x[outpost_height_index(id)] = measured_h;

    // EKF measurement update.
    update_ypda(armors[i], id);
  }

  // Cross-validate identification against EKF VYAW.
  if (
    outpost_id_.state == OutpostPlateId::State::kIdentified && convergened() &&
    std::abs(ekf_.x[STATE_VYAW]) > OUTPOST_DIRECTION_GATING_SPEED) {
    int vyaw_dir = (ekf_.x[STATE_VYAW] > 0) ? -1 : 1;
    if (vyaw_dir != outpost_id_.direction) {
      tools::logger()->debug(
        "[Target] Outpost id direction mismatch — resetting identification");
      outpost_id_ = {};
    }
  }

  if (switch_count_ > 0) jumped = true;
  if (!best_match.ids.empty()) last_id = best_match.ids[0];
  update_count_ += armor_count;
}

bool Target::is_outpost() const { return name == ArmorName::outpost && armor_num_ == 3; }

void Target::rotate_outpost_reference(int id)
{
  if (!is_outpost() || id <= 0 || id >= armor_num_) return;

  Eigen::MatrixXd permutation = Eigen::MatrixXd::Identity(kStateSize, kStateSize);
  for (int row = 0; row < armor_num_; row++) {
    for (int col = 0; col < armor_num_; col++) {
      permutation(outpost_height_index(row), outpost_height_index(col)) = 0.0;
    }
  }
  for (int new_slot = 0; new_slot < armor_num_; new_slot++) {
    const int old_slot = (new_slot + id) % armor_num_;
    permutation(outpost_height_index(new_slot), outpost_height_index(old_slot)) = 1.0;
  }

  ekf_.x[STATE_YAW] = tools::limit_rad(ekf_.x[STATE_YAW] + id * 2 * CV_PI / armor_num_);
  ekf_.x = permutation * ekf_.x;
  rotate_outpost_covariance(id);
}

double Target::outpost_match_cost(const Armor & armor, int id) const
{
  // Compute predicted angle/xyz for the given slot directly (armor_xyza_list()
  // only returns the visible plate so can't be indexed by arbitrary id).
  auto angle = tools::limit_rad(ekf_.x[STATE_YAW] + id * 2 * CV_PI / armor_num_);
  Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, id);

  // Angle is the ONLY reliable discriminator — slots are 120° apart,
  // so the correct slot always has ~0 angle error while wrong slots have ~2.09 rad.
  const auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - angle));

  // z_error as a weak tiebreaker (rarely needed since angle_error already separates slots).
  const auto z_error = std::abs(armor.xyz_in_world[2] - xyz[2]) / OUTPOST_HEIGHT_STEP;

  return angle_error + 0.1 * z_error;
}

int Target::outpost_expected_shift() const
{
  // Prioritize identified direction (available after 2–3 transitions),
  // fall back to VYAW-based prediction.
  if (outpost_id_.state == OutpostPlateId::State::kIdentified) {
    return (outpost_id_.direction > 0) ? 1 : 2;  // CW→1, CCW→2
  }
  if (ekf_.x[STATE_VYAW] > 0) return 2;
  if (ekf_.x[STATE_VYAW] < 0) return 1;
  return -1;
}

int Target::outpost_height_index(int id) const { return kOutpostHeightBaseIndex + id; }

double Target::outpost_height_offset(const Eigen::VectorXd & x, int id) const
{
  return x[outpost_height_index(id)];
}

void Target::rotate_outpost_covariance(int id)
{
  Eigen::MatrixXd permutation = Eigen::MatrixXd::Identity(kStateSize, kStateSize);
  for (int row = 0; row < armor_num_; row++) {
    for (int col = 0; col < armor_num_; col++) {
      permutation(outpost_height_index(row), outpost_height_index(col)) = 0.0;
    }
  }
  for (int new_slot = 0; new_slot < armor_num_; new_slot++) {
    const int old_slot = (new_slot + id) % armor_num_;
    permutation(outpost_height_index(new_slot), outpost_height_index(old_slot)) = 1.0;
  }
  ekf_.P = permutation * ekf_.P * permutation.transpose();
}

void Target::outpost_identify(double measured_z)
{
  auto & id = outpost_id_;
  if (id.state == OutpostPlateId::State::kIdentified) return;

  double dh = measured_z - id.prev_z;
  double abs_dh = std::abs(dh);

  switch (id.state) {
    case OutpostPlateId::State::kInit:
      id.first_z = measured_z;
      id.state = OutpostPlateId::State::kFirstObserved;
      id.transition_count = 1;
      tools::logger()->debug(
        "[Target] Outpost init: first_z={:.3f}", measured_z);
      break;

    case OutpostPlateId::State::kFirstObserved: {
      id.transition_count = 2;
      if (abs_dh > OUTPOST_GAP_HIGH_THRESHOLD) {
        // HIGH gap (~0.20m): the two plates span the full height range (lowest↔highest).
        // Observed sequence analysis: lowest→highest is +0.20, which occurs in CW rotation.
        // So: +dh → CW(+1), -dh → CCW(-1)
        int new_dir = (dh > 0) ? 1 : -1;

        // Cross-validate: if EKF has a stable VYAW, check consistency.
        if (convergened() && std::abs(ekf_.x[STATE_VYAW]) > OUTPOST_DIRECTION_GATING_SPEED) {
          int vyaw_dir = (ekf_.x[STATE_VYAW] > 0) ? -1 : 1;
          if (vyaw_dir != new_dir) {
            tools::logger()->debug(
              "[Target] Outpost HIGH gap direction mismatch EKF — resetting id "
              "(dh={:.3f}, id_dir={}, vyaw_dir={})",
              dh, (new_dir > 0) ? "CW" : "CCW", (vyaw_dir > 0) ? "CW" : "CCW");
            outpost_id_ = {};
            return;
          }
        }

        id.direction = new_dir;
        id.state = OutpostPlateId::State::kIdentified;
        tools::logger()->debug(
          "[Target] Outpost identified (HIGH gap): dh={:.3f}, dir={}, transitions={}",
          dh, (id.direction > 0) ? "CW" : "CCW", id.transition_count);
      } else if (abs_dh > OUTPOST_GAP_LOW_THRESHOLD) {
        // LOW gap (~0.10m): adjacent sorted plates (lowest→middle or middle→highest).
        // Observed sequence analysis: lowest→middle is +0.10, which occurs in CCW rotation.
        // So: +dh → CCW(-1), -dh → CW(+1)
        int new_dir = (dh > 0) ? -1 : 1;

        // Cross-validate against EKF VYAW if available.
        if (convergened() && std::abs(ekf_.x[STATE_VYAW]) > OUTPOST_DIRECTION_GATING_SPEED) {
          int vyaw_dir = (ekf_.x[STATE_VYAW] > 0) ? -1 : 1;
          if (vyaw_dir != new_dir) {
            tools::logger()->debug(
              "[Target] Outpost LOW gap direction mismatch EKF — resetting id "
              "(dh={:.3f}, id_dir={}, vyaw_dir={})",
              dh, (new_dir > 0) ? "CW" : "CCW", (vyaw_dir > 0) ? "CW" : "CCW");
            outpost_id_ = {};
            return;
          }
        }

        id.direction = new_dir;
        id.state = OutpostPlateId::State::kSecondObserved;
        tools::logger()->debug(
          "[Target] Outpost LOW gap: dh={:.3f}, tentative dir={}, waiting for 3rd plate",
          dh, (id.direction > 0) ? "CW" : "CCW");
      } else {
        // Gap too small — likely re-observing the same plate or noise.
        // Stay in kFirstObserved, keep first_z unchanged.
        tools::logger()->debug(
          "[Target] Outpost gap too small: dh={:.3f}, staying in kFirstObserved", dh);
        return;  // Don't update prev_z — keep the original reference
      }
      break;
    }

    case OutpostPlateId::State::kSecondObserved: {
      id.transition_count = 3;
      // Third plate observed — confirms the tentative direction from step 2.
      // Cross-validate with EKF VYAW before confirming.
      if (convergened() && std::abs(ekf_.x[STATE_VYAW]) > OUTPOST_DIRECTION_GATING_SPEED) {
        int vyaw_dir = (ekf_.x[STATE_VYAW] > 0) ? -1 : 1;
        if (vyaw_dir != id.direction) {
          tools::logger()->debug(
            "[Target] Outpost 3rd-plate direction mismatch EKF — resetting id "
            "(dir={}, vyaw_dir={})",
            (id.direction > 0) ? "CW" : "CCW", (vyaw_dir > 0) ? "CW" : "CCW");
          outpost_id_ = {};
          return;
        }
      }
      id.state = OutpostPlateId::State::kIdentified;
      tools::logger()->debug(
        "[Target] Outpost identified (3-plate confirm): dh={:.3f}, dir={}, transitions={}",
        dh, (id.direction > 0) ? "CW" : "CCW", id.transition_count);
      break;
    }

    case OutpostPlateId::State::kIdentified:
      break;
  }
  id.prev_z = measured_z;
}

void Target::update_ypda(const Armor & armor, int id)
{
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
  auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
  Eigen::VectorXd R_dig{
    {4e-3, 4e-3, log(std::abs(delta_angle) + 1) + 1,
     log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2}};

  Eigen::MatrixXd R = R_dig.asDiagonal();

  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
    auto angle = tools::limit_rad(x[STATE_YAW] + id * 2 * CV_PI / armor_num_);
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  const Eigen::VectorXd & ypd = armor.ypd_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};

  ekf_.update(z, H, R, h, z_subtract);
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> xyza_list;

  // Outpost: return all 3 modeled plates (green boxes for visualization).
  // The aimer chooses which one to predict (red box) based on closest-to-camera.
  if (is_outpost()) {
    for (int i = 0; i < armor_num_; i++) {
      auto angle = tools::limit_rad(ekf_.x[STATE_YAW] + i * 2 * CV_PI / armor_num_);
      Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
      xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
    }
    return xyza_list;
  }

  for (int i = 0; i < armor_num_; i++) {
    auto angle = tools::limit_rad(ekf_.x[STATE_YAW] + i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
    xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return xyza_list;
}

bool Target::diverged() const
{
  auto r_ok = ekf_.x[STATE_R] > 0.05 && ekf_.x[STATE_R] < 0.5;
  if (is_outpost()) {
    std::array<double, 3> heights{
      ekf_.x[outpost_height_index(0)], ekf_.x[outpost_height_index(1)],
      ekf_.x[outpost_height_index(2)]};
    std::sort(heights.begin(), heights.end());

    const bool heights_in_range =
      heights.front() > OUTPOST_HEIGHT_MIN && heights.back() < OUTPOST_HEIGHT_MAX;
    const bool gaps_reasonable =
      heights[1] - heights[0] > OUTPOST_HEIGHT_MIN_GAP &&
      heights[2] - heights[1] > OUTPOST_HEIGHT_MIN_GAP &&
      heights[1] - heights[0] < OUTPOST_HEIGHT_MAX_GAP &&
      heights[2] - heights[1] < OUTPOST_HEIGHT_MAX_GAP;

    if (r_ok && heights_in_range && gaps_reasonable) return false;

    tools::logger()->debug(
      "[Target] outpost diverged r={:.3f}, heights=[{:.3f}, {:.3f}, {:.3f}]",
      ekf_.x[STATE_R], heights[0], heights[1], heights[2]);
    return true;
  }

  auto l_ok = ekf_.x[STATE_R] + ekf_.x[STATE_L] > 0.05 && ekf_.x[STATE_R] + ekf_.x[STATE_L] < 0.5;

  if (r_ok && l_ok) return false;

  tools::logger()->debug(
    "[Target] r={:.3f}, l={:.3f}", ekf_.x[STATE_R], ekf_.x[STATE_R] + ekf_.x[STATE_L]);
  return true;
}

bool Target::convergened()
{
  if (this->name != ArmorName::outpost && update_count_ > 3 && !this->diverged()) {
    is_converged_ = true;
  }

  if (this->name == ArmorName::outpost && update_count_ > 10 && !this->diverged()) {
    is_converged_ = true;
  }

  return is_converged_;
}

Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  if (is_outpost()) {
    auto angle = tools::limit_rad(x[STATE_YAW] + id * 2 * CV_PI / armor_num_);
    auto armor_x = x[STATE_X] - x[STATE_R] * std::cos(angle);
    auto armor_y = x[STATE_Y] - x[STATE_R] * std::sin(angle);
    auto armor_z = x[STATE_Z] + outpost_height_offset(x, id);

    return {armor_x, armor_y, armor_z};
  }

  auto angle = tools::limit_rad(x[STATE_YAW] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[STATE_R] + x[STATE_L] : x[STATE_R];
  auto armor_x = x[STATE_X] - r * std::cos(angle);
  auto armor_y = x[STATE_Y] - r * std::sin(angle);
  auto armor_z = (use_l_h) ? x[STATE_Z] + x[STATE_H] : x[STATE_Z];


  return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  if (is_outpost()) {
    auto angle = tools::limit_rad(x[STATE_YAW] + id * 2 * CV_PI / armor_num_);
    auto dx_da = x[STATE_R] * std::sin(angle);
    auto dy_da = -x[STATE_R] * std::cos(angle);
    auto dx_dr = -std::cos(angle);
    auto dy_dr = -std::sin(angle);

    Eigen::MatrixXd H_armor_xyza = Eigen::MatrixXd::Zero(4, kStateSize);
    H_armor_xyza(0, STATE_X) = 1.0;
    H_armor_xyza(0, STATE_YAW) = dx_da;
    H_armor_xyza(0, STATE_R) = dx_dr;
    H_armor_xyza(1, STATE_Y) = 1.0;
    H_armor_xyza(1, STATE_YAW) = dy_da;
    H_armor_xyza(1, STATE_R) = dy_dr;
    H_armor_xyza(2, STATE_Z) = 1.0;
    H_armor_xyza(2, outpost_height_index(id)) = 1.0;
    H_armor_xyza(3, STATE_YAW) = 1.0;

    Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
    Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
    Eigen::MatrixXd H_armor_ypda{
      {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
      {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
      {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
      {0, 0, 0, 1}
    };

    return H_armor_ypda * H_armor_xyza;
  }

  auto angle = tools::limit_rad(x[STATE_YAW] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[STATE_R] + x[STATE_L] : x[STATE_R];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;
  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  Eigen::MatrixXd H_armor_xyza = Eigen::MatrixXd::Zero(4, kStateSize);
  H_armor_xyza(0, STATE_X) = 1.0;
  H_armor_xyza(0, STATE_YAW) = dx_da;
  H_armor_xyza(0, STATE_R) = dx_dr;
  H_armor_xyza(0, STATE_L) = dx_dl;
  H_armor_xyza(1, STATE_Y) = 1.0;
  H_armor_xyza(1, STATE_YAW) = dy_da;
  H_armor_xyza(1, STATE_R) = dy_dr;
  H_armor_xyza(1, STATE_L) = dy_dl;
  H_armor_xyza(2, STATE_Z) = 1.0;
  H_armor_xyza(2, STATE_H) = dz_dh;
  H_armor_xyza(3, STATE_YAW) = 1.0;

  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {0, 0, 0, 1}
  };

  return H_armor_ypda * H_armor_xyza;
}

bool Target::checkinit() { return isinit; }

}  // namespace auto_aim
