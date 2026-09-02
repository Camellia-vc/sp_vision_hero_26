#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

class Target
{
public:
  ArmorName name{ArmorName::not_armor};
  ArmorType armor_type{ArmorType::small};
  ArmorPriority priority{ArmorPriority::fifth};
  bool jumped{false};
  int last_id{0};  // debug only

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig);
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void update(const Armor & armor);
  void update(const std::vector<Armor> & armors);

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  bool diverged() const;

  bool convergened();

  bool isinit = false;

  bool checkinit();

private:
  static constexpr int kStateSize = 13;
  static constexpr int kOutpostHeightBaseIndex = 10;

  int armor_num_;
  int switch_count_;
  int update_count_;
  int frames_since_switch_{999};  // hysteresis cooldown for outpost slot switching

  bool is_switch_, is_converged_;

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_{std::chrono::steady_clock::now()};

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle
  bool is_outpost() const;
  void rotate_outpost_reference(int id);
  double outpost_match_cost(const Armor & armor, int id) const;
  int outpost_expected_shift() const;
  int outpost_height_index(int id) const;
  double outpost_height_offset(const Eigen::VectorXd & x, int id) const;
  void rotate_outpost_covariance(int id);

  // ---- Outpost plate identification via height-difference sequencing (per flowchart) ----
  struct OutpostPlateId {
    enum class State { kInit, kFirstObserved, kSecondObserved, kIdentified };
    State state = State::kInit;
    int direction = 0;         // +1 = CW, -1 = CCW, 0 = unknown
    double prev_z = 0.0;       // Z of previously observed plate
    double first_z = 0.0;      // Z of first plate in current sequence (for 3-plate case)
    int transition_count = 0;
  };
  OutpostPlateId outpost_id_;

  void outpost_identify(double measured_z);

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP
