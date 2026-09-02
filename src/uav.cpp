#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/dm_imu/dm_imu.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

const std::string keys =
  "{help h usage ? |                  | 输出命令行参数说明}"
  "{@config-path   | configs/uav.yaml | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  auto_aim::Detector detector(config_path);
  auto_aim::Solver solver(config_path);
  // auto_aim::YOLO yolo(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  auto_buff::Buff_Detector buff_detector(config_path);
  auto_buff::Solver buff_solver(config_path);
  auto_buff::SmallTarget buff_small_target;
  auto_buff::BigTarget buff_big_target;
  auto_buff::Aimer buff_aimer(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::GimbalMode::IDLE;
  auto last_mode = io::GimbalMode::IDLE;
  auto last_enemy_color = static_cast<uint8_t>(0xFF);

  while (!exiter.exit()) {
    camera.read(img, t);
    q = gimbal.q(t);
    mode = gimbal.mode();
    auto gs = gimbal.state();
    if (gs.enemy_color != last_enemy_color) {
      tracker.set_enemy_color(
        gs.enemy_color == 0 ? auto_aim::Color::red : auto_aim::Color::blue);
      last_enemy_color = gs.enemy_color;
    }
    // recorder.record(img, q, t);
    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", gimbal.str(mode));
      last_mode = mode;
    }

    /// 自瞄
    if (mode == io::GimbalMode::AUTO_AIM) {
      solver.set_R_gimbal2world(q);

      Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

      auto armors = detector.detect(img);

      auto targets = tracker.track(armors, t);

      auto command = aimer.aim(targets, t, gs.bullet_speed);

      command.shoot = shooter.shoot(command, aimer, targets, ypr);

      gimbal.send(command);
    }

    /// 打符
    else if (mode == io::GimbalMode::SMALL_BUFF || mode == io::GimbalMode::BIG_BUFF) {
      buff_solver.set_R_gimbal2world(q);

      auto power_runes = buff_detector.detect(img);

      buff_solver.solve(power_runes);

      io::Command buff_command;
      if (mode == io::GimbalMode::SMALL_BUFF) {
        buff_small_target.get_target(power_runes, t);
        auto target_copy = buff_small_target;
        buff_command = buff_aimer.aim(target_copy, t, gs.bullet_speed, true);
      } else if (mode == io::GimbalMode::BIG_BUFF) {
        buff_big_target.get_target(power_runes, t);
        auto target_copy = buff_big_target;
        buff_command = buff_aimer.aim(target_copy, t, gs.bullet_speed, true);
      }
      gimbal.send(buff_command);
    } else {
      gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
    }
  }

  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
  return 0;
}
