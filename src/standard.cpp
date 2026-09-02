#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

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

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", gimbal.str(mode));
      last_mode = mode;
    }

    if (mode != io::GimbalMode::AUTO_AIM) {
      gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
      continue;
    }

    solver.set_R_gimbal2world(q);

    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = detector.detect(img);

    auto targets = tracker.track(armors, t);

    auto command = aimer.aim(targets, t, gs.bullet_speed);
    command.shoot = shooter.shoot(command, aimer, targets, ypr);

    gimbal.send(command.control, command.shoot, command.yaw, 0, 0, command.pitch, 0, 0);
  }

  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}
