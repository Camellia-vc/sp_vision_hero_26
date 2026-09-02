#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::atomic<bool> quit = false;
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      auto target = target_queue.front();
      auto gs = gimbal.state();
      auto plan = planner.plan(target, gs.bullet_speed);

      gimbal.send(
        plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
        plan.pitch_acc, plan.distance);

      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
      data["rx_mode"] = static_cast<int>(gimbal.mode());
      data["tx_mode"] = plan.control ? (plan.fire ? 2 : 1) : 0;

      data["gimbal_yaw"] = gs.yaw;
      data["gimbal_yaw_vel"] = gs.yaw_vel;
      data["gimbal_pitch"] = gs.pitch;
      data["gimbal_pitch_vel"] = gs.pitch_vel;

      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;
      data["target_distance"] = plan.distance;

      data["plan_yaw"] = plan.yaw;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;

      data["plan_pitch"] = plan.pitch;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      data["fire"] = plan.fire ? 1 : 0;
      data["fired"] = fired ? 1 : 0;

      if (target.has_value()) {
        data["target_z"] = target->ekf_x()[4];   //z
        data["target_vz"] = target->ekf_x()[5];  //vz
        data["tracked_armor_id"] = target->last_id;

        const auto armor_xyza_list = target->armor_xyza_list();
        if (!armor_xyza_list.empty()) {
          data["armor_0_z"] = armor_xyza_list[0][2];
          data["armor_0_yaw"] = armor_xyza_list[0][3];
        }
        if (armor_xyza_list.size() > 1) {
          data["armor_1_z"] = armor_xyza_list[1][2];
          data["armor_1_yaw"] = armor_xyza_list[1][3];
        }
        if (armor_xyza_list.size() > 2) {
          data["armor_2_z"] = armor_xyza_list[2][2];
          data["armor_2_yaw"] = armor_xyza_list[2][3];
        }
        if (armor_xyza_list.size() > 3) {
          data["armor_3_z"] = armor_xyza_list[3][2];
          data["armor_3_yaw"] = armor_xyza_list[3][3];
        }
      }

      if (target.has_value()) {
        data["w"] = target->ekf_x()[7];
      } else {
        data["w"] = 0.0;
      }

      data["aim_z"] = planner.debug_xyza[2];

      plotter.plot(data);

      std::this_thread::sleep_for(10ms);
    }
  });

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  auto last_enemy_color = static_cast<uint8_t>(0xFF);
  auto fps_t0 = std::chrono::steady_clock::now();
  int fps_frame_count = 0;
  double fps = 0;

  while (!exiter.exit()) {
    camera.read(img, t);
    auto q = gimbal.q(t);
    auto gs = gimbal.state();
    if (gs.enemy_color != last_enemy_color) {
      tracker.set_enemy_color(
        gs.enemy_color == 0 ? auto_aim::Color::red : auto_aim::Color::blue);
      last_enemy_color = gs.enemy_color;
    }

    solver.set_R_gimbal2world(q);
    auto armors = yolo.detect(img);
    auto targets = tracker.track(armors, t);
    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);

    if (!targets.empty()) {
      auto target = targets.front();

      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      Eigen::Vector4d aim_xyza = planner.debug_xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      tools::draw_points(img, image_points, {0, 0, 255});
    }

    // 帧率计算
    fps_frame_count++;
    auto fps_now = std::chrono::steady_clock::now();
    auto fps_dt = tools::delta_time(fps_now, fps_t0);
    if (fps_dt >= 1.0) {
      fps = fps_frame_count / fps_dt;
      fps_frame_count = 0;
      fps_t0 = fps_now;
    }
    tools::draw_text(img, fmt::format("FPS: {:.1f}", fps), {10, 30}, {0, 255, 0});
    tools::draw_text(
      img, fmt::format("[{}]", tracker.state()), {10, 60}, {255, 255, 255});

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}
