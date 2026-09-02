#include "trajectory.hpp"

#include <cmath>
#include <cstdio>
#include <vector>
#include <iostream>
namespace tools
{
// constexpr double g = 9.7833;
constexpr double GRAVITY = 9.78;
constexpr double MAX_ITERATE_COUNT = 60;
constexpr double ITERATE_SCALE_FACTOR = 0.2;
constexpr double PRECISION = 0.0001;

/**
 * @brief 计算弹道落点 -- 完全空气阻力模型 该模型适用于大仰角击打的击打
 * @author yuanluochen
 *
 * @param solve_trajectory 弹道解算结构体
 * @param x 距离
 * @param bullet_speed 弹速
 * @param theta 仰角
 * @return 弹道落点
 */
static float calc_bullet_drop_in_complete_air(solve_trajectory_t *solve_trajectory, float x, float bullet_speed, float theta)
{
  // 子弹落点高度
  fp32 bullet_drop_z = 0;
  // 计算总飞行时间
  solve_trajectory->flight_time = (float)((exp(solve_trajectory->k1 * x) - 1) / (solve_trajectory->k1 * bullet_speed * cos(theta)));
  // printf("飞行时间%f", solve_trajectory->flight_time);
  if (theta > 0.02)
  {
    // 补偿空气阻力系数 对竖直方向
    // 上升过程中 子弹速度方向向量的角度逐渐趋近于0，竖直空气阻力 hat(f_z) = f_z * sin(theta) 会趋近于零 ，水平空气阻力 hat(f_x) = f_x * cos(theta) 会趋近于 f_x ，所以要对竖直空气阻力系数进行补偿
    fp32 k_z = solve_trajectory->k1 * (1 / sin(theta));
    // 上升段
    // 初始竖直飞行速度
    fp32 v_z_0 = bullet_speed * sin(theta);
    // 计算上升段最大飞行时间
    fp32 max_flight_up_time = (1 / sqrt(k_z * GRAVITY)) * atan(sqrt(k_z / GRAVITY) * v_z_0);
    // 判断总飞行时间是否小于上升最大飞行时间
    if (solve_trajectory->flight_time <= max_flight_up_time)
    {
      // 子弹存在上升段
      bullet_drop_z = (1 / k_z) * log(cos(sqrt(k_z * GRAVITY) * (max_flight_up_time - solve_trajectory->flight_time)) / cos(sqrt(k_z * GRAVITY) * max_flight_up_time));
    }
    else
    {
      // 超过最大上升飞行时间 -- 存在下降段
      // 计算最大高度
      fp32 z_max = (1 / (2 * k_z)) * log(1 + (k_z / GRAVITY) * pow(v_z_0, 2));
      // 计算下降
      bullet_drop_z = z_max - 0.5f * GRAVITY * pow((solve_trajectory->flight_time - max_flight_up_time), 2);
    }
  }
  else
  {
    bullet_drop_z = (float)(bullet_speed * sin(theta) * solve_trajectory->flight_time - 0.5f * GRAVITY * pow(solve_trajectory->flight_time, 2));
  }

  return bullet_drop_z;
}

/**
 * @brief 二维平面弹道模型，计算pitch轴的仰角，
 * @author yuanluochen
 *
 * @param solve_tragectory 弹道计算结构体
 * @param x 水平距离
 * @param y 竖直距离
 * @param x_offset 以机器人转轴坐标系为父坐标系，以发射最大速度点为子坐标系的x轴偏移量
 * @param y_offset 以机器人转轴坐标系为父坐标系，以发射最大速度点为子坐标系的y轴偏移量
 * @param bullet_speed 弹速
 * @param mode 计算模式：
          置 1 完全空气阻力模型
          置 0 单方向空气阻力模型
 * @return 返回pitch轴数值
 */
bool calc_target_position_pitch_angle(solve_trajectory_t *solve_trajectory, fp32 x, fp32 z, fp32 x_offset, fp32 z_offset){
  int count = 0;
  // 计算落点高度
  float bullet_drop_z = 0;
  // 云台瞄准向量
  float aim_z = z;


  // 二维平面的打击角
  float theta = 0;
  // 计算值与真实值之间的误差
  float calc_and_actual_error = 0;
  // 比例迭代法
  int i = 0;
  for (i = 0; i < MAX_ITERATE_COUNT; i++)
  {
    // 计算仰角
    theta = atan2(aim_z, x);
    // 坐标系变换，从机器人转轴系变为发射最大速度位置坐标系
    // 计算子弹落点高度
    //保证数值稳定性,角度太小
    bullet_drop_z =
        calc_bullet_drop_in_complete_air(
            solve_trajectory,
            x - (cos(theta) * x_offset -
                 sin(theta) * z_offset),
            solve_trajectory
            ->current_bullet_speed, theta) +
        (sin(theta) * x_offset + cos(theta) * z_offset);

    // 计算误差
    calc_and_actual_error = z - bullet_drop_z;
    // 对瞄准高度进行补偿
    aim_z += calc_and_actual_error * ITERATE_SCALE_FACTOR;
    // printf("第%d次瞄准，发射系x:%f, z补偿%f, z发射系落点%f ,z机体系落点%f\n", count, x - (cos(theta) * x_offset), (sin(theta) * x_offset + cos(theta) * z_offset), bullet_drop_z - (sin(theta) * x_offset + cos(theta) * z_offset), bullet_drop_z);
    if (!std::isfinite(theta) || !std::isfinite(bullet_drop_z) || !std::isfinite(aim_z) || !std::isfinite(solve_trajectory->flight_time)){
      solve_trajectory->flight_time = 0;
      solve_trajectory->theta = 0;
      return false;
    }
    // 判断误差是否符合精度要求
    count++;
    if (fabs(calc_and_actual_error) < PRECISION)
    {
      break;
    }
  }
  solve_trajectory->theta = theta;
  // printf("solver count i %d\n", i);
  if (i >= MAX_ITERATE_COUNT && std::fabs(calc_and_actual_error) > PRECISION * 1000)
  {
    return false;
  }
  return true;
}

Trajectory::Trajectory(const double v0, const double d, const double h)
{
  // std::cout << d << " " << h << std::endl;
  // auto a = g * d * d / (2 * v0 * v0);
  // auto b = -d;
  // auto c = a + h;
  // auto delta = b * b - 4 * a * c;

  // if (delta < 0) {
  //   unsolvable = true;
  //   return;
  // }

  // unsolvable = false;
  // auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
  // auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
  // auto pitch_1 = std::atan(tan_pitch_1);
  // auto pitch_2 = std::atan(tan_pitch_2);
  // auto t_1 = d / (v0 * std::cos(pitch_1));
  // auto t_2 = d / (v0 * std::cos(pitch_2));

  // pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
  // fly_time = (t_1 < t_2) ? t_1 : t_2;
  // std::cout<< "speed:" << v0 << std::endl;
  unsolvable = false;
  float k1 = 0.0086f;              //空气阻力系数 0.0086  0.004
  float x_offset = 0.133936f;          //惯性坐标系中心到摩擦轮中心的水平距离
  float z_offset = 0.018018f;         //惯性坐标系中心到摩擦轮中心的垂直距离
   solve_trajectory_t st = {
      float(v0),
      k1,
      0.0f,
      0.0f
    };
  bool issolved = calc_target_position_pitch_angle(&st, d, h, x_offset, z_offset);
  if(issolved != false)
     pitch = st.theta;
  if (issolved == false){
    // unsolvable = true;
    pitch = 0.1f;
  }
  fly_time = st.flight_time;
  // std::cout << "bullet_speed:" << v0<< "pitch:" << pitch << std::endl;
  
  // std::cout << pitch << std::endl;
}

}  // namespace tools