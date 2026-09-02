#ifndef TOOLS__TRAJECTORY_HPP
#define TOOLS__TRAJECTORY_HPP


namespace tools
{
typedef float fp32;
// 弹道计算结构体
typedef struct
{
  // 当前弹速
  float current_bullet_speed;
  // 弹道系数
  float k1;
  // 子弹飞行时间
  float flight_time;
  //计算角度
  float theta;

} solve_trajectory_t;

struct Trajectory
{
  bool unsolvable;
  double fly_time;
  double pitch;  // 抬头为正

  // 不考虑空气阻力
  // v0 子弹初速度大小，单位：m/s
  // d 目标水平距离，单位：m
  // h 目标竖直高度，单位：m
  Trajectory(const double v0, const double d, const double h);
};

}  // namespace tools

#endif  // TOOLS__TRAJECTORY_HPP