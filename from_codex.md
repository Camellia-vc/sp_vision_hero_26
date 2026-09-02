我把主程序入口、配置、通信层、测试程序和调试工具都过了一遍。结论先说：

这个仓库不是“只有一个赛场二进制”，而是按兵种分了多套入口。CMakeLists.txt (line 30) 里步兵有 standard / mt_standard / standard_mpc，无人机有 uav，哨兵有 sentry / sentry_bp / sentry_debug / sentry_multithread，CMakeLists.txt (line 112) 还写明哨兵这组只有在 ROS2 和 sp_msgs 都存在时才会编译。你当前这个工作区里，build/ 实际已经编出的只有 standard、mt_standard、standard_mpc、auto_aim_debug_mpc、mt_auto_aim_debug、uav、uav_debug，没有 sentry*。所以如果你问“当前这份工程最像真正上场在跑的哪个”，我会给两个层次的答案：

对 25 赛季国赛里文档明确提到的 3/4 号步兵，最像实战主程序的是 standard_mpc，配 configs/standard3.yaml (line 1) 或 configs/standard4.yaml。理由是 README 明确说国赛上场的是 3、4 号步兵，readme.md (line 39)；而 standard_mpc 正是文档主打的轨迹规划版入口，src/standard_mpc.cpp (line 44)。
如果你问的是你现在打开的这个文件对应哪个二进制，那就是 sentry_bp，入口在 src/sentry_bp.cpp (line 32)。但这套哨兵程序在你当前环境里没编出来，因为缺 ROS2 依赖。
串口 / CAN 都传什么

新串口链路是 io::Gimbal，给步兵新链路用。协议定义在 io/gimbal/gimbal.hpp (line 17)。

下位机发给视觉 GimbalToVision：
head='SP'、mode、四元数 q[4]（wxyz）、yaw、yaw_vel、pitch、pitch_vel、bullet_speed、bullet_count、crc16。io/gimbal/gimbal.hpp (line 17)

视觉发给下位机 VisionToGimbal：
head='SP'、mode（0不控/1控云台不开火/2控云台并开火）、yaw、yaw_vel、yaw_acc、pitch、pitch_vel、pitch_acc、distance、crc16。io/gimbal/gimbal.hpp (line 33)

实际发送时会计算 CRC16 后整包写串口，io/gimbal/gimbal.cpp (line 80) 和 io/gimbal/gimbal.cpp (line 99)。注意一个细节：常用的 send(control, fire, ...) 没有给 distance 赋值，所以这个字段当前大概率没被实际用上。io/gimbal/gimbal.cpp (line 99)

串口设备名来自 com_port，步兵配置里是 /dev/gimbal。configs/standard3.yaml (line 83)

旧链路/哨兵链路不是串口，是 CBoard 走 SocketCAN。发送和接收格式在 io/cboard.cpp (line 47)。

下位机到视觉：
quaternion_canid 帧里放 x,y,z,w，每个都是 int16 / 1e4；bullet_speed_canid 帧里放 bullet_speed、mode、shoot_mode、ft_angle。io/cboard.cpp (line 68)

视觉到下位机：
send_canid 帧 8 字节，依次是 control、shoot、yaw(int16/1e4)、pitch(int16/1e4)、horizon_distance(int16/1e4)。io/cboard.cpp (line 47)

哨兵配置里的 CAN ID 在 configs/sentry.yaml (line 54)，步兵旧 CAN 配置也保留在 configs/standard3.yaml (line 77)。

还有第三条独立串口：DM_IMU，主要给最小系统/测试程序用。它只收不发，921600 波特率，从 /dev/ttyACM0 读加速度、角速度、欧拉角三段帧，再转四元数。io/dm_imu/dm_imu.hpp (line 19) io/dm_imu/dm_imu.cpp (line 37)

怎么 debug

先查通信链路。
新串口步兵先跑 tests/gimbal_test.cpp (line 19)，它会持续发命令、看 yaw/pitch/bullet_speed/bullet_count，最适合先确认串口通没通。
旧 CAN/哨兵先跑 tests/cboard_test.cpp (line 17)，先确认四元数和弹速能不能稳定收到。
再查控制响应。
CAN 链路直接用 tests/gimbal_response_test.cpp (line 39)，它能发三角波、阶跃、圆轨迹并把 cmd_yaw/cmd_pitch 和实测 gimbal_yaw/gimbal_pitch 一起画出来。
再查算法。
步兵 MPC 调试首选 src/auto_aim_debug_mpc.cpp (line 28)，它会开图像窗口画重投影，还会把规划量通过 Plotter 发出去。
哨兵调试首选 src/sentry_debug.cpp (line 32)，它比 sentry_bp 多了 imshow、重投影、EKF 内部量和发射状态曲线；sentry_bp 本身更像运行版，不像调试版。src/sentry_bp.cpp (line 63)
曲线工具就是 PlotJuggler。代码里 Plotter 把 JSON 通过 UDP 发到 127.0.0.1:9870。tools/plotter.cpp (line 9) 仓库里已经给了布局文件 mpc_layout.xml 和 buff_layout.xml，README 也明确写了现场是 NoMachine + PlotJuggler 调参。readme.md (line 33)
日志和录包也都现成。
日志自动写到 logs/。tools/logger.cpp (line 15)
录像+四元数会写到 records/*.avi 和 records/*.txt。tools/recorder.cpp (line 18)
如果你要，我下一步可以继续直接帮你整理一份“你这台机器该跑哪条命令”的清单，按“步兵 / 哨兵 / 仅调串口 / 仅调识别”四种场景给你列出来。


我们使用gimbal而非cboard。请你帮我看一下哨兵和步兵在gimbal通讯协议下通讯接口是否一样，并且该运行什么文件
已处理 38s
结论很直接：你们现在如果走 gimbal，那步兵能直接跑，哨兵这套代码目前不能直接跑。

步兵这边已经完整接到了 io::Gimbal。入口见 standard.cpp (line 41)、standard_mpc.cpp (line 44)、auto_aim_debug_mpc.cpp (line 40)。它们都是 gimbal.q() 取姿态、gimbal.state() 取 bullet_speed/yaw/pitch、gimbal.send(...) 发控制。gimbal.hpp (line 67) 定义的收发协议对这些入口是统一的。

哨兵这边没有接 gimbal，还是全套 cboard。入口见 sentry.cpp (line 46) 和 sentry_bp.cpp (line 46)。它们都在用 cboard.imu_at()、cboard.bullet_speed、cboard.shoot_mode、cboard.send(command)，不是 gimbal 接口。而且哨兵的 aimer 还显式用了 shoot_mode 重载，sentry.cpp (line 93)；这个字段在 GimbalState 里根本没有，gimbal.hpp (line 57)。所以“哨兵和步兵在当前仓库里、在 gimbal 协议下接口是否一样”的答案是：不一样，因为哨兵实际上还没切到 gimbal。

如果只问“通信包格式一不一样”，那只要都走 io::Gimbal，底层包当然一样，都是 gimbal.hpp (line 17) 里的 GimbalToVision / VisionToGimbal。但当前代码并不是这种状态。

你现在该运行的文件：

实战步兵优先跑 ./build/standard_mpc configs/standard3.yaml，对应 standard_mpc.cpp (line 31)。
如果要看图像窗口和规划 debug，跑 ./build/auto_aim_debug_mpc configs/standard3.yaml，对应 auto_aim_debug_mpc.cpp (line 28)。
如果只是最基础的步兵自瞄，不带 MPC，跑 ./build/standard configs/standard3.yaml，对应 standard.cpp (line 28)。
不建议现在跑哨兵 sentry / sentry_bp，因为它们还是 cboard 路线，而且当前环境里也没编出来，哨兵目标还受 ROS2 条件限制。CMakeLists.txt (line 112)
如果你们下一步真要把哨兵改成 gimbal，最关键的阻塞点只有两个：

把 sentry.cpp (line 46) / sentry_bp.cpp (line 46) 的 io::CBoard 全部替换成 io::Gimbal 的 q()/state()/send() 流程。
处理 shoot_mode。要么扩展 gimbal 协议把左右枪信息也带上，要么先把哨兵改成不用 shoot_mode 的 aimer.aim(targets, timestamp, bullet_speed) 版本。aimer.hpp (line 23)
如果你愿意，我下一步可以直接帮你把 sentry_bp.cpp 改成一版可跑的 gimbal 版本。


./build/standard_mpc这个可以处理setmode吗



可以，但要分清“处理”是什么意思。

./build/standard_mpc 会响应下位机发来的 mode 切换，不会主动给下位机发一个“setmode”命令。它在主循环里一直读 gimbal.mode()，然后按模式分支处理：standard_mpc.cpp (line 90)

AUTO_AIM：走自瞄流程，standard_mpc.cpp (line 104)
SMALL_BUFF / BIG_BUFF：走打符流程，standard_mpc.cpp (line 114)
其他模式：发零指令，不控制，standard_mpc.cpp (line 136)
而这个 mode 来自串口协议里下位机发给视觉的 GimbalToVision.mode，gimbal.hpp (line 17)；接收后在 read_thread() 里映射成 GimbalMode::{IDLE,AUTO_AIM,SMALL_BUFF,BIG_BUFF}，gimbal.cpp (line 177)。

所以如果你说的 setmode 是：

下位机切档，让视觉跟着切：standard_mpc 可以。
视觉主动发指令去切下位机档位：standard_mpc 不可以，当前 VisionToGimbal 里也没有这个语义，gimbal.hpp (line 33)。
顺带一提，standard.cpp 只处理 AUTO_AIM，别的模式基本直接清零；要处理切自瞄/打符，应该跑 standard_mpc。





## 串口
当前总长度是 33 字节：

byte 0: head[0] = 'S'，十六进制 0x53
byte 1: head[1] = 'P'，十六进制 0x50
byte 2: mode
byte 3-6: yaw，float
byte 7-10: yaw_vel，float
byte 11-14: yaw_acc，float
byte 15-18: pitch，float
byte 19-22: pitch_vel，float
byte 23-26: pitch_acc，float
byte 27-30: distance，float
byte 31-32: crc16，uint16_t
mode 的含义是：

0: 不控制
1: 控制云台，不开火
2: 控制云台，开火
当前 standard_mpc 就是这样发的：standard_mpc.cpp (line 80)

yaw / pitch: 目标角度，单位是弧度
yaw_vel / pitch_vel: 角速度，单位是弧度每秒
yaw_acc / pitch_acc: 角加速度，单位是弧度每秒平方
distance: 现在已经改成“目标中心到自身的三维距离”，单位和世界坐标一致，通常就是米；无目标时默认发 -1：planner.hpp (line 19)
还有一个很重要的实现细节：

这包是 reinterpret_cast<uint8_t *>(&tx_data_) 直接写串口的，所以 float 和 uint16_t 都按本机内存字节序发
你这台机器现在是 x86_64，通常就是“小端”
所以下位机如果也是常见 STM32/小端平台，按同样结构体解析通常没问题
crc16 也是小端顺序发，也就是低字节在前、高字节在后


## 下位机发给上位机
的是 GimbalToVision，定义在 gimbal.hpp (line 17)，视觉接收和解析在 gimbal.cpp (line 132)。

这包同样是 packed 结构体，当前总长度是 43 字节：
下位机发给上位机的是 GimbalToVision，定义在 gimbal.hpp (line 17)，视觉接收和解析在 gimbal.cpp (line 132)。

这包同样是 packed 结构体，当前总长度是 43 字节：

byte 0: head[0] = 'S'，0x53
byte 1: head[1] = 'P'，0x50
byte 2: mode
byte 3-6: q[0]，四元数 w
byte 7-10: q[1]，四元数 x
byte 11-14: q[2]，四元数 y
byte 15-18: q[3]，四元数 z
byte 19-22: yaw，float
byte 23-26: yaw_vel，float
byte 27-30: pitch，float
byte 31-34: pitch_vel，float
byte 35-38: bullet_speed，float
byte 39-40: bullet_count，uint16_t
byte 41-42: crc16，uint16_t
mode 含义是：

0: 空闲
1: 自瞄
2: 小符
3: 大符
byte 0: head[0] = 'S'，0x53
byte 1: head[1] = 'P'，0x50
byte 2: mode
byte 3-6: q[0]，四元数 w
byte 7-10: q[1]，四元数 x
byte 11-14: q[2]，四元数 y
byte 15-18: q[3]，四元数 z
byte 19-22: yaw，float
byte 23-26: yaw_vel，float
byte 27-30: pitch，float
byte 31-34: pitch_vel，float
byte 35-38: bullet_speed，float
byte 39-40: bullet_count，uint16_t
byte 41-42: crc16，uint16_t
mode 含义是：

0: 空闲
1: 自瞄
2: 小符
3: 大符