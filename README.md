# hero_vision_26 — 前哨站预测与选板逻辑说明

> 本文档**完全基于代码**整理，聚焦于「前哨站（Outpost）预测」与「选板（瞄准点选择）」两条核心逻辑，并在文末与上游仓库 [TongjiSuperPower/sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25) 做了逐项对比。

---

## 0. 项目定位

本仓库是**英雄机器人（Hero）**的视觉系统，由 `sp_vision_25` 演进而来。英雄在 RoboMaster 中的核心职责是**打前哨站（Outpost）和基地（Base）**，因此相对步兵/哨兵版本，本仓库对前哨站做了大量专门建模。

- 主入口：`src/standard.cpp`（直接解算）、`src/standard_mpc.cpp`（MPC 规划，实际部署入口，见 `start_sp_vision.sh` 运行 `./build/standard_mpc`）。
- 关键目录：`tasks/auto_aim/`（自瞄）、`tasks/auto_buff/`（打符）、`tasks/omniperception/`（全向感知/决策）、`io/`（云台/相机/串口/CAN 驱动）、`tools/`（EKF、弹道、数学库等）。

---

## 1. 整体数据流

自瞄主循环（`src/standard.cpp`）：

```
camera.read() → YOLO.detect() → Tracker.track() → Aimer.aim() → Shooter.shoot() → Gimbal.send()
```

| 阶段 | 类 | 作用 |
|------|-----|------|
| 检测 | `YOLO` / `Detector` | 从图像中检出装甲板（神经网络或传统灯条法），给颜色/编号/大小 |
| 位姿解算 | `Solver` | `solvePnP` + 重投影优化，得到装甲板在世界/云台系下的 `xyz`、`ypr` |
| 跟踪滤波 | `Tracker` + `Target`(EKF) | 状态机管理目标、EKF 估计整车旋转中心、半径、转速、**前哨站各板高度** |
| 瞄准选板 | `Aimer` | 预测目标未来位置 → `choose_aim_point` 选板 → 弹道解算 → 输出 yaw/pitch |
| 射击判定 | `Shooter` | 云台到位 + 指令稳定时给出射击允许 |
| 规划（MPC） | `Planner`(TinyMPC) | `standard_mpc.cpp` 中把目标位置转成 yaw/pitch 的加速度规划，产生开火时机 |

其中「前哨站预测」发生在 `Target`（EKF 建模），「选板」发生在 `Aimer::choose_aim_point` 与 `Target::update`（板匹配）。

---

## 2. 前哨站（Outpost）预测

### 2.1 前哨站的物理特点

前哨站是**固定不动的旋转体**，带 3 块装甲板，绕竖直轴以恒定角速度旋转（约 **0.8π rad/s ≈ 2.51 rad/s**）。与普通机器人装甲板的关键差异：

1. **中心不移动** —— 预测时中心 x/y/z 没有速度耦合。
2. **3 块板高度不同** —— 板之间存在竖直方向的高度差（代码用 `0.10 m` 步长描述，初始三块相对中心高度为 `0.0 / +0.20 / +0.10 m`）。这个高度差是识别「当前看到的是哪块板」「往哪个方向转」的**唯一可靠线索**（因为 3 块板夹角都是 120°，角度本身无法区分，PnP 解出的 yaw 在大角度下又很抖）。

### 2.2 EKF 状态向量（13 维）

`Target` 用扩展卡尔曼滤波器估计整车状态。见 [target.hpp:50-51](tasks/auto_aim/target.hpp#L50-L51) 与 [target.cpp:29-39](tasks/auto_aim/target.cpp#L29-L39)：

```
x = [ x, vx, y, vy, z, vz, yaw, vyaw, r, l, h0, h1, h2 ]   (共 13 维)
索引: 0   1   2   3   4   5    6    7    8   9   10  11  12
```

| 分量 | 含义 | 前哨站备注 |
|------|------|-----------|
| x/vx, y/vy, z/vz | 旋转中心位置/速度 | 前哨站固定，速度项不耦合 |
| yaw / vyaw | 整车朝向角 / 角速度 | 前哨站匀速旋转，vyaw≈0.8π |
| r | 旋转半径（中心到装甲板） | 前哨站 `r=0.2765 m` |
| l | 半径补偿（4 板机器人用） | 前哨站不使用 |
| **h0/h1/h2** | **3 块板的相对高度偏移** | **本仓库相对 sp25 新增的核心状态**，见 `kOutpostHeightBaseIndex=10` |

> 对比：`sp_vision_25` 只有 **11 维**（`x vx y vy z vz a w r l h`），没有 `h0/h1/h2`。

`is_outpost()` 判据：`name == outpost && armor_num_ == 3`（[target.cpp:433](tasks/auto_aim/target.cpp#L433)）。

### 2.3 初始化

[target.cpp:67-72](tasks/auto_aim/target.cpp#L67-L72)：构造时把三个高度槽位按 `0 → 0.0`、`1 → 0.20`、`2 → 0.10` 初始化（即 `OUTPOST_HEIGHT_STEP=0.10` 的 0/1/2 倍，与装甲板 id 对齐）。[tracker.cpp:248-262](tasks/auto_aim/tracker.cpp#L248-L262) 里前哨站的 `P0_dig` 对 `h0/h1/h2` 给了初值方差 `1e-2`，半径方差 `1e-4`（半径几乎锁定）。

### 2.4 状态转移与过程噪声（predict）

[target.cpp:124-185](tasks/auto_aim/target.cpp#L124-L185)：

- **前哨站**：状态转移矩阵 `F` 只让 `yaw` 与 `vyaw` 耦合（`F(yaw,vyaw)=dt`），**中心 x/y/z 与速度完全不耦合**（固定不动）。
- 过程噪声强度区分兵种：
  - 前哨站：`v_pos=5`（中心固定，留一点裕量），`v_yaw=0.02`（vyaw 已知为常量 0.8π，几乎不加噪声）。
  - 普通目标：`v_pos=100`，`v_yaw=400`。
- 前哨站额外给 `h0/h1/h2` 加 `OUTPOST_HEIGHT_PROCESS_NOISE=1e-2` 的过程噪声，`r` 加 `1e-4`。

> 对比 `sp_vision_25`：它在 `predict` 里对前哨站做了**硬限幅**（收敛后把 `w` clamp 到 ±2.51），而本仓库改用「极小过程噪声 + EKF 自然估计」的方式，不再硬 clamp（见下文 5.2 详述）。

### 2.5 观测与板匹配（update）

单板更新 [target.cpp:187-277](tasks/auto_aim/target.cpp#L187-L277)。前哨站匹配分两条路：

1. **常规角度匹配**：对 3 个槽位算 `outpost_match_cost`（[target.cpp:455-470](tasks/auto_aim/target.cpp#L455-L470)）：
   ```
   cost = |观测yaw − (yaw + id·120°)|  +  0.1 · |观测z − 该槽位z| / 0.10
   ```
   角度误差是**主导判据**（3 板相差 120°，正确槽位角度误差≈0，错误槽位≈2.09 rad），高度差只是弱 tie-breaker。

2. **方向切换匹配**（大视角下）：当观测角偏差 `> 60°`（PnP yaw 噪声大，角度不可靠）且方向已识别时，直接用「期望的下一块板」：
   ```
   shift = (方向为CW) ? 1 : 2
   best_id = (last_id + shift) % 3
   ```
   即 CW 转到 `id+1`，CCW 转到 `id+2`（见 [target.cpp:199-226](tasks/auto_aim/target.cpp#L199-L226)、`outpost_expected_shift` [target.cpp:472-482](tasks/auto_aim/target.cpp#L472-L482)）。

3. **滞回防抖**：`frames_since_switch_ < 5` 时不换槽位，避免两块板之间来回跳（[target.cpp:229-247](tasks/auto_aim/target.cpp#L229-L247)）。

槽位切换时，直接把测量高度写进新槽位状态做**快速收敛**（否则 EKF 要 3~5 帧才收敛，预测的 Z 会肉眼可见地跳变），见 [target.cpp:233-247](tasks/auto_aim/target.cpp#L233-L247)。

### 2.6 旋转方向识别（高度差序列状态机）

这是前哨站预测的核心创新。`OutpostPlateId` 用一个 4 状态机（[target.hpp:72-81](tasks/auto_aim/target.hpp#L72-L81)、[target.cpp:506-609](tasks/auto_aim/target.cpp#L506-L609)）通过**相邻两板的高度差**判断旋转方向：

```
kInit → kFirstObserved → kSecondObserved → kIdentified
```

| 高度差 `dh` | 判定 | 方向 |
|------------|------|------|
| `> 0.15 m`（HIGH gap，两板跨整个高度范围） | 最低↔最高 | `+dh → CW(+1)`，`-dh → CCW(-1)` |
| `0.05~0.15 m`（LOW gap，相邻板） | 最低↔中 或 中↔最高 | `+dh → CCW(-1)`，`-dh → CW(+1)` |
| `< 0.05 m`（太小，视为同一块板/噪声） | 保持 `kFirstObserved` 不动 | — |

- 观察到第 3 块板（`kSecondObserved`）后再确认方向 → `kIdentified`。
- 每一步都**与 EKF 估计的 `vyaw` 交叉校验**：若 `vyaw` 方向与推导方向不一致，则复位重新识别（[target.cpp:249-259](tasks/auto_aim/target.cpp#L249-L259)）。

### 2.7 参考槽位旋转（rotate_outpost_reference）

为保证「槽位 0 始终跟踪当前可见的板」，匹配到非 0 槽位时对状态和协方差做置换（[target.cpp:435-504](tasks/auto_aim/target.cpp#L435-L504)）：

- 用置换矩阵把 `h_best → h_0`，同时 `yaw` 加 `id·120°`；
- 协方差 `P` 做同样的 `P' = Π P Πᵀ` 置换。

这样 EKF 的观测模型 `h_armor_xyz(x, 0)` 永远指向可见板。

### 2.8 多板联合匹配（DFS）

`Target::update(const std::vector<Armor>&)`（[target.cpp:319-431](tasks/auto_aim/target.cpp#L319-L431)）在**同一帧检测到多块前哨站板**时启用：用 DFS 枚举「检测板 → 槽位」的所有分配，取总 cost 最小的分配；再用多板高度求**共识 `z_base`** 精化中心高度，并对每个槽位快速收敛 + EKF 更新。

### 2.9 发散与收敛判定

- 收敛（[target.cpp:705-716](tasks/auto_aim/target.cpp#L705-L716)）：普通目标 `update_count>3`，前哨站 `update_count>10`（前哨站板轮流出现，需要更多帧）。
- 发散（[target.cpp:671-703](tasks/auto_aim/target.cpp#L671-L703)）：前哨站在「半径范围」之外，还额外检查 **3 个高度都落在 (-0.08, 0.28) 内、且相邻高度间隙在 0.03~0.17 m**，不满足即判定发散。

---

## 3. 选板（瞄准点选择）

选板发生在 `Aimer::choose_aim_point`（[aimer.cpp:144-264](tasks/auto_aim/aimer.cpp#L144-L264)）。它输入 EKF 状态、输出要瞄准的那块板的 `xyza`。

### 3.1 前哨站专属选板

[aimer.cpp:150-202](tasks/auto_aim/aimer.cpp#L150-L202)。EKF 同时建模 3 块板（绿框），预测/瞄准只喂给**离相机最近的那块板**（红框）：

1. **找最近板**：对 3 块板算 `d² = x² + y²`，取最小者作为「最正对相机」的板。
2. **锁定 + 滞回**：首次进入时锁定最近板（`outpost_aim_lock_id_`）；之后只有当满足
   ```
   (锁定板偏离 > 45°  或  最近板比锁定板更面向相机)  且  锁定帧数 ≥ 5
   ```
   才切换锁定目标（`kSwitchAngle=45°`、`kMinFrames=5`）。锁定帧数不足时不切换，防止在相邻两板之间震荡。
3. 返回锁定板的 `xyza`。

> 对比 `sp_vision_25`：其 `choose_aim_point` **没有前哨站专属分支**，前哨站走通用「小陀螺」分支（`coming_angle=70°`、`leaving_angle=30°`）。本仓库把前哨站选板独立成「最近板 + 45° + 5 帧滞回」。

### 3.2 普通装甲板选板

1. **未跳变**（`!jumped`）：直接瞄准 `armor_xyza_list[0]`（当前唯一已知板）。
2. **非小陀螺**（`|ekf_x[8]| <= 2`，即半径在 2 m 内，普通目标恒成立）：
   - 对每块板算 `delta_angle = limit_rad(板角度 − 中心朝向)`；
   - 只保留 `|delta_angle| ≤ 60°` 的「可射击」板；
   - **锁定模式**：若两块板都在范围内，锁到 `|delta_angle|` 较小的一块，防止在都呈 45° 的两板间来回切；只剩一块时退出锁定。
3. **小陀螺**（else 分支）：板一边不断出现、一边不断消失，出现的一侧更容易命中——用 `coming_angle/leaving_angle`（前哨站 70°/30°，其余用配置 `comming_angle/leaving_angle`）配合旋转方向 `vyaw` 选板。

### 3.3 弹道解算与飞行时间迭代

[aimer.cpp:32-123](tasks/auto_aim/aimer.cpp#L32-L123)：

1. 先把目标预测到 `now + 延迟时间`（延迟按 `vyaw` 是否超 `decision_speed` 区分高低速档）；
2. 用 `Trajectory`（[trajectory.hpp](tools/trajectory.hpp)）解**真空弹道**（已知初速 `v0`、水平距 `d`、高度 `h`，求俯仰角与飞行时间）；
3. **迭代收敛飞行时间**（最多 10 次）：用上一轮飞行时间再预测一次目标位置 → 重新选板/解弹道，直到相邻两次飞行时间差 `< 0.001 s`；
4. 输出 `yaw = atan2(y, x) + yaw_offset`、`pitch = -(弹道俯仰 + pitch_offset)`。

英雄弹速钳制：`bullet_speed` 不在 `[10, 12.5]` 内时置为 `11.5 m/s`（[aimer.cpp:43](tasks/auto_aim/aimer.cpp#L43)），区别于 sp25 的 `23 m/s`（步兵弹速）。

---

## 4. 射击判定（Shooter）

[shooter.cpp](tasks/auto_aim/shooter.cpp)：允许开火需同时满足

1. 有目标、有控制、`auto_fire=true`；
2. 距离 > `judge_distance` 用远容差、否则用近容差；
3. **云台已到位**：`|gimbal_yaw − 上次指令yaw| < 容差`；
4. **指令未突变**：`|上次指令yaw − 本次指令yaw| < 2·容差`（突变说明目标丢失/跳变，不打）；
5. 瞄准点有效（`aimer.debug_aim_point.valid`）。

---

## 5. 与 sp_vision_25 的对比

### 5.1 总体对比表

| 维度 | sp_vision_25 | hero_vision_26（本仓库） |
|------|-------------|------------------------------|
| 面向兵种 | 步兵/通用（CAN 板 `CBoard`） | **英雄**（串口云台 `Gimbal`，`start_sp_vision.sh` 跑 `standard_mpc`） |
| EKF 状态维数 | **11**（`x vx y vy z vz a w r l h`） | **13**（增加 `h0/h1/h2` 三块前哨站板高度） |
| 前哨站建模 | 普通旋转目标，仅 `predict` 里硬 clamp 转速到 ±2.51 | 3 块不同高度板 + 各自高度槽位，EKF 估计 |
| 前哨站转速 | 收敛后 `w` 直接 clamp 到 ±0.8π | 过程噪声极小（0.02）靠 EKF 估计，不硬 clamp |
| 板匹配 | 单板、纯角度误差匹配 | 角度+高度代价；大视角走方向切换；多板 DFS 匹配 |
| 旋转方向识别 | 无 | 高度差序列状态机（HIGH/LOW gap）+ vyaw 交叉校验 |
| 参考槽位旋转 | 无 | `rotate_outpost_reference`（状态+协方差置换） |
| 选板（aimer） | 前哨站无专属分支，走小陀螺分支 | 前哨站专属「最近板 + 45° + 5 帧滞回」 |
| 发散判定 | 仅半径范围 | 前哨站额外校验 3 个高度范围与相邻间隙 |
| 多板联合更新 | 无 `update(vector<Armor>)` | 有（DFS 分配 + 共识 z_base） |
| 弹速 | `23 m/s`（<14 时钳制） | `11.5 m/s`（英雄弹速） |
| 主入口 IO | `io::CBoard`（CAN） | `io::Gimbal`（串口 `/dev/gimbal`） |
| 额外工程 | — | `rm_auto_start/` 自启、`start_sp_vision.sh`、`from_codex.md`、`buff_layout.xml`/`mpc_layout.xml` |

### 5.2 关键差异详解

1. **前哨站从「转速硬编码」升级为「三板高度建模」**。
   `sp_vision_25` 把前哨站当成只有一个半径的普通旋转体，唯一特判是 `predict` 里 `if converged && outpost && |w|>2 → w=±2.51`（对应上游 `tasks/auto_aim/target.cpp` 的 predict 段）。本仓库则显式给三块板各建一个高度状态，用高度差识别板身份与旋转方向，预测更稳。

2. **板身份识别**：本仓库新增 `OutpostPlateId` 状态机（`kInit→kFirstObserved→kSecondObserved→kIdentified`），通过「相邻板高度差是大跳变（>0.15m）还是小跳变（0.05~0.15m）」推断 CW/CCW，并和 EKF 的 `vyaw` 互证。

3. **选板策略独立化**：本仓库把前哨站选板从「小陀螺分支」中拆出，改为「离相机最近板 + 45° 切换阈值 + 5 帧滞回」，专门解决前哨站 3 板轮流出现时的目标震荡问题。

4. **跟踪层多板匹配**：本仓库 `Tracker::update_target` 对前哨站调用 `target_.update(matched_armors)`（多板向量），sp25 只能 `target_.update(armor)` 单板逐个喂。

5. **弹速与硬件**：英雄弹速（11.5 m/s）与步兵（23 m/s）不同；英雄用串口云台而非 CAN 板，因此主入口 `standard.cpp` 里 `io::CBoard` 被替换成 `io::Gimbal`（但 `CBoard` 仍保留给哨兵 `src/sentry*.cpp` 使用）。

---

## 6. 关键参数速查

| 参数 | 值 | 位置 |
|------|-----|------|
| 前哨站半径 | `0.2765 m` | tracker.cpp `set_target` |
| 基地半径 | `0.3205 m` | tracker.cpp `set_target` |
| 前哨站板高步长 | `0.10 m`，初始 `0 / 0.20 / 0.10` | target.cpp 顶部常量 |
| 前哨站转速 | ~`0.8π rad/s`（过程噪声极小） | target.cpp `predict` |
| 大视角切换阈值 | `60°` | target.cpp `update` |
| 板切换滞回帧数 | `5` | target.cpp `update` / aimer.cpp |
| 前哨站选板切换角 | `45°` | aimer.cpp `choose_aim_point` |
| 前哨站收敛帧数 | `update_count > 10` | target.cpp `convergened` |
| 前哨站 temp_lost | `75` 帧 | config `outpost_max_temp_lost_count` |
| 普通 temp_lost | `15`（步兵）/`25`（哨兵） | config `max_temp_lost_count` |
| 英雄弹速 | `11.5 m/s` | aimer.cpp 钳制 |
| 飞行时间收敛阈值 | `0.001 s`，最多 10 次迭代 | aimer.cpp |
