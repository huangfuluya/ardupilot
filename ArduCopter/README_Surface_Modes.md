# 四旋翼+船混合飞行器水面模式说明

本文档介绍 ArduCopter 为四旋翼+船混合飞行器（quad+boat hybrid）新增的三种水面航行模式，以及相关参数和接线设置。

---

## 目录

1. [硬件接线](#1-硬件接线)
2. [地面站参数设置](#2-地面站参数设置)
3. [飞行模式说明](#3-飞行模式说明)
   - [SURFACE（模式 29）](#surface模式-29)
   - [SURFACE\_LOITER（模式 31）](#surface_loiter模式-31)
   - [SURFACE\_AUTO（模式 32）](#surface_auto模式-32)
4. [控制律说明](#4-控制律说明)
5. [参数一览](#5-参数一览)

---

## 1. 硬件接线

本模式使用**双发差动**方式控制航向，无舵机转向通道。

### 左侧电调（左发）

| 项目 | 值 |
|---|---|
| 舵机功能 | `SERVOx_FUNCTION = 73`（ThrottleLeft） |
| 输出范围 | 0 ~ 100（功率百分比） |
| 说明 | 对应 `SRV_Channel::k_throttleLeft` |

### 右侧电调（右发）

| 项目 | 值 |
|---|---|
| 舵机功能 | `SERVOx_FUNCTION = 74`（ThrottleRight） |
| 输出范围 | 0 ~ 100（功率百分比） |
| 说明 | 对应 `SRV_Channel::k_throttleRight` |

> 将上述两个功能分配到对应的 SERVO 通道（例如 SERVO5、SERVO6），确保左右电机方向与机体左右一致。

---

## 2. 地面站参数设置

进入任意水面模式前，请先确认以下参数：

| 参数 | 推荐值 | 说明 |
|---|---|---|
| `WPNAV_RADIUS` | 200 cm（2 m） | 航点到达半径；决定减速开始距离 |
| `SURF_THR_GAIN` | 1.0 | SURFACE 模式油门杆基础推力比例系数 |
| `SURF_STEER_GAIN` | 1.0 | SURFACE 模式偏航杆差动比例系数 |
| `SURF_AUTO_SPD` | 50 % | SURFACE\_LOITER / SURFACE\_AUTO 巡航功率 |
| `SURF_HEAD_KP` | 1.0 | SURFACE\_LOITER / SURFACE\_AUTO 航向 P 增益 |

---

## 3. 飞行模式说明

### SURFACE（模式 29）

**手动水面航行**。四旋翼电机保持 `GROUND_IDLE` 怠速，姿态控制器维持机体水平；飞手通过遥控器直接控制船的推进与转向：

- **油门杆（CH3）** → 基础推力（0 ~ 100%），通过 `SURF_THR_GAIN` 缩放，左右等量分配。
- **偏航杆（CH4）** → 差动修正量（±100%），通过 `SURF_STEER_GAIN` 缩放，左加右减实现转向。

切换到其他飞行模式后，输出自动归零，防止意外移动。

### SURFACE\_LOITER（模式 31）

**GPS 定点水面保持**。进入时以当前 GPS 位置为锁定目标，由自动航向 P 控制器通过差动推力纠偏，油门按距离线性减速：

- 飞手可通过俯仰/横滚杆（体坐标系，自动旋转为 NE 方向）以 2 m/s 速率移动定点目标。
- 到达目标圆内（`WPNAV_RADIUS`）时油门归零，依靠惯性自然停船。

需要 GPS 有效定位方可进入。

### SURFACE\_AUTO（模式 32）

**任务航点自动航行**。执行 ArduCopter 任务列表中的 `NAV_WAYPOINT` 和 `NAV_LOITER_UNLIM` 命令：

- 仅处理上述两类导航命令，跳过其他类型（如 `NAV_TAKEOFF`、`NAV_LAND`）。
- 到达当前航点后，自动加载下一个有效导航命令。
- `NAV_LOITER_UNLIM` 使飞行器在该点无限期盘旋（持续开/关油门保持位置）。
- 最后一个航点执行完毕后，输出归零并停止移动。

需要 GPS 有效定位且任务列表非空方可进入。

---

## 4. 控制律说明

SURFACE\_LOITER 和 SURFACE\_AUTO 共用以下控制律：

### 航向 P 控制器 → 差动混控

```
heading_err = wrap_PI(bearing_to_target - current_yaw)   [rad]
diff        = heading_err × (100 / (π/2)) × SURF_HEAD_KP  [%]
diff        = clamp(diff, -100, +100)

left_motor  = clamp(throttle + diff, 0, 100)
right_motor = clamp(throttle - diff, 0, 100)
```

- `SURF_HEAD_KP = 1.0`：90° 航向误差对应 100% 差动（一侧满功率，另一侧归零）。
- 增大 `SURF_HEAD_KP` 可使转向更激进；如船体出现蛇行振荡，应适当减小。

### 油门距离渐变

设 `R = WPNAV_RADIUS`（m），`spd = SURF_AUTO_SPD`（%）：

```
dist > 3R         →  throttle = spd
R < dist ≤ 3R     →  throttle = spd × (dist - R) / (2R)   （线性减速）
dist ≤ R          →  throttle = 0                          （滑行停止）
```

---

## 5. 参数一览

| 参数名 | 默认值 | 范围 | 单位 | 说明 |
|---|---|---|---|---|
| `SURF_THR_GAIN` | 1.0 | 0.0 ~ 2.0 | — | SURFACE 模式油门杆基础推力比例系数 |
| `SURF_STEER_GAIN` | 1.0 | 0.0 ~ 2.0 | — | SURFACE 模式偏航杆差动比例系数 |
| `SURF_AUTO_SPD` | 50 | 0 ~ 100 | % | SURFACE\_LOITER/AUTO 巡航功率百分比 |
| `SURF_HEAD_KP` | 1.0 | 0.1 ~ 3.0 | — | SURFACE\_LOITER/AUTO 航向 P 增益 |
