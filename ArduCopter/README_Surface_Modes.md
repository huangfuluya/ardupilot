# 四旋翼+船混合飞行器水面模式说明

本文档介绍 ArduCopter 为四旋翼+船混合飞行器（quad+boat hybrid）新增的三种水面航行模式，以及相关代码修改说明、参数和接线设置。

---

## 目录

1. [代码修改说明](#1-代码修改说明)
2. [硬件接线](#2-硬件接线)
3. [地面站参数设置](#3-地面站参数设置)
4. [飞行模式使用介绍](#4-飞行模式使用介绍)
   - [SURFACE（模式 29）](#surface模式-29)
   - [SURFACE\_LOITER（模式 31）](#surface_loiter模式-31)
   - [SURFACE\_AUTO（模式 32）](#surface_auto模式-32)
5. [控制律说明](#5-控制律说明)
6. [参数一览](#6-参数一览)
7. [Pixhawk6X 自动编译工作流](#7-pixhawk6x-自动编译工作流)

---

## 1. 代码修改说明

本次修改在 ArduCopter 中新增了三种水面航行飞行模式，并添加了 Pixhawk6X 固件自动编译工作流。以下列出全部变更文件及改动内容。

### 1.1 新增文件

| 文件 | 说明 |
|---|---|
| `ArduCopter/mode_surface.cpp` | 新增 **SURFACE** 模式（模式编号 29）。飞手手动控制推进与转向，遥控器油门杆驱动基础推力，偏航杆驱动左右差动修正。四旋翼电机保持 `GROUND_IDLE` 怠速。 |
| `ArduCopter/mode_surface_loiter.cpp` | 新增 **SURFACE\_LOITER** 模式（模式编号 31）。进入时以当前 GPS 位置为锁定目标，航向 P 控制器通过差动推力自动纠偏，油门按距离线性减速。飞手可用俯仰/横滚杆移动定点目标。 |
| `ArduCopter/mode_surface_auto.cpp` | 新增 **SURFACE\_AUTO** 模式（模式编号 32）。读取 ArduCopter 任务列表，依次执行 `NAV_WAYPOINT` 和 `NAV_LOITER_UNLIM` 命令，最后一个航点完成后停止。 |
| `ArduCopter/README_Surface_Modes.md` | 本说明文档。 |
| `.github/workflows/build_pixhawk6x.yml` | 新增 Pixhawk6X / Pixhawk6X-bdshot 固件自动编译 GitHub Actions 工作流，详见[第 7 节](#7-pixhawk6x-自动编译工作流)。 |

### 1.2 修改文件

| 文件 | 修改内容 |
|---|---|
| `ArduCopter/mode.h` | 新增 `ModeSurface`、`ModeSurfaceLoiter`、`ModeSurfaceAuto` 类声明，均置于 `#if MODE_SURFACE_ENABLED` 编译守卫内。 |
| `ArduCopter/Copter.h` | 新增三个模式成员变量 `mode_surface`、`mode_surface_loiter`、`mode_surface_auto`，并声明对应 `friend class`。 |
| `ArduCopter/mode.cpp` | 在 `mode_from_mode_num()` 中新增模式编号 29/31/32 的分支，返回对应模式对象指针。在 `Mode::output_to_motors()` 中新增：非水面模式时将 `k_throttleLeft`/`k_throttleRight` 锁定在中位（1500 µs），防止可逆 ESC 在旋翼飞行期间意外转动。 |
| `ArduCopter/config.h` | 新增宏 `MODE_SURFACE_ENABLED`（非直升机机型默认启用）。 |
| `ArduCopter/Parameters.cpp` | 在 `ParametersG2` 参数表中新增四个参数：`SURF_THR_GAIN`（索引 21）、`SURF_STEER_GAIN`（索引 22）、`SURF_AUTO_SPD`（索引 23）、`SURF_HEAD_KP`（索引 24）。 |
| `ArduCopter/Parameters.h` | 新增四个参数成员变量声明（`AP_Float surface_thr_gain` 等）。 |

### 1.3 关键设计变更

**原设计**（单通道舵机转向）：
```
SERVOx_FUNCTION = 70  (Throttle)         → 单路油门
SERVOx_FUNCTION = 26  (GroundSteering)   → 舵机转向，输出范围 ±4500 cdeg
```

**新设计**（双发差动推进，支持倒船）：
```
SERVOx_FUNCTION = 73  (ThrottleLeft)     → 左发可逆电调，scaled -100..+100
SERVOx_FUNCTION = 74  (ThrottleRight)    → 右发可逆电调，scaled -100..+100

scaled  0   → 1500 µs（中位/停止）
scaled +100 → max PWM（全速前进）
scaled -100 → min PWM（全速倒退）

left_motor  = clamp(throttle + diff, -100, +100)
right_motor = clamp(throttle - diff, -100, +100)
```

差动修正量 `diff` 来源：
- **SURFACE 模式**：遥控器偏航杆 × `SURF_STEER_GAIN`
- **SURFACE\_LOITER / AUTO 模式**：航向 P 控制器输出 × `SURF_HEAD_KP`

**旋翼模式中的处理**：`Mode::output_to_motors()` 在非水面模式下将 `k_throttleLeft` / `k_throttleRight` 强制设置为 `set_angle(100)` + scaled 0（= 1500 µs），防止可逆 ESC 在旋翼飞行期间异常输出。

---

---

## 2. 硬件接线

本模式使用**双发差动**方式控制航向，无舵机转向通道。**必须使用支持双向/可逆运行的电调**（中位 = 1500 µs）。

### 左侧电调（左发）

| 项目 | 值 |
|---|---|
| 舵机功能 | `SERVOx_FUNCTION = 73`（ThrottleLeft） |
| 输出范围 | −100 ~ +100（scaled 值，0 = 中位 1500 µs） |
| 说明 | 对应 `SRV_Channel::k_throttleLeft` |

### 右侧电调（右发）

| 项目 | 值 |
|---|---|
| 舵机功能 | `SERVOx_FUNCTION = 74`（ThrottleRight） |
| 输出范围 | −100 ~ +100（scaled 值，0 = 中位 1500 µs） |
| 说明 | 对应 `SRV_Channel::k_throttleRight` |

> 将上述两个功能分配到对应的 SERVO 通道（例如 SERVO5、SERVO6），确保左右电机方向与机体左右一致。
> 电调需完成**中位校准**（1500 µs = 停止），以确保倒船功能正常工作。
> 在旋翼模式下或飞行器**未解锁（disarmed）**时，两路通道自动输出 1500 µs（中位），不会驱动船用电机。

---

## 3. 地面站参数设置

进入任意水面模式前，请先确认以下参数：

| 参数 | 推荐值 | 说明 |
|---|---|---|
| `WPNAV_RADIUS` | 200 cm（2 m） | 航点到达半径；决定减速开始距离 |
| `SURF_THR_GAIN` | 1.0 | SURFACE 模式油门杆基础推力比例系数 |
| `SURF_STEER_GAIN` | 1.0 | SURFACE 模式偏航杆差动比例系数 |
| `SURF_AUTO_SPD` | 50 % | SURFACE\_LOITER / SURFACE\_AUTO 巡航功率 |
| `SURF_HEAD_KP` | 1.0 | SURFACE\_LOITER / SURFACE\_AUTO 航向 P 增益 |

---

## 4. 飞行模式使用介绍

### SURFACE（模式 29）

**手动水面航行**（支持倒船）。飞行器**未解锁时**两路电调输出强制维持在中位（1500 µs）；解锁后方可正常驱动：

- **油门杆（CH3）中位以上** → 前进推力（0 ~ 100%），通过 `SURF_THR_GAIN` 缩放，左右等量分配。
- **油门杆（CH3）中位以下** → 倒退推力（0 ~ −100%），同样等量分配。
- **偏航杆（CH4）** → 差动修正量（±100%），通过 `SURF_STEER_GAIN` 缩放，左加右减实现转向。差动可使任一侧电机输出为负（倒转），实现原地转弯。

切换到其他飞行模式后，输出自动归中位（1500 µs），防止意外移动。

### SURFACE\_LOITER（模式 31）

**GPS 定点水面保持**。飞行器**未解锁时**输出强制维持在中位；解锁后方可正常驱动。进入时以当前 GPS 位置为锁定目标，由自动航向 P 控制器通过差动推力纠偏，油门按距离线性减速：

- 飞手可通过俯仰/横滚杆（体坐标系，自动旋转为 NE 方向）以 2 m/s 速率移动定点目标。
- 到达目标圆内（`WPNAV_RADIUS`）时油门归零，依靠惯性自然停船。
- 差动转弯时，若航向误差较大，差动修正量可超过基础油门，使内侧电机进入倒转以实现更紧的原地转弯。

需要 GPS 有效定位方可进入。

### SURFACE\_AUTO（模式 32）

**任务航点自动航行**。飞行器**未解锁时**输出强制维持在中位；解锁后执行 ArduCopter 任务列表中的 `NAV_WAYPOINT` 和 `NAV_LOITER_UNLIM` 命令：

- 仅处理上述两类导航命令，跳过其他类型（如 `NAV_TAKEOFF`、`NAV_LAND`）。
- 到达当前航点后，自动加载下一个有效导航命令。
- `NAV_LOITER_UNLIM` 使飞行器在该点无限期保持（持续开/关油门维持位置）。
- 差动转弯时，内侧电机可进入倒转以缩短转弯半径。
- 最后一个航点执行完毕后，输出归中位（1500 µs）并停止移动。

需要 GPS 有效定位且任务列表非空方可进入。

---

## 5. 控制律说明

### SURFACE 模式（手动）

```
thr  = (CH3_input - 500) × 0.2 × SURF_THR_GAIN   [%，−100..+100]
diff = CH4_norm_dz × 100 × SURF_STEER_GAIN         [%，−100..+100]

left_motor  = clamp(thr + diff, −100, +100)
right_motor = clamp(thr − diff, −100, +100)
```

- 油门杆中位（500）→ thr = 0 → 两路均 1500 µs（停止）
- 油门杆满上（1000）→ thr = +100 → 全速前进
- 油门杆满下（0）→ thr = −100 → 全速倒退
- 偏航杆可使任一侧电机输出为负（倒转），实现紧凑原地转弯

### SURFACE\_LOITER 和 SURFACE\_AUTO 共用控制律

#### 航向 P 控制器 → 差动混控

```
heading_err = wrap_PI(bearing_to_target − current_yaw)   [rad]
diff        = heading_err × (100 / (π/2)) × SURF_HEAD_KP  [%]
diff        = clamp(diff, −100, +100)

left_motor  = clamp(throttle + diff, −100, +100)
right_motor = clamp(throttle − diff, −100, +100)
```

- `SURF_HEAD_KP = 1.0`：90° 航向误差对应 100% 差动。
- 当 |diff| > throttle 时，内侧电机进入反转，实现原地枢转式转弯（pivot turn）。
- 增大 `SURF_HEAD_KP` 可使转向更激进；如船体出现蛇行振荡，应适当减小。

#### 油门距离渐变

设 `R = WPNAV_RADIUS`（m），`spd = SURF_AUTO_SPD`（%）：

```
dist > 3R         →  throttle = spd
R < dist ≤ 3R     →  throttle = spd × (dist − R) / (2R)   （线性减速）
dist ≤ R          →  throttle = 0                          （滑行停止）
```

---

## 6. 参数一览

| 参数名 | 默认值 | 范围 | 单位 | 说明 |
|---|---|---|---|---|
| `SURF_THR_GAIN` | 1.0 | 0.0 ~ 2.0 | — | SURFACE 模式油门杆基础推力比例系数 |
| `SURF_STEER_GAIN` | 1.0 | 0.0 ~ 2.0 | — | SURFACE 模式偏航杆差动比例系数 |
| `SURF_AUTO_SPD` | 50 | 0 ~ 100 | % | SURFACE\_LOITER/AUTO 巡航功率百分比 |
| `SURF_HEAD_KP` | 1.0 | 0.1 ~ 3.0 | — | SURFACE\_LOITER/AUTO 航向 P 增益 |

---

## 7. Pixhawk6X 自动编译工作流

### 7.1 工作流文件

工作流位于 `.github/workflows/build_pixhawk6x.yml`。

### 7.2 触发条件

| 触发方式 | 说明 |
|---|---|
| `push` | 向任意分支推送代码时自动触发（忽略文档、测试脚本等无关路径） |
| `pull_request` | 提交 PR 时自动触发 |
| `workflow_dispatch` | 可在 GitHub Actions 页面手动触发 |

### 7.3 编译矩阵

每次触发均同时编译以下两个目标板：

| 目标板 | 说明 |
|---|---|
| `Pixhawk6X` | 标准固件 |
| `Pixhawk6X-bdshot` | 支持双向 DShot 的固件 |

每个目标板均依次编译全部五种飞行器固件：`copter`、`plane`、`rover`、`sub`、`antennatracker`。

### 7.4 编译环境

| 项目 | 值 |
|---|---|
| 操作系统 | Ubuntu 22.04 |
| 编译容器 | `ardupilot/ardupilot-dev-chibios:v0.1.3` |
| 编译器 | arm-none-eabi-gcc 10 |
| 编译加速 | ccache（按目标板分别缓存，加速后续增量编译） |

### 7.5 固件下载

每次编译完成后，`bin/` 目录下的所有固件文件会自动作为 GitHub Actions **Artifacts** 上传，保留 **30 天**。下载方法：

1. 在 GitHub 仓库页面点击 **Actions** 标签页；
2. 选择对应的工作流运行记录（`Build Pixhawk6X Firmware`）；
3. 在页面底部 **Artifacts** 区域，点击 `Pixhawk6X-firmware` 或 `Pixhawk6X-bdshot-firmware` 即可下载 `.zip` 压缩包；
4. 解压后获得 `.apj`（ArduPilot JSON 固件）、`.bin`（裸二进制）等格式文件，可直接用 Mission Planner / QGroundControl 烧录。

### 7.6 并发控制

同一分支/PR 上同时运行多个编译任务时，新任务会自动取消旧任务，节省 CI 资源：

```yaml
concurrency:
  group: ci-Build Pixhawk6X Firmware-${{ github.ref }}
  cancel-in-progress: true
```

