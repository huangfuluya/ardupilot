# DroneCAN 外部 AHRS 传感器使用说明书

本文档说明如何将基于 DroneCAN 协议的外部 AHRS（姿态与航向参考系统）设备接入 ArduPilot，以及该设备需要通过 DroneCAN 总线发布哪些消息、包含哪些数据字段。

---

## 1. 概述

`AP_ExternalAHRS_DroneCAN` 是 ArduPilot 外部 AHRS 框架（`AP_ExternalAHRS`）的一个后端驱动。它通过监听 DroneCAN（UAVCAN v0）总线上的标准消息，从外部传感器节点获取姿态、IMU、GPS 和气压计数据，并将这些数据提供给 ArduPilot 飞控的各个子系统（姿态估计、导航、着陆等）。

---

## 2. 硬件连接

- 将外部 AHRS 传感器节点接到飞控的任意一路 CAN 总线接口（CAN1 或 CAN2）。
- 确保总线两端各有一个 120 Ω 终端电阻。
- 传感器节点须与飞控共地、供电稳定（通常 5 V）。

---

## 3. 飞控参数配置

### 3.1 使能 DroneCAN 驱动

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| `CAN_P1_DRIVER` | `1` | CAN1 启用 DroneCAN 驱动 |
| `CAN_D1_PROTOCOL` | `1` | 使用 DroneCAN (UAVCAN v0) 协议 |

若使用 CAN2，相应修改 `CAN_P2_DRIVER` / `CAN_D2_PROTOCOL`。

### 3.2 使能外部 AHRS

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| `EAHRS_TYPE` | `36` | 选择 DroneCAN 外部 AHRS 后端 |
| `EAHRS_RATE` | `50` | 期望的数据更新频率（Hz），建议与传感器发布频率一致 |
| `EAHRS_OPTIONS` | `0` | 选项位掩码，默认 0 即可 |
| `EAHRS_SENSORS` | `0xF` | 使能全部传感器（GPS + IMU + 气压计） |

修改参数后须重启飞控。

### 3.3 GPS 源配置（可选）

若希望使用外部 AHRS 的 GPS 数据：

| 参数 | 推荐值 |
|------|--------|
| `GPS_TYPE` | `21`（ExternalAHRS） |

---

## 4. 传感器须发布的 DroneCAN 消息

外部 AHRS 节点根据自身能力，从下列消息中选择合适的组合进行发布。优先级规则见第 4.1 节。

---

### 4.1 消息优先级与降级策略

| 优先级 | 姿态 + IMU 来源 |
|--------|----------------|
| 1（最高）| `uavcan.navigation.GlobalNavigationSolution` |
| 2 | `uavcan.equipment.ahrs.Solution` |
| 3 | `uavcan.equipment.ahrs.RawIMU` |

当收到更高优先级的消息后，低优先级的同类消息将被忽略（500 ms 超时窗口内）。建议传感器**只发布最高精度的一种**，以避免冗余。

---

### 4.2 `uavcan.navigation.GlobalNavigationSolution`（首选，综合导航解）

**DSDL 数据类型 ID：`uavcan.navigation.GlobalNavigationSolution`**

此消息一包包含姿态、位置、速度和 IMU 数据，是最完整的数据源。

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `timestamp` | `uavcan.Timestamp` | μs | 消息时间戳 |
| `orientation_xyzw[4]` | `float32[4]` | — | 姿态四元数，顺序为 `[x, y, z, w]`（NED 坐标系） |
| `angular_velocity_body[3]` | `float32[3]` | rad/s | 机体系角速度 `[roll_rate, pitch_rate, yaw_rate]` |
| `linear_acceleration_body[3]` | `float32[3]` | m/s² | 机体系线加速度 `[ax, ay, az]` |
| `latitude` | `float64` | deg | 纬度（度，双精度） |
| `longitude` | `float64` | deg | 经度（度，双精度） |
| `height_msl` | `float32` | m | MSL 高度（米） |
| `height_baro` | `float32` | m | 气压高度（米）；驱动据此判断是否有气压计数据，`NaN` 或 ≤ 0 时忽略 |
| `linear_velocity_body[3]` | `float32[3]` | m/s | 机体系速度 `[vx, vy, vz]`；驱动内部转为 NED |

> **注意：** `height_baro` 字段不为 `NaN` 且大于 0 时，驱动才会将气压计数据推送到 ArduPilot 气压子系统。气压数值（Pa）需通过 `uavcan.equipment.air_data.StaticPressure` 单独发送（见第 4.5 节）。

---

### 4.3 `uavcan.equipment.ahrs.Solution`（次选，仅姿态 + IMU）

**DSDL 数据类型 ID：`uavcan.equipment.ahrs.Solution`**

当无法提供完整导航解时使用此消息（例如纯姿态传感器）。

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `timestamp` | `uavcan.Timestamp` | μs | 消息时间戳 |
| `orientation_xyzw[4]` | `float32[4]` | — | 姿态四元数，顺序为 `[x, y, z, w]` |
| `angular_velocity[3]` | `float32[3]` | rad/s | 机体系角速度 |
| `linear_acceleration[3]` | `float32[3]` | m/s² | 机体系线加速度 |

---

### 4.4 `uavcan.equipment.ahrs.RawIMU`（备选，原始 IMU）

**DSDL 数据类型 ID：`uavcan.equipment.ahrs.RawIMU`**

仅在无法提供姿态解算结果时使用（驱动不进行姿态解算，直接透传 IMU 原始数据）。

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `timestamp` | `uavcan.Timestamp` | μs | 消息时间戳 |
| `rate_gyro_latest[3]` | `float32[3]` | rad/s | 陀螺仪测量值 `[gx, gy, gz]` |
| `accelerometer_latest[3]` | `float32[3]` | m/s² | 加速度计测量值 `[ax, ay, az]` |

---

### 4.5 `uavcan.equipment.air_data.StaticPressure`（气压计数据）

**DSDL 数据类型 ID：`uavcan.equipment.air_data.StaticPressure`**

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `static_pressure` | `float32` | Pa | 静压（气压计绝对气压值） |
| `static_pressure_variance` | `float32` | Pa² | 气压方差（可选，驱动当前未使用） |

---

### 4.6 `uavcan.equipment.air_data.StaticTemperature`（气压温度，可选）

**DSDL 数据类型 ID：`uavcan.equipment.air_data.StaticTemperature`**

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `static_temperature` | `float32` | K（开尔文） | 环境温度；驱动转换为摄氏度后传递给气压子系统 |
| `static_temperature_variance` | `float32` | K² | 温度方差（可选，驱动当前未使用） |

若不发送此消息，驱动将使用飞控内置气压计温度值作为替代。

---

### 4.7 `uavcan.equipment.gnss.Fix2`（GPS 数据）

**DSDL 数据类型 ID：`uavcan.equipment.gnss.Fix2`**

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `timestamp` | `uavcan.Timestamp` | μs | 消息时间戳 |
| `status` | `uint2` | — | 定位状态：0=无定位，1=仅时间，2=2D，3=3D |
| `mode` | `uint4` | — | 定位模式：0=单点，1=DGPS，2=RTK；驱动据此映射 fix_type |
| `sub_mode` | `uint6` | — | RTK 子模式：0=Float，1=Fixed |
| `latitude_deg_1e8` | `int64` | deg×10⁻⁸ | 纬度（单位：度×10⁻⁸，驱动除以 10 转为 deg×10⁻⁷） |
| `longitude_deg_1e8` | `int64` | deg×10⁻⁸ | 经度 |
| `height_msl_mm` | `int32` | mm | MSL 高度（毫米，驱动除以 10 转为厘米） |
| `ned_velocity[3]` | `float32[3]` | m/s | NED 速度 `[north, east, down]` |
| `sats_used` | `uint7` | — | 参与定位的卫星数量 |
| `pdop` | `float16` | — | 位置精度因子（同时用于 HDOP / VDOP） |
| `covariance` | `float16[≤9]` | — | 协方差矩阵对角元素（可选）：`[var_n, var_e, var_d, var_vn, var_ve, var_vd, ...]`；驱动用前 6 元素计算水平/垂直位置和速度精度 |

---

## 5. 推荐消息发布频率

| 消息 | 推荐频率 |
|------|---------|
| `GlobalNavigationSolution` 或 `ahrs.Solution` | 50–200 Hz |
| `ahrs.RawIMU` | 100–400 Hz（仅在无姿态解时使用） |
| `gnss.Fix2` | 5–20 Hz |
| `air_data.StaticPressure` | 10–50 Hz |
| `air_data.StaticTemperature` | 1–10 Hz |

> 频率高于 `EAHRS_RATE` 参数值时，多余数据会被丢弃；低于该值时系统将判定传感器超时（500 ms 无数据视为故障）。

---

## 6. 健康状态与起飞前检查

- 驱动以 **500 ms** 为超时门限：若超过 500 ms 未收到任何 IMU 相关消息（`GlobalNavigationSolution` / `ahrs.Solution` / `RawIMU`），`healthy()` 返回 `false`，飞控将报告 `DroneCAN AHRS not healthy` 并阻止解锁。
- 成功初始化后，GCS 会显示：`DroneCAN ExternalAHRS initialised`。

---

## 7. 坐标系约定

- **四元数** 均使用 **`[x, y, z, w]`** 顺序（消息字段）；驱动内部存储时转换为 ArduPilot 的 **`[w, x, y, z]`** 顺序。
- **角速度、加速度、速度** 均为 **机体系（Body Frame）**，X 轴向前、Y 轴向右、Z 轴向下。
- **位置、NED 速度** 使用北东地坐标系。
- 温度消息使用 **开尔文（K）**；驱动自动减去 273.15 转为摄氏度。

---

## 8. 典型接入示例（最小消息集）

对于一个提供完整导航解的外部 AHRS 节点，最小消息集为：

1. `uavcan.navigation.GlobalNavigationSolution`（50 Hz，包含姿态 + IMU + 位置 + 速度 + 气压高度）
2. `uavcan.equipment.air_data.StaticPressure`（10 Hz）
3. `uavcan.equipment.gnss.Fix2`（10 Hz）
4. `uavcan.equipment.air_data.StaticTemperature`（1 Hz，可选）

对于仅提供姿态的传感器节点：

1. `uavcan.equipment.ahrs.Solution`（100 Hz）
2. `uavcan.equipment.air_data.StaticPressure`（10 Hz）
3. `uavcan.equipment.gnss.Fix2`（10 Hz，若有 GPS）
