# CLAUDE.md - ArduPilot 品灵吊舱 Viewpro 集成项目

## 项目概述

基于 ArduPilot 4.7.0-beta7，为品灵（Pingling/Viewpro）吊舱实现 Viewlink 通信协议集成，支持吊舱目标跟踪与地理定位功能。

分支: `ArduPilot-4.7_viewpro_follow`

通信协议参考: `品灵吊舱viewlink通信协议V3.4.9.pdf`

## 当前已完成的工作

### 1. AP_Mount_Viewpro 吊舱驱动 (`libraries/AP_Mount/AP_Mount_Viewpro.cpp/.h`)

- **D1 数据包解析增强**:
  - 激光测距仪距离解析升级为3字节格式（兼容协议 v3.4.5+），保留对旧2字节格式的向后兼容
  - 新增 D1 包中水平/垂直视场角（HFOV/VFOV）的解析（精度 0.01°）
  - 新增 D1 包中飞行器相对高度（rel_height）字段的解析

- **CAMERA_INFORMATION 消息完善**:
  - 根据解析到的 FOV 和假定传感器尺寸（1/2.3" 传感器）计算焦距
  - 补充 sensor_size_h/sensor_size_v 和 focal_length 等之前为 NaNf/0 的字段
  - 传感器尺寸: 宽 6.17mm, 高 4.55mm（1/2.3" 传感器典型值）

- **调试日志**:
  - 关闭 DEBUG 模式（AP_MOUNT_VIEWPRO_DEBUG = 0），正式发布前状态
  - F1 字节跟踪状态日志已注释掉

- **目标位置保护**:
  - 仅在跟踪状态为 TRACKING 时更新目标经纬度和高度
  - 非跟踪状态下（SEARCHING/LOST/STOPPED）保留最后一次有效跟踪位置
  - 防止吊舱上报的过期/无效数据污染目标位置

- **M_AHRS 发送逻辑修复**:
  - 移除跟踪期间跳过 M_AHRS 发送的条件判断
  - 始终发送 M_AHRS 数据包，确保吊舱持续获得飞行器姿态信息

- **跟踪状态切换处理**:
  - 当检测到跟踪状态变化时，发送 A1 包将吊舱舵机状态切换为 TRACKING_MODE（0x06）
  - 确保吊舱内部跟踪器接管云台控制权，避免手动模式/FOLOW_YAW 与跟踪冲突导致的振荡

- **CAMERA_TRACKING_GEO_STATUS 消息修复**:
  - 使用 `_target_lat`/`_target_lng`/`_target_alt_m` 替代之前的天顶硬编码占位值
  - tracking_status 字段反映实际跟踪状态

- **B1 数据包结构修复**:
  - 移除 `rel_height_be`（相对高度）字段
  - 将 `reserved2to4[3]` 改为 `unused2to8[7]`，正确对齐后续字段偏移

- **新增方法**: `is_tracking_target()` — 判断吊舱是否正在主动跟踪目标

- **新增成员变量**: `_hfov_deg`（水平视场角）、`_vfov_deg`（垂直视场角）

### 2. AP_Mount / AP_Mount_Backend 基类 (`libraries/AP_Mount/`)

- **新增接口方法**:
  - `AP_Mount::is_tracking_target(uint8_t instance)` — 查询指定吊舱实例是否正在跟踪
  - `AP_Mount_Backend::is_tracking_target()` — 虚方法，默认返回 false

### 3. ArduPlane Guided 模式增强 (`ArduPlane/mode.h`, `ArduPlane/mode_guided.cpp`)

- **高度锁定机制**:
  - 当吊舱目标跟随激活时，锁定当前飞行器高度作为目标高度
  - 后续跟随过程中保持此锁定高度，不再使用飞行器当前高度
  - 新增成员变量: `_mount_target_follow_alt_cm`、`_mount_target_follow_alt_locked`

- **目标丢失处理**:
  - 当吊舱失去目标（SEARCHING/LOST/STOPPED）超 5 秒时，自动设置 Loiter 于当前位置
  - 发送 GCS 消息通知 "Mount target lost, loitering at current position"
  - 新增成员变量: `_mount_target_lost_start_ms`、`_mount_target_lost_loiter_set`

- **状态重置**:
  - Guided 模式进入时清空所有跟随相关状态变量
  - `set_mount_target_follow(false)` 时重置高度锁定标志

### 4. GCS_MAVLink 消息调度优化 (`libraries/GCS_MAVLink/`)

- **CAMERA_TRACKING_GEO_STATUS 独立调度**:
  - 将该消息从 STREAM_EXTRA3 流中移除，改为通过 deferred_message 机制独立调度
  - 设置固定 1Hz 发送频率，不受流速率（SRx_*）参数影响
  - 在高延迟模式和正常模式下均启用

### 5. 先前提交（已合并到分支）

- 目标点经纬度解析
- 摇杆控制跟随目标点
- 目标点上报消息（CAMERA_TRACKING_GEO_STATUS）

## 待完成 / 已知问题

- 吊舱跟踪目标丢失自动备降策略（低电量/通信中断等场景）
- 目标位置过滤/平滑，减少噪声引起的飞行器位置抖动
- 传感器尺寸为假定值，不同吊舱型号可能需要不同配置参数
- mavlink submodule 标记为 dirty（本地修改），需确认是否需要上游同步

## 构建命令

```bash
# SITL 构建
./waf configure --board sitl
./waf copter

# Plane SITL 构建（主要测试车辆）
./waf configure --board sitl
./waf plane

# 清理
./waf clean
./waf distclean
```

## 注意事项

- 传感器尺寸（6.17mm x 4.55mm）为 1/2.3" 传感器假定值，实际值可能因吊舱型号而异
- FOV 数据依赖于吊舱固件版本，需确保固件支持 D1 包中 FOV 字段
- 测距仪 3 字节格式向后兼容旧版 2 字节格式
- 调试模式（AP_MOUNT_VIEWPRO_DEBUG）当前为关闭状态（0）
- B1 数据包结构已移除 rel_height_be 字段，确认与当前吊舱固件版本兼容
- Guided 模式的目标丢失 Loiter 保护机制仅在 mount_target_follow 启用时生效
