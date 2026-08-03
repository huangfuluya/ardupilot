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
  - 开启 DEBUG 模式（AP_MOUNT_VIEWPRO_DEBUG = 1），便于开发调试
  - 为 camera_tracking_geo_status 发送增加跳过原因和目标位置日志

- **新增成员变量**: `_hfov_deg`（水平视场角）、`_vfov_deg`（垂直视场角）

### 2. GCS_MAVLink 消息调度优化 (`libraries/GCS_MAVLink/`)

- **CAMERA_TRACKING_GEO_STATUS 独立调度**:
  - 将该消息从 STREAM_EXTRA3 流中移除，改为通过 deferred_message 机制独立调度
  - 设置固定 1Hz 发送频率，不受流速率（SRx_*）参数影响
  - 在高延迟模式和正常模式下均启用

### 3. 先前提交（已合并到分支）

- 目标点经纬度解析
- 摇杆控制跟随目标点
- 目标点上报消息（CAMERA_TRACKING_GEO_STATUS）

## 构建命令

```bash
# SITL 构建
./waf configure --board sitl
./waf copter

# 清理
./waf clean
./waf distclean
```

## 注意事项

- 传感器尺寸（6.17mm x 4.55mm）为 1/2.3" 传感器假定值，实际值可能因吊舱型号而异
- FOV 数据依赖于吊舱固件版本，需确保固件支持 D1 包中 FOV 字段
- 测距仪 3 字节格式向后兼容旧版 2 字节格式
- 调试模式（AP_MOUNT_VIEWPRO_DEBUG）当前为开启状态，正式发布前应关闭
