# XL-H100 毫米波雷达 → DroneCAN 转换固件

## 项目概述

基于 ArduPilot AP_Periph 的 CAN 外设固件，运行在 Vimdrones VM-L431-Periph-Pico 板
（STM32L431）上，将 XL-H100 77GHz 毫米波雷达（TTL 串口，NJXL 协议）的测距数据
转换为 DroneCAN 标准消息，供无人机定高使用。

- 广播消息：`uavcan.equipment.range_sensor.Measurement`（DS-015，DTID=1050）
- 广播频率：跟随雷达约 20Hz（受参数 `RNGFND_MAX_RATE` 上限 50Hz 控制）
- 飞控侧接收：`RNGFND1_TYPE = 26`（DroneCAN）

## 硬件连接

| 信号 | Pico 板 | 雷达（数据手册针脚） |
|---|---|---|
| 供电 | 5V | 针1（XL-H100 为 5V，其他型号 9-36V） |
| 地 | GND | 针2（必须共地） |
| 数据 | TELEM1 TX (PB6, USART1) | 针3 TTL-RX |
| 数据 | TELEM1 RX (PB7, USART1) | 针4 TTL-TX |

- TELEM1 = USART1 = 串口索引 1（`SERIAL_ORDER EMPTY USART1 USART2`）
- 调试：DroneCAN 调试器接 COM83（SLCAN，CAN 1Mbps）

## 雷达协议（NJXL 交互协议 V1.0.7 + 厂商 TTL 示例代码）

帧格式：`A5 A5 | 命令字 | 操作码 | 数据长度(2B LE) | 数据 | 校验`

- **校验 = 从帧头开始所有字节累加和 & 0xFF**
- 开机：`A5 A5 80 01 00 00 CB`（cmd=128 write）；雷达先回 ACK（op=0x02）再输出数据
- 关机：`A5 A5 81 01 00 00 CC`；**开关机状态存入雷达 Flash**，断电保持
- 检测输出：`A5 A5 82 00 len ...`，载荷 = 2字节周期号 + N×目标
- 目标 8 字节：X(i16) Y(u16) 速度(i16) SNR(u16)，全部小端，单位 cm
- **雷达会用全零填充未使用的目标槽（SNR=0），解析时必须按 SNR>0 过滤**
- 扫描周期 ≤100ms（20Hz）
- **实际默认波特率 115200**（数据手册规格；交互协议 PDF §3.1 的 921600 不适用本批次）
- 数据手册明确：TTL 与 CAN 输出二选一

示例帧解码：`A5 A5 82 00 1A 00 16 3F 53 F0 54 09 00 00 45 00 47 02 19 12 00 00 31 00 00...00 C5`
→ 周期号 0x3F16；目标1 X=-40.13m Y=23.88m SNR=69；目标2 X=5.83m Y=46.33m SNR=49；目标3为全零填充。

## 节点关键参数（Node ID 125）

| 参数 | 值 | 说明 |
|---|---|---|
| `RNGFND1_TYPE` | 49 | XLH100_Serial 驱动 |
| `RNGFND_PORT` | 1 | TELEM1 (USART1) |
| `RNGFND_BAUDRATE` | 115200 | 雷达波特率 |
| `RNGFND1_MIN/MAX` | 0.1 / 100 | 量程（米） |
| `CAN_NODE` | 125 | 静态节点 ID |

默认值已写入 `libraries/AP_HAL_ChibiOS/hwdef/VM-L431-Periph-Pico/defaults.parm`。

## 驱动逻辑（`libraries/AP_RangeFinder/AP_RangeFinder_XLH100_Serial.*`）

- 未收到首帧前每 5s 发一次开机命令；数据静默 10s 后重发
- 状态机解析 A5A5 帧，校验通过后：取帧内 **SNR>0 目标中的最小 Y**（最近目标）作为高度
- 全零帧（无有效目标）→ 报 `max+1m`（飞控侧显示 TOO_FAR）
- `reading_type` 映射：量程内=VALID_RANGE，<0.1m=TOO_CLOSE，无目标=TOO_FAR，超时=UNDEFINED
- 诊断（CAN LogMessage 每 5s）：`rx` 字节数/速率、`sync` 帧头数、`crc` 错误数、`ok` 有效帧数、`st` 解析状态、`b:` 最近 8 字节 hex

## 构建 / 烧录 / 测试（本机环境）

```bash
# 编译（WSL；Windows Python 直接跑 waf 会失败/破坏配置）
C:/Windows/System32/bash.exe -lc "cd /mnt/d/work_temp/27_ardupilot_vimdrones_radar && ./waf AP_Periph"

# DroneCAN 烧录（引导器 256 字节块、echo 节点分配器）
C:/Python312/python.exe -u build/can_flash2.py

# 远程设参
C:/Python312/python.exe -u build/can_param.py 125 RNGFND_BAUDRATE 115200 [--reboot]

# 监控（日志+RANGE+参数枚举）
C:/Python312/python.exe -u build/can_check.py 15
```

注意事项：

- **本仓库的 `param.Value` 是定制 DSDL（无 `numeric_value` 字段）**，设参必须直接赋
  `integer_value` / `real_value`，用 pip pydronecan 的 `numeric_value` 会静默编码为空值
- Windows Python 跑 waf 需 `PYTHONUTF8=1`，且与 WSL 环境不要混用 configure
- `build/can_*.py` 为本机调试脚本，不入库
- DroneCAN 相关子模块指针变更为 fork 既有状态，未包含在本项目提交中

## 已验证结果

- 雷达 20Hz 数据流解析零错误（`ok=7198 crc=0`）
- 全零填充目标导致 RANGE 恒 0.00 的 bug 已修复（SNR 过滤）
- DroneCAN 无线烧录、远程参数配置、诊断日志全部打通
