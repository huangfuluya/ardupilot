# CLAUDE.md — 项目工作记录

## 项目背景

基于 ArduPilot 固件，实现飞控通过 UART 串口驱动 **Emm_V5.0 / Emm42_V5.0 / ZDT_X42** 步进电机闭环伺服驱动板（下称"驱动板"），控制步进电机旋转，用于拉力台等测试装置。

- 驱动板参考资料：`ref_docs/`（说明书 PDF、通讯校验算法、Modbus-RTU 指令）
- 工作目录：`d:\work_temp\15_ardupilot_thrust_step_motor`

## 已完成工作

### 1. AP_StepMotor 驱动库完善（核心）

驱动库位置：`libraries/AP_StepMotor/AP_StepMotor.{h,cpp}`

修复/重构内容：
- **方向问题**：位置指令 `dir` 原先写死 0（恒 CW），现按目标角度增量符号取 CW/CCW
- **线程安全**：删除独立 IO 线程（`thread_create`），`update()` 改由 `SRV_Channels::push()` 主线程调用，按 `SRV_STM_DT` 周期发送，消除跨线程数据竞争
- **越界修复**：接收缓冲写入 `i<=128` 改为 `<128` 语义（帧缓冲上限按帧长约束）
- **阻塞式接收重写**：删除 `while(1)` 轮询，改为非阻塞帧解析状态机（按功能码确定帧长）
- **安全机制**：
  - 新增使能命令（`0xF3 0xAB`，上电自动使能）
  - 新增立即停止（`0xFE 0x98`）：RC 失控 + 硬件安全开关触发
  - 急停恢复后自动以反馈位置重新同步基准
- **协议功能**：
  - 校验支持 0x6B / XOR / CRC8 三种（`SRV_STM_CKSUM`）
  - 位置反馈轮询（250ms 读 `0x36` 实时位置）
  - 发送前检查 `txspace()`；E2 拒答限频告警

### 1.5 遥测解析与日志（STPM）

- **轮询（v2）**：`update()` 每 250ms 发送单一 0x43 系统状态命令（取代旧 0x36@250ms + 0x35/0x37/0x33/0x34@500ms 轮换），一次解析出 `_cur_deg`/`_cur_vel_rpm`/`_tpos_deg`/`_pos_err_deg` 及 `_bus_mv`/`_phase_ma`/`_flags_ready`/`_flags_motor`
- **帧解码**：`handle_frame()` 0x43 分支按实测 31B 布局解码（详见 1.10）；旧 0x35/0x33/0x34/0x37 分支保留但不再轮询
- **日志**：`Log_Write()` 每 200ms 写 `STPM` 消息（`LOG_STEPMOTOR_MSG`）：TimeUS / CPos / TPos / CurTPos / Vel / PErr；`_feedback_valid` 为 false 时不写；CurTPos 无数据源恒为 0
- **日志注册**：新建 `libraries/AP_StepMotor/LogStructure.h`（`LOG_IDS_FROM_STEPMOTOR` / `LOG_STRUCTURE_FROM_STEPMOTOR`），在 `AP_Logger/LogStructure.h` 注册 include 与宏引用
- **头文件注意**：`AP_StepMotor.h` 需先 `#include <AP_Logger/AP_Logger_config.h>` 再使用 `#if HAL_LOGGING_ENABLED`，否则声明被跳过导致 `Log_Write` 未声明

### 1.6 遥测诊断（GCS 周期打印）

- `read_incoming()` 记录：`_rx_bytes`（原始收到的字节数，地址匹配前就计数）、`_frames_ok`（校验通过并解析）、`_frames_bad`（校验失败/未知功能码，含错误命令返回 `01 00 EE 6B`）
- `write_cmd()` 记录：`_tx_bytes`（实际写入字节数）、`_tx_fail`（txspace 不足被丢弃次数）
- `update()` 每 5s 打印：`StepMotor: fb=0/1 cpos= vel= tpos= tx= txf= rx= ok= bad= age=s`（age=距最后收到数据的时间）
  - `tx=0` → 命令从未发出（软件：update 没跑 / uart 未找到）
  - `tx` 增长但 `rx` 冻结 → 板子停止回复（实测：两次不同发送速率下 rx 都精确冻结在 114/ok=14，确定性触发，约上电 2~3s 电机到位前后）
  - `rx>0, ok=0 bad=0` → 有字节但首字节 ≠ `SRV_STM_ADDR`（地址不匹配）
  - `bad` 增长 → 地址匹配但功能码未知/校验失败（CKSUM 不一致）
  - `ok` 增长且 cpos/vel 变化 → 链路正常

### 1.7 已修复：目标不变仍按 DT 周期重发绝对位置命令

- 现象：电机到位后固件仍每 DT（用户设 20ms）重发相同的 0xFD 帧（668B/s 无效流量）
- 修复：仅当目标相对上次命令变化 ≥1 脉冲时才发送；空闲时串口只剩轮询流量（18B/s）
- 待验证：该重发洪泛是否为板子停止回复的触发条件（实验：`SRV_STM_CHAN=0` 纯轮询看 rx 是否持续增长；或只重启驱动板看 rx 是否再涨一截后停）

### 1.8 TX/RX 调试打印（STM TX / STM RX）

- `write_cmd()` 成功写入后调用 `print_tx()`：HEX 打印完整命令到 GCS（`STM TX: 01 36 6B`）
- `read_incoming()` 三条路径调用 `print_rx()`：校验通过（`STM RX ok:`）、校验失败（`STM RX bad:`）、未知功能码（`STM RX bad:`，2 字节）
- 限流（v3，按功能码分槽）：`fn_slot()` 映射 0x33/0x34/0x35/0x36/0x37/0xFD/其他 → 槽 0..6，每槽 2s 限一次，另加 TX/RX 各自 200ms 全局间隔
  - v1/v2 缺陷：仅比较"与上次打印的功能码是否相同"（500ms/5s），250ms 的 0x36 与 500ms 遥测轮换交错时 0x36 被系统性饿死（实际已发送：tx 增量 90B/5s = 20×3B + 10×3B 可证明），v3 已修复
- 用途：人工核对第 1 字节=ADDR、第 2 字节=功能码、末字节=校验（0x6B 模式固定 6B）；板子静默后最后一条 `STM RX` 即板子发出的最后一帧
- 实测线索：rx=114 = 13×8B(0x36) + 1×6B(0x35) + 1×4B(ACK) + 1×错误帧，几乎全是读取返回、无 0xFD ACK 流 → Response 菜单（用户设 Both）的到位返回机制是板子停止回复的头号嫌疑，建议改 None/Receive 测试
- 曾遇链接错误：`Log_Write` 只声明未定义 → 在 cpp 补定义；`-fsyntax-only` 无法发现此类问题，需编译出目标文件验证
- **新增参数**：`ADDR`（电机地址）、`CKSUM`（校验方式）、`FSACT`（失控动作）；`DIV` 默认值由 64 改回 16（与板子出厂一致）
- **保留速度模式**：`Emm_V5_Speed_Control`（0xF6）已实现但未启用，供拉力台恒速转动扩展

### 1.9 2026-08-17 测试结果与鉴别矩阵（进行中）

- **PC 直连（USB-TTL）测试通过**：板子对驱动指令与读取指令均有正确回复；Response=None/Receive 均正常 → Response 菜单嫌疑排除
- **新固件实测**：tx 每 5s +90B（完整轮询速率，发送侧 100% 正常）。注意：诊断行超 50 字符被 GCS 拆成两条显示（`tx=738` + `6 txf=0...` → 实际 tx=7386）
- **打印缺 0x37**：statustext 队列溢出按相位系统性丢弃（字节增量证明 0x37 在发送），仅显示层问题
- **rx 仍精确冻结 114/ok=14/bad=1**，age 持续增长 → 触发点仍在启动后 ~2~3s
- **当前两个不可区分假设**：A=板子停止发送（对 6req/s 混合流停摆）；B=板子在发、飞控 RX 通道死亡（EMI 帧错误致 ChibiOS RX 停摆等）
- **鉴别测试矩阵**（本轮不改代码，避免混淆变量）：
  1. 上电抓全程日志 → 看静默前最后一条 `STM RX`
  2. `SRV_STM_CHAN=0` 纯轮询（轮询先于通道检查，不受影响）→ 隔离电机动作因素
  3. USB-TTL RX 并接板子 TX 线旁听 → 区分 A/B（决定性）
  4. PC 重放 `01 36 6B`@250ms 持续 ≥1min + 单发 `01 34 6B`/`01 37 6B` → 验证板端停摆与 bad=1 元凶（`01 00 EE 6B` NAK 对应不支持的指令码）
- 后续方向：若 B → 查 Telem2 接线/换串口验证/查 UARTDriver RX 错误处理；若 A 且元凶为 0x34/0x37 → 遥测轮换表删除该指令
- **11:52 重启复测（rx=216/ok=27/bad=1）新结论**：
  - 字节分解唯一解：13×8B(0x36)+2×6B(0x35)+12×8B(0x33/34/37)+1×4B(错误帧) → 板子**能回复 0x34/0x37**，"0x34 不支持"排除；bad=1 大概率为开机使能命令被 NAK（`01 00 EE 6B`），良性
  - 本轮电机从未动作（cpos=0、纯轮询 tx+90B/5s）→ 电机到位/电流冲击理论排除
  - **冻结点 114→216 漂移（存活 ~3.4s→~7s）**：同固件同参数仅重启即 2 倍差异 → 确定性板端逻辑基本排除，指向物理层/时序边缘性（线缆间歇、斩波电流 EMI、飞控 RX 通道帧错误后停摆）
  - PC 直连用的是另一套物理链路，证明不了 FC↔板线缆无问题
- **更新测试清单**：
  1. Telem2 TX-RX 短接自环：rx 增长 → FC RX 完好（查外部链路）；rx 冻结 → FC 端口/驱动问题（换 SERIAL4 复测）
  2. DataFlash .BIN 日志查 MSG 项 → 抓开机阶段 STM RX 突发与静默前最后一帧（GCS 后连看不到开机消息）
  3. USB-TTL 旁听板子 TX 线（区分板停发/飞控收不到）
  4. PC 重放 `01 36 6B`@250ms×60s
  5. 无论结果先换一根 FC↔板 RX 线并确认共地
- **11:52+ 新问题：CHAN≠0 后电机不动作**。排查分流（不改代码）：
  1. 驱动电机时看是否有 `STM TX: 01 FD`（tx 每命令 +13B）→ 区分"飞控未发/板子不理"
  2. 未发的四个门：安全开关未按下（MSG 有 `stopped (safety switch)`）；无 RC 时 `in_rc_failsafe()`=真 + `FSACT=1` → 持续急停（改 FSACT=0）；被跟随通道无 PWM 输出；`SRV_STM_VEL=0`
  3. 已发不动 → 上电 2s 内立刻驱动：动=板子接收通道随静默一起死（与 rx 冻结同根因）；不动=回查第 2 步
  - 注意：PC 手动零散发指令撑不到 15~28 帧死亡计数，证明不了板子无"计数/定时死亡"；PC 连续重放测试仍必做
- **14:xx 重大进展：SERIAL3→SERIAL2 换口后电机控制恢复正常**，但反馈仍收不到
  - 换口修复控制 → 飞控软件栈+板子收发能力全部证伪嫌疑，问题收敛到端口物理链路层
  - 当前两模式分流（看 rx）：rx=0 → Telem2 RX 路径断（换口时 RX 针脚插错/压接不良为头号嫌疑，TX 与共地已被"控制正常"证明完好）；rx 冻结 → 老静默问题复现，走旁听测试
  - 一分钟测试：①万用表/目视核对 Telem2 RX 针位与线序 ②Telem2 TX-RX 自环看 rx 是否增长（bad 涨正常）
  - 待用户回报 rx/ok/bad/age 四值后定下一步
- **23:15 0x43 固件实测（静默照旧）**：rx 冻结 283 = 9×31(0x43 完整帧) + 4(开机使能 NAK)，9 帧÷4Hz ≈ 上电 2.3s 后死；vb/st/cpos 均为真实解码（vb=12214、st=0x83）。0x36 时代 13 帧停 vs 0x43 时代 9 帧停 → **与功能码无关，板端混合功能码理论正式排除**
- 剩余两类解释：物理链路边缘性（线缆/EMI 致板子接收死；控制正常仅因命令集中在开机 2s 窗口内）或板端固件"开机窗口/回复计数"怪癖
- **决定性实验 A（PC 高频重放）**：USB-TTL 定时自动发 `01 43 7A 6B`@250ms×60s 数回复 → 复现=板端固件实锤（对策：窗口期利用 50ms 快轮询抢初始化/厂家固件/换 Modbus 或换板）；不复现=FC↔板链路病 → 实验 B（三根线全换新+远离动力线）
- 辅助：DataFlash .BIN 查 MSG 项可看到开机 STM RX 突发与最后回复时间戳（GCS 后连看不到）

### 1.11 FC 接收链路逐层审计（2026-08-17，用户报"接收函数可能有问题"）

- **逻辑定位**：`_rx_bytes` 在解析前计数，rx 冻结 = `UARTDriver::available()/read()` 无数据 → 与帧解析状态机无关，嫌疑只能在 UART 驱动层及以下
- **UARTDriver（ChibiOS）RX 路径审计结论：无缺陷**
  - DMA 双 bounce 缓冲（64B×2）；IDLE 中断 `rx_irq_cb` 关 DMA → `rxbuff_full_irq` 拷贝入 `_readbuf` 并重开 DMA；缓冲满同样路径
  - 1kHz `_rx_timer_tick` 看门狗：发现 DMA 流被关则拷贝残余字节并重新武装（已防"中断被吞"）
  - `RX_BOUNCE_BUFSIZE=64` > 31B 0x43 帧；H7 TRBUFF errata 位已处理；bounce 缓冲 MEM_DMA_SAFE + cache invalidate
- **ChibiOS serial_lld（USARTv2）审计结论：无缺陷**
  - 每次 USART IRQ 入口 `isr=u->ISR; u->ICR=isr;` 立即清 ORE/FE/NE（EIE 已由 ChibiOS 强制置位，错误必触发 IRQ）
  - ArduPilot 的 `irq_cb` 只挂 IDLE 事件（`_serial_irq_code`），不绕过错误清理
- **现有 A/B 开关**：`SERIALx_OPTIONS` bit8 = RX_NoDMA（+256，需重启）→ 强制 `read_bytes_NODMA` 中断逐字节路径，绕开整个 DMA RX 链
- **新诊断（已实施→已回滚）**：曾给 `AP_HAL::UARTDriver` 基类加 `rx_stats_bytes()` 虚函数并在 ChibiOS override，诊断行加 `dma=`；**用户明确要求不得改动 AP_HAL / AP_HAL_ChibiOS（底层接口完全正确），已全部回滚，git diff 确认两目录零改动**。诊断行恢复 `vb/st` 版
- **约束（长期有效）**：排查与修复只能动 `libraries/AP_StepMotor`、`libraries/AP_SerialManager`（既有协议接入）、参数与外部实验，不动 HAL 层
- 分层定位改由零代码 A/B 完成：自环测试 + `SERIAL2_OPTIONS` bit8(RX_NoDMA, +256) 切换 + USB-TTL 旁听板子 TX 线
- 注意反常特征：历次冻结都精确停在**帧边界**（字节数可整除分解）、无乱码、bad 恒为开机 NAK——不像噪声致 USART 错误的典型表现；旁听测试（USB-TTL 挂板子 TX 线）仍需做
- waf 编译通过（objs/AP_StepMotor + objs/AP_HAL_ChibiOS）

### 1.10 0x43 读取系统状态参数（说明书 Rev1.3 p51-52，已核对）

- 发送 `01 43 7A 6B`（4B）；**用户板实测返回 31B**（比说明书 29B 示例多 2 字节头部字段）
- 实测帧 `01 43 1F 09 2F FD 00 15 BA A6 01 00 00 00 03 00 00 00 01 00 00 00 01 01 00 00 00 02 03 83 6B` 试配结果（内部自洽：tpos−cpos=perr 精确成立）：
  - [2-3] 未知头部 u16 = 0x1F09=7945（疑似固件版本 v31.9，可核 OLED 版本页）
  - [4-5] 母线电压 12285mV | [6-7] 相电流 21mA | [8-9] 编码器 47782
  - [10] 符号+u32 目标位置 −0.0165° | [15] 符号+u16 转速 0RPM | [18] 符号+u32 实时位置 −0.0055° | [23] 符号+u32 位置误差 −0.011°
  - [28] 就绪标志 0x03 | [29] 电机状态 0x83（使能+到位，bit7 为本固件扩展）
- **已实施**（用户批准 2026-08-17）：单一 `S_State` 0x43@250ms 取代 0x36@250ms + 35/37/33/34 遥测轮换（总线功能码 5→1 种，6→4 请求/秒）；`_rxbuf` 12→32；`expected_frame_len` 加 0x43→31；`handle_frame` 加 0x43 解码分支（新增 `emm_angle()` 辅助）；`fn_slot` 0x43→槽 0；诊断行追加 `vb=%umV st=%02X`；`_tpos_cur_deg` 无源保持 0（用户批准）；删除 `_last_poll2_ms`/`_poll_index`/`STEP_MOTOR_POLL2_MS`
- 旧 0x36/0x35/0x33/0x34/0x37 解码分支保留（不再轮询，收到仍可解析）
- waf 单库编译通过（2026-08-17）
- **事故与修复（2026-08-17）**：驱动板上电即飞控死机 → 根因是 `print_rx()` 的 `char hex[3*12+1]` 仍按旧 12B 帧定长，31B 的 0x43 帧经 `hex_dump` 写入 94B → 栈溢出 HardFault（板子上电回复第一帧即触发首帧打印，时序吻合）。修复：`char hex[3*sizeof(_rxbuf)+1]`（97B）。教训：帧长变更必须同步审计所有按帧长定长的栈缓冲区；编译器无法捕获运行时长度写入固定数组

- `libraries/AP_SerialManager/AP_SerialManager.cpp`：init 分支新增 `SerialProtocol_StepMotor`（115200、128/128 缓冲、禁流控）；`SERIALx_PROTOCOL` @Values 文档补充 `49:StepMotor`
- `libraries/AP_SerialManager/AP_SerialManager_config.h`：新增 `AP_SERIALMANAGER_STEPMOTOR_*` 宏

### 3. 集成点（未改动，确认可用）

`SRV_Channels` 已挂载驱动（`SRV_Channel.h` 中 `stepmotor` 对象 + `SRV_Channels.cpp` 中 `_STM_` 参数组和 `update()` 调用），模式与 AP_Volz_Protocol / AP_RobotisServo 一致。

### 4. 文档

- 根目录 `固件使用说明.md`：AP_StepMotor 完整参数说明、接线、配置流程、STPM 日志说明、FAQ

## 参数总览（前缀 SRV_STM_）

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| CHAN | 0 | 控制通道（0=禁用） |
| DT | 100 | 指令发送周期 (ms) |
| SC | 1.0 | 角度比例（目标角 = norm × SC × 360°） |
| VEL | 500 | 速度 (RPM) |
| ACC | 0 | 加速度档位 (0=直接启动) |
| DIV | 16 | 细分（须与驱动板 P_Pul 一致） |
| ADDR | 1 | 电机地址（须与驱动板 ID 一致） |
| CKSUM | 0 | 校验（0:0x6B / 1:XOR / 2:CRC8） |
| FSACT | 1 | 失控动作（0=无 / 1=急停） |

## 配置流程

1. `SERIALx_PROTOCOL = 49`（StepMotor）、`SERIALx_BAUD = 115`
2. 被跟随通道设 `SERVOx_FUNCTION`（如脚本输出）
3. `SRV_STM_CHAN` 指向该通道
4. 核对 `SRV_STM_ADDR`、`SRV_STM_DIV` 与驱动板 OLED 菜单一致
5. 无 RC 纯 MAVLink 控制时设 `SRV_STM_FSACT = 0`

## 验证情况

- 修改文件已用 arm-none-eabi-g++（GNU Arm Embedded Toolchain 10-2020-q4-major）语法检查通过：`AP_StepMotor.cpp`、`AP_SerialManager.cpp`、`SRV_Channels.cpp`（0 error）
- 新增遥测解码/日志后再次语法检查通过（exit=0，仅无关头文件的 trigraph 警告）
- waf 单库增量编译（推荐验证方式，带全量 -Werror，需 Cygwin Python3.9）：
  ```
  cd /cygdrive/d/work_temp/15_ardupilot_thrust_step_motor
  (export PATH=/usr/bin:$PATH; ./modules/waf/waf-light build --targets=objs/AP_StepMotor)
  ```
  2026-08-16 验证通过（print_tx v3 分槽限流 + print_rx 版本）
- waf 构建注意：Windows 原生 Python 下 `waf configure` 因 `/dev/null` 报错，需在 MSYS/Cygwin 环境或 Linux 下构建

## 待办 / 后续建议

- [ ] 多电机支持（按 SerialProtocol_StepMotor 多 instance 或 ADDR 数组化）
- [ ] 波特率参数化（当前固定 115200）
- [ ] 若需速度模式恒速控制，启用 `Emm_V5_Speed_Control`（0xF6）
