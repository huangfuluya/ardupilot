# 双机协同吊挂 · SITL 仿真（两机都跑 clift.lua，载荷由两绳模型真算）

分支 **`clift-sitl`**（基于 `lthall/Leonard_ardupilot` 的 `master`），两架 ArCopter SITL
从地面解锁、起飞、合队、绕圈、降落，全程约 108 s。

- 载荷由扩展后的 `SIM_SlungPayload` **双挂点模型**在 SITL 物理循环里计算，两根绳的力分别反馈给两架飞机；
- **两架飞机都运行 `clift.lua`**：领航机广播 `FOLLOW_TARGET`（`custom_state` 里带电池状态），
  跟随机解码后做加速倾斜 + 能量均衡，形成控制层的闭环。

## 本次运行

| 阶段 | 时间 |
| --- | --- |
| 两机在地面解锁 | t ≈ 0 s |
| 起飞（GUIDED，目标 20 m） | t ≈ 2 s |
| 两机到 20 m、从机切 FOLLOW | t ≈ 14 s |
| 载荷离地（两绳绷紧） | t ≈ 9 s |
| 圆形巡查（R=18 m，周期 30 s） | t ≈ 17 – 82 s |
| LAND、下降 | t ≈ 83 s |
| 落地（载荷最后触地） | t ≈ 108 s |

| 指标 | 数值 |
| --- | --- |
| 日志行数 | 2163（约 19.5 Hz），FOLLOW_TARGET 转发 928 条 |
| 编队间距（稳态） | 约 9 m |
| 载荷高度 | 离地 ~9 m，随摆动 ±2 m |
| 载荷到领航机 / 跟随机距离 | 4.5–10.1 m / 4.7–10.2 m（绳长 10 m） |
| 载荷相对编队中点 | 低 0–9.0 m |

## 控制层闭环验证（GCS 里能看到）

两机脚本每 5 s 打一条状态：

```
SYS1: clift: lead lean -0.74m
SYS2: clift: 237/264mAh 39/36A L-0.81 B-0.08
```

- `237/264mAh`、`39/36A`：自己/领航机的已消耗电量与电流（来自 `FOLLOW_TARGET.custom_state` 的 gossip）；
- `L-0.81`：加速倾斜偏移（m，正 = 爬升）；
- `B-0.08`：能量均衡偏移（m，正 = 爬升）。

平衡量的符号是对的：领航机电流更大（44 A vs 33 A）时跟随机会往上抬、多分担载荷。

### 人为制造能量差，验证平衡真的在动

SITL 里两机电池模型完全一样，平时 `B` 只有 ±0.1 m 左右。用一个测试参数
`CLIFT_GOSSIP_OFS`（默认 0，只加在广播出去的电量上）让领航机"假装多耗 2 Ah"：

| 架次 | 从机 B 偏移 | 从机比领航机平均高 |
| --- | --- | --- |
| 正常 | ±0.1 m | +0.18 m |
| 领航机 +2 Ah | +0.3 … +0.6 m | **+0.88 m** |

从机确实按 `BAL_P·ΔAh` 抬高了自己（0.5 m/(Ah)·1 Ah ≈ 0.5 m），说明"领航机电量 → gossip →
从机偏移"这条控制回路是通的，不是只打印数字。

## 双挂点模型（`libraries/SITL/SIM_SlungPayload.*`）

| 参数 | 含义 |
| --- | --- |
| `SIM_SLUP_MODE` | 0 单机 / 1 双机主（算载荷+两绳）/ 2 双机从（收力） |
| `SIM_SLUP_PPORT` | 两机 UDP 端口基址（主绑基址，从绑基址+1） |
| `SIM_SLUP_LINE2` | 第二根绳长 |
| `SIM_SLUP_POFS_N/E` | 从机 EKF 原点在主机的 NED 偏移（本架次东 14.5 m） |

```
从机 SITL ──(位置/速度 'S', 400 Hz)──► 主机 SITL
主机：两绳几何约束 + 静力分配(2×2 最小二乘) + 动态项
      ──(作用在从机上的力 'F')──► 从机 SITL
```

- 静力分配保证两绳竖直支撑之和 = 载荷重量；
- 两球约束不相交（间距 > L1+L2）时把载荷放到两挂点连线上；
- 从机与主机的 EKF 原点不同，用 `POFS_N/E` 换算。

## 飞行日志（每架飞机各一个）

| 飞机 | 数据闪存日志 |
| --- | --- |
| 1 号（领航机，sysid 1，双机主） | `run0\logs\00000001.BIN`（9.6 MB） |
| 2 号（跟随机，sysid 2，双机从） | `run1\logs\00000001.BIN`（7.8 MB） |

配套 CSV：`sim_log.csv`（两机合并）、`sim_log_lead.csv`、`sim_log_follower.csv`、
`sim_payload.csv`（SITL 算出的载荷轨迹）。

## 复现

```powershell
python clift_sim\run_sim.py --repo C:\Users\86913\Desktop\mh\ardupilot
python clift_sim\make_animation.py --fps 20 --step 0.15
```

## 这次修掉的三个问题（跟随机脚本相关）

1. **"exceeded time limit" 的真正原因**：之前中断的运行留下了两个 `arducopter.exe`
   进程占着 UDP 端口，新实例连到的是旧进程、跑的是旧脚本。脚本本身早已改成
   手写 `string.pack/unpack`（不再 `require` MAVLink 模块）+ 有界收包循环，
   现在两机都能正常加载。`run_sim.py` 启动前会 `taskkill` 并 `--wipe`。
2. **跟随机不施加偏移**：`is_taking_off()` 在从 GUIDED 切到 FOLLOW 后仍为真
   （GUIDED 起飞状态机没结束），脚本一直走"等待起飞"分支。修法：FOLLOW 模式下
   忽略该标志；同时 run_sim 改成两机到 20 m 并稳定 3 s 后再切 FOLLOW。
3. **gossip 通道**：`FOLLOW_TARGET.custom_state` 由领航机发出，`run_sim` 转发到从机
   （真实系统里是同一遥测链路），从机脚本 `mavlink:init(5,1)` + `register_rx_msgid(144)`
   收包并解码，实测每 5 s 一条，数值与实际电池一致。

## 仍然存在的限制（如实说明）

- **平衡收益在 SITL 里看不出来**：两架飞机的电池模型完全相同，消耗差只有几 mAh，
  所以 `B` 只有 ±0.3 m。真机上两机负载不同才会有明显效果。
- **载荷模型是研究级近似**：单绳版本已用 SITL 自身校准到 3 cm（见 `../slung_calib/`），
  双绳版本的静力分配与动态项是新写的，没有真实飞行数据可对标。
- 编队倾斜曲线仍有振荡（±1.5 m）：两机各自控制器对绳力的反应不同步，
  说明离"主动负载均衡"还有距离——这是下一步值得做的。
