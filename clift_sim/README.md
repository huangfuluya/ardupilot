# 双机协同吊挂 · SITL 仿真（载荷由两绳模型真算出来）

分支 **`clift-sitl`**（基于 `lthall/Leonard_ardupilot` 的 `master`），两架 ArduCopter SITL
从地面解锁、起飞、合队、绕圈、降落，全程约 108 s。载荷由扩展后的
`SIM_SlungPayload` 双挂点模型在 SITL 物理循环里计算，**两根绳的力分别反馈给两架飞机**。

## 本次运行

| 阶段 | 时间 |
| --- | --- |
| 两机在地面解锁 | t ≈ 0 s |
| 起飞（GUIDED，目标 20 m） | t ≈ 2 s |
| 两机升到 18 m，从机切 FOLLOW | t ≈ 12 s |
| 载荷离地（两绳绷紧） | t ≈ 9 s |
| 圆形巡查（R=18 m，周期 30 s） | t ≈ 17 – 82 s |
| LAND、下降 | t ≈ 83 s |
| 落地（载荷最后触地） | t ≈ 108 s |

日志 2138 行（约 19.5 Hz），FOLLOW_TARGET 转发 925 条。

| 指标 | 数值 |
| --- | --- |
| 编队间距（稳态） | 约 9.1 m |
| 载荷高度 | 离地 10 m 左右，随摆动 ±2 m |
| 载荷到领航机距离 | 4.5 – 10.3 m（绳长 10 m） |
| 载荷到跟随机距离 | 4.3 – 10.7 m |
| 载荷相对编队中点 | 低 0 – 9.3 m |

## 双挂点模型（`libraries/SITL/SIM_SlungPayload.*`）

新增参数（`SIM_SLUP_*`）：

| 参数 | 含义 |
| --- | --- |
| `MODE` | 0 单机 / 1 双机主（算载荷+两绳）/ 2 双机从（收力） |
| `PPORT` | 两机 UDP 端口基址（主绑基址，从绑基址+1） |
| `LINE2` | 第二根绳长（0 = 与 `LINELEN` 相同） |
| `POFS_N` / `POFS_E` | 从机 EKF 原点在主机的 NED 坐标系里的偏移（m） |

数据流：

```
从机 SITL ──(52 Hz: 自身位置/速度, 'S')──► 主机 SITL
主机：两绳几何约束 + 静力分配(2×2 最小二乘) + 动态项
      ──(每步: 作用在从机上的力, 'F')──► 从机 SITL
```

- 静力分配保证两绳竖直支撑之和 = 载荷重量（这是第一版"每根绳都算一份重力"的 bug 修正点）。
- 两球（绳长约束）不相交时（两机间距 > L1+L2），载荷被放到两挂点连线上，避免约束来回打架。
- 从机的 EKF 原点和主机不同，用 `POFS_N/E` 做坐标换算。

## 飞行日志（每架飞机各一个）

| 飞机 | 数据闪存日志 |
| --- | --- |
| 1 号（领航机，sysid 1，双机主） | `run0\logs\00000001.BIN`（9.2 MB） |
| 2 号（跟随机，sysid 2，双机从） | `run1\logs\00000001.BIN`（7.3 MB） |

配套 CSV（20 Hz 遥测）：

- `sim_log.csv` —— 两机合并（`l_*` 领航机，`f_*` 跟随机）
- `sim_log_lead.csv` / `sim_log_follower.csv` —— 单机各一份
- `sim_payload.csv` —— **SITL 算出的载荷轨迹**（`t,n,e,d,land,tens`，时间基准与本日志一致）

## 交付物

| 文件 | 说明 |
| --- | --- |
| `clift_sim.mp4` | 动画：地面待命 → 起飞 → 载荷离地 → 绕圈 → 降落（35.7 s，20 fps） |
| `clift_summary.png` | 四联图：俯视轨迹 / 高度（含载荷）/ 编队倾斜 / 间距 |
| `frame_takeoff.png` / `frame_survey.png` | 起飞、绕圈时刻截帧 |

> 动画里的载荷不再离线重建，直接读 `sim_payload.csv`；右下角标注
> "payload simulated by SITL (two-line model)"。

## 复现

```powershell
python clift_sim\run_sim.py --repo C:\Users\86913\Desktop\mh\ardupilot
python clift_sim\make_animation.py --fps 20 --step 0.15
```

`run_sim.py` 会先杀掉残留的 `arducopter.exe`、以 `--wipe` 启动两个实例，
结束后自动从主机日志里导出 `sim_payload.csv`。

## 限制（如实说明）

- **跟随机没有跑 `clift.lua`**：该 SITL 上跟随机加载脚本会触发 scripting 指令预算
  （`SCR_VM_I_COUNT`）超限，pre-arm 报 "exceeded time limit" 拒绝解锁。本架次只在
  领航机加载脚本（广播 FOLLOW_TARGET + 加速倾斜），跟随机用 `FOLL_OFS_X=-10` 保持编队。
  所以"能量均衡"这条协同逻辑没有真正闭环，只有载荷的力是双向耦合的。
- **载荷模型是研究级近似**：`SIM_SlungPayload` 的原始注释也说明它只是"rough approximation"，
  单绳版本已用 SITL 自身做校准（误差 3 cm，见 `../slung_calib/`），双绳版本的静力分配
  与动态项是新写的，尚未有真实飞行数据可对标。
- 编队倾斜曲线有明显振荡（±1.5 m），这是两机各自控制器对绳力反应不同造成的，
  说明"协同"还停留在几何跟随 + 被动受力，离真正的负载均衡控制还有距离。
