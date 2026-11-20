# DM6220 双轴云台上位机

基于 **达妙 DM6220 中空云台电机**（滑环、绝对编码器，额定 1.3 N·m / 峰值 2.7 N·m）+ **达妙 USB2CAN 串口模块**（DM-USBCU1 一类）的 PC 端双轴云台控制软件。

> 当前状态：**控制架构已切换（2026-11-20）**。生产路径只使用**位置-速度双环**：`pos_vel`（0x100+ID，驱动器位置环→速度环→电流环）或 `velocity_pos`（PC 位置环→0x200+ID 驱动器速度环→电流环）。**MIT 控制模式已从 service/config 移除**，MIT 编解码只保留在历史 bring-up 脚本中。
>
> 2026-11-19 已运行官方 `DMTool-v2.1.9.4` 与 `USB2CAN_2.0.0.3` 做界面观察：两者均为 **COM + 921600 8N1**，确认应按串口型 USB2CAN（`55 AA` 帧）设计，而不是 libusb 型 USB2CANFD。
>
> 2026-11-20 真机确认：**COM10**，yaw = CAN 0x01/MST 0x11，pitch = CAN 0x02/MST 0x12，速度模式 3；`PMAX/VMAX/TMAX = 12.5/45/10`，`NPP=14`，`Gr=1.0`。可用帧为 `cmd=0x01, CAN ID@13–14, DLEN@18, CRC=0`。
>
> 连续旋转实测：yaw -0.5 rad/s × 45 s，16-bit 反馈在 ±12.5 rad 回绕，累计 -22.38 rad（-3.56 圈）；`p_m`（0x50）为多圈绝对位置，不回绕。详见 [`docs/hardware-notes.md`](docs/hardware-notes.md) 与 [`docs/usb2can-serial.md`](docs/usb2can-serial.md)。

## 目标

1. 用 **位置-速度双环** 驱动两台 G6220，完成双轴云台的姿态控制；生产路径不再使用 MIT 模式。
2. `pos_vel`：PC 只发关节目标 + 速度限幅，驱动器内部完成位置环 → 速度环 → 电流环；`velocity_pos`：PC 位置环输出 `v_des`，驱动器完成速度环 → 电流环。
3. 实现双轴云台的 **姿态正/逆运动学**、角度解缠、软限位与轨迹规划。
4. 提供 **本地 Web UI + HTTP/WebSocket API + MCP Server + CLI**，让人和 AI 都能安全地操控云台。
5. 支持无硬件仿真测试，硬件到位后可直接切换到真机。

## 关键结论（先看这里）

| 项目 | 决定 | 原因 |
|---|---|---|
| 语言 | **Python 3.11+**（当前环境 3.13 已验证可用） | 生态、AI 集成、开发效率最好；500 Hz 控制环足够，Windows 上需专门的定时线程 |
| CAN 传输 | **`pyserial` + 达妙 USB2CAN 串口协议（`55 AA` 帧）** + **`python-can` `BusABC` 适配层** | 桌面的 DMTool 2.1.9.4 与 USB2CAN 2.0.0.3 都是 **COM + 921600 8N1**；Seeed 达妙教程也把这两个工具作为一套串口型 USB-CAN 方案。`dmcan-sdk`/libusb 仅作为 USB2CANFD 备选后端 |
| USB2CANFD 备选 | `dmcan-sdk` 2.1.x + `libdmcan` | 如果实物是 USB2CANFD_S/D 或 LinkX4C（VID 34B7），改用 libusb 后端；上层电机/控制代码不变 |
| 电机驱动 | **自行实现 POS_VEL / 速度帧 + 反馈 + 寄存器访问** | 生产路径使用驱动器原生位置/速度环；MIT 编解码仅保留给历史 bring-up 脚本 |
| 控制周期 | **默认 500 Hz**，串口带宽下不建议 1 kHz | 实测 **500.0 Hz、p99 2.78 ms、0 overrun、0 坏帧**（两轴 MIT 保持，1500 周期）；500 Hz 已占约 46 KB/s（921600 8N1 约 92 KB/s 的 50%），1 kHz 会打满串口 |
| 控制结构 | **位置-速度双环，MIT 已停用** | 默认 `pos_vel`：驱动器位置环→速度环→电流环；备选 `velocity_pos`：PC 位置环→驱动器速度环→电流环。速度模式实测抗 0.42 N·m 负载；MIT 位置 PD 在高速/负载下出现 46 Hz，已从生产路径移除 |
| 运动学 | **解析式 2-DOF 偏航/俯仰逆解** + 四元数/Euler 接口 | 两轴正交云台无需数值 IK；保留坐标变换层，方便以后扩展第三轴 |
| 上位机形态 | **本地常驻 daemon + Web UI** | AI、CLI、GUI 共享同一份状态和安全层，避免多个进程抢占 USB 设备 |
| AI 接口 | **MCP Server（stdio）作为 daemon 的客户端** | Claude Code 可直接调用；同时保留 HTTP/JSON，任何 AI/脚本都能接 |
| 配置 | YAML + Pydantic 校验 | 电机 ID、零点、增益、限位都需要现场标定，配置文件比硬编码合适 |
| 测试 | pytest + `python-can` virtual bus + 电机仿真模型 | 硬件没到之前就能把协议、控制、API 全链路跑通 |

## 真机验证结果（2026-11-20）

| 项目 | 结果 |
|---|---|
| 串口 | COM10，VID_2E88/PID_4603，921600 8N1 |
| 串口帧 | `cmd=0x01`，CAN ID 2 字节 @13–14，DLEN@18，CRC=0 |
| yaw 电机 | CAN 0x01 / MST 0x11，模式 3 |
| pitch 电机 | CAN 0x02 / MST 0x12，模式 3 |
| 限幅 | PMAX 12.5，VMAX 45，TMAX 10，NPP 14，Gr 1.0 |
| 速度模式 | 0.1 rad/s × 0.3 s，两台平滑转动约 0.02 rad |
| MIT 模式（历史，已停用） | yaw `Kp=0.5, Kd=0.1`；pitch `Kp=3.0, Kd=0.3`；高速/负载下出现 46 Hz，生产路径不再使用 |
| POS_VEL | `KP_APR=20`（yaw）；整定测试用 0.8 限幅，无 6/46 Hz、带载 0.97 N·m 稳定；整定后配置限幅 **10 rad/s** |
| 参数固化 | yaw `KP_APR=20` / pitch `KP_APR=54` / `KI_APR=0` 已发 flash 保存命令并回读通过（`store_params.py`） |
| **4 rad/s 连续旋转** | yaw +4.0 rad/s × 20 s，行走 **+79.99 rad（12.73 圈）**，3 次 wrap；位置差分峰值 **4.33 rad/s**，力矩峰值 **0.232 N·m**，HF 0.018–0.058；pitch 保持 −1.884 rad（偏差 ≤0.001），无 6/46 Hz |
| 高速 POS_VEL | ±0.5 rad / 2 s，实际峰值 yaw 2.04 / pitch 1.67 rad/s，HF 0.10–0.13，无 6/46 Hz |
| 速度模式 + 软负载 | 45 s，yaw 力矩峰值 0.42 N·m，HF 0.02–0.03，无 46 Hz |
| 反向连续旋转 | yaw -0.5 rad/s × 45 s，1 次 wrap，累计 -22.38 rad（-3.56 圈） |
| 正向连续旋转 | yaw +0.5 rad/s × 45 s，1 次 wrap，累计 +22.42 rad（+3.57 圈） |
| 编码器 | 16-bit 反馈在 ±12.5 rad 回绕；`p_m` 0x50 为多圈绝对位置；`xout` 0x51=0 |
| 500 Hz 基准 | **500.0 Hz，p99 2.78 ms，0 overrun，1501 帧/轴，3002 回执，0 坏帧** |
| pitch 小步 | 0.03 rad，`Kp=3.0/Kd=0.3`，误差约 0.005–0.008 rad，最大速度 0.165 rad/s |
| 纯力矩 `tau_ff` | 0.05 N·m 指令 → 反馈约 0.0415 N·m，yaw 加速到 0.582 rad/s；`KT_Value=0` 不影响纯力矩 |
| 最大实测 | 速度 0.582 rad/s，力矩 0.0464 N·m，温度 33/29 °C |
| 当前状态 | 两台已恢复速度模式 3、失能、timeout 0；增益已固化到 flash |
| 当前位置 | 4 rad/s 连续旋转后 yaw ≈ +67.5 rad（关节坐标，未回零），pitch ≈ −1.884 rad（pitch 失能后会缓慢下落） |

## 实现状态（2026-11-20）

MVP 已落地到 `src/dm6220_gimbal/`：

- `transport.py`：串口 USB2CAN `can.BusABC` 后端（自动识别 COM10/VID 2E88:4603）
- `protocol.py`：POS_VEL/速度帧、反馈解析、30 字节串口帧；MIT 编解码仅保留给历史脚本
- `motor.py`：`DmMotor` + `CanRouter`（POS_VEL/速度发送、寄存器读写、反馈分发、清故障）
- `control.py`：PID、低通、位置差分测速、`pos_vel` / `velocity_pos` 两种位置-速度双环
- `kinematics.py`：2-DOF 正逆解、yaw unwrap、`p_m` 重同步
- `service.py`：500 Hz 控制环、状态机、安全层、CAN 看门狗、TTL、遥测
- `api.py` + `ui/`：FastAPI HTTP/WebSocket + 本地 Web 仪表盘
- `cli.py`：`serve / probe / scan / status / enable / disable / estop / attitude / joints`
- `mcp_server.py`：MCP stdio 工具，供 Claude Code 等 AI 调用
- `tests/`：协议、运动学、PID、配置，19 个测试通过

默认配置使用 **`pos_vel`**：PC 发关节目标 + 速度限幅（整定后 **10 rad/s**），驱动器内部跑位置环→速度环→电流环。yaw 的驱动器位置环已整定为 `KP_APR=20`（`motors.yaw.position_kp_apr`，service 在 enable 时写入 RAM）；掉电保持已用 `scripts/store_params.py` 写入 flash（yaw 20/pitch 54，`KI_APR=0`）。备选 **`velocity_pos`**：PC 位置环输出速度，驱动器跑速度环→电流环。**MIT 模式已从 service/config 移除。**

> **pitch 机械警告**：实测 pitch 轴在 disabled 状态会因重力缓慢下落。请先机械配平/配重；`estop` 会立即失能，pitch 可能下落。POS_VEL/velocity_pos 下驱动器速度环能抵抗负载，但失能后没有保持力矩。
>
> **振动警告（2026-11-20）**：MIT 位置正弦 + 外部负载会出现约 **46 Hz** 振动；同速/同负载的速度模式对照完全干净（45 s、yaw 0.42 N·m、HF 0.02–0.03）。生产路径已改为 **POS_VEL/velocity_pos**，MIT 位置环停用。POS_VEL 的 yaw 6.2 Hz 小振荡通过 **`KP_APR 54 → 20`** 基本消除；带载 0.97 N·m 测试无 6/46 Hz，位置误差 ≤0.065 rad、峰值速度 0.65 rad/s。service 在 enable 时写入该增益（RAM）；掉电保持已用 `scripts/store_params.py` 固化到 flash。详见 [`docs/hardware-notes.md`](docs/hardware-notes.md) 2.7/2.8 节。
>
> **DMTool 兼容性（2026-11-20）**：DMTool v2.1.9.4 无法与当前 USB2CAN 模块固件通信——官方串口帧用 `cmd=0x03`，本模块只接受 `cmd=0x01`；“读参数”收到的只是模块回显（改用不存在的 CAN ID 后计数仍一一对等）。参数烧录因此使用项目内 `scripts/store_params.py`（官方 `save_motor_param` 同一条命令）。详见 [`docs/hardware-notes.md`](docs/hardware-notes.md) 2.8 节。

运行：

```powershell
pip install -e .
python -m dm6220_gimbal serve --config config/gimbal.example.yaml
# 浏览器打开 http://127.0.0.1:8765/ui/
python -m dm6220_gimbal probe --config config/gimbal.example.yaml
```

MCP（先运行 daemon）：

```powershell
pip install -e ".[ai]"
dm6220-gimbal-mcp --url http://127.0.0.1:8765
```

## 架构一览

```text
┌──────────────┐   ┌──────────────┐   ┌─────────────────────┐
│ 本地 Web UI  │   │     CLI      │   │   MCP Server / AI   │
└──────┬───────┘   └──────┬───────┘   └──────────┬──────────┘
       │                  │                      │
       └──────────────┬───┴──────────────────────┘
                      │  HTTP / WebSocket / MCP
              ┌───────▼─────────────────────────┐
              │         GimbalService            │
              │  状态机 / 命令邮箱 / 安全限幅    │
              │  遥测快照 / 审计日志             │
              └───────┬──────────────────────────┘
                      │
              ┌───────▼─────────────────────────┐
              │   ControlLoop（独立线程，500 Hz）│
              │轨迹 → 逆解 → 位置 PID → 力矩  │
              └───────┬──────────────────────────┘
                      │
              ┌───────▼─────────────────────────┐
              │   DmMotor × 2（POS_VEL/速度帧）  │
              └───────┬──────────────────────────┘
                      │
              ┌───────▼─────────────────────────┐
              │ python-can BusABC                │
              │ ├─ SerialUsb2CanBus（pyserial）  │
              │ ├─ DmCanBus（USB2CANFD 备选）    │
              │ ├─ SocketCAN（Linux 备选）       │
              │ └─ virtual（仿真/测试）          │
              └───────┬──────────────────────────┘
                      │
              达妙 USB2CAN（COM）→ CAN 1 Mbps → DM-G6220 × 2
```

## 计划中的仓库结构

```text
DM6220Gimbal/
├─ pyproject.toml
├─ README.md
├─ .gitignore
├─ config/
│  ├─ gimbal.example.yaml
│  └─ calibration.example.yaml
├─ docs/
│  ├─ architecture.md        # 详细设计
│  ├─ hardware-notes.md      # 电机/协议/模块资料
│  ├─ usb2can-serial.md      # 55 AA 串口帧协议与抓包方法
│  ├─ decisions.md           # 技术选型 ADR
│  └─ ai-control.md          # API / MCP / 安全约束
├─ src/dm6220_gimbal/
│  ├─ transport/             # CAN 后端与 python-can 适配
│  ├─ motor/                 # POS_VEL/速度帧、G6220 参数、DmMotor
│  ├─ control/               # PID、滤波、串级、轨迹、安全
│  ├─ kinematics/            # 正逆解、坐标变换、角度解缠
│  ├─ service/               # 状态机、控制循环、遥测、审计
│  ├─ api/                   # FastAPI、WebSocket、MCP、CLI
│  └─ ui/                    # 本地 Web 仪表盘
├─ scripts/
│  ├─ usb2can_probe.py       # 55 AA 帧探测/只读寄存器
│  ├─ yaw_continuous_test.py # 连续 yaw + pitch MIT 保持测试
│  ├─ pos_vel_sine_test.py   # 生产路径：POS_VEL 正弦 + HF 振动检测
│  ├─ pos_vel_continuous_test.py # 生产路径：yaw 4 rad/s 连续旋转 + pitch 保持
│  ├─ store_params.py        # 写驱动增益并保存到 flash（官方 save_motor_param）
│  ├─ velocity_sine_test.py  # velocity_pos/速度正弦 + HF 振动检测
│  ├─ mit_oscillation_test.py # 历史 MIT 对照（生产路径不使用）
│  ├─ serial_500hz_bench.py  # 双电机 500 Hz 串口/反馈基准
│  ├─ capture_usb2can.py     # 串口抓包/帧解析（后续）
│  ├─ fetch_dmcan_dlls.ps1   # 仅 USB2CANFD 备选后端需要
│  └─ can_scan.py            # 扫描总线上的电机（后续）
└─ tests/
   ├─ unit/                  # 协议、PID、运动学
   ├─ sim/                   # 虚拟 CAN + 电机模型
   └─ hardware/              # 真机测试（pytest -m hardware）
```

## 待确认的问题

已确认：

- [x] 串口型 USB2CAN，COM10，VID_2E88/PID_4603，921600 8N1。
- [x] 两台电机 CAN ID 0x01/0x02，Master ID 0x11/0x12。
- [x] `PMAX/VMAX/TMAX = 12.5/45/10`，`NPP=14`，`Gr=1.0`。
- [x] 串口帧格式：`cmd=0x01, CAN ID@13–14, DLEN@18, CRC=0`。
- [x] 速度模式、POS_VEL 与 MIT 对照均已实测；MIT 已从生产路径移除。
- [x] yaw 可连续旋转；16-bit 反馈在 ±12.5 rad 回绕，`p_m` 0x50 为多圈绝对位置，`xout` 0x51=0。
- [x] yaw 双方向连续旋转均正常：反向 -22.38 rad、正向 +22.42 rad；各 1 次 wrap。
- [x] 此前 +10 rad 的 0.696 N·m 尖峰确认是外部触碰，不是机械/控制问题。
- [x] 纯力矩 `tau_ff` 已验证：`KT_Value=0` 不影响；0.05 N·m 指令产生约 0.0415 N·m 反馈。
- [x] 500 Hz 双电机基准通过：500.0 Hz、p99 2.78 ms、0 overrun、0 坏帧。

仍需确认：

- [ ] 机械零位、两轴正方向、俯仰软限位。
- [ ] CAN 总线拓扑与 120 Ω 终端电阻；双 CAN 口是并联还是级联。
- [ ] 负载惯量、重心、俯仰轴重力矩，用于重力补偿和前馈。
- [ ] Windows 上 500 Hz 控制环的实际抖动与串口往返延迟。
- [ ] `KT_Value=0.0` 时纯力矩 `tau_ff` 的实际换算；需要低速、低力矩单独验证。
- [ ] 实物丝印/固件版本；是否支持高于 921600 的串口速率。

## 参考资料

- [DM-G6220 说明书（OpenELAB Wiki）](https://wiki.openelab.io/damiao/damiao-dm-g6220-motor-instruction-manual)
- [DM-G6220 中文说明书（Gitee）](https://gitee.com/kit-miao/DM-G6220)
- [Seeed Studio DAMIAO 电机教程](https://wiki.seeedstudio.com/damiao_series/)
- [DM-USBCU1 USB-CAN 模块（串口协议）](https://damiao.enactic.ai/en/products/hardware/dm-usbcu1/)
- [达妙 DMTool 发布仓](https://github.com/dmBots/motor-debugging-tool)
- [DM_CAN.py 串口驱动（55 AA 帧参考）](https://github.com/cmjang/DM_Control_Python/blob/main/DM_CAN.py)
- [达妙 DM_DeviceSDK / dmcan（USB2CANFD 备选）](https://gitee.com/kit-miao/dm-device-sdk)
- [dmcan-sdk（PyPI）](https://pypi.org/project/dmcan-sdk/)
- [python-can 文档](https://python-can.readthedocs.io/)
- [达妙 MIT 协议整理（python-damiao-driver）](https://github.com/jia-xie/python-damiao-driver/blob/main/docs/concept/communication-protocol.md)
- [LeRobot Damiao 驱动与参数表](https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/damiao/tables.py)

## 许可证

本项目使用 MIT License。第三方依赖的许可证（`pyserial` BSD-3-Clause、`python-can` LGPL-3.0、`dmcan-sdk` Apache-2.0 等）在实现阶段单独记录。
