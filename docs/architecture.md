# 系统架构设计

> 版本：v0.2（2026-11-19，根据官方 DMTool/USB2CAN 工具实测修正传输层）
> 适用范围：DM-G6220 ×2 + 达妙 USB2CAN 串口模块（DM-USBCU1 一类；USB2CANFD 作为备选后端）+ Windows PC 上位机。

## 1. 目标与边界

### 1.1 必须实现

1. 两台 G6220 的 MIT 模式驱动：使能、失能、清零、模式切换、MIT 收发、反馈解析、错误处理。
2. 双轴云台姿态控制：目标姿态（yaw/pitch 或四元数）→ 关节角 → 电机指令。
3. 位置环在上位机、电流环在驱动器内的串级控制；可选速度环。
4. 安全层：软限位、速度/加速度/力矩限幅、反馈超时、温度保护、急停、状态机。
5. AI 可控：稳定的本地 API + MCP 工具，带审计和 TTL。
6. 无硬件也能开发：虚拟 CAN + 电机/云台仿真。

### 1.2 非目标（第一版）

- 不在 PC 上实现高频 FOC 电流环；该环由 G6220 驱动器完成。
- 不做 3 轴/冗余机械臂的数值 IK；先做 2-DOF 解析解，保留扩展点。
- 不做云端控制；默认只监听 localhost。
- 不做自动 PID 整定（先提供阶跃响应记录和手动调参）。

## 2. 运行形态与线程模型

```text
┌─────────────────────────── gimbald 进程 ───────────────────────────┐
│                                                                    │
│  [uvicorn / FastAPI]  ← HTTP / WebSocket ← Web UI / CLI / MCP      │
│          │                                                         │
│          │ 命令邮箱（离散命令队列 + 最新目标覆盖）                  │
│          ▼                                                         │
│  ┌──────────────────────── GimbalService ───────────────────────┐  │
│  │ 状态机 / 安全层 / 遥测快照 / 审计日志 / 配置                  │  │
│  └───────────────┬───────────────────────────────┬──────────────┘  │
│                  │ 控制周期触发                   │ 状态读取        │
│                  ▼                               │                 │
│  ┌───────────────────────────────┐               │                 │
│  │ ControlLoop 线程（500 Hz）     │               │                 │
│  │ 轨迹 → 逆解 → PID → MIT 帧    │               │                 │
│  └───────────────┬───────────────┘               │                 │
│                  │ send()                        │                 │
│                  ▼                               │                 │
│  ┌───────────────────────────────┐   ┌───────────▼──────────────┐  │
│  │ python-can BusABC             │   │ RX 线程                   │  │
│  │ SerialUsb2CanBus / DmCanBus   │──▶│ bus.recv() → 解析反馈     │  │
│  │ SocketCAN / virtual           │   │ → 更新 MotorState 快照    │  │
│  └───────────────┬───────────────┘   └──────────────────────────┘  │
│                  ▼                                                 │
│        达妙 USB2CAN（COM）→ CAN 1 Mbps → G6220 ×2                  │
└────────────────────────────────────────────────────────────────────┘
```

### 线程职责

| 线程 | 职责 | 实时性 |
|---|---|---|
| `control` | 按固定周期生成并发送两台电机的 MIT 帧；执行 PID、轨迹、限幅 | 高（默认 500 Hz） |
| `rx` | 阻塞 `bus.recv(timeout)`，解析反馈，更新状态快照与超时计数 | 中高 |
| `api` | uvicorn 事件循环；处理 HTTP/WS；不直接碰 CAN | 低 |
| `telemetry` | 按 50–100 Hz 写 JSONL/CSV、推送 WS；可合并进 api | 低 |

- **只有 `control` 线程调用 `bus.send()`**；急停由控制线程在下一个周期优先处理。发送加锁仅用于防止未来其他线程误用。
- 串口型后端有一个独立的**串口读线程**：按 `AA ... 55` 切帧、校验长度、解析 CAN ID/数据，然后放入 `queue.Queue`；`bus.recv()` 只是从队列取。读线程不写串口，避免与发送线程竞争。
- 如果使用 USB2CANFD 备选后端，SDK 的 RX 回调**只做拷贝和入队**；不调用阻塞 SDK 接口，避免 `REENTRANT`。
- 状态快照使用不可变 dataclass；RX 线程整体替换引用，API 读取最新引用，避免长锁。
- 离散命令用 `queue.Queue`；连续目标用“最新值 + 序号 + 时间戳”覆盖，避免队列堆积导致动作滞后。

### 控制周期与 Windows 定时

- 默认 `rate_hz: 500`（2 ms）。串口型 USB2CAN 下**不建议超过 500 Hz**；除非确认模块支持更高串口波特率，否则 1 kHz 会打满 921600 8N1 链路。
- 启动时调用 `timeBeginPeriod(1)`；每周期：
  1. `deadline += period`
  2. 先 `sleep` 到 deadline 前约 0.5–1 ms
  3. 剩余时间自旋等待
  4. 记录实际 `dt` 和抖动
- `dt` 实测值用于 PID；限制在 `[0.5, 2] × nominal_dt`，异常时记录 overrun。
- 不在热路径分配大对象、不写日志、不做 JSON；日志进入无锁环形缓冲，由低优先级线程落盘。
- 可选：`SetThreadPriority(THREAD_PRIORITY_TIME_CRITICAL)`；仅在实测需要时启用。

> 为什么不是 1 kHz 起步：除了 Windows Python 线程尾延迟和 USB 往返，串口协议的固定开销也很大。每个电机每周期需要 30 字节 TX + 16 字节 RX；两台电机 500 Hz 合计约 `(30+16)*2*500 = 46,000 B/s`，921600 8N1 理论约 `92,160 B/s`，已占约 50%。1 kHz 会达到约 100%，没有余量。云台机械带宽低，500 Hz 足够；真正的高频环在驱动器内。

## 3. 模块划分

```text
src/dm6220_gimbal/
├─ config.py                 # Pydantic 配置模型、YAML 加载、校验
├─ transport/
│  ├─ base.py                # 后端工厂、错误类型
│  ├─ serial_usb2can.py      # SerialUsb2CanBus(can.BusABC)，55 AA 串口帧
│  ├─ dmcan_backend.py       # USB2CANFD 备选：dmcan-sdk 适配
│  ├─ socketcan_backend.py   # Linux 备选
│  └─ virtual.py             # 虚拟总线/仿真节点
├─ motor/
│  ├─ g6220.py               # 型号常量、限幅、默认值
│  ├─ mit.py                 # MIT 编码/解码（纯函数，易测）
│  ├─ feedback.py            # 反馈帧解析、状态码
│  └─ driver.py              # DmMotor：使能/失能/模式/寄存器/发送
├─ control/
│  ├─ pid.py                 # PID + 抗饱和 + 微分滤波
│  ├─ filters.py             # 一阶低通、限幅、速率限制
│  ├─ cascade.py             # pos_current / pos_vel_current / impedance
│  ├─ trajectory.py          # 梯形/S 曲线、姿态轨迹
│  ├─ gravity.py             # 重力/惯量前馈（可选）
│  └─ safety.py              # 限幅、超时、温度、急停策略
├─ kinematics/
│  ├─ transforms.py          # 旋转矩阵/四元数/Euler（小工具，不引 scipy）
│  └─ gimbal2d.py            # 2-DOF 正逆解、解缠、标定
├─ service/
│  ├─ state.py               # 状态枚举、不可变快照
│  ├─ gimbal_service.py      # 编排：连接、使能、控制、故障
│  ├─ loop.py                # ControlLoop 线程
│  ├─ telemetry.py           # 环形缓冲、记录、WS 推送
│  └─ audit.py               # 命令审计
├─ api/
│  ├─ http.py                # FastAPI 路由
│  ├─ schemas.py             # 请求/响应模型
│  ├─ ws.py                  # WebSocket 遥测
│  ├─ mcp_server.py          # MCP stdio server（HTTP 客户端）
│  └─ cli.py                 # Typer CLI
└─ ui/                       # 静态 HTML/JS/Chart.js
```

**依赖方向**：`api → service → control/kinematics/motor → transport`。`motor` 不依赖 `service`；`transport` 不依赖电机协议。

## 4. CAN 与电机驱动

### 4.1 `SerialUsb2CanBus(can.BusABC)`：串口型 USB2CAN（主路径）

这是当前实物工具对应的后端。串口参数为 **921600, 8, None, 1**。

**TX：30 字节 `55 AA` 帧**

2026-11-20 真机确认的模板（`cmd=0x01`，CAN ID 2 字节放在 13–14）：

```python
def pack_usb2can_frame(can_id: int, data: bytes) -> bytes:
    assert len(data) <= 8
    frame = bytearray(30)
    frame[0:2] = b"\x55\xAA"          # header
    frame[2] = 0x1E                  # total length 30
    frame[3] = 0x01                  # data frame (0x03 did not work on this firmware)
    frame[4:8] = (1).to_bytes(4, "little")   # send count
    frame[8:10] = (10).to_bytes(2, "little") # interval ms
    frame[10] = 0x00                 # 0=standard, 1=extended
    frame[11] = 0x00                 # reserved
    frame[12] = 0x00                 # reserved
    frame[13] = can_id & 0xFF        # CAN ID low
    frame[14] = (can_id >> 8) & 0xFF # CAN ID high
    frame[15] = 0x00                 # 0=data, 1=remote
    frame[16] = 0x00                 # idAcc
    frame[17] = 0x00                 # dataAcc
    frame[18] = len(data)            # DLEN
    frame[19:21] = b"\x00\x00"       # reserved
    frame[21:29] = data.ljust(8, b"\x00")
    frame[29] = 0x00                 # CRC=0 confirmed
    return bytes(frame)
```

> 官方字段表写的 4 字节 CAN ID（11–14）在本固件上会把 ID 读成 0；实测必须用 2 字节放在 13–14。`cmd=0x03` 本固件无响应，`cmd=0x01` 可用。

**RX：16 字节 `AA ... 55` 帧**

```text
AA | CMD | FORMAT | CAN_ID[4] LE | DATA[8] | 55
```

- `CMD=0x11`：收到 CAN 帧；`0x12`：发送成功；`0x02/0x01`：失败；其余可只记录。
- `FORMAT`：bit0–5 = 数据长度；bit6 = 扩展帧；bit7 = 远程帧。
- `CAN_ID`：4 字节小端。
- `DATA`：固定 8 字节，按 `FORMAT` 的低 6 位取有效长度。

**2026-11-20 真机验证**：

- COM10（VID_2E88/PID_4603），921600 8N1，CAN 1 Mbps。
- 电机 1：CAN 0x01 / MST 0x11；电机 2：CAN 0x02 / MST 0x12。
- 速度模式 0.1 rad/s 短时测试正常；MIT 位置保持 `Kp=0.5, Kd=0.1` 正常。
- 两台电机已恢复速度模式并失能。

**后端骨架**

```python
class SerialUsb2CanBus(can.BusABC):
    def __init__(self, port="COM10", baud=921600, can_bitrate=1_000_000, **kwargs):
        super().__init__(**kwargs)
        self._ser = serial.Serial(
            port, baudrate=baud, bytesize=8, parity="N", stopbits=1,
            timeout=0.01, write_timeout=0.1,
        )
        self._tx_lock = threading.Lock()
        self._rxq = queue.Queue(maxsize=4096)
        self._stop = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        self._send_baud_config(can_bitrate)   # 55 05 <index> AA 55，index 0=1M
        self._send_heartbeat()                # 55 04 AA 55

    def send(self, msg, timeout=None):
        frame = pack_usb2can_frame(msg.arbitration_id, bytes(msg.data))
        with self._tx_lock:
            self._ser.write(frame)

    def _rx_loop(self):
        buf = bytearray()
        while not self._stop.is_set():
            chunk = self._ser.read(64)
            if not chunk:
                continue
            buf.extend(chunk)
            while True:
                i = buf.find(b"\xAA")
                if i < 0:
                    buf.clear()
                    break
                if i:
                    del buf[:i]
                if len(buf) < 16:
                    break
                pkt = bytes(buf[:16])
                if pkt[15] != 0x55:
                    del buf[:1]
                    continue
                del buf[:16]
                self._handle_rx(pkt)

    def _recv_internal(self, timeout):
        try:
            return self._rxq.get(timeout=timeout), False
        except queue.Empty:
            return None, False

    def shutdown(self):
        self._stop.set()
        self._rx_thread.join(timeout=1.0)
        self._ser.close()
```

要点：

- **发送和读取必须分线程**；串口读线程只解析、入队，不写串口。
- `serial.write()` 可能阻塞；超时设置短，异常时进入 FAULT，不要静默丢帧。
- 队列满时丢弃最旧帧并计数；控制环始终使用最新反馈。
- 串口帧损坏、长度不符、尾字节不是 `0x55`：丢弃并计数；连续错误超过阈值进入 FAULT。
- 启动时记录 COM 口、驱动描述、USB VID/PID、固件/握手响应。
- 需要确认 CRC：先用 `0x00`；若模块拒绝，抓 DMTool/USB2CAN 工具的串口流量，确定算法后补上。

**备选：`DmCanBus(can.BusABC)`（USB2CANFD）**

如果实物是 USB2CANFD_S/D/LinkX4C，使用 `dmcan-sdk` 2.1.x + `libdmcan` 2.1.0：

```python
class DmCanBus(can.BusABC):
    def __init__(self, channel=0, bitrate=1_000_000, device_index=0, **kwargs):
        ...
        self._ctx = DmCanContext()
        self._dev = self._ctx.get_device(device_index)
        self._dev.open()
        info = dmcan_channel_can_info()
        info.canfd = False
        info.can_baudrate = bitrate
        info.can_sp = 0.8
        info.retrans = True
        self._dev.set_channel_baudrate(channel, info)
        self._dev.enable_channel(channel, True)
        self._dev.set_send_profile(dmcan_send_profile.LOW_LATENCY)
        self._sub = self._dev.subscribe(dmcan_event_flag.RX_FRAME, self._on_rx,
                                        channel_mask=1 << channel)
        self._rxq = queue.Queue()

    def _on_rx(self, dev, event):
        f = event.frame
        self._rxq.put(can.Message(
            arbitration_id=f.can_id,
            data=bytes(f.payload[:f.dlen]),
            is_extended_id=bool(f.ext),
            is_remote_frame=bool(f.rtr),
            timestamp=time.time(),
        ))

    def send(self, msg, timeout=None):
        self._dev.send_can(self._channel, msg.arbitration_id, bytes(msg.data),
                           canfd=False, ext=msg.is_extended_id,
                           rtr=msg.is_remote_frame, brs=False)
```

两种后端都实现 `can.BusABC`，`DmMotor` 和上层完全不知道底层是串口还是 libusb。

### 4.2 `DmMotor`

职责：

- 保存 `can_id`, `feedback_id`, `model`, `limits`, `direction`, `zero_offset`。
- 发送系统命令：enable/disable/set_zero/clear_error。
- 读写寄存器；设置 MIT 模式（0x0A=1）。
- 发送 MIT 帧；校验反馈 ID 与 `motor_id`。
- 维护 `MotorState`：`pos, vel, tau, t_mos, t_rotor, status, rx_time, error_count, tx_count`。
- 反馈超时、状态异常检测。

使能流程（严格顺序）：

```text
1. 确认总线已连接、反馈正常（至少收到一帧，状态=0 失能）
2. 读取/写入 CTRL_MODE=1（MIT）；切换前确认零速、低力矩
3. 发送 clear_error
4. 发送 enable
5. 等待状态字节变为 1（超时则失败）
6. 启动控制环，先输出 0 力矩，再斜坡到目标
```

失能流程：

```text
1. 控制环输出按配置斜坡降到 0 力矩（或保持阻尼）
2. 发送 disable
3. 确认状态变为 0
4. 停止控制环
```

## 5. 控制设计

### 5.1 被控对象与约束

- 每轴近似为刚体：`J * q̈ + b * q̇ + τ_g(q) = τ_motor - τ_friction`。
- G6220 直驱、低减速比、力矩密度有限；输出力矩必须限幅。
- 编码器分辨率高（16-bit），但速度由驱动器估算，可能含噪声；需要低通滤波。
- 俯仰轴有重力矩，偏航轴通常没有；重力前馈能显著改善稳态误差。
- 串口型 USB2CAN 的往返延迟预计约 2–5 ms（待实测），控制周期 2 ms；相位滞后是主要限制。
- 921600 8N1 下，两电机 500 Hz 已占约 50% 带宽；不要盲目提高控制频率。

### 5.2 MIT 模式下的三种用法

| 模式 | MIT 帧字段 | 内环 | 适用 |
|---|---|---|---|
| `torque`（默认） | `Kp=0, Kd=0, tau_ff=tau_cmd` | 驱动器 FOC 电流环 | PC 位置环/速度环输出力矩 |
| `impedance` | `p_des, v_des, Kp, Kd, tau_ff` | 驱动器 MIT PD + 电流环 | 安全保持、遥操作、参数未标定 |
| `velocity` | `Kp=0, Kd>0, v_des` | 驱动器速度/电流环 | 调速、回零辅助 |

> MIT 模式下无法直接给 `iq`；`tau_ff` 经 `T_MAX` 映射后由驱动器换算为 `iq_ref`。所谓“电流环内环”就是驱动器内部的 FOC 电流环。
>
> 真机 bring-up 先用 `impedance`：yaw 已验证 `Kp=0.5, Kd=0.1`；pitch 有重力/摩擦，需 `Kp=3.0, Kd=0.3`（0.03 rad 小步静差约 0.005–0.008 rad）。
>
> **纯力矩已验证（2026-11-20）**：`KT_Value=0.0` 不影响 `tau_ff`；yaw 上 0.05 N·m 指令产生约 0.0415 N·m 反馈并明显加速。`pos_current` 在原理上可行，下一步做 PC 位置环整定和力矩标定。

### 5.3 串级结构

#### A. `pos_current`（默认，最贴近需求）

```text
q_target ──▶ [位置 PID] ──▶ tau_cmd ──▶ [限幅] ──▶ MIT(tau_ff) ──▶ [驱动器电流环] ──▶ motor
                ▲                                                    │
                │                                                    │
                └──────────────── q_feedback ◀──────────────────────┘
```

- 位置 PID 可配置为 P / PD / PID；微分作用于测量值，避免目标阶跃冲击。
- 可选速度阻尼项：`tau_cmd -= Kd_v * q̇_filtered`。
- 积分项用于补偿重力/摩擦；必须抗饱和。
- 这是“位置环（PC）+ 电流环（驱动器）”的两环结构。

#### B. `pos_vel_current`（可选，三环）

```text
q_target → [位置 PID] → v_target → [速度 PID] → tau_cmd → MIT(tau_ff) → [驱动器电流环]
```

- 速度环能抑制扰动、限制速度，但增加调参维度和相位滞后。
- 速度反馈先低通（默认 50 Hz），再进入 PID。
- 位置环输出限幅为 `±max_joint_velocity`。

#### C. `impedance`（安全/调试）

```text
MIT(p_des=q_target, v_des=0, Kp=Kp_imp, Kd=Kd_imp, tau_ff=gravity)
```

- 不经过 PC PID；适合手动摆位、示教、参数辨识。
- `Kp/Kd` 必须在 MIT 全局范围内，且从小值开始。

### 5.4 PID 实现要求

- 位置式 PID，输出力矩（N·m）。
- 抗饱和：条件积分 + 输出限幅回算；积分限幅独立配置。
- 微分：作用于测量值（`-Kd * dq/dt`），一阶低通滤波（默认 20–50 Hz）。
- 死区/静摩擦补偿可选，默认关闭。
- 增益调度不做第一版；如需，按姿态/速度分段。
- 每个轴独立 PID；参数来自 YAML。
- 提供 `reset()`：使能、模式切换、急停恢复时清零积分和微分状态。

### 5.5 轨迹与前馈

- 目标姿态接口支持：
  - 阶跃（内部仍做速率限制）
  - 定时轨迹（`duration_s`，梯形/S 曲线）
  - 路点序列（未来）
- 轨迹在关节空间生成，保证速度/加速度限制；两轴可按同一时长同步。
- 前馈（可选）：
  - 重力：`tau_g(pitch) = m*g*l*cos(pitch)` 或查表/辨识
  - 惯量：`J * q̈_ref`
  - 摩擦：`b * q̇ + coulomb`
- 第一版先实现重力前馈占位，参数标定后启用。

### 5.6 时序与数据流

每个控制周期：

```text
1. 处理离散命令（enable/disable/estop/clear_fault）——最高优先级
2. 读取最新反馈快照；检查反馈年龄、状态码、温度
3. 安全检查：软限位、速度、力矩、目标变化率
4. 轨迹更新 → q_target
5. 运动学逆解（若目标是姿态）→ q_target_joint
6. 每轴 PID → tau_cmd
7. 限幅 + 前馈 → MIT 帧
8. 发送两台电机
9. 更新遥测快照、抖动统计
```

- 反馈年龄 > `feedback_timeout_ms`（默认 100 ms）→ 进入 FAULT，输出 0 力矩并失能。
- 任一电机状态码为故障（8–E）→ FAULT，失能全部电机。
- 温度超过阈值 → 先降额（力矩按比例），超过硬阈值 → FAULT。
- 控制周期 overrun 连续超过阈值 → 记录并降级（降低目标速度/报警），不静默继续。

### 5.7 安全策略

| 保护 | 默认 | 行为 |
|---|---|---|
| 关节软限位 | 配置 | 目标限幅；接近时减速；越界进入 FAULT |
| 速度限幅 | 配置 | 目标/轨迹限幅 |
| 力矩限幅 | ≤2.0 N·m（初始） | 输出限幅；可配置降额 |
| 反馈超时 | 100 ms | FAULT，0 力矩，失能 |
| 电机故障码 | 8–E | FAULT，失能 |
| MOS/线圈温度 | 80/90 °C 警告，硬阈值 FAULT | 降额/失能 |
| 命令 TTL | 500 ms | 超时保持最后目标（位置保持），不继续运动 |
| 急停 | 最高优先级 | 斜坡到 0 力矩 → 失能 → 锁存 ESTOP |
| 目标阶跃 | 每周期最大变化 | 限幅，避免冲击 |

急停语义需要结合机械结构确认：如果失能后云台会因重力自由下落，默认应采用“先保持/阻尼减速，再失能”，并提供“立即断电”外部急停。配置项 `estop_behavior: ramp_and_disable | hold | disable`。

## 6. 运动学

### 6.1 坐标系与关节

- 世界系 `W`：z 轴向上。
- 基座系 `B`：固定在云台底座。
- 偏航关节 `q_yaw` 绕 z 轴；俯仰关节 `q_pitch` 绕偏航后的 y 轴。
- 相机/负载系 `C`：`R_WC = Rz(q_yaw) * Ry(q_pitch) * R_offset`。
- G6220 减速比 1:1，关节角 = 电机角（乘以方向系数）。

### 6.2 逆解（2-DOF 解析）

输入：目标姿态 `(yaw, pitch)` 或四元数。
输出：`q_yaw, q_pitch`。

```text
q_yaw   = yaw_sign   * yaw   + yaw_offset
q_pitch = pitch_sign * pitch + pitch_offset
```

- 四元数输入先转 ZYX Euler；若 `|roll|` 超过容差，返回“不可达/需要三轴”。
- 俯仰接近 ±90° 时存在万向锁；软限位默认 ±85°（可配置）。
- 偏航角解缠：反馈角与目标角取最短路径；连续旋转时维护 `yaw_unwrapped`。
- 正解用于 UI 显示和校验：`R_WC = Rz(q_yaw) * Ry(q_pitch)`，再转 Euler/四元数。

### 6.3 标定

1. **软件零偏**（默认）：记录当前电机位置作为云台零位，写入本地 calibration 文件；不写电机 flash。
2. **方向系数**：手动正转，确认 yaw/pitch 增大方向；配置 `sign = ±1`。
3. **机械零位**（产线）：使用调试助手或 `set_zero` 命令写电机零点；仅必要时执行，注意 flash 寿命。
4. **限位**：手动缓慢移动到机械极限，记录并留安全余量。
5. **重力辨识**：在多个俯仰角下保持，记录保持力矩，拟合 `tau_g(pitch)`。

### 6.4 扩展性

- `kinematics/transforms.py` 提供 4×4 齐次变换和四元数工具。
- 未来第三轴：在 `R_WC` 链中插入 `Rz/Ry/Rx`，逆解仍可解析；若机械轴不正交，再引入数值 IK。
- 不在第一版引入 scipy/spatial；用 numpy + 手写小函数，减少依赖。

### 6.5 连续偏航与多圈位置（真机确认）

- yaw 轴带滑环，可连续旋转；pitch 轴仍按有限行程处理。
- **16-bit MIT 反馈位置在 ±12.5 rad 回绕**（量程 25 rad）。
- 寄存器 **0x50 `p_m` 是多圈绝对位置**，不回绕；0x51 `xout` 本机型读回 0，不可用。
- 控制层维护 `yaw_unwrapped`：
  - 每个反馈周期用 16-bit 位置，跨 ±12.5 时加/减 25 rad；
  - 以 1–5 Hz 读 `p_m`（0x50），与 unwrapped 位置做慢速重同步，防止长时间丢帧后累计误差；
  - 目标 yaw 也使用 unwrapped 角度，软限位对 yaw 设为“连续”（`yaw_continuous: true`），pitch 保留有限限位。
- 正反两个方向各 45 s 连续旋转均已通过；此前 +10 rad 附近的 0.696 N·m 尖峰确认是外部触碰，不是机械/控制问题。
- **连续 yaw 的控制模式**：不要用 `impedance` 把 `p_des` 直接跨 ±12.5 回绕（MIT 位置字段会跳变）。连续 yaw 应使用 `pos_current`（PC 位置环输出力矩）或速度模式；`impedance` 只适合有限行程/保持。

## 7. 服务与状态机

```text
DISCONNECTED ──connect──▶ CONNECTED ──enable──▶ ENABLED ──target──▶ RUNNING
      ▲                       ▲                   │                    │
      │                       │                   │ disable            │ disable
      │                       │                   ▼                    ▼
      └──────disconnect───────┴──────────────── CONNECTED ◀────────────┘
                              │
                      fault/estop
                              ▼
                         FAULT / ESTOP（锁存）
                              │ clear_fault（确认安全后）
                              ▼
                         CONNECTED
```

- `ENABLED`：电机已使能，输出 0 力矩或保持当前位置；可以改 PID/限位。
- `RUNNING`：跟踪目标；持续接收新目标。
- `FAULT`：反馈超时、电机错误、温度/限位异常；电机失能，必须 `clear_fault`。
- `ESTOP`：急停锁存；只有显式 `clear_fault` + `enable` 才能恢复。
- 所有状态转换由控制线程执行；API 只提交请求并返回 `accepted`，最终状态以快照为准。

## 8. API、MCP 与 AI 控制

### 8.1 HTTP API（草案）

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/v1/health` | 进程/总线/控制环健康 |
| GET | `/api/v1/status` | 完整状态快照 |
| POST | `/api/v1/connect` | 打开总线、探测电机 |
| POST | `/api/v1/disconnect` | 失能并关闭 |
| POST | `/api/v1/enable` | 使能（需已连接、无故障） |
| POST | `/api/v1/disable` | 斜坡停止并使能 |
| POST | `/api/v1/estop` | 急停，最高优先级 |
| POST | `/api/v1/clear-fault` | 清除锁存故障（需确认安全） |
| POST | `/api/v1/target/attitude` | `{yaw_deg, pitch_deg, duration_s, ttl_ms}` |
| POST | `/api/v1/target/joints` | `{yaw_rad, pitch_rad, duration_s, ttl_ms}` |
| POST | `/api/v1/trajectory` | 路点序列（后续） |
| GET/PUT | `/api/v1/config/pid` | 读取/设置 PID（有限幅） |
| POST | `/api/v1/record/start` / `stop` | 遥测记录 |
| WS | `/ws/telemetry` | 20–50 Hz 状态/曲线推送 |

统一响应：

```json
{
  "ok": true,
  "accepted": true,
  "request_id": "…",
  "state": "RUNNING",
  "message": null
}
```

### 8.2 MCP 工具（Claude Code 可直接调用）

| 工具 | 作用 |
|---|---|
| `gimbal_status` | 返回状态、姿态、关节、力矩、温度、错误、控制环统计 |
| `gimbal_connect` / `gimbal_disconnect` | 连接/断开 |
| `gimbal_enable` / `gimbal_disable` | 使能/失能 |
| `gimbal_estop` | 急停（始终可用） |
| `gimbal_set_attitude` | 目标 yaw/pitch（度），可带 duration/ttl |
| `gimbal_set_joints` | 目标关节角（弧度） |
| `gimbal_set_pid` | 设置某轴 PID（受硬限幅） |
| `gimbal_run_trajectory` | 路点序列 |
| `gimbal_get_config` | 读取当前配置/标定 |
| `gimbal_start_recording` / `stop` | 记录遥测 |
| `gimbal_self_test` | 低力矩自检（需确认） |

MCP Server 作为独立 stdio 进程，通过 HTTP 连接 `gimbald`；**不直接打开 USB 设备**。示例 Claude Code 配置：

```json
{
  "mcpServers": {
    "gimbal": {
      "command": "C:\\path\\to\\DM6220Gimbal\\.venv\\Scripts\\python.exe",
      "args": ["-m", "dm6220_gimbal.api.mcp_server", "--url", "http://127.0.0.1:8765"]
    }
  }
}
```

### 8.3 AI 安全约束

- 只能调用白名单工具；不能写任意寄存器、不能改安全硬上限、不能关闭看门狗。
- 每个目标命令带 `ttl_ms`；超时后保持最后位置（位置环继续工作），不继续执行运动。
- 目标变化率限幅（例如每周期 ≤ 配置的最大角速度），防止 AI 发阶跃。
- 所有命令写审计日志：来源、工具、参数、时间、接受/拒绝原因。
- 急停不需要确认，且不受 TTL/队列限制。
- 默认只监听 `127.0.0.1`；如需远程，必须加 token + TLS 反向代理，并明确风险。

## 9. UI

第一版本地 Web 仪表盘：

- 连接/断开、使能/失能、急停、清除故障。
- 姿态目标：yaw/pitch 滑条 + 数值输入 + “ Go” 按钮；显示当前姿态。
- 实时曲线：目标/实际角度、关节误差、力矩、温度、控制环抖动。
- PID 调参面板：读取/写入，带范围提示和“恢复默认”。
- 日志/事件列表：故障、超时、限幅、命令审计。
- 响应式布局，暗色主题；Chart.js 本地打包，不依赖 CDN。

技术：FastAPI 静态文件 + 原生 JS + Chart.js。后续可加 Three.js 3D 姿态显示，或用 pywebview 包成桌面窗口。

## 10. 配置

见 `config/gimbal.example.yaml`。原则：

- 所有硬件相关参数可配置；安全硬上限在代码中兜底。
- 启动时 Pydantic 校验：ID 唯一、限位 `min < max`、`rate_hz` 范围、力矩 ≤ 硬上限、温度阈值合理。
- 本地 `config/gimbal.yaml` / `calibration.yaml` 不入库。
- 配置变更在 `CONNECTED`/`ENABLED` 状态下生效；运行中改 PID 允许，改 ID/限幅需重新连接。

## 11. 测试与验收

### 11.1 测试层次

| 层次 | 内容 |
|---|---|
| 单元 | MIT 编解码 golden vectors、反馈解析、PID、滤波、运动学、限位 |
| 仿真 | `python-can` virtual bus + 两轴刚体模型；验证控制环、状态机、API |
| 集成 | 启动 daemon + HTTP 客户端 + MCP 客户端；命令/状态一致性 |
| 真机 | 扫描 ID、读寄存器、低力矩使能、阶跃响应、温升、急停、反馈超时 |

### 11.2 验收指标（初稿）

- 控制环 500 Hz：已实测 **500.0 Hz、p99 2.778 ms（抖动约 0.78 ms）、0 overrun、0 坏帧**（两轴 MIT 保持，1500 周期）。
- 两台电机反馈丢失率 0；MIT 帧发送错误可计数并可恢复。
- 位置阶跃（小角度）无持续振荡；超调/调节时间在整定后记录。
- 软限位、力矩限幅、反馈超时、急停均通过故障注入测试。
- AI 无法绕过限幅；TTL 生效；审计日志完整。

## 12. 性能与风险

| 风险 | 影响 | 缓解 |
|---|---|---|
| Windows 定时抖动 | 控制周期不稳、噪声 | 已实测 500 Hz：p99 2.778 ms、0 overrun；仍保留 timeBeginPeriod/自旋和抖动统计，必要时 C++/Rust 核心 |
| 串口带宽打满 | 延迟增大、丢帧 | 500 Hz 上限；30B TX + 16B RX 开销固定；确认模块是否支持更高串口速率 |
| 55 AA 帧偏移/CRC（已解决） | 无法通信或偶发丢帧 | 已确认 `cmd=0x01, CAN ID@13–14, DLEN@18, CRC=0`；换固件时仍需抓包复核 |
| 串口驱动/COM 口被占用 | 无法打开设备 | 使用 USB-CDC 驱动；启动时枚举 COM；DMTool 关闭后再运行 daemon |
| USB2CANFD 误判 | 选错后端 | 设备管理器/VID-PID 检测；`can.backend` 显式配置；启动日志记录 |
| libdmcan/DLL 版本不匹配（仅 FD） | 无法打开设备 | 固定 `dmcan-sdk 2.1.1` + `libdmcan 2.1.0`，启动校验版本 |
| 模块型号不匹配 | SDK 不支持 | 先 probe；抽象 transport，必要时新增后端 |
| G6220 参数未知 | MIT 映射错误、力矩异常 | 低力矩首次使能；读寄存器/调试助手确认；配置化 |
| 偏航多圈/回绕 | 角度跳变、控制冲击 | 解缠 + 软限位；确认编码器多圈能力；必要时速度模式回零 |
| 重力/惯量未知 | 稳态误差、超调 | 重力前馈辨识；先低增益；记录阶跃响应 |
| CAN 终端/接地问题 | 随机错误帧 | 接线检查清单；示波器/万用表验证 60–70 Ω |
| AI 误操作 | 危险运动 | 安全层、TTL、限幅、审计、急停、localhost |

## 13. 实施路线

1. **硬件 bring-up**：插上 USB2CAN，确认新 COM 口和驱动；用 DMTool 读参数/设 ID/确认 1 Mbps 与 MIT 模式；用 `capture_usb2can.py` 核对 `55 AA` 帧偏移与 CRC。
2. **协议与电机驱动**：MIT 编解码 + 单元测试；真机低力矩使能、单电机位置保持；验证反馈解析。
3. **控制环与仿真**：PID、限幅、轨迹、状态机；虚拟总线下闭环。
4. **双轴运动学**：逆解/正解、标定、软限位、角度解缠。
5. **服务/API/CLI**：daemon、HTTP/WS、审计、配置。
6. **Web UI**：状态、控制、曲线、调参。
7. **MCP 与 AI 使用**：工具封装、TTL、安全测试。
8. **真机整定与验收**：阶跃、扰动、温升、急停、长时间运行。

每一步都保留“无硬件可测试”的入口；硬件到位后只替换 transport 和参数。
