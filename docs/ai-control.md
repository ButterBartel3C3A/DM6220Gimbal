# AI 控制接口设计

目标：让 Claude Code、脚本或其他 AI 通过稳定接口控制云台，同时保证安全边界不被绕过。

## 1. 原则

1. **AI 不是硬件所有者**：`gimbald` 独占 USB-CAN 的 COM 口；DMTool/串口助手必须先关闭，AI 通过 HTTP/MCP 操作 daemon。
2. **安全层在 daemon 内**：AI 不能关闭限幅、看门狗、反馈超时、温度保护。
3. **命令可审计**：来源、参数、时间、结果全部落盘。
4. **默认本地**：只监听 `127.0.0.1`；远程必须加 token/TLS，并明确风险。
5. **失败安全**：反馈丢失、电机故障、控制环异常时，daemon 自行进入 FAULT 并失能。

## 2. 典型 AI 工作流

```text
gimbal_status()
  → 确认 state=CONNECTED、无故障、反馈正常
gimbal_connect()（若未连接）
gimbal_enable()（低力矩，斜坡使能）
gimbal_set_attitude(yaw_deg=15, pitch_deg=-5, duration_s=0.8)
gimbal_status() / 订阅遥测，确认到位
...
gimbal_disable() 或 gimbal_estop()
```

AI 不应：

- 在未读取状态的情况下直接使能或发目标。
- 连续高频刷目标（daemon 会限流，但会记录）。
- 试图修改安全硬上限或写原始寄存器。
- 在有人/物接近云台时做大幅运动；首次调试应低力矩、小角度。

## 3. HTTP API

### 3.1 状态查询

```http
GET /api/v1/status
```

```json
{
  "state": "RUNNING",
  "connected": true,
  "fault": null,
  "attitude": {"yaw_deg": 12.4, "pitch_deg": -4.8, "roll_deg": 0.0},
  "joints": {"yaw_rad": 0.216, "yaw_raw_rad": 0.216,
             "yaw_abs_rad": 0.216, "pitch_rad": -0.084},
  "target": {"yaw_deg": 15.0, "pitch_deg": -5.0, "age_ms": 120},
  "motors": {
    "yaw":   {"pos_rad": 0.216, "vel_rad_s": 0.01, "tau_nm": 0.12,
              "t_mos_c": 41, "t_rotor_c": 39, "status": 1, "rx_age_ms": 2},
    "pitch": {"pos_rad": -0.084, "vel_rad_s": -0.02, "tau_nm": 0.31,
              "t_mos_c": 42, "t_rotor_c": 40, "status": 1, "rx_age_ms": 2}
  },
  "loop": {"rate_hz": 500, "jitter_ms_p99": 0.6, "overruns": 0},
  "limits": {"torque_nm": 0.8, "velocity_rad_s": 1.0}
}
```

> 连续 yaw 时 `yaw_rad`/`yaw_deg` 是 **unwrapped** 角度，可以超过 ±π/±180°；`yaw_raw_rad` 是 16-bit 反馈解码值（±12.5 回绕），`yaw_abs_rad` 是 `p_m` 多圈绝对位置。AI 设置连续 yaw 目标时使用 unwrapped 角度，不要自行归一化到 ±180°。

### 3.2 目标命令

```http
POST /api/v1/target/attitude
Content-Type: application/json

{
  "yaw_deg": 15.0,
  "pitch_deg": -5.0,
  "duration_s": 0.8,
  "ttl_ms": 500,
  "source": "mcp"
}
```

响应：

```json
{"ok": true, "accepted": true, "request_id": "01J...", "state": "RUNNING"}
```

- `duration_s=0` 表示尽快（仍受最大速度/加速度限制）。
- `ttl_ms` 后无新目标则保持最后位置；不会继续执行未完成的无限运动。
- 目标超出软限位、变化率过大、状态不允许时返回 `accepted=false`，并给出原因。

### 3.3 离散命令

| 路径 | 说明 |
|---|---|
| `POST /api/v1/connect` | 打开总线、探测电机；不使能 |
| `POST /api/v1/enable` | 使能；需 `CONNECTED` 且无故障 |
| `POST /api/v1/disable` | 斜坡停止后失能 |
| `POST /api/v1/estop` | 急停；最高优先级，不需要确认 |
| `POST /api/v1/clear-fault` | 清除锁存故障；需 `confirm=true` |

### 3.4 PID 调参

```http
PUT /api/v1/config/pid
Content-Type: application/json

{
  "axis": "pitch",
  "kp": 8.0,
  "ki": 0.3,
  "kd": 0.4,
  "output_limit_nm": 1.5
}
```

- 只允许在 `ENABLED`/`RUNNING` 状态修改；写入后控制线程在下一周期生效。
- 值受硬上限约束；超出返回 `accepted=false`。
- 修改会写审计日志；不自动持久化到 YAML（需显式保存）。

### 3.5 遥测

- `GET /api/v1/status`：轮询，适合低频。
- `WS /ws/telemetry`：20–50 Hz 推送，适合实时曲线和 AI 监控。
- `POST /api/v1/record/start` / `stop`：记录 JSONL/CSV。

## 4. MCP 工具

MCP Server 用 stdio 传输，作为 daemon 的 HTTP 客户端。工具定义建议：

```python
@mcp.tool()
def gimbal_status() -> dict:
    """读取云台完整状态。调用前先确认 state 和 fault。"""

@mcp.tool()
def gimbal_connect() -> dict:
    """打开 USB-CAN 并探测两台电机；不会使能。"""

@mcp.tool()
def gimbal_enable() -> dict:
    """使能两台电机，先输出 0 力矩再斜坡到保持力矩。需无故障。"""

@mcp.tool()
def gimbal_disable() -> dict:
    """斜坡降到 0 力矩后失能。"""

@mcp.tool()
def gimbal_estop() -> dict:
    """急停并锁存。任何时候都可调用。"""

@mcp.tool()
def gimbal_set_attitude(yaw_deg: float, pitch_deg: float,
                        duration_s: float = 0.5,
                        ttl_ms: int = 500) -> dict:
    """设置目标姿态（度）。受软限位/速度/力矩/变化率限制。"""

@mcp.tool()
def gimbal_set_joints(yaw_rad: float, pitch_rad: float,
                      duration_s: float = 0.5,
                      ttl_ms: int = 500) -> dict:
    """设置目标关节角（弧度）。"""

@mcp.tool()
def gimbal_set_pid(axis: str, kp: float, ki: float, kd: float) -> dict:
    """设置某轴 PID；值受硬上限限制。"""
```

工具描述要明确单位、范围、前置条件和安全后果，方便模型正确调用。

Claude Code 配置：

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

## 5. 安全语义

| 机制 | 行为 |
|---|---|
| 命令 TTL | 目标超过 `ttl_ms` 未刷新 → 保持最后位置，不继续运动 |
| 速率限制 | API 层限流；目标变化率在控制层再限幅 |
| 目标步长 | 单次命令最大跳变（默认 10°），超出拒绝或分段 |
| 软限位 | 目标先限幅；接近限位减速；越界拒绝 |
| 力矩/速度 | 配置上限 + 代码硬上限双重约束 |
| 反馈超时 | 100 ms 无反馈 → FAULT，0 力矩，失能 |
| 电机故障 | 状态码 8–E → FAULT，失能 |
| 温度 | 警告降额，硬阈值 FAULT |
| 急停 | 斜坡到 0 力矩 → 失能 → 锁存；需人工确认后清除 |
| 审计 | 所有命令含来源、参数、结果、时间 |

## 6. 审计日志格式

```json
{
  "ts": "2026-11-19T15:42:03.123+08:00",
  "source": "mcp",
  "client": "claude-code",
  "request_id": "01J...",
  "action": "set_attitude",
  "params": {"yaw_deg": 15.0, "pitch_deg": -5.0, "duration_s": 0.8},
  "accepted": true,
  "state_before": "ENABLED",
  "state_after": "RUNNING",
  "reject_reason": null
}
```

## 7. 无硬件时的 AI 测试

- `can.backend: virtual` + 仿真电机模型：AI 可以完整走 connect/enable/set_attitude/status/disable。
- 仿真模型应支持：位置跟随、力矩限幅、反馈延迟、故障注入（超时/过温/错误码）。
- 用 `pytest -m sim` 验证 AI 工具不会绕过安全层。
