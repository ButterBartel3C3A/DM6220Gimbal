"""MCP server that exposes the gimbal HTTP API as tools.

Run this as a separate stdio process; it does not open the USB device itself.
Configure Claude Code / another MCP client to start:

    dm6220-gimbal-mcp --url http://127.0.0.1:8765
"""

from __future__ import annotations

import argparse
import json
import urllib.error
import urllib.request
from typing import Any

try:  # mcp 1.x
    from mcp.server.fastmcp import FastMCP as _MCPServer
except ModuleNotFoundError:  # mcp 2.x renamed FastMCP to MCPServer
    from mcp.server.mcpserver import MCPServer as _MCPServer

mcp = _MCPServer("dm6220-gimbal")

BASE_URL = "http://127.0.0.1:8765"
TOKEN: str | None = None


def _request(method: str, path: str, body: dict[str, Any] | None = None) -> str:
    data = None if body is None else json.dumps(body).encode("utf-8")
    req = urllib.request.Request(
        BASE_URL.rstrip("/") + path,
        data=data,
        method=method,
        headers={"Content-Type": "application/json"},
    )
    if TOKEN:
        req.add_header("Authorization", f"Bearer {TOKEN}")
    try:
        with urllib.request.urlopen(req, timeout=10.0) as resp:
            return resp.read().decode("utf-8")
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        return json.dumps({"ok": False, "http_status": exc.code, "detail": detail})
    except Exception as exc:
        return json.dumps({"ok": False, "error": f"daemon unreachable at {BASE_URL}: {exc}"})


@mcp.tool()
def gimbal_status() -> str:
    """Read the full gimbal state: state machine, attitude, joints, motors, faults, loop stats."""
    return _request("GET", "/api/v1/status")


@mcp.tool()
def gimbal_connect() -> str:
    """Open the USB2CAN and probe both motors. Does not enable the motors."""
    return _request("POST", "/api/v1/connect")


@mcp.tool()
def gimbal_disconnect() -> str:
    """Disable the motors and close the USB2CAN."""
    return _request("POST", "/api/v1/disconnect")


@mcp.tool()
def gimbal_enable() -> str:
    """Enable both motors in MIT mode. Requires CONNECTED and no active fault."""
    return _request("POST", "/api/v1/enable")


@mcp.tool()
def gimbal_disable() -> str:
    """Stop the control loop, send zero torque, and disable both motors."""
    return _request("POST", "/api/v1/disable")


@mcp.tool()
def gimbal_estop() -> str:
    """Emergency stop: zero torque and disable both motors immediately. Always available."""
    return _request("POST", "/api/v1/estop")


@mcp.tool()
def gimbal_clear_fault() -> str:
    """Clear a latched fault/estop after confirming the mechanism is safe."""
    return _request("POST", "/api/v1/clear-fault")


@mcp.tool()
def gimbal_set_attitude(
    yaw_deg: float,
    pitch_deg: float,
    roll_deg: float = 0.0,
    duration_s: float = 0.5,
    ttl_ms: int = 500,
) -> str:
    """Set target attitude in degrees. Yaw is unwrapped/continuous; pitch is limited.

    The command is rejected unless the gimbal is ENABLED/RUNNING. The controller
    rate-limits motion to the configured joint velocity (1 rad/s in bring-up).
    """
    return _request(
        "POST",
        "/api/v1/target/attitude",
        {
            "yaw_deg": yaw_deg,
            "pitch_deg": pitch_deg,
            "roll_deg": roll_deg,
            "duration_s": duration_s,
            "ttl_ms": ttl_ms,
            "source": "mcp",
        },
    )


@mcp.tool()
def gimbal_set_joints(
    yaw_rad: float,
    pitch_rad: float,
    duration_s: float = 0.5,
    ttl_ms: int = 500,
) -> str:
    """Set target joint angles in radians. Requires ENABLED/RUNNING."""
    return _request(
        "POST",
        "/api/v1/target/joints",
        {
            "yaw_rad": yaw_rad,
            "pitch_rad": pitch_rad,
            "duration_s": duration_s,
            "ttl_ms": ttl_ms,
            "source": "mcp",
        },
    )


@mcp.tool()
def gimbal_set_pid(axis: str, kp: float, ki: float, kd: float) -> str:
    """Update the PC position PID for yaw or pitch. Values are clamped to safe bounds."""
    return _request(
        "PUT",
        "/api/v1/config/pid",
        {"axis": axis, "kp": kp, "ki": ki, "kd": kd},
    )


@mcp.tool()
def gimbal_get_config() -> str:
    """Return the active configuration (limits, modes, PID gains, kinematics)."""
    return _request("GET", "/api/v1/config")


def main() -> None:
    global BASE_URL, TOKEN
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:8765")
    parser.add_argument("--token", default=None)
    args = parser.parse_args()
    BASE_URL = args.url
    TOKEN = args.token
    mcp.run()


if __name__ == "__main__":
    main()
