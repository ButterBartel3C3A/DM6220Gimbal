"""Command line interface."""

from __future__ import annotations

import json
import logging
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

import typer
from rich.console import Console
from rich.table import Table

from .config import AppConfig, load_config
from .service import GimbalService, GimbalServiceError
from .transport import list_serial_ports

app = typer.Typer(add_completion=False, help="DM6220 dual-axis gimbal controller")
console = Console()


def _setup_logging(cfg: AppConfig) -> None:
    handlers: list[logging.Handler] = []
    if cfg.logging.console:
        handlers.append(logging.StreamHandler())
    if cfg.logging.file:
        path = Path(cfg.logging.file)
        path.parent.mkdir(parents=True, exist_ok=True)
        handlers.append(logging.FileHandler(path, encoding="utf-8"))
    logging.basicConfig(
        level=getattr(logging, cfg.logging.level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        handlers=handlers or None,
        force=True,
    )


def _service(config: Path | None) -> tuple[AppConfig, GimbalService]:
    cfg = load_config(config)
    _setup_logging(cfg)
    return cfg, GimbalService(cfg)


def _request(
    method: str,
    path: str,
    url: str = "http://127.0.0.1:8765",
    body: dict[str, Any] | None = None,
    token: str | None = None,
) -> dict[str, Any]:
    data = None if body is None else json.dumps(body).encode("utf-8")
    req = urllib.request.Request(
        url.rstrip("/") + path,
        data=data,
        method=method,
        headers={"Content-Type": "application/json"},
    )
    if token:
        req.add_header("Authorization", f"Bearer {token}")
    try:
        with urllib.request.urlopen(req, timeout=5.0) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        raise typer.BadParameter(f"HTTP {exc.code}: {detail}") from exc
    except urllib.error.URLError as exc:
        raise typer.BadParameter(f"daemon not reachable at {url}: {exc}") from exc


@app.command()
def serve(
    config: Path | None = typer.Option(None, "--config", "-c", help="YAML config path"),
    host: str | None = typer.Option(None, help="HTTP host override"),
    port: int | None = typer.Option(None, help="HTTP port override"),
    connect: bool = typer.Option(False, help="connect to the USB2CAN on startup"),
) -> None:
    """Run the gimbal daemon and local web UI."""
    cfg, service = _service(config)
    if host:
        cfg.api.host = host
    if port:
        cfg.api.port = port
    if connect:
        try:
            service.connect()
        except GimbalServiceError as exc:
            raise typer.BadParameter(str(exc)) from exc
    from .api import create_app

    application = create_app(service, cfg)
    import uvicorn

    console.print(f"[bold]DM6220 Gimbal[/bold]  http://{cfg.api.host}:{cfg.api.port}/ui/")
    try:
        uvicorn.run(application, host=cfg.api.host, port=cfg.api.port, log_level="info")
    finally:
        try:
            service.disconnect()
        except Exception:
            pass


@app.command()
def probe(
    config: Path | None = typer.Option(None, "--config", "-c", help="YAML config path"),
    json_out: bool = typer.Option(False, "--json", help="print raw JSON"),
) -> None:
    """Connect, read one status snapshot, and disconnect (read-only)."""
    _, service = _service(config)
    try:
        snap = service.connect()
        if json_out:
            console.print_json(json.dumps(snap))
        else:
            console.print(f"state: {snap['state']}")
            for name, motor in snap["motors"].items():
                console.print(
                    f"{name}: status={motor['status']} ({motor['status_name']}) "
                    f"pos={motor['pos_rad']:+.4f} raw={motor['raw_pos_rad']:+.4f} "
                    f"vel={motor['vel_rad_s']:+.4f} tau={motor['tau_nm']:+.4f} "
                    f"T={motor['t_mos_c']:.0f}/{motor['t_rotor_c']:.0f} "
                    f"rx_age={motor['rx_age_ms']} ms"
                )
    finally:
        try:
            service.disconnect()
        except Exception:
            pass


@app.command()
def scan() -> None:
    """List serial ports and mark the USB2CAN adapter."""
    table = Table("device", "description", "VID:PID", "USB2CAN")
    for p in list_serial_ports():
        vidpid = ""
        if p["vid"] is not None and p["pid"] is not None:
            vidpid = f"{p['vid']:04X}:{p['pid']:04X}"
        table.add_row(str(p["device"]), str(p["description"]), vidpid, "yes" if p["is_usb2can"] else "")
    console.print(table)


@app.command()
def status(
    url: str = typer.Option("http://127.0.0.1:8765", help="daemon URL"),
    token: str | None = typer.Option(None, help="API token"),
) -> None:
    """Read status from a running daemon."""
    snap = _request("GET", "/api/v1/status", url=url, token=token)
    console.print_json(json.dumps(snap))


@app.command()
def connect(
    url: str = typer.Option("http://127.0.0.1:8765"),
    token: str | None = typer.Option(None),
) -> None:
    console.print_json(json.dumps(_request("POST", "/api/v1/connect", url, token=token)))


@app.command()
def disconnect(
    url: str = typer.Option("http://127.0.0.1:8765"),
    token: str | None = typer.Option(None),
) -> None:
    console.print_json(json.dumps(_request("POST", "/api/v1/disconnect", url, token=token)))


@app.command()
def enable(
    url: str = typer.Option("http://127.0.0.1:8765"),
    token: str | None = typer.Option(None),
) -> None:
    console.print_json(json.dumps(_request("POST", "/api/v1/enable", url, token=token)))


@app.command()
def disable(
    url: str = typer.Option("http://127.0.0.1:8765"),
    token: str | None = typer.Option(None),
) -> None:
    console.print_json(json.dumps(_request("POST", "/api/v1/disable", url, token=token)))


@app.command()
def estop(
    url: str = typer.Option("http://127.0.0.1:8765"),
    token: str | None = typer.Option(None),
) -> None:
    console.print_json(json.dumps(_request("POST", "/api/v1/estop", url, token=token)))


@app.command()
def attitude(
    yaw_deg: float,
    pitch_deg: float,
    url: str = typer.Option("http://127.0.0.1:8765"),
    token: str | None = typer.Option(None),
) -> None:
    body = {"yaw_deg": yaw_deg, "pitch_deg": pitch_deg, "source": "cli"}
    console.print_json(json.dumps(_request("POST", "/api/v1/target/attitude", url, body, token)))


@app.command()
def joints(
    yaw_rad: float,
    pitch_rad: float,
    url: str = typer.Option("http://127.0.0.1:8765"),
    token: str | None = typer.Option(None),
) -> None:
    body = {"yaw_rad": yaw_rad, "pitch_rad": pitch_rad, "source": "cli"}
    console.print_json(json.dumps(_request("POST", "/api/v1/target/joints", url, body, token)))


if __name__ == "__main__":
    app()
