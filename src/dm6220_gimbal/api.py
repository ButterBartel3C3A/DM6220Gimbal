"""FastAPI HTTP/WebSocket API and local web UI mount."""

from __future__ import annotations

import asyncio
from pathlib import Path
from typing import Any

from fastapi import APIRouter, Depends, FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import RedirectResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from .config import AppConfig
from .service import GimbalService, GimbalServiceError


class AttitudeRequest(BaseModel):
    yaw_deg: float
    pitch_deg: float
    roll_deg: float = 0.0
    duration_s: float = Field(default=0.5, ge=0.0, le=60.0)
    ttl_ms: int = Field(default=2000, ge=50, le=60_000)
    source: str = "api"


class JointsRequest(BaseModel):
    yaw_rad: float
    pitch_rad: float
    duration_s: float = Field(default=0.5, ge=0.0, le=60.0)
    ttl_ms: int = Field(default=2000, ge=50, le=60_000)
    source: str = "api"


class PidRequest(BaseModel):
    axis: str
    kp: float = Field(ge=0, le=100)
    ki: float = Field(ge=0, le=10)
    kd: float = Field(ge=0, le=10)


def create_app(service: GimbalService, cfg: AppConfig | None = None) -> FastAPI:
    cfg = cfg or service.cfg
    app = FastAPI(title="DM6220 Gimbal", version="0.1.0")
    app.state.service = service
    app.state.cfg = cfg

    if cfg.api.cors_origins:
        app.add_middleware(
            CORSMiddleware,
            allow_origins=cfg.api.cors_origins,
            allow_credentials=False,
            allow_methods=["*"],
            allow_headers=["*"],
        )

    def require_token(request: Request) -> None:
        token = cfg.api.token
        if not token:
            return
        supplied = request.headers.get("x-api-token")
        auth = request.headers.get("authorization", "")
        if auth.lower().startswith("bearer "):
            supplied = auth[7:].strip()
        if supplied != token:
            raise HTTPException(status_code=401, detail="invalid API token")

    router = APIRouter(prefix="/api/v1", dependencies=[Depends(require_token)])

    def action(fn, *args, **kwargs) -> dict[str, Any]:
        try:
            snap = fn(*args, **kwargs)
            return {"ok": True, "state": snap.get("state"), "snapshot": snap}
        except GimbalServiceError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        except Exception as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc

    @router.get("/health")
    def health() -> dict[str, Any]:
        snap = service.snapshot()
        return {
            "ok": True,
            "state": snap["state"],
            "fault": snap["fault"],
            "connected": snap["connected"],
        }

    @router.get("/status")
    def status() -> dict[str, Any]:
        return service.snapshot()

    @router.post("/connect")
    def connect() -> dict[str, Any]:
        return action(service.connect)

    @router.post("/disconnect")
    def disconnect() -> dict[str, Any]:
        return action(service.disconnect)

    @router.post("/enable")
    def enable() -> dict[str, Any]:
        return action(service.enable)

    @router.post("/disable")
    def disable() -> dict[str, Any]:
        return action(service.disable)

    @router.post("/estop")
    def estop() -> dict[str, Any]:
        return action(service.estop)

    @router.post("/clear-fault")
    def clear_fault() -> dict[str, Any]:
        return action(service.clear_fault)

    @router.post("/target/attitude")
    def target_attitude(req: AttitudeRequest) -> dict[str, Any]:
        return action(
            service.set_attitude,
            req.yaw_deg,
            req.pitch_deg,
            req.roll_deg,
            req.source,
            req.duration_s,
            req.ttl_ms,
        )

    @router.post("/target/joints")
    def target_joints(req: JointsRequest) -> dict[str, Any]:
        return action(
            service.set_joints,
            req.yaw_rad,
            req.pitch_rad,
            req.source,
            req.duration_s,
            req.ttl_ms,
        )

    @router.put("/config/pid")
    def set_pid(req: PidRequest) -> dict[str, Any]:
        return action(service.set_pid, req.axis, req.kp, req.ki, req.kd)

    @router.get("/config")
    def get_config() -> dict[str, Any]:
        return cfg.model_dump(exclude={"source_path"})

    app.include_router(router)

    @app.websocket("/ws/telemetry")
    async def telemetry(ws: WebSocket) -> None:
        await ws.accept()
        try:
            while True:
                await ws.send_json(service.snapshot())
                await asyncio.sleep(0.05)
        except (WebSocketDisconnect, RuntimeError):
            return

    ui_dir = Path(__file__).resolve().parent / "ui"
    if ui_dir.is_dir():
        app.mount("/ui", StaticFiles(directory=ui_dir, html=True), name="ui")

        @app.get("/")
        def root() -> RedirectResponse:
            return RedirectResponse(url="/ui/")

    return app
