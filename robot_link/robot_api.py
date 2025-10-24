"""High level helpers for controlling the Husky simulator over HTTP.

The original implementation in this project imported :mod:`husky_drive` and
manipulated its shared state directly. That worked when everything ran in the
same process, but the simulator now exposes a REST API.  This module provides
small convenience wrappers around those HTTP endpoints so the rest of the code
base can continue to issue simple Python function calls.
"""

from __future__ import annotations

import os
from typing import Any, Dict, Optional

import requests

__all__ = [
    "start",
    "shutdown",
    "drive",
    "forward",
    "backward",
    "turn_left",
    "turn_right",
    "stop",
    "reset",
    "get_pose",
]

DEFAULT_BASE_URL = "http://127.0.0.1:5000"
DEFAULT_TIMEOUT = 5.0

_base_url = os.environ.get("HUSKY_API_URL", DEFAULT_BASE_URL).rstrip("/") or DEFAULT_BASE_URL
_timeout = float(os.environ.get("HUSKY_API_TIMEOUT", DEFAULT_TIMEOUT))
_session: Optional[requests.Session] = None


def _get_session() -> requests.Session:
    global _session
    if _session is None:
        _session = requests.Session()
    return _session


def _make_url(path: str) -> str:
    if not path.startswith("/"):
        path = "/" + path
    return f"{_base_url}{path}"


def _request(method: str, path: str, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    response = _get_session().request(
        method,
        _make_url(path),
        json=payload,
        timeout=_timeout,
    )
    response.raise_for_status()
    if not response.content:
        return {}
    data = response.json()
    if isinstance(data, dict):
        return data
    raise ValueError(f"Unexpected response type: {type(data)!r}")


def _normalise_duration(duration: Optional[float]) -> Optional[float]:
    if duration is None:
        return None
    value = float(duration)
    if value <= 0.0:
        return 0.0
    return value


def start() -> Dict[str, Any]:
    """Ping the health endpoint to ensure the simulator is reachable."""

    return _request("get", "/health")


def shutdown(timeout: Optional[float] = None) -> Dict[str, Any]:  # pragma: no cover - thin wrapper
    """Stop the robot and release the HTTP session.

    ``timeout`` is accepted for backward compatibility but currently unused.
    """

    result: Dict[str, Any]
    try:
        result = stop()
    except requests.RequestException:
        result = {"stopped": False}

    global _session
    if _session is not None:
        _session.close()
        _session = None

    return result


def drive(vx: float, wz: float, duration: Optional[float] = None) -> Dict[str, Any]:
    """Drive the robot using differential velocity commands."""

    payload: Dict[str, Any] = {"vx": float(vx), "wz": float(wz)}
    duration_value = _normalise_duration(duration)
    if duration_value is not None:
        payload["duration"] = duration_value
    return _request("post", "/drive", payload)


def forward(speed: float = 0.6, duration: float = 1.0) -> Dict[str, Any]:
    """Drive forward for ``duration`` seconds at ``speed`` m/s."""

    return drive(speed, 0.0, duration)


def backward(speed: float = 0.6, duration: float = 1.0) -> Dict[str, Any]:
    """Drive backwards for ``duration`` seconds at ``speed`` m/s."""

    return drive(-speed, 0.0, duration)


def turn_left(rate: float = 1.0, duration: float = 0.8) -> Dict[str, Any]:
    """Rotate counter-clockwise for ``duration`` seconds."""

    return drive(0.0, rate, duration)


def turn_right(rate: float = 1.0, duration: float = 0.8) -> Dict[str, Any]:
    """Rotate clockwise for ``duration`` seconds."""

    return drive(0.0, -rate, duration)


def stop() -> Dict[str, Any]:
    """Immediately stop the robot."""

    return _request("post", "/stop")


def reset() -> Dict[str, Any]:
    """Reset the Husky to the origin."""

    return _request("post", "/reset")


def get_pose() -> Dict[str, Any]:
    """Return the most recently observed robot pose."""

    return _request("get", "/pose")

