# link.py — load templates from file + MQTT bridge
"""MQTT bridge that wires OmniLink commands to the robot API helpers."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List

from omnilink import (
    OmniLinkEngine,
    OmniLinkMQTTBridge,
    TypeRegistry,
    load_patterns_from_file,
)
from robot_api import backward, forward, stop, turn_left, turn_right

HERE = Path(__file__).resolve().parent
PATTERNS_FILE = HERE / "robot_commands.txt"

types = TypeRegistry()
TEMPLATES = load_patterns_from_file(PATTERNS_FILE, types)
engine = OmniLinkEngine(TEMPLATES, types=types)

_NUMBER_RE = re.compile(r"-?\d+(?:\.\d+)?")


def _extract_numbers(command: str) -> List[float]:
    """Return all numeric values embedded in ``command`` as floats."""

    return [float(match) for match in _NUMBER_RE.findall(command)]


def _call_motion(
    evt: Dict[str, Any],
    handler: Callable[[float, float], Dict[str, Any]],
    *,
    expected: int = 2,
) -> Dict[str, Any]:
    """Run ``handler`` with ``expected`` numeric arguments parsed from the event."""

    numbers = _extract_numbers(evt.get("command", ""))
    if len(numbers) < expected:
        print(
            "[link_mqtt] Not enough numeric arguments for template",
            evt.get("template"),
        )
        return {"ack": False}

    args: Iterable[float] = numbers[:expected]
    try:
        result = handler(*args)
    except Exception as exc:  # pragma: no cover - defensive logging
        print(f"[link_mqtt] Robot command failed: {exc}")
        return {"ack": False, "error": str(exc)}

    return {"ack": True, "result": result}


def _handle_stop(_evt: Dict[str, Any]) -> Dict[str, Any]:
    try:
        result = stop()
    except Exception as exc:  # pragma: no cover - defensive logging
        print(f"[link_mqtt] Failed to stop robot: {exc}")
        return {"ack": False, "error": str(exc)}
    return {"ack": True, "result": result}


_DISPATCH: Dict[str, Callable[[Dict[str, Any]], Dict[str, Any]]] = {
    "move_forward_at_[number]_m/s_for_[number]_seconds": lambda evt: _call_motion(
        evt, forward
    ),
    "move_backward_at_[number]_m/s_for_[number]_seconds": lambda evt: _call_motion(
        evt, backward
    ),
    "turn_right_at_[number]_ras/s_for_[number]_seconds": lambda evt: _call_motion(
        evt, turn_right
    ),
    "turn_left_at_[number]_ras/s_for_[number]_seconds": lambda evt: _call_motion(
        evt, turn_left
    ),
    "stop": _handle_stop,
}


def handle_any(evt: Dict[str, Any]) -> Dict[str, Any]:
    """Dispatch recognised templates to the robot control helpers."""

    template = evt.get("template")
    if not template:
        print("[link_mqtt] Event did not match a known template")
        return {"ack": False}

    handler = _DISPATCH.get(template)
    if handler is None:
        print(f"[link_mqtt] No handler for template: {template}")
        return {"ack": False}

    return handler(evt)


engine.on(lambda _evt: True, handle_any)

bridge = OmniLinkMQTTBridge(engine)
bridge.loop_forever()
