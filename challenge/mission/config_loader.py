"""Editable mission config loader.

Reads a JSON file (default: `challenge/config/default.json`) and applies it to
a `MissionConfig` instance. Operators can tune line speeds, thresholds, and
servo timings without touching Python code.

Unknown keys (and any key starting with `_comment`) are silently ignored, so
the same file can carry inline notes for humans.
"""

from __future__ import annotations

import json
from dataclasses import fields
from pathlib import Path
from typing import Any

from .config import MissionConfig


def load_config_file(path: str | Path) -> dict[str, Any]:
    p = Path(path)
    with p.open("r", encoding="utf-8") as fh:
        data = json.load(fh)
    if not isinstance(data, dict):
        raise ValueError(f"config file must be a JSON object: {p}")
    return data


def apply_config_dict(cfg: MissionConfig, data: dict[str, Any]) -> list[str]:
    """Apply known keys from `data` onto `cfg`.

    Returns the list of keys that were skipped (unknown or comments) so callers
    can log them. Type-coerces line_command_map keys to int, since JSON only
    supports string keys.
    """
    valid = {f.name: f.type for f in fields(cfg)}
    skipped: list[str] = []
    for key, value in data.items():
        if key.startswith("_") or key not in valid:
            skipped.append(key)
            continue
        if key == "line_command_map" and isinstance(value, dict):
            coerced: dict[int, tuple[int, int]] = {}
            for k, pair in value.items():
                try:
                    code = int(k)
                except (TypeError, ValueError):
                    skipped.append(f"line_command_map[{k!r}]")
                    continue
                if not (isinstance(pair, (list, tuple)) and len(pair) == 2):
                    skipped.append(f"line_command_map[{k!r}]")
                    continue
                coerced[code] = (int(pair[0]), int(pair[1]))
            setattr(cfg, key, coerced)
            continue
        setattr(cfg, key, value)
    return skipped


__all__ = ["apply_config_dict", "load_config_file"]
