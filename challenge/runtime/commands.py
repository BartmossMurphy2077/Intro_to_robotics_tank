"""Command dispatch for the runtime console.

Each command maps to a small handler function. Adding a new command is one
entry in `_COMMAND_HANDLERS` (or `_PREFIX_HANDLERS` for `set <param> <val>`
style commands).
"""

from __future__ import annotations

from typing import Callable, Optional

from ..mission import ChallengeMission, MissionConfig


EmitLine = Callable[[str], None]
HandlerResult = bool  # True = consumed, False = not handled


HELP_TEXT = (
    "commands: w a s d (manual drive)  space (pickup toggle)  auto (resume autonomy)  "
    "home (reset anchor)  status  help\n"
    "  set <param> <value>          (pickup-cm, obstacle-cm, line-crawl-speed)\n"
    "  get <param>\n"
    "  setmap <code> <left> <right> (override IR-to-duty mapping)"
)


def handle_command(
    command: str,
    mission: ChallengeMission,
    cfg: MissionConfig,
    emit_line: EmitLine = print,
) -> bool:
    """Try to dispatch `command`. Return True if it was handled."""
    if not command:
        return False

    # Single-key manual drive — latches manual mode (no auto-creep).
    if command in ("w", "a", "s", "d"):
        mission.start_manual_drive(command)
        return True

    if command in (" ", "space"):
        mission.manual_pickup_toggle()
        return True

    handler = _COMMAND_HANDLERS.get(command)
    if handler is not None:
        handler(mission, cfg, emit_line)
        return True

    for prefix, prefix_handler in _PREFIX_HANDLERS:
        if command.startswith(prefix + " ") or command == prefix:
            prefix_handler(command, mission, cfg, emit_line)
            return True

    return False


# ---------- single-word commands ----------


def _cmd_home(mission: ChallengeMission, cfg: MissionConfig, emit: EmitLine) -> None:
    mission.reset_home_anchor()
    emit("[challenge] home anchor reset")


def _cmd_auto(mission: ChallengeMission, cfg: MissionConfig, emit: EmitLine) -> None:
    if mission.is_manual_mode():
        mission.resume_autonomous()
        emit("[challenge] autonomous resumed")
    else:
        emit("[challenge] already autonomous")


def _cmd_status(mission: ChallengeMission, cfg: MissionConfig, emit: EmitLine) -> None:
    s = mission.get_status()
    mode = "manual" if s.get("manual") else "auto"
    emit(
        f"[challenge] mode={mode} state={s['state']} reason={s['state_reason']} "
        f"age={float(s['state_age_s']):.2f}s ir={s['ir']} dist={s['distance_cm']:.1f}cm "
        f"carry={s['carrying']} home={s['home_m']:.2f}m"
    )


def _cmd_help(mission: ChallengeMission, cfg: MissionConfig, emit: EmitLine) -> None:
    emit("[challenge] " + HELP_TEXT)


def _cmd_map(mission: ChallengeMission, cfg: MissionConfig, emit: EmitLine) -> None:
    try:
        emit(mission.get_map_string(size_m=2.0, resolution=41))
    except Exception as exc:
        emit(f"[challenge] map error: {exc}")


_COMMAND_HANDLERS: dict[str, Callable[[ChallengeMission, MissionConfig, EmitLine], None]] = {
    "home": _cmd_home,
    "auto": _cmd_auto,
    "resume": _cmd_auto,
    "status": _cmd_status,
    "help": _cmd_help,
    "?": _cmd_help,
    "map": _cmd_map,
}


# ---------- prefix commands (set/get/setmap) ----------


_NUMERIC_PARAMS: dict[str, str] = {
    # alias -> attribute name on MissionConfig
    "pickup-cm": "pickup_distance_cm",
    "obstacle-cm": "obstacle_distance_cm",
    "line-crawl-speed": "line_crawl_speed",
    "home-radius-m": "home_radius_m",
}


def _cmd_set(
    command: str, mission: ChallengeMission, cfg: MissionConfig, emit: EmitLine
) -> None:
    parts = command.split()
    if len(parts) != 3:
        emit("[challenge] set usage: set <param> <value>")
        return
    _, alias, raw = parts
    attr = _NUMERIC_PARAMS.get(alias)
    if attr is None:
        emit(f"[challenge] unknown param: {alias}")
        return
    try:
        value: float | int = float(raw)
    except ValueError:
        emit("[challenge] set value must be numeric")
        return
    if isinstance(getattr(cfg, attr), int):
        value = int(value)
    setattr(cfg, attr, value)
    emit(f"[challenge] set {alias} = {value}")


def _cmd_get(
    command: str, mission: ChallengeMission, cfg: MissionConfig, emit: EmitLine
) -> None:
    parts = command.split()
    if len(parts) != 2:
        emit("[challenge] get usage: get <param>")
        return
    _, alias = parts
    attr = _NUMERIC_PARAMS.get(alias)
    if attr is None:
        emit(f"[challenge] unknown param: {alias}")
        return
    emit(f"{alias} = {getattr(cfg, attr)}")


def _cmd_setmap(
    command: str, mission: ChallengeMission, cfg: MissionConfig, emit: EmitLine
) -> None:
    parts = command.split()
    if len(parts) != 4:
        emit("[challenge] setmap usage: setmap <code> <left> <right>")
        return
    _, code_s, left_s, right_s = parts
    try:
        code = int(code_s)
        left = int(left_s)
        right = int(right_s)
    except ValueError:
        emit("[challenge] setmap arguments must be integers")
        return
    try:
        cfg.line_command_map[code] = (left, right)
        emit(f"[challenge] line map[{code}] = ({left}, {right})")
    except Exception as exc:
        emit(f"[challenge] failed to set map: {exc}")


_PREFIX_HANDLERS: list[tuple[str, Callable[[str, ChallengeMission, MissionConfig, EmitLine], None]]] = [
    ("set", _cmd_set),
    ("get", _cmd_get),
    ("setmap", _cmd_setmap),
]


__all__ = ["handle_command", "HELP_TEXT"]
