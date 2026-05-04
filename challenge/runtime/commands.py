"""Command dispatch for the runtime console.

Primary keys are handled in `runtime.loop` (Q / E / I). Here: WASD (when loop
passes `operator_auto=False`), space (pick/drop), H (home), typed tuning.
"""

from __future__ import annotations

from typing import Callable

from ..mission import ChallengeMission, MissionConfig


EmitLine = Callable[[str], None]


HELP_TEXT = (
    "controls:  Q toggle AUTO/MANUAL | E stop | WASD (manual only) | "
    "space pickup | I ir-debug | H home\n"
    "tuning (typed):  status | help | set <param> <value> | get <param> | "
    "setmap <code> <left> <right>\n"
    "  params:  pickup-cm  obstacle-cm  line-crawl-speed  home-radius-m  "
    "manual-speed-forward  manual-speed-turn  manual-dwell-s  ir-majority-window"
)


def handle_command(
    command: str,
    mission: ChallengeMission,
    cfg: MissionConfig,
    emit_line: EmitLine = print,
    *,
    operator_auto: bool = False,
) -> bool:
    """Try to dispatch `command`. Return True if it was handled."""
    if not command:
        return False

    # Single-key manual drive — ignored while operator AUTO is on.
    if command in ("w", "a", "s", "d"):
        if operator_auto:
            emit_line("[challenge] WASD only in MANUAL (press Q)")
            return True
        mission.start_manual_drive(command)
        return True

    if command.startswith("manual-axis "):
        if operator_auto:
            emit_line("[challenge] manual drive only in MANUAL (press Q)")
            return True
        parts = command.split()
        if len(parts) != 3:
            return True
        try:
            forward_axis = int(parts[1])
            turn_axis = int(parts[2])
        except ValueError:
            return True
        mission.start_manual_vector(forward_axis, turn_axis)
        return True

    if command in (" ", "space"):
        mission.manual_pickup_toggle()
        return True

    # H = reset home anchor.
    if command == "h":
        mission.reset_home_anchor()
        emit_line("[challenge] home anchor reset")
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


# ---------- typed helper commands ----------


def _cmd_status(mission: ChallengeMission, cfg: MissionConfig, emit: EmitLine) -> None:
    s = mission.get_status()
    op = "AUTO" if s.get("operator_auto") else "MAN"
    emit(
        f"[challenge] {op} state={s['state']} reason={s['state_reason']} "
        f"x={s['x_m']:.2f} y={s['y_m']:.2f} hdg={s['heading_deg']:.0f}° "
        f"L/R={s['duty_l']}/{s['duty_r']} "
        f"ir={s['ir']:03b} raw={s.get('ir_raw', 0):03b} inv={s.get('ir_inverted', 0)} "
        f"line={'yes' if s.get('line_seen', 0) else 'NO'} "
        f"dist={s['distance_cm']:.1f}cm carry={s['carrying']} home={s['home_m']:.2f}m "
        f"tuned={s.get('tuned') or '-'}"
    )


def _cmd_help(mission: ChallengeMission, cfg: MissionConfig, emit: EmitLine) -> None:
    emit("[challenge]\n" + HELP_TEXT)


_COMMAND_HANDLERS: dict[str, Callable[[ChallengeMission, MissionConfig, EmitLine], None]] = {
    "status": _cmd_status,
    "help": _cmd_help,
    "?": _cmd_help,
}


# ---------- prefix commands (set/get/setmap) ----------


_NUMERIC_PARAMS: dict[str, str] = {
    "pickup-cm": "pickup_distance_cm",
    "obstacle-cm": "obstacle_distance_cm",
    "line-crawl-speed": "line_crawl_speed",
    "home-radius-m": "home_radius_m",
    "manual-speed-forward": "manual_speed_forward",
    "manual-speed-turn": "manual_speed_turn",
    "manual-dwell-s": "manual_dwell_s",
    "ir-majority-window": "ir_majority_window",
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
