"""Main run loop for the challenge.

Control model:

    Q     toggle operator AUTO (tuned mission: line → ball → home) vs MANUAL
    E     stop — drop to MANUAL with wheels idle (aborts in-progress pickup)
    W A S D   manual drive (only in MANUAL)
    space     pickup / drop (works in both modes)
    I / H     IR debug / home anchor

`mission.step()` always runs so manual dwell auto-stop and incremental pickup
stay coherent. The simulator world ticks whenever `sim_world` is present so
manual WASD moves the mock robot."""
from __future__ import annotations

import time
from typing import Any

from ..mission import ChallengeMission, MissionConfig
from .commands import handle_command
from .console import RuntimeConsole, coalesce_movement_commands, format_hud_line


def run_mission(
    mission: ChallengeMission,
    cfg: MissionConfig,
    *,
    sim_world: Any = None,
    visualizer: Any = None,
    status_interval_s: float = 1.0,
) -> None:
    console = RuntimeConsole()
    last_status = 0.0

    mission.set_ir_debug(cfg.ir_debug_log, emit=console.print_info_line)
    mission.enter_manual_idle_at_start()

    print(format_startup_banner(cfg))
    console.print_info_line(
        "[challenge] MANUAL — Q=enter AUTO (tuned params)  "
        "Q again=MANUAL  E=stop  WASD=drive"
    )

    try:
        console.start()
        while True:
            commands = console.poll_commands()
            if visualizer is not None:
                gui = visualizer.poll_commands()
                if gui:
                    commands = list(commands) + list(gui)
            commands = coalesce_movement_commands(commands)

            op_auto = mission.is_operator_auto()
            for command in commands:
                cmd = (command or "").strip().lower()

                if cmd in ("q", "start"):
                    now_auto = mission.toggle_operator_auto()
                    if now_auto:
                        src = cfg.trained_params_source or "defaults"
                        console.print_info_line(
                            f"[challenge] AUTO on (params: {src})"
                        )
                    else:
                        console.print_info_line("[challenge] MANUAL (WASD)")
                    op_auto = now_auto
                    continue

                if cmd in ("e", "stop", "pause"):
                    mission.stop_drive_latched()
                    op_auto = False
                    console.print_info_line(
                        "[challenge] E-stop → MANUAL (Q toggles AUTO)"
                    )
                    continue

                if cmd in ("i", "irdebug"):
                    new_state = not cfg.ir_debug_log
                    cfg.ir_debug_log = new_state
                    mission.set_ir_debug(new_state, emit=console.print_info_line)
                    console.print_info_line(
                        f"[challenge] ir debug {'ON' if new_state else 'off'}"
                    )
                    continue

                if handle_command(
                    command,
                    mission,
                    cfg,
                    emit_line=console.print_info_line,
                    operator_auto=op_auto,
                ):
                    continue

            mission.step()

            if sim_world is not None:
                speed = (
                    getattr(visualizer, "speed", 1.0)
                    if visualizer is not None
                    else 1.0
                )
                sim_world.tick(
                    cfg.loop_sleep_s * max(0.25, min(8.0, float(speed)))
                )

            if visualizer is not None:
                visualizer.draw(mission)
                if visualizer.should_quit():
                    break

            now = time.monotonic()
            if status_interval_s > 0 and now - last_status >= status_interval_s:
                console.print_status_line(
                    format_hud_line(mission.get_status(), mission.is_operator_auto())
                )
                last_status = now

            time.sleep(cfg.loop_sleep_s)
    except KeyboardInterrupt:
        console.print_info_line("[challenge] stopping")
    finally:
        console.stop()
        if visualizer is not None:
            visualizer.close()
        mission.car.close()


def format_startup_banner(cfg: MissionConfig) -> str:
    tune = ""
    if cfg.trained_params_source:
        tune = f" tuned={cfg.trained_params_source}"
    return (
        f"[challenge] obstacle_cm={cfg.obstacle_distance_cm:.1f} "
        f"pickup_cm={cfg.pickup_distance_cm:.1f} home_radius_m={cfg.home_radius_m:.2f} "
        f"vision={cfg.use_vision}{tune}\n"
        "[challenge] Q toggle AUTO/MANUAL  E stop  WASD (manual only)  "
        "space pickup  I ir-debug  H home"
    )


__all__ = ["run_mission", "format_startup_banner"]
