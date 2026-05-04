"""Main run loop for the challenge.

Single function `run_mission(mission, cfg, ...)` that owns the read → handle
→ step → tick → render → status cadence. Visualizer is optional and
injected as a parameter; sim world is also optional (None on real hardware).
"""

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
    running = False

    print(format_startup_banner(cfg))
    console.print_info_line("[challenge] waiting for 'start' command")

    try:
        console.start()
        while True:
            commands = console.poll_commands()
            if visualizer is not None:
                gui = visualizer.poll_commands()
                if gui:
                    commands = list(commands) + list(gui)
            commands = coalesce_movement_commands(commands)

            for command in commands:
                cmd = (command or "").strip().lower()
                if cmd == "start":
                    if not running:
                        running = True
                        console.print_info_line("[challenge] started")
                    else:
                        console.print_info_line("[challenge] already started")
                    continue
                if cmd in ("stop", "pause"):
                    if running:
                        running = False
                        mission.stop_drive()
                        console.print_info_line("[challenge] paused; type 'start' to continue")
                    else:
                        console.print_info_line("[challenge] already paused")
                    continue
                if handle_command(
                    command, mission, cfg, emit_line=console.print_info_line
                ):
                    continue

            if running:
                mission.step()

            if running and sim_world is not None:
                speed = getattr(visualizer, "speed", 1.0) if visualizer is not None else 1.0
                sim_world.tick(cfg.loop_sleep_s * max(0.25, min(8.0, float(speed))))

            if visualizer is not None:
                visualizer.draw(mission)
                if visualizer.should_quit():
                    break

            now = time.monotonic()
            if status_interval_s > 0 and now - last_status >= status_interval_s:
                console.print_status_line(format_hud_line(mission.get_status()))
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
    return (
        f"[challenge] obstacle_cm={cfg.obstacle_distance_cm:.1f} "
        f"pickup_cm={cfg.pickup_distance_cm:.1f} home_radius_m={cfg.home_radius_m:.2f} "
        f"vision={cfg.use_vision}\n"
        "[challenge] commands: start  WASD drive (latches manual)  space pickup  "
        "auto resume  stop/pause  home/status/help + Enter"
    )


__all__ = ["run_mission", "format_startup_banner"]
