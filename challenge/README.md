# Challenge Runtime

The challenge mission is composed of small, focused modules with a clear
seam between behavior logic, runtime glue, and hardware adapters.

## Run

From repo root:

```bash
python3 -m challenge.main
python3 -m challenge.main --gui                # pygame visualizer (sim mode)
python3 -m challenge.main --mode real          # real Freenove tank (Linux/Pi only)
```

## Layout

```
challenge/
├── main.py                  # thin entry: parse args, build mission, hand off to runtime.loop
├── mission/                 # mission state machine + behaviors
│   ├── config.py            # MissionConfig (tunable parameters)
│   ├── state.py             # MissionState enum
│   ├── pose.py              # Pose2D, normalize_angle, dead-reckoning
│   ├── memory.py            # MapPoint, GraphNode, LineGraph
│   ├── controller.py        # ChallengeMission (composes behaviors)
│   └── behaviors/           # one file per behavior, all implement Behavior
│       ├── base.py          # Behavior + MissionContext interface
│       ├── line_follow.py   # IR follow (mirrors vendor mode_infrared mapping)
│       ├── pickup.py        # wraps vendor mode_clamp_up/down for pick + drop
│       ├── avoid.py         # phased reflex avoidance
│       ├── seek_ball.py     # vision-driven red-ball seek
│       └── return_home.py   # dead-reckoned return to anchor
├── runtime/                 # interactive runtime
│   ├── console.py           # cbreak terminal HUD + non-blocking input
│   ├── commands.py          # command dispatcher (set/get/setmap/auto/status/help)
│   └── loop.py              # main read → handle → step → tick → render loop
├── hardware/                # CarBase adapters: real_car, mock_car, base
├── sim/                     # SimWorld, scenarios, physics, visualizer
├── vision/                  # red-ball detector
└── tests/
```

## Mission Behavior

The controller's state machine is intentionally small. Each tick it picks
exactly one behavior to run:

1. **FOLLOW_LINE** — `LineFollower` reads IR and maps to motor outputs using
   the same 3-bit code mapping as vendor `Code/Server/car.py:mode_infrared`,
   with a runtime-tunable override in `MissionConfig.line_command_map`.
2. **AVOID_OBSTACLE** — `ObstacleAvoidance` runs a phased reflex
   (backup → turn → bypass → return → settle).
3. **PICK_BALL** — `BallPickup` calls `set_mode_clamp(1)` and pumps
   `mode_clamp()` until the vendor `mode_clamp_up` lifecycle finishes
   (it does the ultrasonic alignment + servo lift internally).
4. **RETURN_HOME** — `ReturnHome` turns toward and drives to the home anchor.
5. **DROP_BALL** — `BallPickup.drop` runs the vendor `mode_clamp_down` lifecycle.
6. **SEEK_BALL** — `VisionSeeker` (gated by `--use-vision`) takes precedence
   when it has a confident lock and other filters agree.

If the line is lost, `LineFollower` runs a short spiral search before falling
back to a slow forward crawl.

## Manual Override (no auto-creep)

Pressing any of `w a s d` **latches** the mission into manual mode and stops
all autonomous behaviors. The wheels follow the last WASD command for
`manual_dwell_s` (default 0.30 s) and then stop at (0, 0) — they do *not*
auto-creep. Type `auto` (or `resume`) to re-enable autonomous behaviors.

## Runtime Commands

| Command                        | Action                                                              |
| ------------------------------ | ------------------------------------------------------------------- |
| `w` `a` `s` `d`                | Drive (latches manual, no auto-creep)                               |
| `space`                        | Toggle pickup/drop using vendor clamp lifecycle                     |
| `auto` / `resume`              | Resume autonomous behaviors                                          |
| `home`                         | Reset home anchor to current pose                                    |
| `status`                       | Print one-line status                                                |
| `map`                          | Print ASCII map (lines, obstacles, balls, home, current pose)        |
| `help`, `?`                    | Print command list                                                   |
| `set <param> <value>`          | `pickup-cm`, `obstacle-cm`, `line-crawl-speed`, `home-radius-m`      |
| `get <param>`                  | Show a config param                                                  |
| `setmap <code> <l> <r>`        | Override IR-to-duty mapping at runtime                               |

## Useful Options

```bash
python3 -m challenge.main --obstacle-cm 18 --pickup-cm 15 --home-radius-m 0.22
python3 -m challenge.main --status-interval 0.5 --loop-sleep 0.05
python3 -m challenge.main --line-crawl-speed 260 --ir-zero-lost
python3 -m challenge.main --calibrate --calibrate-arm --calibrate-seconds 6
```

## Adding a New Behavior

1. Add a file under `mission/behaviors/`, e.g. `my_behavior.py`.
2. Implement `step(self, ctx: MissionContext)` (and any helpers).
3. Register in `mission/behaviors/__init__.py`.
4. Wire it into `mission/controller.py` (state branch, or a new `MissionState`).
