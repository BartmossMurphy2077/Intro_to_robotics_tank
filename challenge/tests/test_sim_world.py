from challenge.sim.scenarios import build_world


def test_ultrasonic_distance_tracks_ball_ahead():
    world = build_world("line-with-ball", seed=1)
    world.pose.x_m = 3.0
    world.pose.y_m = 2.0
    world.pose.heading_rad = 0.0

    distance_cm = world.read_sonic_cm()

    assert 5.0 <= distance_cm <= 8.0


def test_ir_code_changes_when_crossing_line():
    world = build_world("straight-line", seed=1)
    world.pose.x_m = 1.0
    world.pose.y_m = 2.0
    world.pose.heading_rad = 0.0

    assert world.read_ir_code() == 7

    world.pose.y_m = 2.20

    assert world.read_ir_code() == 0


def test_clamp_pickup_requires_ball_in_reach():
    world = build_world("line-with-ball", seed=1)
    world.pose.x_m = 3.18
    world.pose.y_m = 2.0
    world.pose.heading_rad = 0.0
    world.set_clamp_mode(1)

    for _ in range(40):
        world.tick(world.config.dt_s)

    assert world.carrying_ball
    assert len(world.arena.balls) == 0


def test_full_course_starts_on_base_off_line_then_reaches_track():
    world = build_world("full-course", seed=1)

    assert world.arena.bases
    assert world.read_ir_code() == 0

    for _ in range(int(0.55 / world.config.dt_s)):
        world.set_motor_cmd(360, 360)
        world.tick(world.config.dt_s)

    assert world.read_ir_code() != 0
