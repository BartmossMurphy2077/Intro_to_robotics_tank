#!/usr/bin/env python3
"""
competition.py — Autonomous Competition Controller
====================================================
Freenove Tank Robot Kit  •  PCB V2.0  •  Raspberry Pi 5
Pi IP: 10.205.9.10

OVERVIEW
--------
The robot starts at "home" (0, 0) and follows a black line on the ground using
its three infrared sensors.  While following the line it simultaneously watches
for two conditions:

  1. An obstacle is detected by the ultrasonic sensor → drive around it on the
     LEFT side and re-acquire the line.
  2. The camera sees a RED BALL → approach it, pick it up with the arm/clamp,
     drive straight back to home (dead-reckoning), drop the ball, then find the
     line again and continue.

FEATURE FLAGS (default: all enabled)
--------------------------------------
Three module-level booleans let you test each subsystem independently:

  ENABLE_INFRARED   — IR line-following motors
  ENABLE_ULTRASONIC — ultrasonic obstacle detection
  ENABLE_VISION     — camera red-ball detection

Override them from the command line (see --help).

USAGE
-----
  # On the Pi (SSH in, cd to Code/Server first):
  python3 competition.py                               Full competition mode
  python3 competition.py --no-vision --no-ultrasonic   IR line-follow only
  python3 competition.py --no-infrared --no-ultrasonic Vision/ball test only
  python3 competition.py --no-vision                   Line + obstacle test
  python3 competition.py --calibrate                   Sensor readout (no drive)

DEPENDENCIES (one-time install on the Pi)
------------------------------------------
  pip3 install opencv-python-headless
  # picamera2, lgpio, gpiozero, rpi-hardware-pwm are already present

STATE MACHINE
-------------
  LINE_FOLLOW → (obstacle) → OBSTACLE_AVOID → LINE_FOLLOW
  LINE_FOLLOW → (ball)     → BALL_APPROACH  → BALL_PICKUP
  BALL_PICKUP → RETURN_HOME → DROP_BALL → FIND_LINE → LINE_FOLLOW

HARDWARE SUMMARY (PCB V2.0 / Pi 5)
------------------------------------
  Arm servo  ch0 GPIO12: 90 ° = raised (home),  130 ° = lowered
  Clamp servo ch1 GPIO13: 140 ° = closed/gripping, 90 ° = open
  IR sensors: IR01 GPIO16 (left), IR02 GPIO26 (centre), IR03 GPIO21 (right)
              read_all_infrared() → 3-bit: bit2=left, bit1=centre, bit0=right
              1 = line detected
  Ultrasonic: GPIO27 (trig), GPIO22 (echo) — distance in cm
  Motors:     duty range ±4095; positive = forward
"""

import sys
import os
import time
import math
import signal
import argparse
import threading
from dataclasses import dataclass
from enum import Enum, auto

# ── Ensure the Server directory is on sys.path so relative imports work ───────
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

# ── Existing hardware drivers (none of these files are modified) ───────────────
from motor     import tankMotor
from servo     import Servo
from infrared  import Infrared
from ultrasonic import Ultrasonic
from led       import Led


# ══════════════════════════════════════════════════════════════════════════════
#  FEATURE FLAGS  —  overridden by CLI args in main()
# ══════════════════════════════════════════════════════════════════════════════

ENABLE_INFRARED   = True   # IR sensor line-following
ENABLE_ULTRASONIC = True   # Ultrasonic obstacle avoidance
ENABLE_VISION     = True   # Camera-based red-ball detection


# ══════════════════════════════════════════════════════════════════════════════
#  CALIBRATION CONSTANTS
#  ─────────────────────
#  HOW TO CALIBRATE on competition day:
#
#  TURN_90_S  : Run `python3 competition.py --calibrate`.  Manually drive the
#               robot with setMotorModel(-1500, 1500) for some time and measure
#               how many degrees it turns per second.  Set TURN_90_S = 90 /
#               degrees_per_second.  Typical range: 0.65 – 0.90 s.
#
#  FORWARD_MPS: Drive at duty 2000 for exactly 1.0 s; measure the distance
#               travelled in metres.  That value is FORWARD_MPS.
#
#  WHEEL_BASE_M: Measure the distance (in metres) between the centres of the
#               left and right tracks.
#
#  SPEED_SCALE: Computed automatically from FORWARD_MPS.  Do not change it
#               directly.
# ══════════════════════════════════════════════════════════════════════════════

TURN_90_S        = 0.75    # seconds to rotate 90 ° at motor duty ±1500
FORWARD_MPS      = 0.30    # metres/second at motor duty 2000
WHEEL_BASE_M     = 0.155   # metres between left and right track centres
SPEED_SCALE      = FORWARD_MPS / 2000.0   # m/s per duty unit (auto-computed)

# Detection thresholds
OBSTACLE_DIST_CM = 35.0    # cm — ultrasonic triggers avoidance below this
PICKUP_DIST_CM   = 12.0    # cm — switch from BALL_APPROACH to BALL_PICKUP

# Obstacle bypass timings (tune alongside TURN_90_S)
BYPASS_FORWARD_S = 0.60    # seconds to drive straight past obstacle side/front

# Find-line / search limits
FIND_LINE_TIMEOUT_S  = 5.0   # seconds to spin before giving up in FIND_LINE
BALL_LOST_TIMEOUT_S  = 2.0   # seconds without ball detection → abandon approach

# Ball approach PID
BALL_APPROACH_KP   = 3.0    # proportional gain: steer = Kp × pixel_offset
BALL_APPROACH_BASE = 1200   # base forward duty during ball approach
MAX_STEER          = 800    # maximum steer correction (clamps Kp * offset)

# Vision / OpenCV
MIN_BALL_AREA_PX = 800      # minimum contour area (px²) to count as ball
BALL_HYSTERESIS  = 3        # consecutive frames with ball before "confirmed"

# Main loop
LOOP_HZ = 20
LOOP_DT = 1.0 / LOOP_HZ    # 50 ms per tick; max dt cap prevents tracker jump

# ── Servo angle positions ─────────────────────────────────────────────────────
#   Arm  ch0: 90° = raised (home/parked),  130° = lowered to ground
#   Clamp ch1: 140° = closed/gripping,      90° = open
ARM_UP       = 90
ARM_DOWN     = 130
CLAMP_CLOSED = 140
CLAMP_OPEN   = 90

# ── Line-follow motor commands (left_duty, right_duty) ───────────────────────
LINE_FORWARD    = ( 1500,  1500)
LINE_HARD_LEFT  = (-1500,  2500)
LINE_SOFT_LEFT  = (  800,  2000)
LINE_HARD_RIGHT = ( 2500, -1500)
LINE_SOFT_RIGHT = ( 2000,   800)
LINE_SEARCH     = ( 1200,  1200)   # lost line — creep forward searching


# ══════════════════════════════════════════════════════════════════════════════
#  STATE MACHINE ENUM
# ══════════════════════════════════════════════════════════════════════════════

class State(Enum):
    LINE_FOLLOW    = auto()   # following black line with IR sensors
    OBSTACLE_AVOID = auto()   # blocking: drive around obstacle then resume
    BALL_APPROACH  = auto()   # camera-guided approach toward red ball
    BALL_PICKUP    = auto()   # blocking: lower arm, grip ball, raise arm
    RETURN_HOME    = auto()   # blocking: dead-reckoning drive to (0, 0)
    DROP_BALL      = auto()   # blocking: open clamp, raise arm, park
    FIND_LINE      = auto()   # spin until IR sensor finds a line


# ══════════════════════════════════════════════════════════════════════════════
#  DEAD-RECKONING POSITION TRACKER
# ══════════════════════════════════════════════════════════════════════════════

class PositionTracker:
    """
    Differential-drive dead-reckoning.

    Coordinate system:
      x, y in metres;  heading in radians (0 = initial forward direction).
      Robot starts at (0, 0, 0) = home.

    Call update() once per loop tick with the current motor duty values and
    elapsed time.  Call reset() after the robot returns home and drops the ball.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.x       = 0.0
        self.y       = 0.0
        self.heading = 0.0   # radians; 0 = original forward direction

    def update(self, left_duty: float, right_duty: float, dt: float):
        """Integrate wheel velocities to update (x, y, heading)."""
        vl = left_duty  * SPEED_SCALE   # m/s
        vr = right_duty * SPEED_SCALE   # m/s
        v      = (vl + vr) / 2.0
        omega  = (vr - vl) / WHEEL_BASE_M
        self.heading += omega * dt
        self.x       += v * math.cos(self.heading) * dt
        self.y       += v * math.sin(self.heading) * dt

    def distance_to_home(self) -> float:
        """Euclidean distance from current position to (0, 0) in metres."""
        return math.hypot(self.x, self.y)

    def angle_to_home(self) -> float:
        """
        Signed angle (radians) the robot must turn to face (0, 0).
        Positive = turn left (counter-clockwise);
        Negative = turn right (clockwise).
        Normalised to (−π, +π].
        """
        target = math.atan2(-self.y, -self.x)
        delta  = target - self.heading
        while delta >  math.pi:  delta -= 2 * math.pi
        while delta < -math.pi:  delta += 2 * math.pi
        return delta


# ══════════════════════════════════════════════════════════════════════════════
#  THREAD-SAFE BALL DETECTION RESULT
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class BallInfo:
    detected:  bool  = False
    offset_x:  int   = 0       # pixels from image centre; + = ball is right
    area:      float = 0.0     # contour area in pixels²
    timestamp: float = 0.0     # time.time() when last updated


class VisionThread:
    """
    Background daemon thread: reads from picamera2 at 320×240 RGB888,
    applies a HSV red-colour mask (two hue ranges wrapping around 0°),
    finds the largest qualifying contour, and publishes a BallInfo.

    Uses a 3-frame hysteresis counter so single-frame noise is ignored.
    If ENABLE_VISION is False this thread is never created.
    """

    FRAME_W = 320
    FRAME_H = 240

    def __init__(self):
        self._lock    = threading.Lock()
        self._result  = BallInfo()
        self._running = False
        self._thread  = None
        self._consec  = 0   # consecutive frames with a valid detection

    def start(self):
        self._running = True
        self._thread  = threading.Thread(
            target=self._loop, daemon=True, name="vision"
        )
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)

    def get(self) -> BallInfo:
        """Thread-safe snapshot of the latest detection result."""
        with self._lock:
            b = self._result
            return BallInfo(
                detected  = b.detected,
                offset_x  = b.offset_x,
                area      = b.area,
                timestamp = b.timestamp,
            )

    def _loop(self):
        try:
            import cv2
            import numpy as np
            from picamera2 import Picamera2
        except ImportError as e:
            print(f"[Vision] Import error: {e}  —  vision subsystem disabled")
            return

        cam    = Picamera2()
        config = cam.create_preview_configuration(
            main={"format": "RGB888", "size": (self.FRAME_W, self.FRAME_H)}
        )
        cam.configure(config)
        cam.start()
        time.sleep(0.5)   # let exposure settle

        # Red HSV ranges: hue wraps, so we need two bands
        RED_LO1 = np.array([  0, 120,  70], dtype=np.uint8)
        RED_HI1 = np.array([ 10, 255, 255], dtype=np.uint8)
        RED_LO2 = np.array([170, 120,  70], dtype=np.uint8)
        RED_HI2 = np.array([180, 255, 255], dtype=np.uint8)

        kernel  = np.ones((3, 3), np.uint8)
        cx_half = self.FRAME_W // 2

        print("[Vision] Camera started — watching for red ball")

        while self._running:
            try:
                frame = cam.capture_array()                          # RGB888
                hsv   = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
                mask1 = cv2.inRange(hsv, RED_LO1, RED_HI1)
                mask2 = cv2.inRange(hsv, RED_LO2, RED_HI2)
                mask  = cv2.bitwise_or(mask1, mask2)
                mask  = cv2.erode (mask, kernel, iterations=2)
                mask  = cv2.dilate(mask, kernel, iterations=2)

                contours, _ = cv2.findContours(
                    mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )

                found = False
                if contours:
                    largest = max(contours, key=cv2.contourArea)
                    area    = cv2.contourArea(largest)
                    if area >= MIN_BALL_AREA_PX:
                        M = cv2.moments(largest)
                        if M["m00"] > 0:
                            cx       = int(M["m10"] / M["m00"])
                            offset_x = cx - cx_half
                            self._consec += 1
                            detected = self._consec >= BALL_HYSTERESIS
                            with self._lock:
                                self._result = BallInfo(
                                    detected  = detected,
                                    offset_x  = offset_x,
                                    area      = area,
                                    timestamp = time.time(),
                                )
                            found = True

                if not found:
                    self._consec = 0
                    with self._lock:
                        self._result = BallInfo(
                            detected  = False,
                            timestamp = time.time(),
                        )

            except Exception as exc:
                print(f"[Vision] Frame error: {exc}")

            time.sleep(LOOP_DT)

        cam.stop()
        cam.close()
        print("[Vision] Camera stopped")


# ══════════════════════════════════════════════════════════════════════════════
#  COMPETITION ROBOT — main controller
# ══════════════════════════════════════════════════════════════════════════════

class CompetitionRobot:
    """
    Integrates all hardware drivers and runs the 7-state competition FSM.

    Instantiate, then call either:
      robot.run()            — full autonomous loop
      robot.calibrate_loop() — sensor readout without driving
    """

    def __init__(self):
        print("[Robot] Initialising hardware ...")
        print(f"[Robot] Feature flags:  IR={ENABLE_INFRARED}  "
              f"Ultrasonic={ENABLE_ULTRASONIC}  Vision={ENABLE_VISION}")

        # ── Hardware ─────────────────────────────────────────────────────────
        self.motor    = tankMotor()
        self.servo    = Servo()
        self.infrared = Infrared()    if ENABLE_INFRARED   else None
        self.sonic    = Ultrasonic()  if (ENABLE_ULTRASONIC or ENABLE_VISION) else None
        self.led      = Led()
        self.vision   = VisionThread() if ENABLE_VISION else None

        # ── State ─────────────────────────────────────────────────────────────
        self.tracker  = PositionTracker()
        self.state    = State.LINE_FOLLOW
        self.has_ball = False

        # Internal bookkeeping
        self._left_duty      = 0
        self._right_duty     = 0
        self._ball_last_seen = 0.0   # timestamp of last positive ball detection

        # Graceful shutdown on Ctrl-C or SIGTERM
        signal.signal(signal.SIGINT,  self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        # Park arm and close clamp at startup
        self._arm_home()
        print("[Robot] Ready.  Starting in LINE_FOLLOW state.")
        self._set_state_led(State.LINE_FOLLOW)

    # ─────────────────────────────────────────────────────────────────────────
    # Low-level helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _drive(self, left: int, right: int):
        self._left_duty  = left
        self._right_duty = right
        self.motor.setMotorModel(left, right)

    def _stop(self):
        self._drive(0, 0)

    def _drive_timed(self, left: int, right: int, duration: float):
        """Drive for `duration` seconds, updating the position tracker."""
        t0 = time.time()
        self._drive(left, right)
        while time.time() - t0 < duration:
            self.tracker.update(left, right, LOOP_DT)
            time.sleep(LOOP_DT)

    def _turn_timed(self, left: int, right: int, duration: float):
        """
        Spin in-place for `duration` seconds.
        For a true in-place spin (left=-right), v=0 so only heading changes.
        """
        self._drive(left, right)
        t0 = time.time()
        while time.time() - t0 < duration:
            self.tracker.update(left, right, LOOP_DT)
            time.sleep(LOOP_DT)
        self._stop()

    # ─────────────────────────────────────────────────────────────────────────
    # LED helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _led_solid(self, r: int, g: int, b: int):
        """Set all 4 LEDs to one solid colour."""
        self.led.ledIndex(0b1111, r, g, b)

    def _led_off(self):
        self.led.ledIndex(0b1111, 0, 0, 0)

    def _led_flash(self, r: int, g: int, b: int,
                   n: int = 3, on_s: float = 0.18, off_s: float = 0.12):
        """Flash all LEDs n times."""
        for _ in range(n):
            self._led_solid(r, g, b);  time.sleep(on_s)
            self._led_off();           time.sleep(off_s)

    def _set_state_led(self, state: State):
        """
        Set LED colour for the given state.

        Colour guide:
          LINE_FOLLOW    Blue  (0, 0, 180)      — following the line
          OBSTACLE_AVOID Yellow (180, 180, 0)   — going around something
          BALL_APPROACH  Red   (180, 0, 0)      — closing in on ball
          BALL_PICKUP    Bright red (255, 0, 0) — gripping
          RETURN_HOME    Purple (150, 0, 150)   — driving back to home
          DROP_BALL      Green (0, 200, 0)      — releasing ball
          FIND_LINE      White (150, 150, 150)  — searching for line
        """
        colours = {
            State.LINE_FOLLOW:    (0,   0,   180),
            State.OBSTACLE_AVOID: (180, 180, 0  ),
            State.BALL_APPROACH:  (180, 0,   0  ),
            State.BALL_PICKUP:    (255, 0,   0  ),
            State.RETURN_HOME:    (150, 0,   150),
            State.DROP_BALL:      (0,   200, 0  ),
            State.FIND_LINE:      (150, 150, 150),
        }
        c = colours.get(state, (0, 0, 0))
        self._led_solid(*c)

    # ─────────────────────────────────────────────────────────────────────────
    # State transitions
    # ─────────────────────────────────────────────────────────────────────────

    def _transition(self, new_state: State):
        if new_state != self.state:
            print(f"[FSM] {self.state.name} → {new_state.name}")
            self.state = new_state
            self._set_state_led(new_state)

    # ─────────────────────────────────────────────────────────────────────────
    # Arm / clamp sequences  (blocking)
    # ─────────────────────────────────────────────────────────────────────────

    def _arm_home(self):
        """Park arm up (ch0=90°) and clamp closed (ch1=140°)."""
        self.servo.setServoAngle('0', ARM_UP)
        time.sleep(0.3)
        self.servo.setServoAngle('1', CLAMP_CLOSED)
        time.sleep(0.3)

    def _arm_pickup(self):
        """
        Grab a ball sitting in front of the robot:
          1. Open clamp    ch1: 140° → 90°
          2. Lower arm     ch0:  90° → 130°
          3. Close clamp   ch1:  90° → 140°  (now gripping)
        """
        print("[Arm] Pickup: open clamp")
        for angle in range(CLAMP_CLOSED, CLAMP_OPEN - 1, -2):
            self.servo.setServoAngle('1', angle)
            time.sleep(0.01)

        print("[Arm] Pickup: lower arm")
        for angle in range(ARM_UP, ARM_DOWN + 1, 1):
            self.servo.setServoAngle('0', angle)
            time.sleep(0.01)

        print("[Arm] Pickup: close clamp")
        for angle in range(CLAMP_OPEN, CLAMP_CLOSED + 1, 2):
            self.servo.setServoAngle('1', angle)
            time.sleep(0.01)

        print("[Arm] Ball gripped")

    def _arm_drop(self):
        """
        Release ball and park arm:
          1. Open clamp    ch1: 140° → 90°
          2. Raise arm     ch0: 130° → 90°
          3. Close clamp   ch1:  90° → 140°  (empty / parked)
        """
        print("[Arm] Drop: open clamp")
        for angle in range(CLAMP_CLOSED, CLAMP_OPEN - 1, -2):
            self.servo.setServoAngle('1', angle)
            time.sleep(0.01)

        print("[Arm] Drop: raise arm")
        for angle in range(ARM_DOWN, ARM_UP - 1, -1):
            self.servo.setServoAngle('0', angle)
            time.sleep(0.01)

        print("[Arm] Drop: close clamp (park)")
        for angle in range(CLAMP_OPEN, CLAMP_CLOSED + 1, 2):
            self.servo.setServoAngle('1', angle)
            time.sleep(0.01)

        print("[Arm] Ball released")

    # ─────────────────────────────────────────────────────────────────────────
    # IR line-following  (one tick, non-blocking)
    # ─────────────────────────────────────────────────────────────────────────

    def _step_line_follow(self) -> tuple:
        """
        Read all three IR sensors and return (left_duty, right_duty).

        IR bit encoding (read_all_infrared returns a 3-bit int):
          bit 2 = IR01 (GPIO16) = LEFT  sensor  — 1 means line detected
          bit 1 = IR02 (GPIO26) = CENTRE sensor
          bit 0 = IR03 (GPIO21) = RIGHT  sensor

        Decision table:
          0 (000) none     → creep forward (search)
          1 (001) right    → hard right turn
          2 (010) centre   → straight forward
          3 (011) c+r      → gentle right
          4 (100) left     → hard left turn
          5 (101) l+r      → T-junction / branch → go straight
          6 (110) l+c      → gentle left
          7 (111) all      → wide junction / intersection → go straight
                             NOTE: original car.py stops at 7; we continue
                             through intersections as the competition course
                             may have branches.
        """
        ir = self.infrared.read_all_infrared()
        mapping = {
            0: LINE_SEARCH,      # no line — creep/search
            1: LINE_HARD_RIGHT,  # right only
            2: LINE_FORWARD,     # centre only
            3: LINE_SOFT_RIGHT,  # centre + right
            4: LINE_HARD_LEFT,   # left only
            5: LINE_FORWARD,     # left + right  (T-junction)
            6: LINE_SOFT_LEFT,   # left + centre
            7: LINE_FORWARD,     # all sensors   (wide junction)
        }
        return mapping.get(ir, LINE_SEARCH)

    # ─────────────────────────────────────────────────────────────────────────
    # Obstacle avoidance  (blocking, always turns LEFT)
    # ─────────────────────────────────────────────────────────────────────────

    def _do_obstacle_avoid(self):
        """
        Drive around the obstacle on the left side using a fixed timed
        sequence, then re-acquire the line.

        Manoeuvre steps (all at ±1500 duty):
          1. Reverse briefly           (avoid bumping obstacle)
          2. Turn left  90°            (face left of obstacle)
          3. Drive forward             (clear the obstacle's side)
          4. Turn right 90°            (now parallel to original heading)
          5. Drive forward             (clear the obstacle's front)
          6. Turn right 90°            (now facing original heading)
          7. Creep forward             (until IR detects line, max 3 s)
          8. Small left correction     (straighten up on line)

        All timings are controlled by TURN_90_S and BYPASS_FORWARD_S.
        Calibrate TURN_90_S first for accurate 90° turns.
        """
        print("[Obstacle] Starting left-side bypass")
        self._stop()
        time.sleep(0.15)

        # 1. Reverse
        self._drive_timed(-1500, -1500, 0.30)

        # 2. Left 90°
        self._turn_timed(-1500, 1500, TURN_90_S)

        # 3. Clear obstacle side
        self._drive_timed(1500, 1500, BYPASS_FORWARD_S)

        # 4. Right 90° (parallel to original heading)
        self._turn_timed(1500, -1500, TURN_90_S)

        # 5. Clear obstacle front
        self._drive_timed(1500, 1500, BYPASS_FORWARD_S)

        # 6. Right 90° (back to original heading)
        self._turn_timed(1500, -1500, TURN_90_S)

        # 7. Creep forward until IR hits the line
        print("[Obstacle] Searching for line ...")
        self._drive(1200, 1200)
        t0 = time.time()
        found = False
        while time.time() - t0 < FIND_LINE_TIMEOUT_S:
            if self.infrared and self.infrared.read_all_infrared() > 0:
                found = True
                break
            time.sleep(0.05)
        self._stop()

        # 8. Small correction — nudge left to centre on line
        if found:
            self._turn_timed(-1500, 1500, TURN_90_S * 0.12)
            print("[Obstacle] Line re-acquired")
        else:
            print("[Obstacle] WARNING — line not found after bypass")

    # ─────────────────────────────────────────────────────────────────────────
    # Ball approach  (one tick, non-blocking)
    # ─────────────────────────────────────────────────────────────────────────

    def _step_ball_approach(self, ball: BallInfo) -> bool:
        """
        Proportional steering toward the ball centre.
        Returns True when the robot is close enough to pick up the ball
        (ultrasonic distance ≤ PICKUP_DIST_CM).
        """
        steer = int(BALL_APPROACH_KP * ball.offset_x)
        steer = max(-MAX_STEER, min(MAX_STEER, steer))

        left  = BALL_APPROACH_BASE - steer
        right = BALL_APPROACH_BASE + steer
        self._drive(left, right)

        if self.sonic:
            dist = self.sonic.get_distance()
            if 0 < dist <= PICKUP_DIST_CM:
                return True
        return False

    # ─────────────────────────────────────────────────────────────────────────
    # Return home  (blocking dead-reckoning)
    # ─────────────────────────────────────────────────────────────────────────

    def _do_return_home(self):
        """
        Turn to face (0, 0) then drive straight there.

        Accuracy depends on TURN_90_S and FORWARD_MPS being well calibrated.
        The robot does not need to be perfectly on top of (0,0) — it just
        needs to be close enough to find the start line again in FIND_LINE.
        """
        dist  = self.tracker.distance_to_home()
        angle = self.tracker.angle_to_home()
        print(f"[Home] Position ({self.tracker.x:.2f}, {self.tracker.y:.2f})  "
              f"dist={dist:.2f} m  angle={math.degrees(angle):+.1f}°")

        # Turn to face home
        if abs(angle) > 0.15:   # ignore tiny corrections (< ~9°)
            # TURN_90_S is for pi/2 radians; scale linearly
            turn_s = abs(angle) / (math.pi / 2.0) * TURN_90_S
            if angle > 0:
                print(f"[Home] Turning left {math.degrees(angle):.1f}° ({turn_s:.2f} s)")
                self._turn_timed(-1500, 1500, turn_s)
            else:
                print(f"[Home] Turning right {-math.degrees(angle):.1f}° ({turn_s:.2f} s)")
                self._turn_timed(1500, -1500, turn_s)

        # Drive straight to home
        if dist > 0.05:   # ignore if already within 5 cm
            drive_s = dist / FORWARD_MPS
            print(f"[Home] Driving {dist:.2f} m  ({drive_s:.1f} s)")
            self._drive_timed(2000, 2000, drive_s)

        self._stop()
        print("[Home] Arrived")

    # ─────────────────────────────────────────────────────────────────────────
    # Find line  (blocking spin)
    # ─────────────────────────────────────────────────────────────────────────

    def _do_find_line(self) -> bool:
        """
        Spin left slowly until any IR sensor detects a line, or timeout.
        Returns True if line found, False if timed out.
        """
        print("[FindLine] Spinning left to find line ...")
        self._drive(-800, 800)
        t0 = time.time()
        while time.time() - t0 < FIND_LINE_TIMEOUT_S:
            if self.infrared and self.infrared.read_all_infrared() > 0:
                self._stop()
                print("[FindLine] Line found")
                return True
            time.sleep(0.05)
        self._stop()
        print("[FindLine] TIMEOUT — no line found within "
              f"{FIND_LINE_TIMEOUT_S:.0f} s")
        return False

    # ─────────────────────────────────────────────────────────────────────────
    # LINE_FOLLOW tick
    # ─────────────────────────────────────────────────────────────────────────

    def _run_line_follow(self):
        """
        Execute one LINE_FOLLOW tick.

        Priority order (highest first):
          1. Vision   — ball confirmed → transition to BALL_APPROACH
          2. Ultrasonic — obstacle    → transition to OBSTACLE_AVOID
          3. IR       — line steer    → update motor duty
        """
        # ── Priority 1: vision ───────────────────────────────────────────────
        if ENABLE_VISION and self.vision:
            ball = self.vision.get()
            if ball.detected:
                print(f"[LineFollow] Red ball confirmed — "
                      f"offset={ball.offset_x:+d}  area={ball.area:.0f} px²")
                self._ball_last_seen = ball.timestamp
                self._stop()
                self._transition(State.BALL_APPROACH)
                return

        # ── Priority 2: ultrasonic ───────────────────────────────────────────
        if ENABLE_ULTRASONIC and self.sonic:
            dist = self.sonic.get_distance()
            if 0 < dist <= OBSTACLE_DIST_CM:
                print(f"[LineFollow] Obstacle at {dist:.1f} cm")
                self._stop()
                self._transition(State.OBSTACLE_AVOID)
                return

        # ── Priority 3: IR line steer ─────────────────────────────────────────
        if ENABLE_INFRARED and self.infrared:
            left, right = self._step_line_follow()
            self._drive(left, right)
        else:
            # IR disabled — remain stationary
            self._stop()

    # ─────────────────────────────────────────────────────────────────────────
    # BALL_APPROACH tick
    # ─────────────────────────────────────────────────────────────────────────

    def _run_ball_approach(self):
        """
        Execute one BALL_APPROACH tick.
        Transitions to BALL_PICKUP when close enough, or back to LINE_FOLLOW
        if the ball is lost for more than BALL_LOST_TIMEOUT_S seconds.
        """
        ball = self.vision.get() if self.vision else BallInfo()

        if ball.detected:
            self._ball_last_seen = ball.timestamp
            close_enough = self._step_ball_approach(ball)
            if close_enough:
                self._stop()
                self._transition(State.BALL_PICKUP)
        else:
            # Ball not visible this tick
            lost_for = time.time() - self._ball_last_seen
            if lost_for > BALL_LOST_TIMEOUT_S:
                print(f"[Approach] Ball lost for {lost_for:.1f} s — "
                      "abandoning, returning to LINE_FOLLOW")
                self._stop()
                self._transition(State.LINE_FOLLOW)
            # else: keep last motor command briefly while searching

    # ─────────────────────────────────────────────────────────────────────────
    # Error halt  (blink red until reset)
    # ─────────────────────────────────────────────────────────────────────────

    def _error_halt(self, msg: str):
        print(f"[ERROR] {msg}")
        print("[ERROR] Robot halted — press Ctrl-C to exit")
        self._stop()
        try:
            while True:
                self._led_solid(255, 0, 0); time.sleep(0.10)
                self._led_off();            time.sleep(0.10)
        except KeyboardInterrupt:
            pass

    # ─────────────────────────────────────────────────────────────────────────
    # Main autonomous loop
    # ─────────────────────────────────────────────────────────────────────────

    def run(self):
        """
        Start the competition FSM.  Runs until KeyboardInterrupt (Ctrl-C) or
        SIGTERM.  Calls shutdown() in the finally block regardless.
        """
        if self.vision:
            self.vision.start()
            time.sleep(1.0)   # give camera time to expose before loop starts

        self._transition(State.LINE_FOLLOW)
        t_last = time.time()

        try:
            while True:
                tick_start = time.time()

                # Cap dt to 2 loop ticks so a blocking call returning after
                # several seconds doesn't feed a huge dt into the tracker.
                dt     = min(tick_start - t_last, LOOP_DT * 2)
                t_last = tick_start

                # ── Dispatch current state ────────────────────────────────────
                if self.state == State.LINE_FOLLOW:
                    self._run_line_follow()

                elif self.state == State.OBSTACLE_AVOID:
                    # Blocking: handles full avoidance manoeuvre internally
                    self._do_obstacle_avoid()
                    t_last = time.time()   # reset timer after blocking call
                    self._transition(State.LINE_FOLLOW)

                elif self.state == State.BALL_APPROACH:
                    self._run_ball_approach()

                elif self.state == State.BALL_PICKUP:
                    self._stop()
                    self._arm_pickup()
                    self.has_ball = True
                    t_last = time.time()
                    self._transition(State.RETURN_HOME)

                elif self.state == State.RETURN_HOME:
                    self._do_return_home()
                    t_last = time.time()
                    self._transition(State.DROP_BALL)

                elif self.state == State.DROP_BALL:
                    self._stop()
                    self._arm_drop()
                    self.has_ball = False
                    self._led_flash(0, 255, 0, n=3)      # green success flash
                    self.tracker.reset()                  # back at (0, 0)
                    t_last = time.time()
                    self._transition(State.FIND_LINE)

                elif self.state == State.FIND_LINE:
                    found = self._do_find_line()
                    t_last = time.time()
                    if found:
                        self._transition(State.LINE_FOLLOW)
                    else:
                        self._error_halt(
                            "Could not find line after returning home"
                        )

                # ── Update dead-reckoning for non-blocking states ─────────────
                # Blocking states call _drive_timed/_turn_timed which update
                # the tracker internally; only update here for the fast ticks.
                if self.state in (State.LINE_FOLLOW, State.BALL_APPROACH):
                    self.tracker.update(self._left_duty, self._right_duty, dt)

                # ── Maintain loop rate ─────────────────────────────────────────
                elapsed   = time.time() - tick_start
                remaining = LOOP_DT - elapsed
                if remaining > 0:
                    time.sleep(remaining)

        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    # ─────────────────────────────────────────────────────────────────────────
    # Calibration / sensor readout mode
    # ─────────────────────────────────────────────────────────────────────────

    def calibrate_loop(self):
        """
        Print sensor values every 0.5 s.  Never drives the robot.
        Use this to verify IR readings, ultrasonic distances, and vision
        before a competition run.

        Output columns (enabled subsystems only):
          IR=NNN (dec)  — 3-bit integer; NNN = binary representation
          US=XX.X cm    — ultrasonic distance
          Ball=YES/no   — ball confirmed, pixel offset from centre, area
        """
        print("=" * 64)
        print("CALIBRATION MODE  —  sensors only, no driving")
        print("Ctrl-C to exit")
        print("=" * 64)

        if self.vision:
            self.vision.start()
            time.sleep(1.2)

        try:
            while True:
                parts = []

                if ENABLE_INFRARED and self.infrared:
                    ir = self.infrared.read_all_infrared()
                    parts.append(f"IR={ir:03b}({ir})")

                if ENABLE_ULTRASONIC and self.sonic:
                    dist = self.sonic.get_distance()
                    parts.append(f"US={dist:5.1f} cm")

                if ENABLE_VISION and self.vision:
                    b = self.vision.get()
                    parts.append(
                        f"Ball={'YES' if b.detected else 'no '}"
                        f"  offset={b.offset_x:+4d}  area={b.area:6.0f}"
                    )

                print("  |  ".join(parts) if parts else "(all subsystems disabled)")
                time.sleep(0.5)

        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    # ─────────────────────────────────────────────────────────────────────────
    # Shutdown
    # ─────────────────────────────────────────────────────────────────────────

    def shutdown(self):
        """
        Stop all actuators, park the arm, turn off LEDs, release hardware.
        Called automatically by run() and calibrate_loop() on exit.
        """
        print("\n[Robot] Shutting down ...")
        try:
            self._stop()
            self._arm_home()
            self._led_off()
        except Exception as e:
            print(f"[Robot] Shutdown warning: {e}")

        if self.vision:
            self.vision.stop()

        try:
            if self.sonic:
                self.sonic.close()
            if self.infrared:
                self.infrared.close()
            self.motor.close()
        except Exception as e:
            print(f"[Robot] Hardware close warning: {e}")

        print("[Robot] Shutdown complete")

    def _signal_handler(self, sig, frame):
        print(f"\n[Robot] Signal {sig} received — shutting down")
        raise KeyboardInterrupt


# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

def main():
    global ENABLE_INFRARED, ENABLE_ULTRASONIC, ENABLE_VISION

    parser = argparse.ArgumentParser(
        prog="competition.py",
        description=(
            "Freenove Tank — Autonomous Competition Controller\n"
            "PCB V2.0 / Raspberry Pi 5  •  Pi IP: 10.205.9.10"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
EXAMPLES
--------
  python3 competition.py
        Full competition: line follow + obstacle avoidance + ball pickup

  python3 competition.py --no-vision --no-ultrasonic
        IR line-follow only (test line tracking without other subsystems)

  python3 competition.py --no-infrared --no-ultrasonic
        Vision/ball only (test camera approach + pickup without driving a line)

  python3 competition.py --no-vision
        Line follow + obstacle avoidance, no ball detection

  python3 competition.py --no-infrared --no-vision
        Ultrasonic obstacle test only (robot creeps forward until obstacle)

  python3 competition.py --calibrate
        Print all sensor values every 0.5 s — DOES NOT DRIVE the robot.
        Use this to verify sensors before a competition run.

CALIBRATION CONSTANTS  (edit at top of file)
----------------------------------------------
  TURN_90_S    seconds for a 90-degree turn at duty ±1500
  FORWARD_MPS  metres/second at duty 2000
  WHEEL_BASE_M distance between tracks in metres
        """,
    )

    parser.add_argument(
        "--no-infrared",
        action="store_true",
        help="Disable IR sensor line-following",
    )
    parser.add_argument(
        "--no-ultrasonic",
        action="store_true",
        help="Disable ultrasonic obstacle avoidance",
    )
    parser.add_argument(
        "--no-vision",
        action="store_true",
        help="Disable camera red-ball detection",
    )
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="Print sensor readings only — robot does NOT drive",
    )

    args = parser.parse_args()

    # Apply CLI overrides to module-level flags
    if args.no_infrared:
        ENABLE_INFRARED = False
        print("[Config] IR line-following   DISABLED")
    if args.no_ultrasonic:
        ENABLE_ULTRASONIC = False
        print("[Config] Ultrasonic avoidance DISABLED")
    if args.no_vision:
        ENABLE_VISION = False
        print("[Config] Vision / ball        DISABLED")

    robot = CompetitionRobot()

    if args.calibrate:
        robot.calibrate_loop()
    else:
        robot.run()


if __name__ == "__main__":
    main()
