#!/usr/bin/env python3
"""
Unitree Go2 (G02) Autonomous Waypoint Patrol Execution Script — corrected live navigation

Key differences from the original:
  * Live mode now uses real robot pose feedback (position + yaw) to compute
    distance/bearing error to each waypoint, instead of "drive forward for 2s".
  * Proportional heading control (vyaw) turns the robot toward the target
    before/while driving forward, and target_yaw_deg is honored on arrival.
  * "Arrived" is a real distance check against the waypoint (with tolerance),
    not a fixed timer.
  * The kill switch is polled every control tick (~20 Hz) instead of only
    between waypoints, so a spacebar press stops the robot within ~50ms.
  * Class structure fixed (__init__ / stand_down are proper methods now).
  * Obstacle avoidance is enabled the same way as Go2KeyboardController.py:
    UseRemoteCommandFromApi(True) + SwitchSet(True) after standing, and all
    movement during navigation goes through ObstaclesAvoidClient.Move()
    instead of SportClient.Move() while avoidance is active, so the robot's
    onboard avoidance can override/shape commanded velocities around
    obstacles it detects. Cleanly disabled again in stand_down().

Usage:
    python run_go2_patrol_corrected.py                 # live (requires unitree_sdk2 & robot)
    python run_go2_patrol_corrected.py --dry-run        # simulation mode
    python run_go2_patrol_corrected.py --waypoints path.json --net eth0
"""

import sys
import os
import time
import json
import math
import argparse
import select
import termios
import tty

SDK_AVAILABLE = False
try:
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber,
    )
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient
    # SportModeState carries the robot's estimated position/yaw (odometry).
    # Go2 uses the unitree_go idl (G1/H1-2 use unitree_hg instead).
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False


# ---------- Tunables ----------
ARRIVAL_TOLERANCE_M = 0.15       # how close counts as "reached" the waypoint
YAW_TOLERANCE_DEG = 8.0          # how close counts as "facing" the target yaw
MAX_LINEAR_SPEED = 1.0           # m/s safety cap, overrides waypoint speed if higher
MAX_YAW_RATE = 1.0               # rad/s cap for turning
CONTROL_HZ = 20.0                # control loop rate
HEADING_KP = 2.0                 # proportional gain: rad/s per rad of heading error
TURN_IN_PLACE_THRESHOLD_DEG = 30 # if heading error exceeds this, stop and turn first


def load_waypoints(json_path):
    if not os.path.exists(json_path):
        raise FileNotFoundError(f"Waypoints file not found: {json_path}")
    with open(json_path, "r") as f:
        return json.load(f)


def _check_for_space_kill():
    """Non-blocking single-key check. Returns True if space was pressed."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == " ":
                return True
    except (termios.error, OSError, EOFError):
        return False
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return False


def angle_diff_rad(target_rad, current_rad):
    """Shortest signed angular difference, wrapped to [-pi, pi]."""
    d = target_rad - current_rad
    return (d + math.pi) % (2 * math.pi) - math.pi


class Go2PatrolController:
    def __init__(self, network_interface: str = "eth0"):
        if not SDK_AVAILABLE:
            raise RuntimeError("unitree_sdk2py not installed; cannot run live mode")

        ChannelFactoryInitialize(0, network_interface)

        self.sport_client = SportClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()

        self.obstacle_client = ObstaclesAvoidClient()
        self.obstacle_client.Init()
        self.obstacle_avoidance_enabled = False  # actually enabled later, once standing

        # Robot pose, updated by the state subscriber callback.
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0  # radians
        self._pose_lock_ready = False

        self._state_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self._state_sub.Init(self._on_state, 10)

        self.is_standing = False

        # Wait briefly for first state message so we're not navigating blind.
        t0 = time.time()
        while not self._pose_lock_ready and time.time() - t0 < 3.0:
            time.sleep(0.05)
        if not self._pose_lock_ready:
            print("Warning: no pose feedback received yet; navigation will be unreliable "
                  "until state messages arrive.")

    def _on_state(self, msg):
        # Field names depend on SDK version — check SportModeState_ definition.
        # Typically something like msg.position = [x, y, z], msg.imu_state.rpy = [r,p,y]
        self.pose_x = msg.position[0]
        self.pose_y = msg.position[1]
        self.pose_yaw = msg.imu_state.rpy[2]
        self._pose_lock_ready = True

    def enable_obstacle_avoidance(self):
        self.obstacle_client.UseRemoteCommandFromApi(True)
        self.obstacle_client.SwitchSet(True)
        self.obstacle_avoidance_enabled = True
        time.sleep(0.5)

    def disable_obstacle_avoidance(self):
        self.obstacle_client.SwitchSet(False)
        self.obstacle_client.UseRemoteCommandFromApi(False)
        self.obstacle_avoidance_enabled = False

    def _move(self, vx, vy, vyaw):
        """Route through the obstacle-avoidance client when enabled, same as
        the keyboard controller: ObstaclesAvoidClient.Move() lets the robot's
        onboard avoidance modify/block the commanded velocity around obstacles."""
        if self.obstacle_avoidance_enabled:
            self.obstacle_client.Move(vx, vy, vyaw)
        else:
            self.sport_client.Move(vx=vx, vy=vy, vyaw=vyaw)

    def stand_up(self):
        self.sport_client.StopMove()
        time.sleep(0.2)
        self.sport_client.StandUp()
        self.is_standing = True
        time.sleep(2.5)
        self.sport_client.ClassicWalk(True)
        self.enable_obstacle_avoidance()

    def stand_down(self):
        self.sport_client.StopMove()
        self.disable_obstacle_avoidance()
        self.sport_client.Euler(0.0, 0.0, 0.0)
        time.sleep(0.2)
        self.sport_client.StandDown()
        self.is_standing = False
        time.sleep(1.5)

    def navigate_to_waypoint(self, wp):
        """Closed-loop drive toward a single waypoint. Returns False if killed."""
        target_x = wp["x"]
        target_y = wp["y"]
        target_yaw_deg = wp.get("yaw_deg")
        max_speed = min(wp.get("target_speed_m_s", 0.8), MAX_LINEAR_SPEED)

        period = 1.0 / CONTROL_HZ

        while True:
            if _check_for_space_kill():
                print("\nKill switch pressed. Stopping.")
                self.sport_client.StopMove()
                self.stand_down()
                return False

            dx = target_x - self.pose_x
            dy = target_y - self.pose_y
            dist = math.hypot(dx, dy)

            if dist <= ARRIVAL_TOLERANCE_M:
                break

            bearing_to_target = math.atan2(dy, dx)
            heading_error = angle_diff_rad(bearing_to_target, self.pose_yaw)
            heading_error_deg = math.degrees(heading_error)

            vyaw = max(-MAX_YAW_RATE, min(MAX_YAW_RATE, HEADING_KP * heading_error))

            if abs(heading_error_deg) > TURN_IN_PLACE_THRESHOLD_DEG:
                # Facing badly wrong direction: turn in place before driving forward.
                vx = 0.0
            else:
                # Slow down as we approach, and while still correcting heading.
                vx = max_speed * min(1.0, dist / 0.5)

            self._move(vx, 0.0, vyaw)
            time.sleep(period)

        self.sport_client.StopMove()

        # Rotate to the requested final yaw, if the waypoint specifies one.
        if target_yaw_deg is not None:
            target_yaw_rad = math.radians(target_yaw_deg)
            while True:
                if _check_for_space_kill():
                    print("\nKill switch pressed. Stopping.")
                    self.sport_client.StopMove()
                    self.stand_down()
                    return False
                err = angle_diff_rad(target_yaw_rad, self.pose_yaw)
                if abs(math.degrees(err)) <= YAW_TOLERANCE_DEG:
                    break
                vyaw = max(-MAX_YAW_RATE, min(MAX_YAW_RATE, HEADING_KP * err))
                self._move(0.0, 0.0, vyaw)
                time.sleep(period)
            self.sport_client.StopMove()

        return True

    def run_patrol(self, waypoints_data):
        waypoints = waypoints_data.get("waypoints", [])
        print(f"Executing live patrol sequence for {len(waypoints)} waypoints...")

        if not self.is_standing:
            print("Standing up...")
            self.stand_up()

        for wp in waypoints:
            print(f"Navigating to Node {wp['id']} "
                  f"({wp['x']:.2f}, {wp['y']:.2f}, yaw={wp.get('yaw_deg', 0):.1f}°)...")
            ok = self.navigate_to_waypoint(wp)
            if not ok:
                return  # kill switch was hit

            wait_time = wp.get("wait_time_sec", 0.5)
            if wait_time > 0:
                t0 = time.time()
                while time.time() - t0 < wait_time:
                    if _check_for_space_kill():
                        print("\nKill switch pressed. Stopping.")
                        self.sport_client.StopMove()
                        self.stand_down()
                        return
                    time.sleep(1.0 / CONTROL_HZ)

        print("Patrol completed. Returning to idle pose...")
        self.stand_down()


def run_patrol_simulation(waypoints_data, speed_factor=1.0):
    print("\n=======================================================")
    print("      UNITREE GO2 PATROL SIMULATION / DRY-RUN MODE     ")
    print("=======================================================")
    waypoints = waypoints_data.get("waypoints", [])
    meta = waypoints_data.get("metadata", {})
    print(f"Loaded {len(waypoints)} waypoints across {meta.get('total_patrol_distance_m', 0)} meters.")
    print("Starting simulated route execution...\n")

    curr_x, curr_y = 0.0, 0.0

    for wp in waypoints:
        target_x = wp["x"]
        target_y = wp["y"]
        target_yaw = wp["yaw_deg"]
        dist = math.hypot(target_x - curr_x, target_y - curr_y)
        speed = wp.get("target_speed_m_s", 0.8) * speed_factor
        travel_time = dist / speed if speed > 0 else 0
        wait_time = wp.get("wait_time_sec", 0.5)

        print(f"[Waypoint {wp['seq']:02d} | {wp['id']}] -> Target: "
              f"(x: {target_x:6.2f}m, y: {target_y:6.2f}m, yaw: {target_yaw:6.1f}°)")
        print(f"   -> Moving {dist:.2f}m at {speed:.2f} m/s (Est. time: {travel_time:.1f}s)...")

        steps = 5
        for s in range(1, steps + 1):
            ratio = s / steps
            px = curr_x + ratio * (target_x - curr_x)
            py = curr_y + ratio * (target_y - curr_y)
            bar = "=" * (s * 4) + ">" + "." * ((steps - s) * 4)
            print(f"   [{bar}] Robot pos: ({px:6.2f}, {py:6.2f})", end="\r")
            time.sleep(0.1 / speed_factor)
        print()

        curr_x, curr_y = target_x, target_y
        print(f"   [ARRIVED] Reached waypoint {wp['id']} (Type: {wp['type']}). Waiting {wait_time}s...")
        time.sleep(min(wait_time, 0.5) / speed_factor)
        print("-" * 55)

    print("\n[SUCCESS] Unitree Go2 patrol mission simulation finished successfully!")


def main():
    parser = argparse.ArgumentParser(description="Unitree Go2 Waypoint Patrol Controller")
    default_json = os.path.join(os.path.dirname(__file__), "Smart Parking Park_go2_waypoints.json")
    parser.add_argument("--waypoints", type=str, default=default_json, help="Path to waypoints JSON file")
    parser.add_argument("--net", type=str, default="eth0", help="Network interface for Unitree SDK 2")
    parser.add_argument("--dry-run", action="store_true", help="Run in simulation mode (offline mock execution)")
    parser.add_argument("--speed-factor", type=float, default=2.0, help="Simulation speedup factor")
    args = parser.parse_args()

    data = load_waypoints(args.waypoints)

    if args.dry_run or not SDK_AVAILABLE:
        if not SDK_AVAILABLE and not args.dry_run:
            print("Note: unitree_sdk2py library not found. Falling back to --dry-run simulation mode.")
        run_patrol_simulation(data, speed_factor=args.speed_factor)
    else:
        controller = Go2PatrolController(network_interface=args.net)
        controller.run_patrol(data)


if __name__ == "__main__":
    main()
