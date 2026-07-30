#!/usr/bin/env python3
"""
Unitree Go2 (G02) Autonomous Waypoint Patrol Execution Script

This script loads the exported waypoints JSON file and commands a Unitree Go2 robot dog
to navigate sequentially through all patrol points using `unitree_sdk2` (SportClient).

Usage:
    python run_go2_patrol.py                     # Run live (requires unitree_sdk2 & robot connection)
    python run_go2_patrol.py --dry-run           # Run in simulation mode (offline / mock test)
    python run_go2_patrol.py --waypoints path.json --net eth0
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
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient

# Try importing unitree_sdk2
SDK_AVAILABLE = False
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False

def __init__(self, network_interface: str = "eth0"):#change to 'lo'/'eth0' for local or connected
        self.sport_client = SportClient()
        self.obstacle_client = ObstaclesAvoidClient()
 #setup sport client and obi avoid

        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()
        self.obstacle_client.Init()

        self.is_standing = True #may not be true on startup
        self.pressed_keys = set()
        self.obstacle_avoidance_enabled = False

def load_waypoints(json_path):
    if not os.path.exists(json_path):
        raise FileNotFoundError(f"Waypoints file not found: {json_path}")
    with open(json_path, 'r') as f:
        return json.load(f)

def stand_down(self):
        self.pressed_keys.clear()
        self.sport_client.StopMove()
        self.sport_client.Euler(0.0, 0.0, 0.0)
        time.sleep(0.2)
        self.sport_client.StandDown()
        self.is_standing = False
        time.sleep(1.5)

def run_patrol_simulation(waypoints_data, speed_factor=1.0):
    print("\n=======================================================")
    print("      UNITREE GO2 PATROL SIMULATION / DRY-RUN MODE     ")
    print("=======================================================")
    waypoints = waypoints_data.get('waypoints', [])
    meta = waypoints_data.get('metadata', {})
    print(f"Loaded {len(waypoints)} waypoints across {meta.get('total_patrol_distance_m', 0)} meters.")
    print("Starting simulated route execution...\n")

    curr_x, curr_y = 0.0, 0.0

    for wp in waypoints:
        target_x = wp['x']
        target_y = wp['y']
        target_yaw = wp['yaw_deg']
        dist = math.hypot(target_x - curr_x, target_y - curr_y)
        speed = wp.get('target_speed_m_s', 0.8) * speed_factor

        travel_time = dist / speed if speed > 0 else 0
        wait_time = wp.get('wait_time_sec', 0.5)

        print(f"[Waypoint {wp['seq']:02d} | {wp['id']}] -> Target: (x: {target_x:6.2f}m, y: {target_y:6.2f}m, yaw: {target_yaw:6.1f}°)")
        print(f"   -> Moving {dist:.2f}m at {speed:.2f} m/s (Est. time: {travel_time:.1f}s)...")

        # Simulate movement steps
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


def _check_for_space_kill(sport_client, timeout=0.05):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        if select.select([sys.stdin], [], [], timeout)[0]:
            key = sys.stdin.read(1)
            if key == ' ':
                print("\nKill switch pressed. Stopping patrol and sitting down.")
                sport_client.StopMove()
                sport_client.StandDown()
                return True
    except (termios.error, OSError, EOFError):
        return False
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return False


def run_patrol_live(waypoints_data, net_interface="eth0"):
    if not SDK_AVAILABLE:
        print("Error: unitree_sdk2py is not installed on this system.")
        print("Please install unitree_sdk2py or run with --dry-run")
        sys.exit(1)

    print(f"Initializing Unitree Go2 SDK 2 on network interface: {net_interface}...")
    ChannelFactoryInitialize(0, net_interface)

    sport_client = SportClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    print("Unlocking robot motion (Standing up)...")
    sport_client.StandUp()
    time.sleep(2.0)

    waypoints = waypoints_data.get('waypoints', [])
    print(f"Executing live patrol sequence for {len(waypoints)} waypoints...")

    for wp in waypoints:
        target_x = wp['x']
        target_y = wp['y']
        speed = wp.get('target_speed_m_s', 0.8)
        wait_time = wp.get('wait_time_sec', 0.5)

        print(f"Navigating to Node {wp['id']} ({target_x:.2f}, {target_y:.2f})...")
        sport_client.Move(vx=speed, vy=0.0, vyaw=0.0)

        start_time = time.time()
        while time.time() - start_time < 2.0:
            if _check_for_space_kill(sport_client):
                return
            time.sleep(0.05)

        if wait_time > 0:
            sport_client.StopMove()
            if _check_for_space_kill(sport_client):
                return
            time.sleep(wait_time)
            if _check_for_space_kill(sport_client):
                return

    print("Patrol completed. Returning to idle pose...")
    sport_client.StandDown()


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
        run_patrol_live(data, net_interface=args.net)


if __name__ == "__main__":
    main()
