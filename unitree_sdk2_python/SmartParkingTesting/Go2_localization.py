import sys
import time
import math
import numpy as np
import open3d as o3d

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import PointCloud2_

class Go2Localization:
    def __init__(self, network_interface: str = "eth0"):
        ChannelFactoryInitialize(0, network_interface)
        self.sport_client = SportClient()
        self.obstacle_client = ObstaclesAvoidClient()

        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()
        self.obstacle_client.Init()

        self.is_standing = False
        self.obstacle_avoidance_enabled = False

        self.spin_start_time = 0.0
        self.current_yaw_rate = 0.0

        self.is_recording = False
        self.recorded_point_buffers = []

        # Subscribe to onboard LiDAR stream via DDS
        self.lidar_sub = ChannelSubscriber("rt/utlidar/cloud_deskewed", PointCloud2_)
        self.lidar_sub.Init(self._lidar_callback, 10)

    def _lidar_callback(self, msg: PointCloud2_):
        """Processes raw LiDAR bytes and transforms points to global frame based on estimated yaw."""
        if not self.is_recording:
            return

        try:
            raw_data = np.frombuffer(msg.data, dtype=np.float32)
            floats_per_point = msg.point_step // 4
            if floats_per_point >= 3:
                points = raw_data.reshape(-1, floats_per_point)[:, :3]
                valid_mask = np.isfinite(points).all(axis=1)
                pts = points[valid_mask]

                # Estimate current yaw rotation angle from elapsed spin time
                elapsed = time.time() - self.spin_start_time
                current_yaw = self.current_yaw_rate * elapsed

                # Rotate local body points into static world frame
                cos_y = math.cos(current_yaw)
                sin_y = math.sin(current_yaw)
                rot_matrix = np.array([
                    [cos_y, -sin_y, 0],
                    [sin_y,  cos_y, 0],
                    [    0,      0, 1]
                ], dtype=np.float32)

                transformed_pts = pts @ rot_matrix.T
                self.recorded_point_buffers.append(transformed_pts)
        except Exception:
            pass

    def enable_obstacle_avoidance(self):
        self.obstacle_client.UseRemoteCommandFromApi(True)
        self.obstacle_client.SwitchSet(True)
        self.obstacle_avoidance_enabled = True
        time.sleep(0.5)

    def disable_obstacle_avoidance(self):
        if self.obstacle_avoidance_enabled:
            self.obstacle_client.SwitchSet(False)
            self.obstacle_client.UseRemoteCommandFromApi(False)
            self.obstacle_avoidance_enabled = False

    def _move(self, vx: float, vy: float, vyaw: float):
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
        self.stop()
        self.disable_obstacle_avoidance()
        time.sleep(0.2)
        self.sport_client.StandDown()
        self.is_standing = False
        time.sleep(1.5)

    def stop(self, wait_time: float = 1.0):
        if self.obstacle_avoidance_enabled:
            self.obstacle_client.Move(0.0, 0.0, 0.0)
        else:
            self.sport_client.StopMove()
        time.sleep(wait_time)

    def spin(self, angle_degrees: float = 360.0, yaw_rate: float = 0.8):
        print(f"2. Performing {angle_degrees}° localization scan at {yaw_rate} rad/s...")

        self.recorded_point_buffers.clear()

        angle_rad = math.radians(abs(angle_degrees))
        spin_duration = angle_rad / abs(yaw_rate)

        direction = 1.0 if angle_degrees >= 0 else -1.0
        target_yaw_rate = direction * abs(yaw_rate)

        control_hz = 20
        dt = 1.0 / control_hz
        total_steps = int(spin_duration * control_hz)

        
        self.current_yaw_rate = target_yaw_rate
        self.spin_start_time = time.time()
        self.is_recording = True

        # Stream velocity command to maintain watchdog heartbeat
        for _ in range(total_steps):
            self._move(0.0, 0.0, target_yaw_rate)
            time.sleep(dt)

        self.is_recording = False
        self.stop()

    def export_to_ply(self, output_filename: str = "scan_output.ply"):
        if not self.recorded_point_buffers:
            print("Error: No LiDAR data captured!")
            return

        print("Processing captured point clouds...")
        full_cloud = np.vstack(self.recorded_point_buffers)
        print(f"Total points collected: {len(full_cloud)}")

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(full_cloud)

        # Downsample duplicate points from overlapping scan sweeps
        pcd = pcd.voxel_downsample(voxel_size=0.02)

        o3d.io.write_point_cloud(output_filename, pcd)
        print(f"--> Saved scan to file: {output_filename}")

    def execute_sequence(self, output_filename: str = "scan_output.ply", angle_degrees: float = 360.0):
        print("--- Starting Spin Sequence ---")
        self.stand_up()
        self.spin(angle_degrees=angle_degrees, yaw_rate=0.8)
        self.export_to_ply(output_filename)
        self.stand_down()
        print("--- Sequence Complete ---")

def main():
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} <network_interface>")
        print("Example: python3 test_lidar_spin.py eth0")
        sys.exit(1)

    net_interface = sys.argv[1]
    
    node = Go2Localization(net_interface)
    node.execute_sequence(output_filename="scan_output.ply", angle_degrees=360.0)

if __name__ == "__main__":
    main()