import sys
import time
import math
import struct

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import PointCloud2_

class Go2BasicTest:
    def __init__(self, network_interface: str = "eth0"):
        ChannelFactoryInitialize(0, network_interface)
        self.sport_client = SportClient()
        self.obstacle_client = ObstaclesAvoidClient()

        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()
        self.obstacle_client.Init()

        self.is_recording = False
        self.recorded_points = []
        self.msg_count = 0

        self.spin_start_time = 0.0
        self.current_yaw_rate = 0.0

        # Subscribe to LiDAR topic
        self.lidar_sub = ChannelSubscriber("rt/utlidar/cloud_deskewed", PointCloud2_)
        self.lidar_sub.Init(self._lidar_callback, 10)

    def _lidar_callback(self, msg: PointCloud2_):
        if not self.is_recording:
            return

        self.msg_count += 1
        
        # Calculate horizontal spin angle (Yaw)
        elapsed = time.time() - self.spin_start_time
        current_yaw = self.current_yaw_rate * elapsed
        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)

        # LiDAR pitch compensation (~45 degrees down on Go2 nose)
        pitch_angle = math.radians(45.0) 
        cos_p = math.cos(pitch_angle)
        sin_p = math.sin(pitch_angle)

        data_bytes = bytes(msg.data)
        point_step = msg.point_step
        num_points = len(data_bytes) // point_step

        for i in range(num_points):
            offset = i * point_step
            if offset + 12 <= len(data_bytes):
                x_raw, y_raw, z_raw = struct.unpack_from('<fff', data_bytes, offset)

                # 1. Skip NaN / Inf values
                if math.isnan(x_raw) or math.isnan(y_raw) or math.isnan(z_raw):
                    continue

                # 2. FILTER ZERO RETURNS: Discard points within 10cm of sensor origin
                dist_sq = x_raw * x_raw + y_raw * y_raw + z_raw * z_raw
                if dist_sq < 0.01:  # 0.1 meters squared
                    continue

                # Pitch Correction (LiDAR Frame -> Dog Body Frame)
                x_body = x_raw * cos_p + z_raw * sin_p
                y_body = y_raw
                z_body = -x_raw * sin_p + z_raw * cos_p

                # Yaw Rotation (Dog Body Frame -> World Frame)
                world_x = x_body * cos_yaw - y_body * sin_yaw
                world_y = x_body * sin_yaw + y_body * cos_yaw
                world_z = z_body

                self.recorded_points.append((world_x, world_y, world_z))

    def stand_up(self):
        print("1. Standing up...")
        self.sport_client.StopMove()
        time.sleep(0.2)
        self.sport_client.StandUp()
        time.sleep(2.5)
        self.sport_client.ClassicWalk(True)
        time.sleep(1.0)

    def stand_down(self):
        print("4. Lowering body...")
        self.sport_client.StopMove()
        time.sleep(0.2)
        self.sport_client.StandDown()
        time.sleep(1.5)

    def spin_and_scan(self, angle_degrees: float = 360.0, yaw_rate: float = 0.8):
        print(f"2. Spinning {angle_degrees}° at {yaw_rate} rad/s...")
        self.recorded_points.clear()
        self.msg_count = 0

        spin_duration = math.radians(abs(angle_degrees)) / abs(yaw_rate)
        self.current_yaw_rate = yaw_rate if angle_degrees >= 0 else -yaw_rate
        
        control_hz = 20
        dt = 1.0 / control_hz
        total_steps = int(spin_duration * control_hz)

        self.spin_start_time = time.time()
        self.is_recording = True

        for _ in range(total_steps):
            self.sport_client.Move(vx=0.0, vy=0.0, vyaw=self.current_yaw_rate)
            time.sleep(dt)

        self.is_recording = False
        self.sport_client.StopMove()
        print(f"3. Done. Received {self.msg_count} LiDAR packets ({len(self.recorded_points)} points).")

    def save_ascii_ply(self, filename: str = "scan_output.ply"):
        """Saves points into standard ASCII PLY format without Open3D."""
        if not self.recorded_points:
            print("No points recorded to save!")
            return

        print(f"Writing {len(self.recorded_points)} points to {filename}...")
        header = (
            "ply\n"
            "format ascii 1.0\n"
            f"element vertex {len(self.recorded_points)}\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "end_header\n"
        )

        with open(filename, "w") as f:
            f.write(header)
            for x, y, z in self.recorded_points:
                f.write(f"{x:.4f} {y:.4f} {z:.4f}\n")

        print(f"--> Saved scan to {filename}")

def main():
    net_interface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    robot = Go2BasicTest(net_interface)

    robot.stand_up()
    robot.spin_and_scan(angle_degrees=360.0, yaw_rate=0.8)
    robot.save_ascii_ply("scan_output.ply")
    robot.stand_down()

if __name__ == "__main__":
    main()