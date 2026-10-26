#!/usr/bin/env python3
"""智能小车仿真 60 秒高清录制实时遥测数据记录器。"""

import sys
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class TelemetryLogger(Node):
    def __init__(self, log_path: str):
        super().__init__('telemetry_logger')
        self.log_file = open(log_path, 'w')
        header = f"{'时间(s)':<8} {'X坐标(m)':<10} {'Y坐标(m)':<10} {'线速(m/s)':<10} {'轮速角速':<10} {'IMU角速(rad/s)':<14}\n"
        self.log_file.write(header)
        self.log_file.flush()

        self.start_time = time.time()
        self.x, self.y, self.vx, self.wz = 0.0, 0.0, 0.0, 0.0
        self.imu_wz = 0.0

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.timer = self.create_timer(2.0, self.timer_cb)

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.vx = msg.twist.twist.linear.x
        self.wz = msg.twist.twist.angular.z

    def imu_cb(self, msg: Imu):
        self.imu_wz = msg.angular_velocity.z

    def timer_cb(self):
        elapsed = int(time.time() - self.start_time)
        line = f"{elapsed:<8d} {self.x:+7.3f}   {self.y:+7.3f}   {self.vx:+7.3f}   {self.wz:+7.3f}   {self.imu_wz:+7.3f}"
        self.log_file.write(line + "\n")
        self.log_file.flush()
        print(f"  [{elapsed:2d}s/60s] X={self.x:+6.3f} Y={self.y:+6.3f} Vx={self.vx:+5.2f} Wz={self.wz:+5.2f} IMU_wz={self.imu_wz:+6.3f}", flush=True)


def main():
    log_path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/sim_telemetry.log'
    rclpy.init()
    node = TelemetryLogger(log_path)
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        try:
            node.log_file.close()
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

