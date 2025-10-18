#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped, AccelStamped


class DroneListener(Node):
    def __init__(self):
        super().__init__('drone_listener')

        # Subscribers
        self.create_subscription(String, 'drone_status', self.status_cb, 10)
        self.create_subscription(NavSatFix, 'drone_gps', self.gps_cb, 10)
        self.create_subscription(Float32, 'drone_battery', self.batt_cb, 10)
        self.create_subscription(Float32, 'drone_altitude', self.alt_cb, 10)
        self.create_subscription(Vector3Stamped, 'drone_angle_vel', self.angle_vel_cb, 10)
        self.create_subscription(Vector3Stamped, 'drone_euler', self.euler_cb, 10)
        self.create_subscription(AccelStamped, 'drone_accel', self.accel_cb, 10)

    # ---------------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------------
    def status_cb(self, msg):
        self.get_logger().info(f"[STATUS] {msg.data}")

    def gps_cb(self, msg):
        self.get_logger().info(
            f"[GPS] Lat={msg.latitude:.6f}, Lon={msg.longitude:.6f}, Alt={msg.altitude:.1f} m"
        )

    def batt_cb(self, msg):
        self.get_logger().info(f"[BATT] {msg.data:.1f}%")

    def alt_cb(self, msg):
        self.get_logger().info(f"[ALT] {msg.data:.1f} m")

    def angle_vel_cb(self, msg):
        self.get_logger().info(
            f"[ANG_VEL] Roll={msg.vector.x:.3f} rad/s, Pitch={msg.vector.y:.3f} rad/s, Yaw={msg.vector.z:.3f} rad/s"
        )

    def euler_cb(self, msg):
        self.get_logger().info(
            f"[EULER] Roll={msg.vector.x:.2f}°, Pitch={msg.vector.y:.2f}°, Yaw={msg.vector.z:.2f}°"
        )

    def accel_cb(self, msg):
        a = msg.accel.linear
        self.get_logger().info(
            f"[ACCEL] Fwd={a.x:.2f} m/s², Right={a.y:.2f} m/s², Down={a.z:.2f} m/s²"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DroneListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
