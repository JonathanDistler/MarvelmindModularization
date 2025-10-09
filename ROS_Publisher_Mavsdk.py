#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped, AccelStamped
import asyncio
from mavsdk import System


class MavsdkCommandNode(Node):
    def __init__(self):
        super().__init__('mavsdk_command_node')

        # ROS 2 Publishers
        self.status_pub = self.create_publisher(String, 'drone_status', 10)
        self.gps_pub = self.create_publisher(NavSatFix, 'drone_gps', 10)
        self.alt_pub = self.create_publisher(Float32, 'drone_altitude', 10)
        self.battery_pub = self.create_publisher(Float32, 'drone_battery', 10)
        self.angle_vel_pub = self.create_publisher(Vector3Stamped, 'drone_angle_vel', 10)
        self.euler_angle_pub = self.create_publisher(Vector3Stamped, 'drone_euler', 10)
        self.accel_pub = self.create_publisher(AccelStamped, 'drone_accel', 10)

    async def run(self):
        drone = System()
        await drone.connect(system_address="udp://:14540")

        self.log("Waiting for connection...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                self.log("Connected to PX4")
                break

        # Wait for GPS health
        async for health in drone.telemetry.health():
            if health.is_global_position_ok:
                self.log("Global position OK")
                break

        # Start telemetry publishers concurrently
        asyncio.create_task(self.publish_position(drone))
        asyncio.create_task(self.publish_battery(drone))
        asyncio.create_task(self.publish_angle_vel_bod(drone))
        asyncio.create_task(self.publish_euler_angle(drone))
        asyncio.create_task(self.publish_accel_forward(drone))

        # Basic flight sequence
        self.log("Arming")
        await drone.action.arm()
        self.log("Taking off")
        await drone.action.takeoff()
        await asyncio.sleep(10)
        self.log("Landing")
        await drone.action.land()
        self.log("Finished mission")

    # ---------------------------------------------------------------------
    # Telemetry publishers
    # ---------------------------------------------------------------------

    async def publish_position(self, drone):
        async for pos in drone.telemetry.position():
            gps_msg = NavSatFix()
            gps_msg.latitude = pos.latitude_deg
            gps_msg.longitude = pos.longitude_deg
            gps_msg.altitude = pos.absolute_altitude_m
            self.gps_pub.publish(gps_msg)

            alt_msg = Float32()
            alt_msg.data = pos.relative_altitude_m
            self.alt_pub.publish(alt_msg)

    async def publish_battery(self, drone):
        async for batt in drone.telemetry.battery():
            msg = Float32()
            msg.data = batt.remaining_percent * 100.0
            self.battery_pub.publish(msg)

    # Updated to use imu()
    async def publish_angle_vel_bod(self, drone):
        async for imu in drone.telemetry.imu():
            msg = Vector3Stamped()
            msg.vector.x = imu.angular_velocity_frd.forward_rad_s
            msg.vector.y = imu.angular_velocity_frd.right_rad_s
            msg.vector.z = imu.angular_velocity_frd.down_rad_s
            self.angle_vel_pub.publish(msg)

    async def publish_euler_angle(self, drone):
        async for euler in drone.telemetry.attitude_euler():
            msg = Vector3Stamped()
            msg.vector.x = euler.roll_deg
            msg.vector.y = euler.pitch_deg
            msg.vector.z = euler.yaw_deg
            self.euler_angle_pub.publish(msg)

    # Updated to use imu()
    async def publish_accel_forward(self, drone):
        async for imu in drone.telemetry.imu():
            msg = AccelStamped()
            msg.accel.linear.x = imu.acceleration_frd.forward_m_s2
            msg.accel.linear.y = imu.acceleration_frd.right_m_s2
            msg.accel.linear.z = imu.acceleration_frd.down_m_s2
            self.accel_pub.publish(msg)

    # ---------------------------------------------------------------------
    # Logging helper
    # ---------------------------------------------------------------------
    def log(self, msg):
        self.get_logger().info(msg)
        self.status_pub.publish(String(data=msg))


# -------------------------------------------------------------------------
# Async main loop
# -------------------------------------------------------------------------
async def async_main():
    rclpy.init()
    node = MavsdkCommandNode()

    # Start MAVSDK loop
    await node.run()

    # Keep spinning ROS events concurrently
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            await asyncio.sleep(0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    asyncio.run(async_main())


if __name__ == '__main__':
    main()
