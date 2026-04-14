import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class VehicleStateMonitor(Node):
    def __init__(self):
        super().__init__('vehicle_state_monitor')

        # Subscribers for velocity and steering
        self.create_subscription(Float64, '/velocity', self.velocity_callback, 10)
        self.create_subscription(Float64, '/steering_angle', self.steering_callback, 10)

        # Vehicle state
        self.velocity = 0.0
        self.steering = 0.0

        # Pose integration
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        # Vehicle parameter
        self.wheel_base = 0.35  # Adjust to your vehicle

        # Timer for updating pose (10 Hz)
        self.create_timer(0.1, self.update_pose)
        # Timer for printing every 2 seconds
        self.create_timer(2.0, self.print_state)

    def velocity_callback(self, msg):
        self.velocity = msg.data

    def steering_callback(self, msg):
        self.steering = msg.data

    def update_pose(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0.0:
            return

        # Compute angular velocity from steering
        if abs(self.steering) > 1e-3:
            turning_radius = self.wheel_base / math.tan(self.steering)
            angular_velocity = self.velocity / turning_radius
        else:
            angular_velocity = 0.0

        # Integrate yaw
        self.yaw += angular_velocity * dt

        # Integrate position
        self.x += self.velocity * math.cos(self.yaw) * dt
        self.y += self.velocity * math.sin(self.yaw) * dt

    def print_state(self):
        self.get_logger().info(
            f"Position: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f} rad | "
            f"Velocity: {self.velocity:.2f} m/s, Steering: {self.steering:.2f} rad"
        )


def main(args=None):
    rclpy.init(args=args)
    node = VehicleStateMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()