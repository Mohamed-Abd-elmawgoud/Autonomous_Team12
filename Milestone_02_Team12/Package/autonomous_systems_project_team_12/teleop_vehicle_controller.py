import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist  # Used for teleop commands

class VehicleTeleop(Node):
    def __init__(self):
        super().__init__('vehicle_teleop_node')

        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Publishers
        self.vel_pub = self.create_publisher(Float64, '/velocity', 10)
        self.steer_pub = self.create_publisher(Float64, '/steering_angle', 10)

        # Teleop subscriber
        self.teleop_sub = self.create_subscription(
            Twist,
            '/teleop_cmd',
            self.teleop_callback,
            10
        )

        self.latest_joint_state = None
        self.current_velocity = 0.0
        self.current_steering = 0.0

        # Timer: print joint states every 2 sec
        # self.print_timer = self.create_timer(2.0, self.print_joint_states)

        # Timer: publish commands at 10 Hz
        self.command_timer = self.create_timer(0.1, self.publish_commands)

        self.get_logger().info("Vehicle Teleop Node started (use /teleop_cmd)")

    def joint_callback(self, msg):
        self.latest_joint_state = msg

    def print_joint_states(self):
        if self.latest_joint_state:
            self.get_logger().info(f'Joint positions: {self.latest_joint_state.position}')
        else:
            self.get_logger().info('Waiting for joint states...')

    def teleop_callback(self, msg: Twist):
        # Linear.x -> forward/backward
        # Angular.z -> left/right steering
        self.current_velocity = msg.linear.x
        self.current_steering = msg.angular.z

    def publish_commands(self):
        vel_msg = Float64()
        vel_msg.data = self.current_velocity
        self.vel_pub.publish(vel_msg)

        steer_msg = Float64()
        steer_msg.data = self.current_steering
        self.steer_pub.publish(steer_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()