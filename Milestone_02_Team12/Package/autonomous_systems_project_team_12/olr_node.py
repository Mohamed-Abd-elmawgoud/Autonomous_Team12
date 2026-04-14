import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from rcl_interfaces.msg import SetParametersResult


class OLRNode(Node):

    def __init__(self):
        super().__init__('olr_node')

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.velocity_pub = self.create_publisher(Float64, '/velocity',       10)
        self.steering_pub = self.create_publisher(Float64, '/steering_angle', 10)

        self.latest_joint_state = None

        # ── Read from ROS parameters ──────────────────────────────────────
        self.declare_parameter('velocity',       1.5)
        self.declare_parameter('steering_angle', 0.8)

        self.velocity       = self.get_parameter('velocity').value
        self.steering_angle = self.get_parameter('steering_angle').value

        self.get_logger().info(
            f'OLR Node started — velocity={self.velocity}  '
            f'steering={self.steering_angle}'
        )

        self.command_timer = self.create_timer(0.1, self.publish_commands)

        # ── Watch for parameter changes ───────────────────────────────────────
        self.add_on_set_parameters_callback(self._param_callback)

    def joint_callback(self, msg):
        self.latest_joint_state = msg


    def _param_callback(self, params):
        for param in params:
            if param.name == 'velocity':
                self.velocity = param.value
                self.get_logger().info(f'Velocity updated to {self.velocity}')
            elif param.name == 'steering_angle':
                self.steering_angle = param.value
                self.get_logger().info(f'Steering updated to {self.steering_angle}')
        return SetParametersResult(successful=True)

    def publish_commands(self):
        vel_msg = Float64()
        vel_msg.data = self.velocity
        self.velocity_pub.publish(vel_msg)

        steer_msg = Float64()
        steer_msg.data = self.steering_angle
        self.steering_pub.publish(steer_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OLRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()