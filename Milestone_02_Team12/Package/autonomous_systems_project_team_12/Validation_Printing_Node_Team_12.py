import rclpy
from rclpy.node import Node


class ValidationPrintingNodeTeam12(Node):

    def __init__(self):
        super().__init__('validation_printing_node_team_12')

        # Create a timer that triggers every 2 seconds
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello world, we are team 12")


def main(args=None):
    rclpy.init(args=args)

    node = ValidationPrintingNodeTeam12()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()