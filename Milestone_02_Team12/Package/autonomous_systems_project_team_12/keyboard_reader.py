import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from pynput import keyboard


class keyboardReader(Node):
    def __init__(self):
        super().__init__('keyboard_reader')

        self.vel_pub = self.create_publisher(Float64, '/velocity', 10)
        self.steer_pub = self.create_publisher(Float64, '/steering_angle', 10)

        self.vel_value = 1.0
        self.steer_value = 1.0

        self.pressed_keys = set()

        # 50Hz instead of 10Hz
        self.create_timer(0.02, self.publish_commands)

        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

        self.get_logger().info("Teleop started — w/s to move, a/d to steer, q to quit")

    def on_press(self, key):
        try:
            if key.char in ('w', 's', 'a', 'd'):
                self.pressed_keys.add(key.char)
                self.publish_commands()  # Publish immediately on press
            elif key.char == 'q':
                self.get_logger().info("Exiting teleop...")
                rclpy.shutdown()
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            if key.char in ('w', 's', 'a', 'd'):
                self.pressed_keys.discard(key.char)
                self.publish_commands()  # Publish immediately on release
        except AttributeError:
            pass

    def publish_commands(self):
        velocity = 0.0
        if 'w' in self.pressed_keys:
            velocity += self.vel_value
        if 's' in self.pressed_keys:
            velocity -= self.vel_value

        steering = 0.0
        if 'a' in self.pressed_keys:
            steering += self.steer_value
        if 'd' in self.pressed_keys:
            steering -= self.steer_value

        vel_msg = Float64()
        vel_msg.data = velocity
        self.vel_pub.publish(vel_msg)

        steer_msg = Float64()
        steer_msg.data = steering
        self.steer_pub.publish(steer_msg)


def main(args=None):
    rclpy.init(args=args)
    node = keyboardReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()