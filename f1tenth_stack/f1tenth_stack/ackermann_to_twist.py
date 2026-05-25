import math
import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import Twist, TwistStamped


class AckermannToTwist(Node):
    def __init__(self):
        super().__init__('ackermann_to_twist')

        # Declare parameters
        self.declare_parameter('ackermann_topic', 'ackermann_cmd')
        self.declare_parameter('twist_topic', 'cmd_vel')
        self.declare_parameter('wheelbase', 0.256)  # 0.256 (Traxxas 4-Tec 2.0), 0.33 (Traxxas Slash 4x4) are standard 1/10 scale approx (meters)
        self.declare_parameter('use_stamped_subscriber', True)
        self.declare_parameter('use_stamped_publisher', True)

        # Retrieve parameters
        self.ackermann_topic = self.get_parameter('ackermann_topic').value
        self.twist_topic = self.get_parameter('twist_topic').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.use_stamped_subscriber = self.get_parameter('use_stamped_subscriber').value
        self.use_stamped_publisher = self.get_parameter('use_stamped_publisher').value

        # Setup subscriber based on message type
        if self.use_stamped_subscriber:
            self.subscriber = self.create_subscription(
                AckermannDriveStamped,
                self.ackermann_topic,
                self._stamped_callback,
                10
            )
            self.get_logger().info(f"Subscribed to AckermannDriveStamped on '{self.ackermann_topic}'")
        else:
            self.subscriber = self.create_subscription(
                AckermannDrive,
                self.ackermann_topic,
                self._unstamped_callback,
                10
            )
            self.get_logger().info(f"Subscribed to AckermannDrive on '{self.ackermann_topic}'")
            
        # Setup publisher based on message type
        if self.use_stamped_publisher:
            self.publisher = self.create_publisher(TwistStamped, self.twist_topic, 10)
            self.get_logger().info(f"Publishing to TwistStamped on '{self.twist_topic}'")
        else:
            self.publisher = self.create_publisher(Twist, self.twist_topic, 10)
            self.get_logger().info(f"Publishing to Twist on '{self.twist_topic}'")

    def _stamped_callback(self, msg: AckermannDriveStamped):
        twist_base = self._convert_kinematics(msg.drive)
        if self.use_stamped_publisher:
            twist_msg = TwistStamped()
            twist_msg.header = msg.header
            twist_msg.twist = twist_base
            self.publisher.publish(twist_msg)
        else:
            self.publisher.publish(twist_base)

    def _unstamped_callback(self, msg: AckermannDrive):
        twist_base = self._convert_kinematics(msg)
        if self.use_stamped_publisher:
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.twist = twist_base
            self.publisher.publish(twist_msg)
        else:
            self.publisher.publish(twist_base)

    def _convert_kinematics(self, drive: AckermannDrive) -> Twist:
        twist = Twist()
        twist.linear.x = float(drive.speed)
        
        # Kinematic bicycle model: omega = (v * tan(delta)) / L
        if self.wheelbase > 0.0:
            twist.angular.z = float((drive.speed * math.tan(drive.steering_angle)) / self.wheelbase)
        else:
            # Fallback behavior if wheelbase isn't set properly to avoid ZeroDivisionError
            twist.angular.z = float(drive.steering_angle)
            
        return twist


def main(args=None):
    rclpy.init(args=args)
    node = AckermannToTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()