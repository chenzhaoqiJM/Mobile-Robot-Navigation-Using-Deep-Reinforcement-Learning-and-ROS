import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class WheelVelocityPublisher(Node):
    def __init__(self):
        super().__init__('wheel_velocity_publisher')
        self.pub = self.create_publisher(Float64MultiArray, '/my_velocity_controller/commands', 10)
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.publish_velocity)

    def publish_velocity(self):
        msg = Float64MultiArray()
        msg.data = [1.0, 1.0, 1.0]  # 对应三个轮子的速度
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = WheelVelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
