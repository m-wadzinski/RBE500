import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import sympy as sp

class ForkinSubscriber(Node):

    def __init__(self):
        super().__init__('forkin_subscriber')
        self.subscription = self.create_subscription(Float32MultiArray, 'topic', self.listener_callback, 10)
        self.subscription  # Recommended by tutorial to prevent error

    def listener_callback(self, msg):
        l1,l2,l3 = sp.symbols("l1,l2,l3")
        DH_L1 = (0, l1, msg.data[0] + np.pi/2 , -np.pi/2)   
        DH_L2 = (l2, 0, msg.data[1], 0)
        DH_L3 = (l3, 0, msg.data[2], 0)
        A1 = dh2a(DH_L1)
        A2 = dh2a(DH_L2)
        A3 = dh2a(DH_L3)
        self.get_logger().info("\nInput:\n %s \nTransformation:\n %s" % (msg.data, str(np.matmul(np.matmul(A1, A2), A3))))


def main(args=None):
    rclpy.init(args=args)

    forkin_subscriber = ForkinSubscriber()

    rclpy.spin(forkin_subscriber)
    forkin_subscriber.destroy_node()
    rclpy.shutdown()


def dh2a(dh):
    a, d, theta, alpha = dh
    A = ([np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1])
    return A


if __name__ == '__main__':
    main()
