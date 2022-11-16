import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class ForwardSubscriber(Node):

    def __init__(self):
        super().__init__('forward_subscriber')
        self.subscription = self.create_subscription(Float32MultiArray, 'inputQ', self.listener_callback, 10)
        self.subscription  # Recommended by tutorial to prevent error

    def forward_calculation(self, msg):
        DH = ([1.0, 1.0, msg.data[0], 0.0],
              [1.0, 0.0, msg.data[1], 0.0],
              [0.0, msg.data[2], 0.0, 0.0])
        
        A = []
        for x in DH:
            a = dh2a(x)
            A.append(a)
            
        T = np.matmul(np.matmul(A[0], A[1]), A[2])
        o03 = (T[0][3], T[1][3], T[2][3])
        
        self.get_logger().info("\nInput:\n %s \nPose:\n %s" % (msg.data, str(o03)))


def main(args=None):
    rclpy.init(args=args)

    forward_subscriber = ForwardSubscriber()

    rclpy.spin(forward_subscriber)
    forward_subscriber.destroy_node()
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
