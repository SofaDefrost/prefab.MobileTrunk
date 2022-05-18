# coding: utf8
#!/usr/bin/env python3
import std_msgs
from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.node import Node
import sys

class MinimalPublisher(Node):

    def __init__(self, arg):
        super().__init__('summit_xl_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/summit_xl_control/cmd_vel', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.linear_vel = arg[0]
        self.angular_vel = arg[1]

    def timer_callback(self):

        msg = Float32MultiArray(layout=std_msgs.msg.MultiArrayLayout( data_offset=0), data=[self.linear_vel,0.,
                                                                        0.,0.,0.,self.angular_vel])
        self.publisher_.publish(msg)


def main(args):
    vel = [float(args[1]) ,float(args[2])]
    rclpy.init()

    minimal_publisher = MinimalPublisher(vel)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    print(len(sys.argv))

    main(sys.argv)