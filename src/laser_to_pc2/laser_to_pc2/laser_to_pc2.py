import rclpy
from rclpy.node import Node

from sensor_msgs_py import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import math


class LaserToPc2(Node):

    def __init__(self):
        super().__init__('laser_to_pc2')
        self.publisher_ = self.create_publisher(PointCloud2, '/toybox/pc2', 10)
        self.subscription = self.create_subscription(LaserScan, '/laser_controller/out', 
                                                      self.listener_callback, 10  )
        self.lp = lg.LaserProjection()

    def listener_callback(self, msg):
        pc2_msg = self.lp.projectLaser(msg)
        self.publisher_.publish(pc2_msg)

def main(args=None):
    rclpy.init(args=args)

    laser_to_pc2 = LaserToPc2()

    rclpy.spin(laser_to_pc2)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    laser_to_pc2.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
