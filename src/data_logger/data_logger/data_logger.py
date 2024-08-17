import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import pickle
from toybox_interfaces.srv import Switch
import numpy as np
import open3d




class DataLogger(Node):
    def convertCloudFromRosToOpen3d(self,ros_cloud):
        
        # Get cloud data from ros_cloud
        field_names=[field.name for field in ros_cloud.fields]
        cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

        # Check empty
        open3d_cloud = open3d.geometry.PointCloud()
        if len(cloud_data)==0:
            print("Converting an empty cloud")
            return None

        # Set open3d_cloud
        xyz = [(x,y,z) for x,y,z,a,b in cloud_data ] # get xyz
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

        # return
        return open3d_cloud
    def __init__(self):
        super().__init__('data_logger')
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_subscriber_callback,
            10)
        self.p3d_subscriber = self.create_subscription(
            Odometry,
            '/p3d/odom_p3d',
            self.p3d_subscriber_callback,
            10)
        self.pc2_subscriber = self.create_subscription(PointCloud2, '/toybox/pc2', 
                                                      self.pc2_subscriber_callback, 10  )
        
        self.record_switch = self.create_service(Switch, 'record_switch', self.switch_callback)
        self.odom_data = Odometry
        self.p3d_data = Odometry
        self.pc2_data = PointCloud2
        self.record_switch = False

    def odom_subscriber_callback(self, msg):
        self.odom_data = msg

    def p3d_subscriber_callback(self, msg):
        self.p3d_data = msg

    def pc2_subscriber_callback(self, msg):
        self.pc2_data = msg
        self.record()

    def record(self):
        if self.record_switch:
            open_3d_data = self.convertCloudFromRosToOpen3d(self.pc2_data)
            try:
                with open("/home/sykes/data.pickle", "wb") as f:
                    pickle.dump((self.p3d_data,self.odom_data,open_3d_data), f, protocol=pickle.HIGHEST_PROTOCOL)
            except Exception as ex:
                print("Error during pickling object (Possibly unsupported):", ex)

    def switch_callback(self, request, response):
        self.record_switch = request.switch_cmd
        response.switch_return = self.record_switch
        return response

        

def main(args=None):
    rclpy.init(args=args)

    data_logger = DataLogger()

    rclpy.spin(data_logger)

    rclpy.shutdown()


if __name__ == '__main__':
    main()