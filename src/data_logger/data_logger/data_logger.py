import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import pickle
from toybox_interfaces.srv import Switch
import numpy as np
import math



class DataLogger(Node):
    import math
 
    def yaw_from_quaternion(self,odom_msg):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        x = odom_msg._pose._pose.orientation.x
        y = odom_msg._pose._pose.orientation.y
        z = odom_msg._pose._pose.orientation.z
        w = odom_msg._pose._pose.orientation.w

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
    
        return yaw_z # in radians

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

        self.odom_record_list = []
        self.p3d_record_list = []
        self.pc2_record_list = []
        

    def odom_subscriber_callback(self, msg):
        self.odom_data = msg

    def p3d_subscriber_callback(self, msg):
        self.p3d_data = msg

    def pc2_subscriber_callback(self, msg):
        self.pc2_data = msg
        self.record()

    def record(self):
        if self.record_switch:
            field_names=[field.name for field in self.pc2_data.fields]
            point_cloud_data = list(pc2.read_points(self.pc2_data, skip_nans=True, field_names = field_names))

            pc2_list = [(x,y,z) for x,y,z,a,b in point_cloud_data ]
            
            odom_list = [self.odom_data._pose._pose.position.x, 
                         self.odom_data._pose._pose.position.y, 
                         self.yaw_from_quaternion(self.odom_data)]
            
            p3d_list = [self.p3d_data._pose._pose.position.x, 
                        self.p3d_data._pose._pose.position.y, 
                        self.yaw_from_quaternion(self.p3d_data)]
            
            self.odom_record_list.append(odom_list)
            self.p3d_record_list.append(p3d_list)
            self.pc2_record_list.append(pc2_list)


    def switch_callback(self, request, response):
        self.record_switch = request.switch_cmd
        response.switch_return = self.record_switch
        if not request.switch_cmd:
            try:
                with open("/home/sykes/toybox_ws/scrip/data/odom.pickle", "wb") as file:
                    pickle.dump(self.odom_record_list, file, protocol=pickle.HIGHEST_PROTOCOL)
                with open("/home/sykes/toybox_ws/scrip/data/p3d.pickle", "wb") as file:
                    pickle.dump(self.p3d_record_list, file, protocol=pickle.HIGHEST_PROTOCOL)
                with open("/home/sykes/toybox_ws/scrip/data/pc2.pickle", "wb") as file:
                    pickle.dump(self.pc2_record_list, file, protocol=pickle.HIGHEST_PROTOCOL)    
            except Exception as ex:
                print("Error during pickling object (Possibly unsupported):", ex)  
            self.odom_record_list = []
            self.p3d_record_list = []
            self.pc2_record_list = []
        return response

        

def main(args=None):
    rclpy.init(args=args)

    data_logger = DataLogger()

    rclpy.spin(data_logger)

    rclpy.shutdown()


if __name__ == '__main__':
    main()