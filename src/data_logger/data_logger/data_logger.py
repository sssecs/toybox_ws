import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import pickle


class DataLogger(Node):
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
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.record_switch = self.create_service(bool, 'record_switch', self.switch_callback)
        self.odom_data = Odometry
        self.p3d_data = Odometry
        self.record_switch = False

    def odom_subscriber_callback(self, msg):
        self.odom_data = msg

    def p3d_subscriber_callback(self, msg):
        self.p3d_data = msg

    def timer_callback(self):
        if self.record_switch:
            try:
                with open("/home/sykes/data.pickle", "wb") as f:
                    pickle.dump((self.p3d_data,self.odom_data), f, protocol=pickle.HIGHEST_PROTOCOL)
            except Exception as ex:
                print("Error during pickling object (Possibly unsupported):", ex)

    def switch_callback(self, request, response):
        self.record_switch = request
        

def main(args=None):
    rclpy.init(args=args)

    data_logger = DataLogger()

    rclpy.spin(data_logger)

    rclpy.shutdown()


if __name__ == '__main__':
    main()