import math
import rclpy
from sensor_msgs.msg import NavSatFix, Imu
from rclpy.node import Node
from nav_msgs.msg import Odometry
from example_interfaces.srv import Trigger
from handy_msgs.msg import Float32Stamped
from rclpy.executors import MultiThreadedExecutor
from handy_msgs.srv import WGS, UTM
import time
import message_filters

class RTKBeardist(Node):
    def __init__(self, args):
        super().__init__("rtk_odometry")
        self.get_logger().info('INITIALIZING RTK ODOMETRY')
        self.odom_odom = self.odom_map = Odometry()

        self.odom_pub = self.create_publisher(Odometry, "/rtk/odom", 10)
        self.map_pub = self.create_publisher(Odometry, "/rtk/map", 10)

        self.curr_sub = message_filters.Subscriber(self, Odometry, "/rtk/curr")
        self.prev_sub = message_filters.Subscriber(self, Odometry, "/rtk/prev")
        self.dist_sub = message_filters.Subscriber(self, Float32Stamped, "/rtk/distance")
        self.bear_sub = message_filters.Subscriber(self, Float32Stamped, "/rtk/radians")

        self.sub = message_filters.TimeSynchronizer([self.curr_sub, self.dist_sub, self.bear_sub, self.prev_sub], 10)
        self.sub.registerCallback(self.sync_message)

        self.map_timer = self.create_timer(10.0, self.map_callback)
        self.odom_timer = self.create_timer(1.0, self.odom_callback)

        self.srv = self.create_service(Trigger, '/rtk/transforms', self.transforms)

    def sync_message(self, curr, dist, bear, prev):
        self.odom_odom.header = curr.header
        self.odom_odom.pose.pose.position.x = dist.data * math.cos(bear.data)
        self.odom_odom.pose.pose.position.y = dist.data * math.sin(bear.data)
        self.odom_odom.pose.pose.position.z = 0.0
        self.odom_odom.pose.pose.orientation.x = 0.0
        self.odom_odom.pose.pose.orientation.y = 0.0
        self.odom_odom.pose.pose.orientation.z = 0.0
        self.odom_odom.pose.pose.orientation.w = 1.0

    def map_callback(self, msg=None):
        self.odom_map.header = self.odom_odom.header
        self.map_pub.publish(self.odom_map)

    def odom_callback(self, msg=None):
        self.odom_pub.publish(self.odom_odom)


    def transforms(self, request, response):
        response.message = "TRANSFORMS SET"
        self.odom_callback()
        self.map_callback()
        return response


def main(args=None):
    rclpy.init(args=args)
    navfix = RTKBeardist(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()


if __name__ == '__main__':
    main()