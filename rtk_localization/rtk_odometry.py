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

def distance(coord1, coord2):
    radius_earth = 6_367_449
    lat1, lon1 = map(math.radians, coord1)
    lat2, lon2 = map(math.radians, coord2)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = radius_earth * c
    return distance

def bearing(coord1, coord2):
    lat1, lon1 = math.radians(coord1[0]), math.radians(coord1[1])
    lat2, lon2 = math.radians(coord2[0]), math.radians(coord2[1])
    diffLong = lon2 - lon1
    x = math.sin(diffLong) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(diffLong))
    initial_bearing = math.atan2(x, y)
    initial_bearing = math.degrees(initial_bearing)
    bearing = (initial_bearing + 360) % 360
    bearing = math.radians(bearing)
    bearing = math.pi - bearing
    return bearing

class RTKBeardist(Node):
    def __init__(self, args):
        super().__init__("rtk_odometry")
        self.gps_sub = message_filters.Subscriber(self, NavSatFix, "/rtk/fix")
        self.dist_sub = message_filters.Subscriber(self, Float32Stamped, "/rtk/distance")
        self.rad_sub = message_filters.Subscriber(self, Float32Stamped, "/rtk/radians")
        self.deg_sub = message_filters.Subscriber(self, Float32Stamped, "/rtk/degrees")
        self.gps_sub = message_filters.ApproximateTimeSynchronizer(
            [
                self.gps_sub, 
                self.dist_sub, 
                self.rad_sub, 
                self.deg_sub
            ], 10, slop=10
        )
        self.gps_sub.registerCallback(self.message_callback)
        self.odom_pub = self.create_publisher(Odometry, "/rtk/odomm", 10)
        self.get_logger().info('MESSAGE CALLBACK')

    def message_callback(self, fix, dist, rad, deg):
        odom_msg = Odometry()
        odom_msg.header = fix.header
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = dist.data * math.cos(rad.data)
        odom_msg.pose.pose.position.y = dist.data * math.sin(rad.data)
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom_msg)
        odom_position = odom_msg

def main(args=None):
    rclpy.init(args=args)
    navfix = RTKBeardist(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()


if __name__ == '__main__':
    main()