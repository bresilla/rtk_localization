import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from example_interfaces.srv import Trigger
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import message_filters


class RTKBeardist(Node):
    def __init__(self, args):
        super().__init__("rtk_odometry")
        self.get_logger().info('INITIALIZING RTK ODOMETRY')
        self.odom_odom = self.odom_map = Odometry()
        self.path = Path()
        self.path.header.frame_id = "map"

        self.srv = self.create_service(Trigger, '/rtk/set_reset', self.transforms)

        self.odom_pub = self.create_publisher(Odometry, "/rtk/odom", 10)
        self.odom_timer = self.create_timer(0.01, self.odom_callback)
        # self.map_pub = self.create_publisher(Odometry, "/rtk/map", 10) 
        # self.map_timer = self.create_timer(10.0, self.map_callback)
        self.path_pub = self.create_publisher(Path, "/rtk/path", 10)
        self.path_timer = self.create_timer(2.0, self.path_callback)

        # self._curr_sub = message_filters.Subscriber(self, Odometry, '/rtk/curr')
        # self._dot_sub = message_filters.Subscriber(self, Odometry, '/rtk/dot')
        # self._syn_pub = message_filters.ApproximateTimeSynchronizer(
        #     [self._curr_sub, self._dot_sub], 10, slop=10
        # )
        # self._syn_pub.registerCallback(self.sync_message)

        self.curr_sub = self.create_subscription(Odometry, "/rtk/curr", self.curr_sub_callback, 10)

    def sync_message(self, curr, dot):
        self.odom_odom = curr
        pose = PoseStamped()
        pose.header = self.odom_odom.header
        pose.pose.position.x = self.odom_odom.pose.pose.position.x
        pose.pose.position.y = self.odom_odom.pose.pose.position.y
        self.path.poses.append(pose)

    def curr_sub_callback(self, msg=None):
        self.odom_odom = msg
        pose = PoseStamped()
        pose.header = self.odom_odom.header
        pose.pose.position.x = self.odom_odom.pose.pose.position.x
        pose.pose.position.y = self.odom_odom.pose.pose.position.y
        self.path.poses.append(pose)

    # def map_callback(self, msg=None):
    #     # self.get_logger().info('PUBLISHING MAP')
    #     self.odom_map.header = self.odom_odom.header
    #     self.map_pub.publish(self.odom_map)

    def odom_callback(self, msg=None):
        # self.get_logger().info('PUBLISHING ODOM')
        self.odom_pub.publish(self.odom_odom)
    
    def path_callback(self, msg=None):
        # self.get_logger().info('PUBLISHING PATH')
        self.path.header = self.odom_odom.header
        self.path_pub.publish(self.path)

    def transforms(self, request, response):
        response.message = "TRANSFORMS SET"
        self.odom_callback()
        # self.map_callback()
        return response

def main(args=None):
    rclpy.init(args=args)
    navfix = RTKBeardist(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()


if __name__ == '__main__':
    main()