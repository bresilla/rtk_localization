import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from example_interfaces.srv import Trigger
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def quaternion_to_euler(q):
    (x, y, z, w) = (q.x, q.y, q.z, q.w)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    # return [yaw, pitch, roll]
    return yaw, math.degrees(yaw)

class RTKBeardist(Node):
    def __init__(self, args):
        super().__init__("rtk_odometry")
        self.get_logger().info('INITIALIZING RTK ODOMETRY')
        self.odom_odom = self.odom_map = Odometry()
        self.prev_val = 0.0
        self.path = Path()

        self.odom_pub = self.create_publisher(Odometry, "/rtk/odom", 10)
        self.odom_timer = self.create_timer(0.01, self.odom_callback)
        self.map_pub = self.create_publisher(Odometry, "/rtk/map", 10)
        self.map_timer = self.create_timer(10.0, self.map_callback)
        self.path_pub = self.create_publisher(Path, "/rtk/path", 10)
        self.path_timer = self.create_timer(2.0, self.path_callback)


        self.curr_sub = self.create_subscription(Odometry, "/rtk/curr", self.sync_message, 10)
        self.srv = self.create_service(Trigger, '/rtk/transforms', self.transforms)

    def sync_message(self, msg=None):
        rad, deg = quaternion_to_euler(msg.pose.pose.orientation)
        if 135 < abs(self.prev_val - deg) < 45: return
        self.odom_odom = msg
        self.prev_val = deg

        pose = PoseStamped()
        pose.header = self.odom_odom.header
        pose.pose.position.x = self.odom_odom.pose.pose.position.x
        pose.pose.position.y = self.odom_odom.pose.pose.position.y
        self.path.poses.append(pose)

    def map_callback(self, msg=None):
        # self.get_logger().info('PUBLISHING MAP')
        self.odom_map.header = self.odom_odom.header
        self.map_pub.publish(self.odom_map)

    def odom_callback(self, msg=None):
        # self.get_logger().info('PUBLISHING ODOM')
        self.odom_pub.publish(self.odom_odom)
    
    def path_callback(self, msg=None):
        # self.get_logger().info('PUBLISHING PATH')
        self.path.header = self.odom_odom.header
        self.path.header.frame_id = "map"
        self.path_pub.publish(self.path)

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