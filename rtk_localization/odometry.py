import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from handy_msgs.msg import Float32Stamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import message_filters

def axangle2quat(vector, theta, is_normalized=False):
    vector = np.array(vector)
    if not is_normalized:
        vector = vector / np.sqrt(np.dot(vector, vector))
    t2 = theta / 2.0
    st2 = np.sin(t2)
    return np.concatenate(([np.cos(t2)],vector * st2))

def quat2axangle(q):
    quat = np.asarray(quat)
    Nq = np.sum(quat ** 2)
    if not np.isfinite(Nq):
        return np.array([1.0, 0, 0]), float('nan')
    if identity_thresh is None:
        try:
            identity_thresh = np.finfo(Nq.type).eps * 3
        except (AttributeError, ValueError):
            identity_thresh = np.finfo(np.float64).eps * 3
    if Nq < np.finfo(np.float64).eps ** 2:
        return np.array([1.0, 0, 0]), 0.0
    if Nq != 1:
        s = np.sqrt(Nq)
        quat = quat / s
    xyz = quat[1:]
    len2 = np.sum(xyz ** 2)
    if len2 < identity_thresh ** 2:
        return np.array([1.0, 0, 0]), 0.0
    theta = 2 * np.acos(max(min(quat[0], 1), -1))
    return  xyz / np.sqrt(len2), theta

class MyNode(Node):
    def __init__(self, args):
        super().__init__("enu_odom")
        self.get_logger().info('INITIALIZING ENU ODOMETRY')
        self.reset_path = False
        self.odom_odom = self.odom_map = Odometry()
        self.path = Path()
        self.path.header.frame_id = "map"

        self.odom_pub = self.create_publisher(Odometry, "/fix/odom", 10)
        self.odom_timer = self.create_timer(0.01, self.odom_callback)
        self.path_pub = self.create_publisher(Path, "/fix/path", 10)
        self.path_timer = self.create_timer(1.0, self.path_callback)

        self._enu_sub = message_filters.Subscriber(self, Odometry, "/fix/enu")
        self._dot_sub = message_filters.Subscriber(self, Float32Stamped, '/fix/rad')
        self._syn_pub = message_filters.ApproximateTimeSynchronizer(
            [self._enu_sub, self._dot_sub], 10, slop=10
        )
        self._syn_pub.registerCallback(self.sync_message)

    def sync_message(self, enu, rad):
        self.odom_odom = enu
        # qw, qx, qy, qz = euler_to_quaternion(0, 0, rad.data)
        qw, qx, qy, qz = axangle2quat([0, 0, 1], -(rad.data), is_normalized=True)
        self.odom_odom.pose.pose.orientation.w = qw
        self.odom_odom.pose.pose.orientation.x = qx
        self.odom_odom.pose.pose.orientation.y = qy
        self.odom_odom.pose.pose.orientation.z = qz
        # self.get_logger().info(f"angle: {quat2axangle([qw, qx, qy, qz])[2]}")
        pose = PoseStamped()
        pose.header = self.odom_odom.header
        pose.pose = self.odom_odom.pose.pose
        self.path.poses.append(pose)
        if len(self.path.poses) > 100 and self.reset_path:
            self.path.poses.pop(0)

    def odom_callback(self, msg=None):
        # self.get_logger().info('PUBLISHING ODOM')
        self.odom_pub.publish(self.odom_odom)
    
    def path_callback(self, msg=None):
        # self.get_logger().info('PUBLISHING PATH')
        # self.path.header = self.odom_odom.header
        self.path_pub.publish(self.path)

    def transforms(self, request, response):
        response.message = "TRANSFORMS SET"
        self.odom_callback()
        # self.map_callback()
        return response

def main(args=None):
    rclpy.init(args=args)
    navfix = MyNode(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()


if __name__ == '__main__':
    main()