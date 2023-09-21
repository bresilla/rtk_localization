import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class Transformerr(Node):
    def __init__(self, args):
        super().__init__("rtk_transform")
        self.tf_dyna_broadcaster = TransformBroadcaster(self)
        self.tf_stat_broadcaster = StaticTransformBroadcaster(self)
        self.odom_sub = self.create_subscription(Odometry, "/rtk/odom", self.dyna_transform, 10)

    def stat_transform(self):
        stat_t = TransformStamped()
        stat_t.header = self.get_clock().now().to_msg()
        stat_t.header.frame_id = "map"
        stat_t.child_frame_id = "odom"
        stat_t.transform.translation.x = 0.0
        stat_t.transform.translation.y = 0.0
        stat_t.transform.translation.z = 0.0
        stat_t.transform.rotation.x = 0.0
        stat_t.transform.rotation.y = 0.0
        stat_t.transform.rotation.z = 0.0
        stat_t.transform.rotation.w = 1.0
        self.tf_stat_broadcaster.sendTransform(stat_t)

    def dyna_transform(self, msg):
        dyna_t = TransformStamped()
        dyna_t.header = msg.header
        dyna_t.header.frame_id = "odom"
        dyna_t.child_frame_id = "base_link"
        dyna_t.transform.translation.x = msg.pose.pose.position.x
        dyna_t.transform.translation.y = msg.pose.pose.position.y
        dyna_t.transform.translation.z = 0.0
        dyna_t.transform.rotation.x = 0.0
        dyna_t.transform.rotation.y = 0.0
        dyna_t.transform.rotation.z = 0.0
        dyna_t.transform.rotation.w = 1.0
        self.tf_dyna_broadcaster.sendTransform(dyna_t)

def main(args=None):
    rclpy.init(args=args)
    navfix = Transformerr(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

