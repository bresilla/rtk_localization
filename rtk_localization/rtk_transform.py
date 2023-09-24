import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from handy_msgs.srv import UTM

class Transformerr(Node):
    def __init__(self, args):
        super().__init__("rtk_transform")
        self.static_transform = Odometry()
        self.tf_dyna_broadcaster = TransformBroadcaster(self)
        self.tf_stat_broadcaster = StaticTransformBroadcaster(self)
        self.odom_sub = self.create_subscription(Odometry, "/rtk/odom", self.dyna_transform, 10)
        self.srv = self.create_service(UTM, '/rtk/map2odom', self.request_stat)
        self.timer = self.create_timer(10.0, self.stat_transform)
        self.stat_transform()

    def request_stat(self, request, response):
        self.static_transform = request.utm_pose
        response.message = "Static transform reseted"
        self.stat_transform()
        self.get_logger().info(f'STATIC TRANSFORM SET')
        return response

    def stat_transform(self):
        msg = self.static_transform
        stat_t = TransformStamped()
        stat_t.header.stamp = self.get_clock().now().to_msg()
        stat_t.header.frame_id = "map"
        stat_t.child_frame_id = "odom"
        stat_t.transform.translation.x = msg.pose.pose.position.x
        stat_t.transform.translation.y = msg.pose.pose.position.y
        stat_t.transform.translation.z = msg.pose.pose.position.z
        stat_t.transform.rotation.x = msg.pose.pose.orientation.x
        stat_t.transform.rotation.y = msg.pose.pose.orientation.y
        stat_t.transform.rotation.z = msg.pose.pose.orientation.z
        stat_t.transform.rotation.w = msg.pose.pose.orientation.w
        self.tf_stat_broadcaster.sendTransform(stat_t)

    def dyna_transform(self, msg):
        dyna_t = TransformStamped()
        dyna_t.header.stamp = msg.header.stamp
        dyna_t.header.frame_id = "odom"
        dyna_t.child_frame_id = "base_link"
        dyna_t.transform.translation.x = msg.pose.pose.position.x
        dyna_t.transform.translation.y = msg.pose.pose.position.y
        dyna_t.transform.translation.z = msg.pose.pose.position.z
        dyna_t.transform.rotation.x = msg.pose.pose.orientation.x
        dyna_t.transform.rotation.y = msg.pose.pose.orientation.y
        dyna_t.transform.rotation.z = msg.pose.pose.orientation.z
        dyna_t.transform.rotation.w = msg.pose.pose.orientation.w
        self.tf_dyna_broadcaster.sendTransform(dyna_t)

def main(args=None):
    rclpy.init(args=args)
    navfix = Transformerr(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

