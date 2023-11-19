import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
# import time

class Transformerr(Node):
    def __init__(self, args):
        super().__init__("rtk_transform")
        self.ecef_msg = None
        self.is_transformed = False
        self.base_tf = TransformBroadcaster(self)
        self.odom_tf = StaticTransformBroadcaster(self)
        self.map_tf = StaticTransformBroadcaster(self)
        self.odom_sub = self.create_subscription(Odometry, "/fix/odom", self.base_transform, 10)
        self._ecef = self.create_subscription(Odometry, "/fix/datum/ecef", self.ecef_callback, 10)
        self.transform_timer = self.create_timer(10.0, self.ecef_timer)

    def ecef_callback(self, msg):
        self.ecef_msg = msg
        # time.sleep(1)
        self.destroy_subscription(self._ecef)
        self.map_transform()
        self.odom_transform()
        self.is_transformed = True
        self.get_logger().info('TRANSFORMS SET')

    def ecef_timer(self):
        if not self.is_transformed:
            self.get_logger().info('TRANSFORMS STILL NOT SET')
            return
        self.map_transform()
        self.odom_transform()

    def map_transform(self):
        stat_t = TransformStamped()
        stat_t.header.stamp = self.get_clock().now().to_msg()
        stat_t.header.frame_id = "world"
        stat_t.child_frame_id = "map"
        stat_t.transform.translation.x = self.ecef_msg.pose.pose.position.x
        stat_t.transform.translation.y = self.ecef_msg.pose.pose.position.y
        stat_t.transform.translation.z = self.ecef_msg.pose.pose.position.z
        stat_t.transform.rotation.x = 0.0
        stat_t.transform.rotation.y = 0.0
        stat_t.transform.rotation.z = 0.0
        stat_t.transform.rotation.w = 1.0
        self.map_tf.sendTransform(stat_t)

    def odom_transform(self):
        stat_t = TransformStamped()
        stat_t.header.stamp = self.get_clock().now().to_msg()
        stat_t.header.frame_id = "map"
        stat_t.child_frame_id = "odom"
        stat_t.transform.translation.x = 0.0
        stat_t.transform.translation.y = 0.0
        stat_t.transform.translation.z = 0.0
        stat_t.transform.rotation.x = 0.0
        stat_t.transform.rotation.y = 0.0
        stat_t.transform.rotation.z = 0.0
        stat_t.transform.rotation.w = 1.0
        self.odom_tf.sendTransform(stat_t)

    def base_transform(self, msg):
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
        self.base_tf.sendTransform(dyna_t)

def main(args=None):
    rclpy.init(args=args)
    navfix = Transformerr(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

