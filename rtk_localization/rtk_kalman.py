import math
import rclpy
from sensor_msgs.msg import NavSatFix
from rclpy.node import Node
from handy_msgs.msg import Float32Stamped
from nav_msgs.msg import Odometry
from handy_msgs.srv import WGS
from example_interfaces.srv import Trigger
import numpy as np
import transforms3d.quaternions as quaternions
import message_filters


class KallmanFilter():
    def __init__(self, estimated_state=0.0, estimated_error=1.0, process_noise=0.01, measurement_noise=0.1):
        self.estimated_state = estimated_state  # Initial state estimate
        self.estimated_error = estimated_error  # Initial error estimate
        self.process_noise = process_noise   # Process noise (adjust as needed)
        self.measurement_noise = measurement_noise # Measurement noise (adjust as needed)
    
    def step(self, value):
        # Prediction step
        predicted_state = self.estimated_state
        predicted_error = self.estimated_error + self.process_noise
        # Update step
        kalman_gain = predicted_error / (predicted_error + self.measurement_noise)
        self.estimated_state = predicted_state + kalman_gain * (value - predicted_state)
        self.estimated_error = (1 - kalman_gain) * predicted_error
        return self.estimated_state

class RTKBeardist(Node):
    def __init__(self, args):
        super().__init__("rtk_kalman")
        self.get_logger().info('INITIALIZING RTK KALMAN')
        self.pre_odom = self.cur_odom = Odometry()
        self.quaternion = [1.0, 0.0, 0.0, 0.0]
        self.kf = KallmanFilter(0.0, 1.0, 0.009, 0.2)

        self._gps_sub = message_filters.Subscriber(self, NavSatFix, '/rtk/fix')
        self._dot_sub = message_filters.Subscriber(self, NavSatFix, '/rtk/dot')
        self._dist_sub = message_filters.Subscriber(self, Float32Stamped, '/rtk/distance')
        self._bear_sub = message_filters.Subscriber(self, Float32Stamped, '/rtk/bearing')
       
        self.curr_pub = self.create_publisher(Odometry, "/rtk/curr", 10)
        self.rad_pub = self.create_publisher(Float32Stamped, "/rtk/radians", 10)
        self.deg_pub = self.create_publisher(Float32Stamped, "/rtk/degrees", 10)
        self.kall_pub = self.create_publisher(Float32Stamped, '/rtk/kallman', 10)


        self._syn_pub = message_filters.ApproximateTimeSynchronizer(
            [self._gps_sub, self._dot_sub, self._dist_sub, self._bear_sub], 10, slop=10
        )
        self._syn_pub.registerCallback(self.sync_callback)

        self.declare_parameter('delta', 0.1)
        self.delta_threshold = self.get_parameter('delta').value
        self._parameter_callback = self.create_timer(1.0, self.parameter_callback)


    def parameter_callback(self):
        new_value = self.get_parameter('delta').value
        if new_value != self.delta_threshold:
            self.get_logger().info(f"delta changed from {self.delta_threshold} to {new_value}")
            self.delta_threshold = new_value


    def sync_callback(self, fix, dot, dist, bear):
        self.cur_odom = Odometry()
        self.cur_odom.header = fix.header
        self.cur_odom.pose.pose.position.x = dist.data * math.cos(bear.data)
        self.cur_odom.pose.pose.position.y = dist.data * math.sin(bear.data)
        self.cur_odom.pose.pose.orientation.w = self.quaternion[0]
        self.cur_odom.pose.pose.orientation.x = self.quaternion[1]
        self.cur_odom.pose.pose.orientation.y = self.quaternion[2]
        self.cur_odom.pose.pose.orientation.z = self.quaternion[3]

        if self.odom_distance(self.cur_odom, self.pre_odom) > self.delta_threshold:
            x = self.cur_odom.pose.pose.position.x - self.pre_odom.pose.pose.position.x
            y = self.cur_odom.pose.pose.position.y - self.pre_odom.pose.pose.position.y
            rad_msg = deg_msg = kall_msg = Float32Stamped()

            radians = math.atan2(y, x)
            rad_msg.data = radians
            self.rad_pub.publish(rad_msg)

            degrees = math.degrees(radians)
            deg_msg.data = degrees
            self.deg_pub.publish(deg_msg)

            kally = self.kf.step(radians)
            kall_msg.data = kally
            self.kall_pub.publish(kall_msg)

            self.quaternion = quaternions.axangle2quat([0, 0, 1], kally)
            self.cur_odom.pose.pose.orientation.w = self.quaternion[0]
            self.cur_odom.pose.pose.orientation.x = self.quaternion[1]
            self.cur_odom.pose.pose.orientation.y = self.quaternion[2]
            self.cur_odom.pose.pose.orientation.z = self.quaternion[3]
            self.pre_odom = self.cur_odom
        self.curr_pub.publish(self.cur_odom)

    def odom_distance(self, odom1, odom2):
        x1, y1 = odom1.pose.pose.position.x, odom1.pose.pose.position.y
        x2, y2 = odom2.pose.pose.position.x, odom2.pose.pose.position.y
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def main(args=None):
    rclpy.init(args=args)
    navfix = RTKBeardist(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()


if __name__ == '__main__':
    main()