import math, numpy as np
import rclpy
from sensor_msgs.msg import NavSatFix
from rclpy.node import Node
import message_filters
from handy_msgs.msg import Float32Stamped
from handy_msgs.srv import Datum
from example_interfaces.srv import Trigger

def midpoint(lat1, long1, lat2, long2):
    # Convert degrees to radians
    lat1 = math.radians(lat1)
    long1 = math.radians(long1)
    lat2 = math.radians(lat2)
    long2 = math.radians(long2)
    delta_long = long2 - long1
    B_x = math.cos(lat2) * math.cos(delta_long)
    B_y = math.cos(lat2) * math.sin(delta_long)
    lat_m = math.atan2(math.sin(lat1) + math.sin(lat2), math.sqrt((math.cos(lat1) + B_x)**2 + B_y**2))
    long_m = long1 + math.atan2(B_y, math.cos(lat1) + B_x)
    # Convert radians back to degrees
    lat_m = math.degrees(lat_m)
    long_m = math.degrees(long_m)
    return lat_m, long_m

def calc_bearing(lat1, long1, lat2, long2):
    # Convert latitude and longitude to radians
    lat1 = math.radians(lat1)
    long1 = math.radians(long1)
    lat2 = math.radians(lat2)
    long2 = math.radians(long2)
    # Calculate the bearing
    bearing_rad = math.atan2(
        math.sin(long2 - long1) * math.cos(lat2),
        math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(long2 - long1)
    )
    #add 90 degrees to get the bearing from north
    bearing_rad += math.pi/2
    # Convert the bearing to degrees
    bearing_deg = math.degrees(bearing_rad)
    # Make sure the bearing is positive
    bearing_deg = (bearing_deg + 360) % 360
    #make sure bearing is between -2pi and 2pi
    # if bearing_rad < -math.pi:
    #     bearing_rad += 2 * math.pi
    # elif bearing_rad > math.pi:
    #     bearing_rad -= 2 * math.pi
    if bearing_rad < 0:
        bearing_rad += 2 * math.pi
    elif bearing_rad > 2 * math.pi:
        bearing_rad -= 2 * math.pi
    return bearing_deg, bearing_rad


class MyNode(Node):
    def __init__(self, args):
        super().__init__("gps_fuse")
        self.get_logger().info('STARTING GPS FUSE')
        self.datum = None
        self.curr_position = None
        self.front_as_main = True

        self._gps_front = message_filters.Subscriber(self, NavSatFix, '/gps/front')
        self._gps_back = message_filters.Subscriber(self, NavSatFix, '/gps/back')
        self._gps_filter = message_filters.ApproximateTimeSynchronizer(
            [self._gps_front, self._gps_back], 10, slop=10
        )
        self._gps_filter.registerCallback(self.callback)

        self.gps_pub = self.create_publisher(NavSatFix, "/fix/gps", 10)
        self.deg = self.create_publisher(Float32Stamped, "/fix/deg", 10)
        self.rad = self.create_publisher(Float32Stamped, "/fix/rad", 10)
        self.dat_pub = self.create_publisher(NavSatFix, "/fix/datum/gps", 10)

        self._datum_gps = self.create_service(Datum, '/datum/gps', self.datum_gps)
        self._datum_set = self.create_service(Trigger, '/datum/set', self.datum_set)
        # self.cli = self.create_client(Trigger, '/fix/trigger')

    def callback(self, front_msg, back_msg):
        if self.front_as_main:
            gps_msg = front_msg
        else:
            gps_msg = NavSatFix()
            gps_msg.header = front_msg.header
            gps_msg.latitude, gps_msg.longitude = midpoint(front_msg.latitude, front_msg.longitude, back_msg.latitude, back_msg.longitude)
        # gps_msg.header.frame_id = "gps"
        self.curr_position = gps_msg

        deg_msg = Float32Stamped()
        rad_msg = Float32Stamped()
        deg_msg.header = front_msg.header
        rad_msg.header = front_msg.header
        deg_msg.data, rad_msg.data = calc_bearing(front_msg.latitude, front_msg.longitude, back_msg.latitude, back_msg.longitude)
        
        self.gps_pub.publish(gps_msg)
        self.deg.publish(deg_msg)
        self.rad.publish(rad_msg)

        if self.datum is not None:
            self.datum.header = gps_msg.header
            self.dat_pub.publish(self.datum)

    def datum_set(self, request, response):
        self.get_logger().info('FIX SET TO CURRENT POSITION')
        self.datum = self.curr_position
        response.message = f"DOT SET TO: {self.datum.latitude}, {self.datum.longitude}"
        # self.request_stat()
        return response
    
    def datum_gps(self, request, response):
        self.get_logger().info('FIX SET TO SPECIFIED POSITION')
        self.datum = request.gps
        response.message = f"DOT SET TO: {self.datum.latitude}, {self.datum.longitude}"
        # # self.request_stat()
        return response

    # def request_stat(self):
    #     self.get_logger().info('SENDING TRIGGERED REQUEST')
    #     return
    #     cli_request = Trigger.Request()
    #     while not self.cli.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().info('service not available, waiting again...')
    #     self.cli.call_async(cli_request)
    #     # try:
    #     #     self.cli.wait_for_service(timeout_sec=1.0)
    #     #     self.cli.call_async(cli_request)
    #     # except Exception as e:
    #     #     self.get_logger().info(f"Service call failed {e}")


def main(args=None):
    rclpy.init(args=args)
    navfix = MyNode(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()

if __name__ == '__main__':
    main()