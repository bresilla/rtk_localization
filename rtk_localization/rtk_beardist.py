import math
import rclpy
from sensor_msgs.msg import NavSatFix
from rclpy.node import Node
from handy_msgs.msg import Float32Stamped
from handy_msgs.srv import WGS, UTM
from example_interfaces.srv import Trigger

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
    radians = math.radians(bearing)
    radians = math.pi - radians
    return radians, bearing

class RTKBeardist(Node):
    def __init__(self, args):
        super().__init__("rtk_beardist")
        self.get_logger().info('INITIALIZING RTK BEARDIST')
        self.distance = self.radians = self.degrees = 0.0
        self.dot_position = self.cur_position = self.tick_position = None
        self.delta = 0.0

        self.gps_sub = self.create_subscription(NavSatFix, "/gps/fix", self.gps_callback, 10)
        self.curr_pub = self.create_publisher(NavSatFix, "/rtk/curr", 10)
        self.prev_pub = self.create_publisher(NavSatFix, "/rtk/prev ", 10)
        self.dot_pub = self.create_publisher(NavSatFix, "/rtk/dot", 10)
        self.dist_pub = self.create_publisher(Float32Stamped, "/rtk/distance", 10)
        self.rad_pub = self.create_publisher(Float32Stamped, "/rtk/radians", 10)
        # self.deg_pub = self.create_publisher(Float32Stamped, "/rtk/degrees", 10)

        self.tmp_srv = self.create_service(Trigger, '/rtk/cur_set', self.cur_set)
        self.fix_srv = self.create_service(WGS, '/rtk/fix_set', self.fix_set)
        self.cli = self.create_client(Trigger, '/rtk/transforms')

    def gps_callback(self, msg):
        # msg.header.frame_id = "rtk"
        if self.dot_position is None: self.dot_position = msg
        if self.tick_position is None: self.tick_position = msg
        self.cur_position = msg
        dist_msg = Float32Stamped()
        rad_msg = Float32Stamped()
        deg_msg = Float32Stamped()
        dist_msg.header = rad_msg.header = deg_msg.header = msg.header
        self.distance = distance(
            (self.dot_position.latitude, self.dot_position.longitude), 
            (msg.latitude, msg.longitude)
        )
        self.radians, self.degrees = bearing(
            (self.dot_position.latitude, self.dot_position.longitude), 
            (msg.latitude, msg.longitude)
        )

        if self.distance >= self.delta:
            self.delta = self.distance + 0.05
            self.prev_pub.publish(self.tick_position)
            self.tick_position = msg

        dist_msg.data = self.distance
        rad_msg.data = self.radians
        deg_msg.data = self.degrees
        self.curr_pub.publish(msg)
        self.dot_position.header = msg.header
        self.dot_pub.publish(self.dot_position)
        self.dist_pub.publish(dist_msg)
        self.rad_pub.publish(rad_msg)
        # self.deg_pub.publish(deg_msg)

    def cur_set(self, request, response):
        self.get_logger().info('FIX SET TO CURRENT POSITION')
        self.dot_position = self.cur_position
        response.message = f"DOT SET TO: {self.dot_position.latitude}, {self.dot_position.longitude}"
        self.request_stat()
        return response
    
    def fix_set(self, request, response):
        self.get_logger().info('FIX SET TO FIXED POSITION')
        self.dot_position = request.wgs_pose
        response.message = f"DOT SET TO: {self.dot_position.latitude}, {self.dot_position.longitude}"
        self.request_stat()
        return response
    
    def request_stat(self):
        cli_request = Trigger.Request()
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.cli.call_async(cli_request)


def main(args=None):
    rclpy.init(args=args)
    navfix = RTKBeardist(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()


if __name__ == '__main__':
    main()