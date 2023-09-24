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
    def __init__(self):
        super().__init__("rtk_beardist")
        self.calc_point = None
        self.distance = 0.0
        self.radians = 0.0
        self.degrees = 0.0
        self.gps_sub = self.create_subscription(NavSatFix, "/gps/fix", self.gps_callback, 10)
        self.gps_pub = self.create_publisher(NavSatFix, "/rtk/fix", 10)
        self.dist_pub = self.create_publisher(Float32Stamped, "/rtk/distance", 10)
        self.rad_pub = self.create_publisher(Float32Stamped, "/rtk/radians", 10)
        self.deg_pub = self.create_publisher(Float32Stamped, "/rtk/degrees", 10)
        self.odom_pub = self.create_publisher(Odometry, "/rtk/odom", 10)
        self.dot_pub = self.create_publisher(NavSatFix, "/rtk/dot", 10)

    def gps_callback(self, msg):
        global odom_position
        global fix_position
        global tmp_position
        if fix_position is None: return
        tmp_position = msg
        self.gps_pub.publish(msg)
        dist_msg = Float32Stamped()
        rad_msg = Float32Stamped()
        deg_msg = Float32Stamped()
        dist_msg.header = rad_msg.header = deg_msg.header = msg.header
        self.distance = distance((fix_position.latitude, fix_position.longitude), (msg.latitude, msg.longitude))
        self.radians, self.degrees = bearing((fix_position.latitude, fix_position.longitude), (msg.latitude, msg.longitude))
        dist_msg.data = self.distance
        rad_msg.data = self.radians
        deg_msg.data = self.degrees
        self.dot_pub.publish(fix_position)
        self.dist_pub.publish(dist_msg)
        self.rad_pub.publish(rad_msg)
        self.deg_pub.publish(deg_msg)

        odom_msg = Odometry()
        odom_msg.header = msg.header
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.distance * math.cos(self.radians)
        odom_msg.pose.pose.position.y = self.distance * math.sin(self.radians)
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom_msg)
        odom_position = odom_msg


class RTKServices(Node):
    def __init__(self):
        super().__init__("rtk_services")
        self.fix_srv = self.create_service(WGS, '/rtk/fix_set', self.fix_set)
        self.tmp_srv = self.create_service(Trigger, '/rtk/cur_set', self.cur_set)
        self.cli = self.create_client(UTM, '/rtk/map2odom')

    def cur_set(self, request, response):
        global fix_position
        global tmp_position
        self.get_logger().info('FIX SET TO CURRENT POSITION')
        fix_position = tmp_position
        response.message = f"FIX SET TO: {fix_position.latitude}, {fix_position.longitude}"
        self.request_stat()
        return response

    def fix_set(self, request, response):
        global fix_position
        global tmp_position
        self.get_logger().info('FIX SET TO DATUM')
        fix_position = request.wgs_pose
        response.message = f"FIX SET TO: {fix_position.latitude}, {fix_position.longitude}"
        self.request_stat()
        return response
    
    def request_stat(self):
        global odom_position
        time.sleep(1)
        while odom_position is None: 
            self.get_logger().info('TRANSFORM NOT AVAILABLE')
        cli_request = UTM.Request()
        cli_request.utm_pose = odom_position
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.cli.call_async(cli_request)

fix_position = None
tmp_position = None
odom_position = None

def main(args=None):
    rclpy.init(args=args)
    try:
        beardist = RTKBeardist()
        servoces = RTKServices()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(beardist)
        executor.add_node(servoces)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            beardist.destroy_node()
            servoces.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':\
    main()