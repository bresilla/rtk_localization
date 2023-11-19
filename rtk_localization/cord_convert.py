import numpy as np
import rclpy
from sensor_msgs.msg import NavSatFix
from rclpy.node import Node
import message_filters
from nav_msgs.msg import Odometry
from handy_msgs.msg import Float32Stamped
from handy_msgs.srv import Datum, Gps2Ecef, Ecef2Gps, Ecef2Enu, Enu2Ecef, Gps2Enu, Enu2Gps
from example_interfaces.srv import Trigger
import pymap3d as pm


def gps_to_ecef(coords):
    lat, lon, alt = coords
    rad_lat = lat * (np.pi / 180.0)
    rad_lon = lon * (np.pi / 180.0)
    a = 6378137.0
    finv = 298.257223563
    f = 1 / finv
    e2 = 1 - (1 - f) * (1 - f)
    v = a / np.sqrt(1 - e2 * np.sin(rad_lat) * np.sin(rad_lat))
    x = (v + alt) * np.cos(rad_lat) * np.cos(rad_lon)
    y = (v + alt) * np.cos(rad_lat) * np.sin(rad_lon)
    z = (v * (1 - e2) + alt) * np.sin(rad_lat)
    return x, y, z

def ecef_to_enu(fix, datum):
    fix_ecef = np.array(gps_to_ecef(fix))
    datum_ecef = np.array(gps_to_ecef(datum))
    # Calculate the differences in coordinates
    dx = fix_ecef[0] - datum_ecef[0]
    dy = fix_ecef[1] - datum_ecef[1]
    dz = fix_ecef[2] - datum_ecef[2]
    # Get observer's geodetic coordinates
    observer_lat = np.arctan2(datum_ecef[2], np.sqrt(datum_ecef[0]**2 + datum_ecef[1]**2))
    observer_lon = np.arctan2(datum_ecef[1], datum_ecef[0])
    # Calculate rotation matrix elements
    sin_lon = np.sin(observer_lon)
    cos_lon = np.cos(observer_lon)
    sin_lat = np.sin(observer_lat)
    cos_lat = np.cos(observer_lat)
    # Calculate rotation matrix
    rotation_matrix = np.array([[-sin_lon, cos_lon, 0],
                                 [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
                                 [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat]])
    # Apply rotation matrix to get ENU coordinates
    enu = np.dot(rotation_matrix, np.array([dx, dy, dz]))
    return enu

class MyNode(Node):
    def __init__(self, args):
        super().__init__("cord_convert")
        self.get_logger().info('STARTING COORDINATE CONVERTER')

        self._gps2ecef = self.create_service(Gps2Ecef, '/fix/convert/gps2ecef', self.gps2ecef)
        self._ecef2gps = self.create_service(Ecef2Gps, '/fix/convert/ecef2gps', self.ecef2gps)

        self._ecef2enu = self.create_service(Ecef2Enu, '/fix/convert/ecef2enu', self.ecef2enu)
        self._enu2ecef = self.create_service(Enu2Ecef, '/fix/convert/enu2ecef', self.enu2ecef)

        self._gps2enu = self.create_service(Gps2Enu, '/fix/convert/gps2enu', self.gps2enu)
        self._enu2gps = self.create_service(Enu2Gps, '/fix/convert/enu2gps', self.enu2gps) 


    def gps2ecef(self, request, response):
        self.get_logger().info('CONVERTING GPS TO ECEF')

    def ecef2gps(self, request, response):
        self.get_logger().info('CONVERTING ECEF TO GPS')

    def ecef2enu(self, request, response):
        self.get_logger().info('CONVERTING ECEF TO ENU')

    def enu2ecef(self, request, response):
        self.get_logger().info('CONVERTING ENU TO ECEF')

    def gps2enu(self, request, response):
        self.get_logger().info('CONVERTING GPS TO ENU')

    def enu2gps(self, request, response):
        self.get_logger().info('CONVERTING ENU TO GPS')



def main(args=None):
    rclpy.init(args=args)
    navfix = MyNode(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()

if __name__ == '__main__':
    main()