import numpy as np
import rclpy
from sensor_msgs.msg import NavSatFix
from rclpy.node import Node
import message_filters
from geometry_msgs.msg import Pose
from handy_msgs.msg import Float32Stamped
from handy_msgs.srv import Datum, Gps2Ecef, Ecef2Gps, Ecef2Enu, Enu2Ecef, Gps2Enu, Enu2Gps
from example_interfaces.srv import Trigger
import pymap3d as pm


def gps_to_ecef(gps):
    lat, lon, alt = gps
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

def gps_to_enu(gps, datum):
    fix_ecef = np.array(gps_to_ecef(gps))
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

def ecef_to_gps(ecef):
    x, y, z = ecef
    a = 6378137.0
    finv = 298.257223563
    f = 1 / finv
    e2 = 1 - (1 - f) * (1 - f)
    lon = np.arctan2(y, x)
    p = np.sqrt(x**2 + y**2)
    lat = np.arctan2(z, p * (1 - e2))
    alt = 0.0
    prev_lat = 2 * np.pi
    while abs(lat - prev_lat) > 1e-9:
        prev_lat = lat
        v = a / np.sqrt(1 - e2 * np.sin(lat)**2)
        new_lat = np.arctan2(z + e2 * v * np.sin(lat), p)
        lat = new_lat
        alt = p / np.cos(lat) - v
    lat = lat * 180.0 / np.pi
    lon = lon * 180.0 / np.pi
    return lat, lon, alt

def enu_to_gps(enu, datum):
    # Retrieve datum ECEF coordinates
    datum_ecef = np.array(gps_to_ecef(datum))
    # Get observer's geodetic coordinates
    observer_lat = np.arctan2(datum_ecef[2], np.sqrt(datum_ecef[0]**2 + datum_ecef[1]**2))
    observer_lon = np.arctan2(datum_ecef[1], datum_ecef[0])
    # Calculate rotation matrix elements
    sin_lon = np.sin(observer_lon)
    cos_lon = np.cos(observer_lon)
    sin_lat = np.sin(observer_lat)
    cos_lat = np.cos(observer_lat)
    # Calculate inverse rotation matrix
    inv_rotation_matrix = np.array([[-sin_lon, -sin_lat * cos_lon, cos_lat * cos_lon],
                                    [cos_lon, -sin_lat * sin_lon, cos_lat * sin_lon],
                                    [0, cos_lat, sin_lat]])
    # Inverse rotate the ENU coordinates to get differences in ECEF coordinates
    ecef_diff = np.dot(inv_rotation_matrix, enu)
    # Add the differences to datum ECEF coordinates to get fix ECEF coordinates
    fix_ecef = datum_ecef + ecef_diff
    # Convert fix ECEF coordinates back to GPS coordinates
    return ecef_to_gps(fix_ecef)

class MyNode(Node):
    def __init__(self, args):
        super().__init__("cord_convert")
        self.get_logger().info('STARTING COORDINATE CONVERTER')

        self.gps2ecef_server = self.create_service(Gps2Ecef, '/fix/convert/gps2ecef', self.gps2ecef)
        self.ecef2gps_server = self.create_service(Ecef2Gps, '/fix/convert/ecef2gps', self.ecef2gps)

        self.ecef2enu_srver = self.create_service(Ecef2Enu, '/fix/convert/ecef2enu', self.ecef2enu)
        self.enu2ecef_server = self.create_service(Enu2Ecef, '/fix/convert/enu2ecef', self.enu2ecef)

        self.gps2enu_server = self.create_service(Gps2Enu, '/fix/convert/gps2enu', self.gps2enu)
        self.enu2gps_server = self.create_service(Enu2Gps, '/fix/convert/enu2gps', self.enu2gps) 

    def gps2ecef(self, request, response):
        self.get_logger().info('CONVERTING GPS TO ECEF')
        ecef_msg = Pose()
        lat, lon, alt = request.gps.latitude, request.gps.longitude, request.gps.altitude
        x, y, z = gps_to_ecef((lat, lon, alt))
        ecef_msg.position.x = x
        ecef_msg.position.y = y
        ecef_msg.position.z = z
        response.message = f"SUCCESSFULLY CONVERTED GPS TO ECEF"
        response.ecef = ecef_msg
        return response

    def ecef2gps(self, request, response):
        self.get_logger().info('CONVERTING ECEF TO GPS')
        gps_msg = NavSatFix()
        x, y, z = request.ecef.position.x, request.ecef.position.y, request.ecef.position.z
        lat, lon, alt = ecef_to_gps((x, y, z))
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = alt
        response.message = f"SUCCESSFULLY CONVERTED ECEF TO GPS"
        response.gps = gps_msg
        return response

    def ecef2enu(self, request, response):
        self.get_logger().info('CONVERTING ECEF TO ENU')
        self.get_logger().info('STILL NOT IMPLEMENTED')

    def enu2ecef(self, request, response):
        self.get_logger().info('CONVERTING ENU TO ECEF')
        self.get_logger().info('STILL NOT IMPLEMENTED')

    def gps2enu(self, request, response):
        self.get_logger().info('CONVERTING GPS TO ENU')
        gps = request.gps
        datum = request.datum
        enu = gps_to_enu((gps.latitude, gps.longitude, gps.altitude), (datum.latitude, datum.longitude, datum.altitude))
        enu_msg = Pose()
        enu_msg.position.x = enu[0]
        enu_msg.position.y = enu[1]
        enu_msg.position.z = enu[2]
        response.message = f"SUCCESSFULLY CONVERTED GPS TO ENU"
        response.enu = enu_msg
        return response

    def enu2gps(self, request, response):
        self.get_logger().info('CONVERTING ENU TO GPS')
        enu = request.enu
        datum = request.datum
        gps = enu_to_gps((enu.position.x, enu.position.y, enu.position.z), (datum.latitude, datum.longitude, datum.altitude))
        gps_msg = NavSatFix()
        gps_msg.latitude = gps[0]
        gps_msg.longitude = gps[1]
        gps_msg.altitude = gps[2]
        response.message = f"SUCCESSFULLY CONVERTED ENU TO GPS"
        response.gps = gps_msg
        return response


def main(args=None):
    rclpy.init(args=args)
    navfix = MyNode(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()

if __name__ == '__main__':
    main()