import numpy as np
import rclpy
from sensor_msgs.msg import NavSatFix
from rclpy.node import Node
import message_filters
from nav_msgs.msg import Odometry
from handy_msgs.msg import Float32Stamped
from handy_msgs.srv import DatumGPS, DatumENU
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
        super().__init__("gps_to_enu")
        self.get_logger().info('STARTING GPS TO ENU')
        self.datum = None

        self._gps = message_filters.Subscriber(self, NavSatFix, '/fix/gps')
        self._datum = message_filters.Subscriber(self, NavSatFix, '/fix/datum/gps')
        self._rad = message_filters.Subscriber(self, Float32Stamped, '/fix/rad')
        self._gps_filter = message_filters.ApproximateTimeSynchronizer(
            [self._gps, self._datum, self._rad], 10, slop=10
        )
        self._gps_filter.registerCallback(self.callback)

        self.ecef_pub = self.create_publisher(Odometry, '/fix/ecef', 10)
        self.enu_pub = self.create_publisher(Odometry, '/fix/enu', 10)
        self.ecef_datum_pub = self.create_publisher(Odometry, '/fix/datum/ecef', 10)

    def callback(self, gps_msg, datum_msg, rad_msg):
        # self.get_logger().info('CALLBACK')
        if self.datum is None:
            self.set_ecef_datum(datum_msg)
        else:
            self.ecef_datum_pub.publish(self.datum)

        lat, lon, alt = gps_msg.latitude, gps_msg.longitude, gps_msg.altitude
        ecef_msg = Odometry()
        ecef_msg.header = gps_msg.header
        ecef_x,ecef_y,ecef_z = pm.geodetic2ecef(lat,lon,alt)
        # ecef_x,ecef_y,ecef_z = gps_to_ecef((lat,lon,alt))
        ecef_msg.pose.pose.position.x = ecef_x
        ecef_msg.pose.pose.position.y = ecef_y
        ecef_msg.pose.pose.position.z = ecef_z

        enu_msg = Odometry()
        enu_msg.header = gps_msg.header
        d_lat, d_lon, d_alt = datum_msg.latitude, datum_msg.longitude, datum_msg.altitude
        enu_x,enu_y,enu_z = pm.geodetic2enu(lat,lon,alt,d_lat,d_lon,d_alt)
        # enu_x,enu_y,enu_z = ecef_to_enu((lat,lon,alt), (d_lat,d_lon,d_alt))
        enu_msg.pose.pose.position.x = enu_x
        enu_msg.pose.pose.position.y = enu_y
        enu_msg.pose.pose.position.z = enu_z

        self.ecef_pub.publish(ecef_msg)
        self.enu_pub.publish(enu_msg)

    def set_ecef_datum(self, datum_msg):
        self.datum = Odometry()
        self.datum.header = datum_msg.header
        lat, lon, alt = datum_msg.latitude, datum_msg.longitude, datum_msg.altitude
        x,y,z = pm.geodetic2ecef(lat,lon,alt)
        self.datum.pose.pose.position.x = x
        self.datum.pose.pose.position.y = y
        self.datum.pose.pose.position.z = z


def main(args=None):
    rclpy.init(args=args)
    navfix = MyNode(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()

if __name__ == '__main__':
    main()