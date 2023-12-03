import numpy as np
import math
import rclpy
from sensor_msgs.msg import NavSatFix
from rclpy.node import Node
import message_filters
from nav_msgs.msg import Odometry
from handy_msgs.msg import Float32Stamped

R = 635_675_2.314_2
f_inv = 298.257_223_563
f = 1.0 / f_inv
e2 = 1 - (1 - f) * (1 - f)

def gps_to_ecef(coords):
    latitude, longitude, altitude = coords
    cosLat = math.cos(latitude * math.pi / 180)
    sinLat = math.sin(latitude * math.pi / 180)
    cosLong = math.cos(longitude * math.pi / 180)
    sinLong = math.sin(longitude * math.pi / 180)
    c = 1 / math.sqrt(cosLat * cosLat + (1 - f) * (1 - f) * sinLat * sinLat)
    s = (1 - f) * (1 - f) * c
    x = (R*c + altitude) * cosLat * cosLong
    y = (R*c + altitude) * cosLat * sinLong
    z = (R*s + altitude) * sinLat
    return x, y, z

def ecef_to_enu(ecef, datum):
    x, y, z = ecef
    latRef, longRef, altRef = datum
    cosLatRef = math.cos(latRef * math.pi / 180)
    sinLatRef = math.sin(latRef * math.pi / 180)
    cosLongRef = math.cos(longRef * math.pi / 180)
    sinLongRef = math.sin(longRef * math.pi / 180)
    cRef = 1 / math.sqrt(cosLatRef * cosLatRef + (1 - f) * (1 - f) * sinLatRef * sinLatRef)
    x0 = (R*cRef + altRef) * cosLatRef * cosLongRef
    y0 = (R*cRef + altRef) * cosLatRef * sinLongRef
    z0 = (R*cRef*(1-e2) + altRef) * sinLatRef
    xEast = (-(x-x0) * sinLongRef) + ((y-y0)*cosLongRef)
    yNorth = (-cosLongRef*sinLatRef*(x-x0)) - (sinLatRef*sinLongRef*(y-y0)) + (cosLatRef*(z-z0))
    zUp = (cosLatRef*cosLongRef*(x-x0)) + (cosLatRef*sinLongRef*(y-y0)) + (sinLatRef*(z-z0))
    return xEast, yNorth, zUp

def geodetic_to_enu(lat, lon, h, lat_ref, lon_ref, h_ref):
    x, y, z = gps_to_ecef((lat, lon, h))
    return ecef_to_enu(x, y, z, lat_ref, lon_ref, h_ref)

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
        ecef_x,ecef_y,ecef_z = gps_to_ecef((lat,lon,alt))
        ecef_msg.pose.pose.position.x = ecef_x
        ecef_msg.pose.pose.position.y = ecef_y
        ecef_msg.pose.pose.position.z = ecef_z

        enu_msg = Odometry()
        enu_msg.header = gps_msg.header
        d_lat, d_lon, d_alt = datum_msg.latitude, datum_msg.longitude, datum_msg.altitude
        enu_x,enu_y,enu_z = ecef_to_enu((ecef_x,ecef_y,ecef_z), (d_lat,d_lon,d_alt))
        enu_msg.pose.pose.position.x = enu_x
        enu_msg.pose.pose.position.y = enu_y
        enu_msg.pose.pose.position.z = enu_z

        self.ecef_pub.publish(ecef_msg)
        self.enu_pub.publish(enu_msg)

    def set_ecef_datum(self, datum_msg):
        self.datum = Odometry()
        self.datum.header = datum_msg.header
        lat, lon, alt = datum_msg.latitude, datum_msg.longitude, datum_msg.altitude
        x,y,z = gps_to_ecef((lat,lon,alt))
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