import math
import rclpy
import numpy as np
from sensor_msgs.msg import NavSatFix
from rclpy.node import Node
from handy_msgs.srv import Threshold

def distance(coord1, coord2):
    radius_earth = 6_367_449.654_657
    lat1, lon1 = map(math.radians, coord1)
    lat2, lon2 = map(math.radians, coord2)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = radius_earth * c
    return distance

class KalmanFilter():
    def __init__(self, process_noise=0.01, measurement_noise=0.1):
        self.estimated_state = [0.0, 0.0]  # Initial state estimate [latitude, longitude]
        self.estimated_error = [[1.0, 0.0], [0.0, 1.0]]  # Initial error estimate
        self.process_noise = process_noise   # Process noise (adjust as needed)
        self.measurement_noise = measurement_noise # Measurement noise (adjust as needed)
    
    def step(self, measurement):
        # Prediction step
        predicted_state = self.estimated_state
        predicted_error = [
            [val + self.process_noise if i == j else val for j, val in enumerate(row)]
            for i, row in enumerate(self.estimated_error)
        ]
        # Update step
        kalman_gain_num = predicted_error
        # Adding a small value to avoid division by near-zero values
        epsilon = 1e-8
        kalman_gain_denom = [
            [
                predicted_error[i][j] + self.measurement_noise + epsilon 
                if i == j 
                else predicted_error[i][j] + epsilon
                for j in range(len(predicted_error))
            ]
            for i in range(len(predicted_error))
        ]
        
        kalman_gain = [
            [
                kalman_gain_num[i][j] / kalman_gain_denom[i][j]
                for j in range(len(kalman_gain_num))
            ]
            for i in range(len(kalman_gain_num))
        ]
        
        innovation = [measurement[0] - predicted_state[0], measurement[1] - predicted_state[1]]
        self.estimated_state = [
            predicted_state[i] + kalman_gain[i][0] * innovation[0] + kalman_gain[i][1] * innovation[1]
            for i in range(len(predicted_state))
        ]
        self.estimated_error = [
            [(1 - kalman_gain[i][j]) * predicted_error[i][j] for j in range(len(predicted_error))]
            for i in range(len(predicted_error))
        ]
        return self.estimated_state


class ExtendedKalmanFilter():
    def __init__(self, process_noise=0.01, measurement_noise=0.1):
        self.estimated_state = np.array([0.0, 0.0])  # Initial state estimate [latitude, longitude]
        self.estimated_error = np.array([[1.0, 0.0], [0.0, 1.0]])  # Initial error estimate
        self.process_noise = process_noise   # Process noise (adjust as needed)
        self.measurement_noise = measurement_noise # Measurement noise (adjust as needed)
    
    def predict(self, delta_t):
        # F matrix represents the Jacobian of the motion model
        F = np.array([[1, delta_t], [0, 1]])
        # Process noise covariance matrix Q
        Q = np.array([[self.process_noise, 0], [0, self.process_noise]])
        # Predict the next state
        predicted_state = F.dot(self.estimated_state)
        # Update the error covariance
        self.estimated_error = F.dot(self.estimated_error).dot(F.T) + Q
        return predicted_state, self.estimated_error
    
    def update(self, measurement):
        # H matrix represents the Jacobian of the measurement model
        H = np.array([[1, 0], [0, 1]])
        # Measurement noise covariance matrix R
        R = np.array([[self.measurement_noise, 0], [0, self.measurement_noise]])
        predicted_state, predicted_error = self.predict(1)  # Assuming a time step of 1 for demonstration
        # Compute the Kalman gain
        K = predicted_error.dot(H.T).dot(np.linalg.inv(H.dot(predicted_error).dot(H.T) + R))
        # Innovation or measurement residual
        innovation = measurement - H.dot(predicted_state)
        # Update the state estimate
        self.estimated_state = predicted_state + K.dot(innovation)
        # Update the error covariance
        self.estimated_error = (np.eye(len(self.estimated_state)) - K.dot(H)).dot(predicted_error)
        return self.estimated_state



class gps_split(Node):
    def __init__(self, args, kalman=0):
        super().__init__("antenna_split")
        self.get_logger().info('-------------------------------------')
        self.get_logger().info('LOCALIZATION WITH SINGLE ANTENNA MODE')
        self.get_logger().info('-------------------------------------')
        self.step = 0
        self.distance = 0.0
        self.positions = []
        self.threshold = 0.2
        self.kalman = kalman
        if self.kalman == 1:
            self.front_filter = KalmanFilter(process_noise=0.01, measurement_noise=0.1)
            self.back_filter = KalmanFilter(process_noise=0.01, measurement_noise=0.1)
        elif self.kalman == 2:
            self.front_filter = ExtendedKalmanFilter(process_noise=0.005, measurement_noise=0.2)
            self.back_filter = ExtendedKalmanFilter(process_noise=0.005, measurement_noise=0.2)
            


        self.gps_sub = self.create_subscription(NavSatFix, "/fix", self.gps_callback, 10)
        self.gps_1 = self.create_publisher(NavSatFix, "/gps/front", 10)
        self.gps_2 = self.create_publisher(NavSatFix, "/gps/back", 10)

        self.gps = self.create_service(Threshold, "/fix/dist_thresh", self.distance_callback)

    def gps_callback(self, msg):
        self.step += 1
        if len(self.positions) < 2:
            self.positions.append(msg)
            self.positions.append(msg)
            return
        
        if self.kalman == 0:
            if distance((msg.latitude, msg.longitude), (self.positions[-1].latitude, self.positions[-1].longitude)) > self.threshold:
                self.positions[0] = self.positions[1]
                self.positions[1] = msg
            self.positions[0].header.stamp = msg.header.stamp
            self.gps_1.publish(msg)
            self.gps_2.publish(self.positions[0])
            return

        elif self.kalman == 1:
            self.front_filter.step((msg.latitude, msg.longitude))
            msg.latitude, msg.longitude = self.front_filter.estimated_state
            if distance((msg.latitude, msg.longitude), (self.positions[-1].latitude, self.positions[-1].longitude)) > self.threshold:
                self.back_filter.step((self.positions[-1].latitude, self.positions[-1].longitude))
                self.positions[1].latitude, self.positions[1].longitude = self.back_filter.estimated_state
                self.positions[0] = self.positions[1]
                self.positions[1] = msg
            self.positions[0].header.stamp = msg.header.stamp
            self.gps_1.publish(msg)
            self.gps_2.publish(self.positions[0])
            return

        elif self.kalman == 2:
            self.front_filter.predict(1)
            self.front_filter.update((msg.latitude, msg.longitude))
            msg.latitude, msg.longitude = self.front_filter.estimated_state

            self.back_filter.predict(1)
            if distance((msg.latitude, msg.longitude), (self.positions[-1].latitude, self.positions[-1].longitude)) > self.threshold:
                self.back_filter.update((self.positions[-1].latitude, self.positions[-1].longitude))
            self.positions[1].latitude, self.positions[1].longitude = self.back_filter.estimated_state
            self.positions[0] = self.positions[1]
            self.positions[1] = msg
    
            self.positions[0].header.stamp = msg.header.stamp
            self.gps_1.publish(msg)
            self.gps_2.publish(self.positions[0])
            return

    def distance_callback(self, req, res):
        self.threshold = req.data
        return res


def main(args=None):
    rclpy.init(args=args)
    # 0 for no kalman filter, 1 for kalman filter, 2 for extended kalman filter
    navfix = gps_split(args=args, kalman=1)
    rclpy.spin(navfix)
    rclpy.shutdown()


if __name__ == '__main__':
    main()