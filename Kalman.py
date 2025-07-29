import numpy as np

class KalmanFilter:
    def __init__(self, dt=1/100, ground = 500): # IMU samples at 160 Hz
        self.dt = dt  # Time step in seconds

        # Initial state: [altitude, vertical_velocity]
        self.x = np.array([[ground],  # Altitude
                           [0.0]]) # Velocity

        # Initial uncertainty (covariance)
        self.P = np.eye(2)

        # State transition matrix
        self.A = np.array([[1, dt],
                           [0, 1]])

        # Observation matrix (only altitude is measured)
        self.H = np.array([[1, 0]])

        # Process noise covariance (tune for responsiveness)
        self.Q = np.array([[2.04e-2, 0],
                           [0, 5]])

        # Measurement noise covariance (altimeter noise)
        self.R = np.array([[20**2]])

    def update(self, z_measured):
        """
        Update the Kalman filter with a new altitude measurement.

        :param z_measured: Altitude measurement (float)
        :return: (altitude_estimate, vertical_velocity_estimate)
        """        
        z = np.array([[z_measured]])

        # Predict
        x_pred = self.A @ self.x
        P_pred = self.A @ self.P @ self.A.T + self.Q

        # Kalman gain
        y = z - (self.H @ x_pred)  # Measurement residual
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)

        # Update estimate
        self.x = x_pred + K @ y
        self.P = (np.eye(2) - K @ self.H) @ P_pred

        # Return the estimated state
        return float(self.x[0][0]), float(self.x[1][0])
