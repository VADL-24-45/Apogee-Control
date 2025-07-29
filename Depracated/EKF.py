import numpy as np

class ExtendedKalmanFilter:
    def __init__(self, dt=1/100, mass=1.0, Cd=5, area=0.00810708056, rho=1.225):
        self.dt = dt  # Time step (s)

        # Rocket parameters
        self.m = mass  # kg
        self.Cd = Cd   # Drag coefficient
        self.A = area  # Cross-sectional area (m^2)
        self.rho = rho # Air density (kg/m^3)
        self.g = 9.81  # Gravity (m/s^2)

        # Initial state: [altitude, velocity]
        self.x = np.array([[0.0], [0.0]])

        # Initial covariance
        self.P = np.eye(2) * 1.0

        # Process noise (tune these!)
        self.Q = np.array([[2.04e-2, 0],
                           [0, 5]])  # Higher velocity process noise allows for coast-phase variability

        # Measurement noise (altimeter variance)
        self.R = np.array([[5**2]])  # measured std dev of 1.648587697

        # Observation matrix (altitude only)
        self.H = np.array([[1, 0]])

    def predict(self, dt):
        vel = self.x[1, 0]

        # Compute drag
        drag = 0.5 * self.Cd * self.rho * self.A * vel**2 / self.m
        drag *= np.sign(vel)

        # State prediction (nonlinear)
        self.x[0, 0] += dt * vel
        self.x[1, 0] += dt * (-self.g - drag)

        # Jacobian (F matrix)
        d_drag_dv = -np.sign(vel) * (self.Cd * self.rho * self.A / self.m) * abs(vel)
        F = np.array([[1, dt],
                    [0, 1 + dt * d_drag_dv]])

        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q


    def update(self, z_measured):
        """
        :param z_measured: Altitude measurement (float)
        :return: (altitude_estimate, velocity_estimate)
        """
        z = np.array([[z_measured]])

        # Measurement residual
        y = z - (self.H @ self.x)

        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R

        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # State update
        self.x = self.x + K @ y

        # Covariance update
        self.P = (np.eye(2) - K @ self.H) @ self.P

        return float(self.x[0, 0]), float(self.x[1, 0])
