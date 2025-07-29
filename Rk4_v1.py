# Import libraries (use pip install numpy pandas matplotlib) if first time 
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

class Rk4:

    def __init__(self, frequency=10, coeeff_drag = 5, mass=5, area=1):
        self.FREQUENCY = frequency
        self.MAX_TIME = 16
        self.GRAV_CONST = 9.80665 # gravity (m/s^2)
        self.COEFF_DRAG = coeeff_drag # drag coefficient **can be changed**
        self.AIR_DENSITY = 1.225 # air density (kg/m^3)
        self.MASS = mass # mass (kg) **can be changed**

        # Rocket Dimensions
        self.AREA = area
        pass

    def rk4_apogee_predictor(self, h0, v0):
        
        dt = 1 / self.FREQUENCY # time steps for physics
        max_steps = int(self.MAX_TIME / dt) # seconds / dt

        h = h0 # height in meters
        v = v0 # velocity in meters per second
        for _ in range(max_steps):
            if v <= 0:
                break

            # Fourth order equations
            k1_v, k1_h = self.find_derivatives(v)
            k2_v, k2_h = self.find_derivatives(v + 0.5*dt*k1_v)
            k3_v, k3_h = self.find_derivatives(v + 0.5*dt*k2_v)
            k4_v, k4_h = self.find_derivatives(v + dt*k3_v)

            v += (dt/6)*(k1_v + 2*k2_v + 2*k3_v + k4_v)
            h += (dt/6)*(k1_h + 2*k2_h + 2*k3_h + k4_h)
        return h

    def find_derivatives(self, velocity):
        drag = .5 * self.COEFF_DRAG * self.AREA * self.AIR_DENSITY * velocity**2 * np.sign(velocity)
        dv = -self.GRAV_CONST - drag / self.MASS
        dh = velocity
        return dv, dh

    def plot_apogee_prediction(self, csv_filename):
        m_to_ft = 3.28084 # conversion factor
        try:
            # Load CSV files 
            data = pd.read_csv(csv_filename).values
        except FileNotFoundError:
            print(f"ERROR: File not found - {csv_filename}")
            return
        
        if data.shape[1] < 11:
            print("ERROR: CSV must have ≥11 columns")
            return
        
        time = data[:, 3]       
        status = data[:, 4]      
        altitude = data[:, 9]    
        velocity = data[:, 10]   
        
        valid_mask = status == 5 # looks at rocket after motor burnout (state 5)
        time = time[valid_mask]
        altitude = altitude[valid_mask]
        velocity = velocity[valid_mask]
        
        # Initialize predicted apogee array
        N = len(time) # Number of data points to process
        predicted_apogees = np.full(N, np.nan) # Initialize with NaN
        
        for i in range(N):
            predicted_apogees[i] = self.rk4_apogee_predictor(altitude[i], velocity[i])
        
        actual_apogee = np.max(altitude) # apogee of rocket
        goal_apogee = 900 # value dependent on data set (csv) used
        
        plt.figure(figsize=(10, 6))
        plt.plot(time, predicted_apogees * m_to_ft, 'b-', lw=1.5, label='Predicted Apogee')
        plt.plot(time, altitude * m_to_ft, 'k-', lw=1.5, label='Current Altitude')
        plt.axhline(actual_apogee * m_to_ft, color='r', ls='--', lw=2, label='Actual Apogee')
        plt.axhline(goal_apogee, color='g', ls='--', lw=2, label='Goal Apogee')
        plt.xlabel('Time (s)')
        plt.ylabel('Altitude (ft)')
        plt.title('Apogee Prediction vs Flight Data')
        plt.legend()
        plt.grid(True)
        plt.xlim(time.min(), time.max())
        plt.tight_layout()
        plt.show()

# Example Usage (change file direction as needed for different data sets)
if __name__ == "__main__":
    Rk4(10).plot_apogee_prediction(r"C:\Users\bigdu\Downloads\Summer 2025\DRSS\MATLAB\Flight Data\4-12 backup flight 2.csv")

