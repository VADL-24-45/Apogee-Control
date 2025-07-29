# Import libraries (use pip install numpy pandas matplotlib) if first time 
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from time import perf_counter

FREQUENCY = 100
MAX_TIME = 16
GRAV_CONST = 9.80665 # gravity (m/s^2)
COEFF_DRAG = 4 # drag coefficient **can be changed**
AIR_DENSITY = 1.225 # air density (kg/m^3)
MASS = 18.6 # mass (kg) **can be changed**

# Rocket Dimensions
DIAMETER = 0.1524 # diameter in meters (6 inches) **can be changed**
AREA = np.pi * (DIAMETER / 2)**2 # Cross-sectional Area (m^2)

def rk4_apogee_predictor(h0, v0):

    dt = 1 / FREQUENCY # time steps for physics
    max_steps = int(MAX_TIME / dt) #seconds / dt


    h = h0 # height in meters
    v = v0 # velocity in meters per second
    for _ in range(max_steps):
        if v <= 0:
            break
        # Fourth order equations

        # k1_v, k1_h = derivatives(v, MASS, GRAV_CONST, COEFF_DRAG, AREA, AIR_DENSITY)
        # k2_v, k2_h = derivatives(v + 0.5*dt*k1_v, MASS, GRAV_CONST, COEFF_DRAG, AREA, AIR_DENSITY)
        # k3_v, k3_h = derivatives(v + 0.5*dt*k2_v, MASS, GRAV_CONST, COEFF_DRAG, AREA, AIR_DENSITY)
        # k4_v, k4_h = derivatives(v + dt*k3_v, MASS, GRAV_CONST, COEFF_DRAG, AREA, AIR_DENSITY)

        k1_v, k1_h = find_derivatives(v)
        k2_v, k2_h = find_derivatives(v + 0.5*dt*k1_v)
        k3_v, k3_h = find_derivatives(v + 0.5*dt*k2_v)
        k4_v, k4_h = find_derivatives(v + dt*k3_v)

        v += (dt/6)*(k1_v + 2*k2_v + 2*k3_v + k4_v)
        h += (dt/6)*(k1_h + 2*k2_h + 2*k3_h + k4_h)
    return h

def derivatives(velocity, mass, grav_const, Cd, A, rho):
    drag = 0.5 * Cd * A * rho * velocity**2 * np.sign(velocity)
    dv = -grav_const - drag / mass
    dh = velocity
    return dv, dh

def find_derivatives(velocity):
    drag = .5 * COEFF_DRAG * AREA * AIR_DENSITY * velocity**2 * np.sign(velocity)
    dv = -GRAV_CONST - drag / MASS
    dh = velocity
    return dv, dh
    

def plot_apogee_prediction(csv_filename):
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
    
    time = data[:, 3]       # Column 4 (0-indexed)
    status = data[:, 4]      # Column 5
    altitude = data[:, 9]    # Column 10
    velocity = data[:, 10]   # Column 11
    
    valid_mask = status == 5 # looks at rocket after motor burnout (state 5)
    time = time[valid_mask]
    altitude = altitude[valid_mask]
    velocity = velocity[valid_mask]
    
    # Initialize predicted apogee array
    N = len(time) # Number of data points to process
    predicted_apogees = np.full(N, np.nan) # Initialize with NaN
    
    sum = 0
    for i in range(N):
        time_start = perf_counter()
        predicted_apogees[i] = rk4_apogee_predictor(altitude[i], velocity[i])
        time_end = perf_counter()
        sum += time_end - time_start
    print(f"Average time to run {sum/N}")
    
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
    plot_apogee_prediction(r"C:\Users\bigdu\Downloads\Summer 2025\DRSS\MATLAB\Flight Data\4-12 backup flight 2.csv")
