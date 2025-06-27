# Import libraries (use pip install numpy pandas matplotlib) if first time 
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

def rk4_apogee_predictor(h0, v0):
    g = 9.80665 # gravity (m/s^2)
    Cd = 4 # drag coefficient **can be changed**
    rho = 1.225 # air density (kg/m^3)
    m = 18.6 # mass (kg) **can be changed**
    
    # Rocket Dimensions
    diameter = 0.1524 # diameter in meters (6 inches) **can be changed**
    A = np.pi * (diameter / 2)**2 # Cross-sectional Area (m^2)


    dt = 0.05 # time steps for physics
    max_steps = 10000

    h = h0
    v = v0
    for _ in range(max_steps):
        if v <= 0:
            break
        # Fourth order equations
        k1_v, k1_h = derivatives(v, m, g, Cd, A, rho)
        k2_v, k2_h = derivatives(v + 0.5*dt*k1_v, m, g, Cd, A, rho)
        k3_v, k3_h = derivatives(v + 0.5*dt*k2_v, m, g, Cd, A, rho)
        k4_v, k4_h = derivatives(v + dt*k3_v, m, g, Cd, A, rho)
        v += (dt/6)*(k1_v + 2*k2_v + 2*k3_v + k4_v)
        h += (dt/6)*(k1_h + 2*k2_h + 2*k3_h + k4_h)
    return h

def derivatives(v, m, g, Cd, A, rho):
    drag = 0.5 * Cd * A * rho * v**2 * np.sign(v)
    dv = -g - drag / m
    dh = v
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
    
    for i in range(N):
        predicted_apogees[i] = rk4_apogee_predictor(altitude[i], velocity[i])
    
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
    plot_apogee_prediction(r"C:\Users\bigdu\Downloads\Summer 2025\MATLAB\Flight Data\4-12 backup flight 2.csv")

