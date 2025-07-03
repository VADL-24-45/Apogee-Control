import math
import time
from IMU import VN100IMU
from Kalman import KalmanFilter
from collections import deque
import os
from Servo import SinceCam

def calculate_ground_altitude(imu):
    print("Calculating ground altitude...")
    altitude_readings = []

    for _ in range(5):
        while True:
            imu.readData()
            if imu.currentData:
                altitude_readings.append(imu.currentData.altitude)
                break
        time.sleep(0.1)

    groundAltitude = sum(altitude_readings) / len(altitude_readings)
    print(f"Ground Altitude: {groundAltitude:.2f} ft")

    return groundAltitude

def combine_files(pre_file, post_file, output_file):
    headers = "Time,Yaw,Pitch,Roll,a_x,a_y,a_z,Temperature,Pressure,Altitude,kf_velocity,kf_altitude,triggerAltitudeAchieved\n"
    with open(output_file, "w") as out_file:
        out_file.write(headers)
        with open(pre_file, "r") as pre_f:
            out_file.write(pre_f.read())
        with open(post_file, "r") as post_f:
            out_file.write(post_f.read())

if __name__ == "__main__":
    imu = VN100IMU()
    servoMotor = SinceCam()

    stop_event = threading.Event()
    triggerAltitudeAchieved = False
    servoMotor.set_angle(0)

    # Initialize Kalman filter
    kf = KalmanFilter(dt=1/100)

    groundAltitude = calculate_ground_altitude(imu)

    try:
        while not stop_event.is_set():
            current_time = time.perf_counter()
            
            if current_time - last_logging_time >= interval:
                current_time = last_logging_time
                imu.readData()
                if imu.currentData:
                    data_str = (
                        f"{current_time:.2f},"
                        f"{imu.currentData.yaw:.2f},"
                        f"{imu.currentData.pitch:.2f},"
                        f"{imu.currentData.roll:.2f},"
                        f"{imu.currentData.a_x:.2f},"
                        f"{imu.currentData.a_y:.2f},"
                        f"{imu.currentData.a_z:.2f},"
                        f"{imu.currentData.temperature:.2f},"
                        f"{imu.currentData.pressure:.2f},"
                        f"{imu.currentData.altitude:.2f},"
                        "0.0,"  # Placeholder for velocity_estimate
                        "0.0,"  # Placeholder for altitude_estimate
                        f"{int(triggerAltitudeAchieved)}\n"
                    )
                    print(data_str)
            
            pass
    except KeyboardInterrupt:
        print("Stopping due to KeyboardInterrupt.")
        
        combine_files(pre_file, post_file, output_file)
        print(f"Data logging completed. Logs combined into {output_file}")

    
