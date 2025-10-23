import math
import time
from IMU import VN100IMU
from Kalman import KalmanFilter
from collections import deque
import os
import threading
import csv
import signal
import sys
from Servo import SinceCam
from Rk4_v1 import Rk4
from erFilter import ERFilter
from datetime import datetime
#############################################################################
"""
Deploys fins based on altitude trigger or RK4 apogee prediction, 
Kalman and erFilter
Has launch detection at 3 G for 0.10s on x axis (axial axis)
Retracts fins at apogee when the velocity is negative for 0.5s after launch and 100 ft of altitude are reached
Detects landing when no new minimum is detected for 10s, and shuts down the program
"""
# Conversions
in_to_m = 0.0254
ft_to_m = 0.3048
m_to_ft = 3.28084
lb_to_kg = 0.453592
kg_to_lb = 1/lb_to_kg
ft2_to_m2 = 0.092903
g_to_kg = 0.001
# Physical Constants
LAUNCH_ACCELERATION = 0.9 # In g
PROPELLANT_MASS = 163 * g_to_kg # kg
ROCKET_DRY_MASS = (13.02 * lb_to_kg) - PROPELLANT_MASS # DRY MASS in kg
ROCKET_DIAMETER = 4.014 * in_to_m # m
ROCKET_AREA = (math.pi/4) * (ROCKET_DIAMETER)**2 # m^2
ACS_CD = 3.45 # CD of rocket with fins deployed
MOTOR_BURN_TIME = 1.0 # 1.1 for I470, 1.5 I366, 1.0 for I357
# Code constants
LOGGER_BUFFER = 1000  # 10 sec (100 Hz)
TARGET_FREQ = 100 # main loop frequency
INTERVAL = 1 / TARGET_FREQ
PRINT_INTERVAL = 60 # Print velocity and altitude readouts to terminal every 30s
IMU_INTERVAL = 1/200 # IMU runs at 200 Hz
PREWRITE_INTERVAL = 1  # Limit writes to pre_file every 1 second
POSTWRITE_INTERVAL = 1  # Limit writes to post_file every 1 second
# Flight Constants
TRIGGER_ALTITUDE = 50000 # ft
TARGET_APOGEE = 10000 # ft
EVAN_LENGTH = 50
VEL_GAP = 15
SERVO_START = 80
SERVO_ANGLE = 125 # deg
#############################################################################

def calculate_ground_altitude(imu):
    print("Calculating ground altitude...")
    altitude_readings = []

    for _ in range(50): # 0.5 s of data to calculate ground level
        while True:
            imu.readData()
            if imu.currentData:
                altitude_readings.append(imu.currentData.altitude)
                break

    groundAltitude = sum(altitude_readings) / len(altitude_readings)
    print(f"Ground Altitude: {groundAltitude:.2f} ft")
    return groundAltitude

def combine_files(pre_file, post_file, output_file):
    with open(output_file, "w", newline='') as out_file:
        writer = csv.writer(out_file)
        headers = ["Time", "Yaw", "Pitch", "Roll", "a_x", "a_y", "a_z",
                   "Pressure", "Altitude",
                   "launch_detected",
                   "kf_velocity", "kf_altitude",
                   "er_velocity", "er_altitude", 
                   "predicted_apogee", "trigger_achieved", "fins_retracted", "er_apogee_pred"]
        writer.writerow(headers)

        for file_path in [pre_file, post_file]:
            if os.path.exists(file_path):
                with open(file_path, "r") as in_file:
                    for line in in_file:
                        writer.writerow(line.strip().split(","))

def imu_reader(imu, imu_deque, stop_event):
    while not stop_event.is_set():
        data = imu.readData()
        if data:
            imu_deque.append((time.perf_counter(), data))
        else:
            pass
            print("[imu_reader] No new data")
        time.sleep(IMU_INTERVAL * 0.05)

def data_logging_process(imu_deque, stop_event, groundAltitude, trigger_flag, kf, servoMotor, launched_flag, er, Rk4_model, apogee_flag):
    output_directory = os.path.join("/home/vadl/Apogee-Control/IMU_DATA")
    os.makedirs(output_directory, exist_ok=True)

    # pre and post based on fin trigger, not launch
    pre_file_path = os.path.join(output_directory, f"data_log_pre_{timestamp}.csv")
    post_file_path = os.path.join(output_directory, f"data_log_post_{timestamp}.csv")
    output_file = os.path.join(output_directory, f"data_log_combined_{timestamp}.csv")

    # Open pre and post files and reader
    pre_file = open(pre_file_path, "w", newline='')
    post_file = open(post_file_path, "a", newline='')

    pre_writer = csv.writer(pre_file)
    post_writer = csv.writer(post_file)

    pre_trigger_buffer = deque(maxlen=LOGGER_BUFFER)
    pre_buffer_finished = False  # flag to prevent pre-file from being written to after trigger
    post_trigger_buffer = []

    last_print_time = 0
    last_prewrite_time = 0
    last_postwrite_time = 0

    consecutive_readings_gs = 0
    consecutive_readings_alt = 0
    consecutive_readings_rk4 = 0
    consecutive_readings_retract = 0
    consecutive_no_new_minimum = 0

    def flush_pre_buffer(pre_file):
        if pre_trigger_buffer:
            pre_file.seek(0)
            pre_file.truncate()
            pre_writer.writerows(pre_trigger_buffer)
            pre_file.flush()

    def flush_post_buffer(post_file):
        if post_trigger_buffer:
            post_writer.writerows(post_trigger_buffer)
            post_trigger_buffer.clear()
            post_file.flush()

    next_logging_time = time.perf_counter()

    while not stop_event.is_set():
        current_time = time.perf_counter()
        sleep_time = next_logging_time - current_time
        if sleep_time > 0:
            time.sleep(sleep_time)
        next_logging_time += INTERVAL

        if not imu_deque:
            continue

        # Only keep the newest data
        while len(imu_deque) > 1:
            imu_deque.popleft()

        current_time, imu_data = imu_deque.pop()
        current_altitude = imu_data.altitude

        #### erFilter ####
        er.process_data(current_time, current_altitude)
        altitude_er, velocity_er = er.get_results()

        #### Kalman Filter ####
        altitude_kf, velocity_kf= kf.update(current_altitude)

        # Launch detection logic
        if not launched_flag[0]:
            if abs(imu_data.a_x) > LAUNCH_ACCELERATION:
                consecutive_readings_gs += 1
            else:
                consecutive_readings_gs = 0
            if consecutive_readings_gs >= 10: # 10 data points (0.1) s
                launched_flag[0] = True
                print(f"[Launch Detection] Acceleration of {imu_data.a_x:.2f} exceeds {LAUNCH_ACCELERATION} G")
                launch_time = current_time

        # Check to ensure no premature fin deployment from RK4
        if launched_flag[0]:
            #### RK4 MODEL #################################################################################
            apogee_prediction_m = Rk4_model.rk4_apogee_predictor(current_altitude * ft_to_m, velocity_kf * ft_to_m)
            apogee_prediction_ft = apogee_prediction_m * m_to_ft # conversion to feet (ft)
            
            er_apogee_m = Rk4_model.rk4_apogee_predictor(current_altitude * ft_to_m, velocity_er * ft_to_m)
            er_apogee_ft = er_apogee_m * m_to_ft
            #################################################################################################
            
            ### Retract Logic ######################################################################
            if (not apogee_flag[0]) and (current_altitude > groundAltitude + 100) : #remove and condition for non lab testing
                if velocity_kf < 0:
                    consecutive_readings_retract += 1
                else:
                    consecutive_readings_retract = 0

                if consecutive_readings_retract >= 50: #0.5s
                    apogee_flag[0] = True
                    print(f"[Apogee Detected]")
                    previous_altitude = current_altitude
                    os.fsync(pre_file.fileno()) # write to disk
                    os.fsync(post_file.fileno())
                    print(f"Fsync to disk")
                    #servoMotor.set_angle(SERVO_START) # disable retract for now
            ########################################################################################
        else:
            apogee_prediction_ft = current_altitude
            er_apogee_ft = current_altitude
        
        # Print at a reduced rate
        if current_time - last_print_time >= PRINT_INTERVAL:
            print(f"[IMU] Alt={current_altitude - groundAltitude:.2f} ft AGL, KF Vel={velocity_kf:.2f} ft/s , ER Vel ={velocity_er:.2f} ft/s")
            print(f"[RK4] Projected Apogee = {apogee_prediction_ft - groundAltitude:.2f} ft AGL")
            last_print_time = current_time

        #### Trigger logic ####################################################################
        if not trigger_flag[0]:
            if current_altitude > groundAltitude + TRIGGER_ALTITUDE:
                consecutive_readings_alt += 1
            else:
                consecutive_readings_alt = 0

            if (apogee_prediction_ft > groundAltitude + TARGET_APOGEE) and (current_time > launch_time + MOTOR_BURN_TIME):
                consecutive_readings_rk4 += 1
            else:
                consecutive_readings_rk4 = 0

            # Fin trigger based on either alt check or goal apogee achieved
            if (consecutive_readings_alt >= 5): # 0.05 s
                trigger_flag[0] = True
                print(f"[Trigger] Target Altitude of {current_altitude - groundAltitude:.2f} ft AGL Achieved!")
                servoMotor.set_angle(SERVO_ANGLE)
            
            if (consecutive_readings_rk4 >= 5): # 0.05 s
                trigger_flag[0] = True
                print(f"[Trigger] Goal Apogee of {apogee_prediction_ft - groundAltitude:.2f} ft AGL Achieved!")
                servoMotor.set_angle(SERVO_ANGLE)
        ########################################################################################
        
        ### Landed Logic #######################################################################
        if apogee_flag[0]:
            if current_altitude < previous_altitude:
                previous_altitude = current_altitude
                consecutive_no_new_minimum = 0

            else:
                consecutive_no_new_minimum += 1
            
            if (consecutive_no_new_minimum >= 1000): #10 seconds
                print(f"[Landing Detection] No new minimum altitude seen for {consecutive_no_new_minimum/TARGET_FREQ:.2f} seconds")
                stop_event.set()
        ########################################################################################

        # Format the data row
        data_row = [
            f"{current_time:.6f}",
            f"{imu_data.yaw:.2f}", f"{imu_data.pitch:.2f}", f"{imu_data.roll:.2f}",
            f"{imu_data.a_x:.2f}", f"{imu_data.a_y:.2f}", f"{imu_data.a_z:.2f}",
            f"{imu_data.pressure:.2f}", f"{imu_data.altitude:.2f}",
            f"{int(launched_flag[0])}", 
            f"{velocity_kf:.2f}", f"{altitude_kf:.2f}",
            f"{velocity_er:.2f}", f"{altitude_er:.2f}",
            f"{apogee_prediction_ft:.2f}", f"{int(trigger_flag[0])}",
            f"{int(apogee_flag[0])}", f"{er_apogee_ft:.2f}"
        ]

        # data logging logic
        if not trigger_flag[0]:
            pre_trigger_buffer.append(data_row)
            if current_time - last_prewrite_time >= PREWRITE_INTERVAL:
                flush_pre_buffer(pre_file)
                last_prewrite_time = current_time
        else:
            if not pre_buffer_finished:
                flush_pre_buffer(pre_file)
                pre_buffer_finished = True

            post_trigger_buffer.append(data_row)
            if current_time - last_postwrite_time >= POSTWRITE_INTERVAL:
                flush_post_buffer(post_file)
                last_postwrite_time = current_time

        last_time = current_time

    # flush after final loop
    flush_pre_buffer(pre_file)
    flush_post_buffer(post_file)
    pre_file.close()
    post_file.close()
    combine_files(pre_file_path, post_file_path, output_file)
    print(f"[Shutdown] Data logging completed. Combined logs into {output_file}")


if __name__ == "__main__":
    # differentiate between files to prevent overwriting
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # directory for terminal .txt
    os.makedirs("/home/vadl/Apogee-Control/IMU_DATA", exist_ok=True)
    log_path = os.path.join("/home/vadl/Apogee-Control/IMU_DATA", f"terminal_{timestamp}.txt")
    sys.stdout = open(log_path, "w", buffering=1)  # line-buffered
    sys.stderr = sys.stdout

    # set constants
    launched_flag = [False]
    apogee_flag = [False]
   
    imu = VN100IMU()
    time.sleep(1.0)

    servoMotor = SinceCam()
    servoMotor.set_angle(SERVO_START)

    stop_event = threading.Event()
    triggerAltitudeAchieved = [False]  

    groundAltitude = calculate_ground_altitude(imu)

    # Initialize classes
    er = ERFilter(evan_length=EVAN_LENGTH, velocity_gap=VEL_GAP)
    kf = KalmanFilter(dt=INTERVAL,ground = groundAltitude)
    Rk4_model = Rk4(frequency=10, coeeff_drag=ACS_CD, mass=ROCKET_DRY_MASS, area=ROCKET_AREA) # 10 Hz for simulation loop (dt = 0.1)
    
    #initialize RK4 variables
    apogee_prediction_m = 0
    apogee_prediction_ft = 0
    er_apogee_m = 0
    er_apogee_ft = 0

    imu_deque = deque(maxlen=1000) # 10 s of imu data

    imu_thread = threading.Thread(target=imu_reader, args=(imu, imu_deque, stop_event))

    logging_thread = threading.Thread(target=data_logging_process,
                                      args=(imu_deque, stop_event, groundAltitude,
                                            triggerAltitudeAchieved, kf, servoMotor, launched_flag , er,
                                            Rk4_model, apogee_flag))

    def handle_shutdown(signum, frame):
        print("Signal received. Shutting down...")
        stop_event.set()

    # Handle SIGINT, SIGTERM, and SIGPIPE
    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)
    signal.signal(signal.SIGPIPE, signal.SIG_DFL)

    imu_thread.start()
    logging_thread.start()

    try:
        while not stop_event.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[Main] KeyboardInterrupt received.")
        stop_event.set()

    imu_thread.join()
    logging_thread.join()
    print("[Main] Shutdown complete.")

