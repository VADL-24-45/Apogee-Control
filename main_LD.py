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
Has launch detection at 3 G for 13 points (0.13s) on x axis
Retracts fins when the velocity is negative for 0.5s
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
LAUNCH_ACCELERATION = 1 # In g
PROPELLANT_MASS = 247.2 * g_to_kg # kg
ROCKET_DRY_MASS = (13.2 *lb_to_kg) - PROPELLANT_MASS # DRY MASS in kg
ROCKET_DIAMETER = 4.014 * in_to_m # m
ROCKET_AREA = (math.pi/4) * (ROCKET_DIAMETER)**2 # m^2
ACS_CD = 5 # CD of rocket with fins deployed
MOTOR_BURN_TIME = 1.1 # 1.1 for I470, 1.5 I366
# Code constants
LOGGER_BUFFER = 3000  # 30 sec (100 Hz)
TARGET_FREQ = 100 # main loop frequency
INTERVAL = 1 / TARGET_FREQ
PRINT_INTERVAL = 5 # Print velocity and altitude readouts to terminal every 5s
IMU_INTERVAL = 1/200 # IMU runs at 200 Hz
PREWRITE_INTERVAL = 0.1  # Limit writes to pre_file every 0.1 seconds
POSTWRITE_INTERVAL = 0.1  # Limit writes to post_file every 0.1 seconds
# Flight Constants
TRIGGER_ALTITUDE = 430 # ft
TARGET_APOGEE = 750 # ft
EVAN_LENGTH = 50
VEL_GAP = 15
SERVO_ANGLE = 90 # deg
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
        time.sleep(IMU_INTERVAL * 0.95)


def data_logging_process(imu_deque, stop_event, groundAltitude, trigger_flag, kf, servoMotor, launched_flag, er, Rk4_model, retract_flag):
    output_directory = os.path.join("IMU_DATA")
    os.makedirs(output_directory, exist_ok=True)

    # pre and post based on fin trigger, not launch
    pre_file = os.path.join(output_directory, "data_log_pre.csv")
    post_file = os.path.join(output_directory, "data_log_post.csv")
    output_file = os.path.join(output_directory, "data_log_combined.csv")
    terminal_file = os.path.join(output_directory, "terminal.txt")

    pre_trigger_buffer = deque(maxlen=LOGGER_BUFFER)
    post_trigger_buffer = []

    last_print_time = 0
    last_prewrite_time = 0
    last_postwrite_time = 0

    consecutive_readings_gs = 0
    consecutive_readings_alt = 0
    consecutive_readings_rk4 = 0
    consecutive_readings_retract = 0
    required_consecutive = 50

    def flush_pre_buffer():
        with open(pre_file, "w", newline='') as pre_f:
            writer = csv.writer(pre_f)
            writer.writerows(pre_trigger_buffer)
        #print("[Log] Pre-trigger buffer flushed to disk.") # commented out to reduce file size

    def flush_post_buffer():
        if post_trigger_buffer:
            with open(post_file, "a", newline='') as post_f:
                writer = csv.writer(post_f)
                writer.writerows(post_trigger_buffer)
            #print(f"[Log] Flushed {len(post_trigger_buffer)} post-trigger rows.")
            post_trigger_buffer.clear()

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
            if consecutive_readings_gs >= required_consecutive / 4: # 13 data points (0.13) s
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
            if not retract_flag[0]:
                #pass
                if velocity_kf < 0:
                    consecutive_readings_retract += 1
                else:
                    consecutive_readings_retract = 0

                if consecutive_readings_retract >= required_consecutive:
                    retract_flag[0] = True
                    servoMotor.set_angle(0)
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
            if (consecutive_readings_alt >= (required_consecutive / 10)): # 0.05 s
                trigger_flag[0] = True
                print(f"[Trigger] Target Altitude of {current_altitude - groundAltitude:.2f} ft AGL Achieved!")
                servoMotor.set_angle(SERVO_ANGLE)
            
            if (consecutive_readings_rk4 >= (required_consecutive / 10 )): # 0.05 s
                trigger_flag[0] = True
                print(f"[Trigger] Goal Apogee of {apogee_prediction_ft - groundAltitude:.2f} ft AGL Achieved!")
                servoMotor.set_angle(SERVO_ANGLE)
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
            f"{int(retract_flag[0])}", f"{er_apogee_ft:.2f}"
        ]

        # data logging logic
        if not trigger_flag[0]:
            pre_trigger_buffer.append(data_row)
            if current_time - last_prewrite_time >= PREWRITE_INTERVAL:
                flush_pre_buffer()
                last_prewrite_time = current_time
        else:
            flush_pre_buffer()
            post_trigger_buffer.append(data_row)
            if current_time - last_postwrite_time >= POSTWRITE_INTERVAL:
                flush_post_buffer()
                last_postwrite_time = current_time

        last_time = current_time

    # flush after final loop
    flush_pre_buffer()
    flush_post_buffer()
    combine_files(pre_file, post_file, output_file)
    print(f"[Shutdown] Data logging completed. Combined logs into {output_file}")


if __name__ == "__main__":
    # directory for terminal .txt
    os.makedirs("IMU_DATA", exist_ok=True)
    log_path = os.path.join("IMU_DATA", "terminal.txt")
    sys.stdout = open(log_path, "w", buffering=1)  # line-buffered
    sys.stderr = sys.stdout

    # set constants
    launched_flag = [False]
    retract_flag = [False]
   
    imu = VN100IMU()
    time.sleep(1.0)

    servoMotor = SinceCam()
    servoMotor.set_angle(0)

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
                                            Rk4_model, retract_flag))

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
