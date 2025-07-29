import time
from IMU import VN100IMU
from collections import deque
import os
import threading
import csv
import signal
from Servo import SinceCam
from Kalman import KalmanFilter
from erFilter import DataProcessor
from Rk4_v1 import Rk4

#############################################################################
# Constants
LOGGER_BUFFER = 18000
GOAL_APOGEE = 5000
TARGET_FREQ = 100
INTERVAL = 1 / TARGET_FREQ
PRINT_INTERVAL = 1
IMU_INTERVAL = 1/200
PREWRITE_INTERVAL = 10
POSTWRITE_INTERVAL = 10
EVAN_LENGTH = 50
VEL_GAP = 15
#############################################################################

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
    with open(output_file, "w", newline='') as out_file:
        writer = csv.writer(out_file)
        headers = ["Time", "Yaw", "Pitch", "Roll", "a_x", "a_y", "a_z",
                   "Pressure", "Altitude", "velocity_er", "smoothed_altitude",
                   "kf_altitude", "velocity_kf",
                   "predicted_apogee", "triggerAltitudeAchieved"]
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
        time.sleep(IMU_INTERVAL * 0.95)

def data_logging_process(imu_deque, stop_event, groundAltitude, trigger_flag, data_processor, rk4_model, servoMotor):
    output_directory = "IMU_DATA"
    os.makedirs(output_directory, exist_ok=True)

    pre_file = os.path.join(output_directory, "data_log_pre.csv")
    post_file = os.path.join(output_directory, "data_log_post.csv")
    output_file = os.path.join(output_directory, "data_log_combined.csv")

    pre_trigger_buffer = deque(maxlen=12000)
    post_trigger_buffer = []

    last_print_time = 0
    last_prewrite_time = 0
    last_postwrite_time = 0

    consecutive_readings = 0
    required_consecutive = 5

    def flush_pre_buffer():
        if pre_trigger_buffer:
            with open(pre_file, "a", newline='') as pre_f:
                writer = csv.writer(pre_f)
                writer.writerows(pre_trigger_buffer)
            pre_trigger_buffer.clear()
            print("[Log] Pre-trigger buffer flushed to disk.")

    def flush_post_buffer():
        if post_trigger_buffer:
            with open(post_file, "a", newline='') as post_f:
                writer = csv.writer(post_f)
                writer.writerows(post_trigger_buffer)
            post_trigger_buffer.clear()
            print(f"[Log] Flushed {len(post_trigger_buffer)} post-trigger rows.")

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

        # Process the current altitude
        data_processor.process_data(current_time, current_altitude)
        smoothed_altitude, velocity_er = data_processor.get_results()

        altitude_kf, velocity_kf = kf.update(current_altitude)

        # RK4 model for apogee prediction
        apogee_prediction_m = rk4_model.rk4_apogee_predictor(smoothed_altitude * 0.3048, velocity_er * 0.3048)
        apogee_prediction_ft = apogee_prediction_m * 3.28084 

        # Print at a reduced rate
        if current_time - last_print_time >= PRINT_INTERVAL:
            print(f"[IMU] Raw Altitude={imu_data.altitude:.2f} ft, Smoothed Altitude={smoothed_altitude:.2f} ft, Velocity={velocity_er:.2f} ft/s")
            print(f"[RK4] Projected Apogee = {apogee_prediction_ft:.2f} ft")
            last_print_time = current_time

        # Trigger logic
        if not trigger_flag[0]:
            if apogee_prediction_ft > groundAltitude + GOAL_APOGEE:
                consecutive_readings += 1
            else:
                consecutive_readings = 0

            if consecutive_readings >= required_consecutive:
                trigger_flag[0] = True
                print("[RK4] Target Apogee Predicted!")
                servoMotor.set_angle(45)

        # Format the data row
        data_row = [
            f"{current_time:.6f}",
            f"{imu_data.yaw:.2f}", f"{imu_data.pitch:.2f}", f"{imu_data.roll:.2f}",
            f"{imu_data.a_x:.2f}", f"{imu_data.a_y:.2f}", f"{imu_data.a_z:.2f}",
            f"{imu_data.pressure:.2f}",
            f"{imu_data.altitude:.2f}", f"{velocity_er:.2f}", f"{smoothed_altitude:.2f}",
            f"{altitude_kf:.2f}", f"{velocity_kf:.2f}",
            f"{apogee_prediction_ft:.2f}", f"{int(trigger_flag[0])}"
        ]

        if not trigger_flag[0]:
            pre_trigger_buffer.append(data_row)
            if current_time - last_prewrite_time >= PREWRITE_INTERVAL:
                flush_pre_buffer()
                last_prewrite_time = current_time
        else:
            post_trigger_buffer.append(data_row)
            if current_time - last_postwrite_time >= POSTWRITE_INTERVAL:
                flush_post_buffer()
                last_postwrite_time = current_time

    # Final flush after loop ends
    flush_pre_buffer()
    flush_post_buffer()
    combine_files(pre_file, post_file, output_file)
    print("[Shutdown] Data logging completed. Combined logs into", output_file)

if __name__ == "__main__":
    imu = VN100IMU()
    time.sleep(1.0)
    servoMotor = SinceCam()
    servoMotor.set_angle(0)

    # Initialize DataProcessor
    data_processor = DataProcessor(evan_length=EVAN_LENGTH, velocity_gap=VEL_GAP)
    kf = KalmanFilter(dt=INTERVAL) # initialize Kalman filter
    rk4_model = Rk4(10)  # RK4 at 10 Hz for simulation
    stop_event = threading.Event()
    triggerAltitudeAchieved = [False] 

    def handle_shutdown(signum, frame):
        print("Signal received. Shutting down...")
        stop_event.set()

    # Handle SIGINT and SIGTERM
    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)

    groundAltitude = calculate_ground_altitude(imu)

    imu_deque = deque(maxlen=50)  # Limit size for better performance

    imu_thread = threading.Thread(target=imu_reader, args=(imu, imu_deque, stop_event))
    logging_thread = threading.Thread(target=data_logging_process,
                                      args=(imu_deque, stop_event, groundAltitude,
                                            triggerAltitudeAchieved, data_processor, rk4_model, servoMotor))

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
