import math
import time
import threading
from IMU import VN100IMU
from Kalman import KalmanFilter
from collections import deque
import os
from Servo import SinceCam
from multiprocessing import Process, Queue

class DataLogger:
    def __init__(self, output_directory="IMU_DATA", target_frequency=100):
        os.makedirs(output_directory, exist_ok=True)
        
        self.pre_file = os.path.join(output_directory, "data_log_pre.txt")
        self.post_file = os.path.join(output_directory, "data_log_post.txt")
        self.output_file = os.path.join(output_directory, "data_log_combined.txt")

        self.interval = 1 / target_frequency

    def file_writer_process(self, queue):
        while True:
            if not queue.empty():
                batch = queue.get()
                triggerAltitudeAchieved = batch['trigger']
                data_strs = batch['data']
                
                if not triggerAltitudeAchieved:
                    with open(self.pre_file, "a") as pre_f:
                        pre_f.write("".join(data_strs))
                else:
                    with open(self.post_file, "a") as post_f:
                        post_f.write("".join(data_strs))

    def combine_files(self):
        headers = "Time,Yaw,Pitch,Roll,a_x,a_y,a_z,Temperature,Pressure,Altitude,kf_velocity,kf_altitude,triggerAltitudeAchieved\n"
        with open(self.output_file, "w") as out_file:
            out_file.write(headers)
            with open(self.pre_file, "r") as pre_f:
                out_file.write(pre_f.read())
            with open(self.post_file, "r") as post_f:
                out_file.write(post_f.read())

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

def read_data_and_queue(imu, queue, triggerAltitudeAchieved, kf, interval):
    rolling_buffer = deque(maxlen=50)
    last_logging_time = time.perf_counter()

    while True:
        current_time = time.perf_counter()
        if current_time - last_logging_time >= interval:
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
                
                rolling_buffer.append(data_str)
                
                if len(rolling_buffer) == 50:
                    queue.put({'data': list(rolling_buffer), 'trigger': triggerAltitudeAchieved})
                    rolling_buffer.clear()
            
            last_logging_time += interval

if __name__ == "__main__":
    imu = VN100IMU()
    servoMotor = SinceCam()

    triggerAltitudeAchieved = False
    servoMotor.set_angle(0)

    data_logger = DataLogger()
    queue = Queue()

    # Start a separate process for writing files
    writer_process = Process(target=data_logger.file_writer_process, args=(queue, ))
    writer_process.start()

    # Calculate ground altitude
    groundAltitude = calculate_ground_altitude(imu)

    # Initialize Kalman filter
    kf = KalmanFilter(dt=data_logger.interval)

    try:
        read_data_and_queue(imu, queue, triggerAltitudeAchieved, kf, data_logger.interval)
    except KeyboardInterrupt:
        print("Stopping due to KeyboardInterrupt.")
        
        writer_process.terminate()
        data_logger.combine_files()
        print("Data logging completed. Logs combined.")
