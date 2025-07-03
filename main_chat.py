import math
import time
import threading
import os
from multiprocessing import Process, Queue
from IMU import VN100IMU
from Kalman import KalmanFilter
from Servo import SinceCam
from collections import deque

class DataLogger:
    def __init__(self, output_directory="IMU_DATA", target_frequency=100):
        os.makedirs(output_directory, exist_ok=True)
        
        self.pre_file = os.path.join(output_directory, "data_log_pre.txt")
        self.post_file = os.path.join(output_directory, "data_log_post.txt")
        self.output_file = os.path.join(output_directory, "data_log_combined.txt")
        
        self.queue = Queue()
        self.buffer = []
        self.buffer_size = 50
        self.interval = 1 / target_frequency

        self.writer_process = Process(target=self._file_writer)
        self.writer_process.start()

    def log_data(self, data_str, triggerAltitudeAchieved):
        self.buffer.append(data_str)
        if len(self.buffer) >= self.buffer_size:
            # Pass the buffer and the trigger status to the separate process
            self.queue.put({"data": list(self.buffer), "trigger": triggerAltitudeAchieved})
            self.buffer.clear()

    def file_writer(self):
        while True:
            if not self.queue.empty():
                batch = self.queue.get()
                triggerAltitudeAchieved = batch["trigger"]
                data_strs = batch["data"]
                
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

def data_logging_process(imu, data_logger, triggerAltitudeAchieved, kf):
    while True:
        imu.readData()
        if imu.currentData:
            current_time = time.perf_counter()
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

            current_time = time.perf_counter()
            if current_time - data_logger.last_logging_time >= data_logger.interval:
                data_logger.log_data(data_str, triggerAltitudeAchieved)
                data_logger.last_logging_time = current_time

if __name__ == "__main__":
    imu = VN100IMU()
    servoMotor = SinceCam()

    stop_event = threading.Event()
    triggerAltitudeAchieved = False
    servoMotor.set_angle(0)

    data_logger = DataLogger()

    # Launch a separate process for file I/O operations
    writer_process = Process(target=data_logger.file_writer_process)
    writer_process.start()

    # Initialize Kalman filter
    kf = KalmanFilter(dt=data_logger.interval)

    groundAltitude = calculate_ground_altitude(imu)

    try:
        while not stop_event.is_set():
            data_logging_process(imu, data_logger.buffer, triggerAltitudeAchieved, data_logger.pre_file, data_logger.post_file)
            time.sleep(data_logger.interval)  # Ensures consistent reading interval
    except KeyboardInterrupt:
        print("Stopping due to KeyboardInterrupt.")
        data_logger.combine_files()
        print("Data logging completed. Logs combined.")

    # Terminate the writer process cleanly
    writer_process.terminate()
    writer_process.join()
