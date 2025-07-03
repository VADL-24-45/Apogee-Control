from IMU_Test import VN100IMU
import time

imu = VN100IMU()
time.sleep(1.0)  # Give thread time to start

for _ in range(500):
    data = imu.readData()
    if data:
        print(f"Altitude: {data.altitude:.2f} ft")
    else:
        print("Waiting for IMU data...")
    time.sleep(0.1)
