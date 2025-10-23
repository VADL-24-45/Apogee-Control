import serial
import time
import math
import struct
import threading
from dataclasses import dataclass

@dataclass
class IMUData:
    """Data structure to hold IMU readings."""
    yaw: float       # deg
    pitch: float     # deg
    roll: float      # deg
    a_x: float       # G
    a_y: float       # G 
    a_z: float       # G
    pressure: float     # kPa
    altitude: float     # ft (derived from pressure)

class VN100IMU:
    """
    VN100IMU class to interface with the VectorNav VN-100 IMU sensor.
    
    This class provides methods to initialize the IMU, read data, validate messages,
    and parse data into structured format. It uses serial communication to receive
    data from the IMU.
    """
    def __init__(self, port='/dev/serial0', baudrate=115200, timeout=1):
        """
        Initializes the VN100IMU class with specified serial port parameters.
        
        :param port: Serial port to connect to the IMU (default: '/dev/serial0').
        :param baudrate: Communication speed (default: 115200).
        :param timeout: Read timeout in seconds (default: 1).
        """
        # Initialize serial connection
        self.serialConnection = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # Allow serial connection to stabilize
        self.initializeIMU()

        self.currentData = None
        self.lock = threading.Lock()
        self.stop_thread = threading.Event()
        self.read_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.read_thread.start()

    def initializeIMU(self):
        """
        Sends the initialization command to the IMU to configure its output.
        """
        self.serialConnection.write(b"$VNWRG,75,2,4,05,0108,0020*XX\r\n")
        ''' 
        explanation of above command:

        $VNWRG - write register
        75 - the register. register 75 is "binary output message configuration"
        2 - message sent out on port 2. port 2 uses 3V logic levels. port 1 uses 12V
        4 - rate divisor. 800 / 4 = 200 Hz
        05 - selecting groups 1 and 3
        0108 - selecting YPR and Acceleration from group 1. convert binary to hex
               (see table 17)
        0020 - selecting pres from group 3. convert binary to hex 
               (see table 17)
        '''
        time.sleep(1)
        print("IMU initialized, starting data monitoring")


    @staticmethod
    def calculateCRC(data, length):
        """
        Calculates the CRC (Cyclic Redundancy Check) for data integrity verification.
        :param data: The data byte array to calculate CRC on.
        :param length: The length of the data.
        :return: Computed CRC value as an integer.
        """
        crc = 0
        for i in range(length):
            crc = (crc >> 8) | ((crc << 8) & 0xFFFF)
            crc ^= data[i]
            crc ^= (crc & 0xFF) >> 4
            crc ^= (crc << 12) & 0xFFFF
            crc ^= ((crc & 0xFF) << 5) & 0xFFFF
        return crc

    def _reader_loop(self):
        """
        Thread to continuously read data from the IMU and send it to currentData
        """
        while not self.stop_thread.is_set():
            if self.serialConnection.in_waiting >= 36: # Check if there is data in the buffer
                sync = self.serialConnection.read(1)  # Read the first byte
                if sync == b'\xFA':
                    message = self.serialConnection.read(35) # Read the next 36 bytes
                    if len(message) == 35:
                        calc_crc = self.calculateCRC(message[:33], 33)
                        recv_crc = int.from_bytes(message[33:35], 'big')
                        if calc_crc == recv_crc:
                            parsed = self.parseMessage(message)
                            with self.lock:
                                self.currentData = parsed

    def parseMessage(self, message):
        """
        Parses the raw message bytes into an IMUData structure.
        
        :param message: The raw byte message received from the IMU.
        :return: IMUData object containing parsed IMU values.
        """
        # Extract YPR (deg)
        yaw = struct.unpack('<f', message[5:9])[0]
        pitch = struct.unpack('<f', message[9:13])[0]
        roll = struct.unpack('<f', message[13:17])[0]

        # Extract acceleration data (G)
        a_x = struct.unpack('<f', message[17:21])[0] / 9.80665
        a_y = struct.unpack('<f', message[21:25])[0] / 9.80665
        a_z = struct.unpack('<f', message[25:29])[0] / 9.80665
        
        # Extract pressure data (used for altitude)
        pressure  = struct.unpack('<f', message[29:33])[0]
        

        # Calculate altitude from pressure (ft MSL). 101.325 kPa ref
        sea_level_ref = 101.325  # kPa
        if pressure <= 0:
            altitude = float('nan')
        else:
            altitude = 145366.45 * (1 - math.pow(pressure / sea_level_ref, 0.190284))

        return IMUData(
            yaw=yaw, pitch=pitch, roll=roll,
            a_x=a_x, a_y=a_y, a_z=a_z,
            pressure=pressure,
            altitude=altitude
        )

    def readData(self):
        with self.lock:
            return self.currentData
        

    def __del__(self):
        """Destructor to ensure serial connection is closed properly."""
        if self.serialConnection and self.serialConnection.is_open:
            self.stop_thread.set()
            self.read_thread.join(timeout=1)
            print("Closing serial connection...")
            self.serialConnection.close()
    """ 
    def wait_for_data(self, timeout=5):
        #Block until first IMU data is available or timeout occurs.
        start_time = time.time()
        while self.currentData is None:
            if time.time() - start_time > timeout:
                raise TimeoutError("IMU did not produce data in time.")
            time.sleep(0.01)
 """
# Usage example
""" if __name__ == "__main__":
    imu = VN100IMU()
    try:
        while True:
            imu.readData()
            # Access currentData directly from other parts of the program if needed
            if imu.currentData:
                print(f"{imu.currentData.yaw:.3f}, {imu.currentData.pitch:.3f}, "
                      f"{imu.currentData.roll:.3f}, "
                      f"{imu.currentData.a_x:.3f}, {imu.currentData.a_y:.3f}, "
                      f"{imu.currentData.a_z:.3f}, "
                      f"{imu.currentData.pressure:.3f},"
                      f"{imu.currentData.altitude:.3f}") 
    except KeyboardInterrupt:
            print("Data monitoring interrupted.")"""

# Print raw data output and save to text file
if __name__ == "__main__":
    imu = VN100IMU()
    output_file = "imu_raw_binary_log.txt"
    try:
        print(f"Logging raw binary IMU data to {output_file}...\n")
        with open(output_file, "w") as f:
            while True:
                if imu.serialConnection.in_waiting:
                    # Read all available bytes
                    raw_bytes = imu.serialConnection.read(imu.serialConnection.in_waiting)

                    # Convert to binary (8 bits per byte)
                    binary_str = ' '.join(f'{b:08b}' for b in raw_bytes)

                    # Write to file (each read is one line)
                    f.write(binary_str + '\n')
                    f.flush()  # force write to disk so data isn't lost

                    # Optional: also print to console
                    print(binary_str)

                else:
                    time.sleep(0.01)

    except KeyboardInterrupt:
        print("Data monitoring interrupted.")