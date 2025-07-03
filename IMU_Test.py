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
    temperature: float  # °C
    pressure: float     # kPa
    altitude: float     # ft (derived from pressure)

class VN100IMU:
    def __init__(self, port='/dev/serial0', baudrate=115200, timeout=1):
        self.serialConnection = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # Allow serial connection to stabilize
        self.initializeIMU()

        self.currentData = None
        self.lock = threading.Lock()
        self.stop_thread = threading.Event()
        self.read_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.read_thread.start()

    def initializeIMU(self):
        # Note: Replace '*XX' with proper checksum if needed
        self.serialConnection.write(b"$VNWRG,75,2,5,05,0108,0030*XX\r\n")
        time.sleep(1)

    @staticmethod
    def calculateCRC(data, length):
        crc = 0
        for i in range(length):
            crc = (crc >> 8) | ((crc << 8) & 0xFFFF)
            crc ^= data[i]
            crc ^= (crc & 0xFF) >> 4
            crc ^= (crc << 12) & 0xFFFF
            crc ^= ((crc & 0xFF) << 5) & 0xFFFF
        return crc

    def _reader_loop(self):
        while not self.stop_thread.is_set():
            if self.serialConnection.in_waiting >= 40:
                sync = self.serialConnection.read(1)
                if sync == b'\xFA':
                    message = self.serialConnection.read(39)
                    if len(message) == 39:
                        calc_crc = self.calculateCRC(message[:37], 37)
                        recv_crc = int.from_bytes(message[37:39], 'big')
                        if calc_crc == recv_crc:
                            parsed = self.parseMessage(message)
                            with self.lock:
                                self.currentData = parsed

    def parseMessage(self, message):
        yaw = struct.unpack('<f', message[5:9])[0]
        pitch = struct.unpack('<f', message[9:13])[0]
        roll = struct.unpack('<f', message[13:17])[0]
        a_x = struct.unpack('<f', message[17:21])[0] / 9.80665
        a_y = struct.unpack('<f', message[21:25])[0] / 9.80665
        a_z = struct.unpack('<f', message[25:29])[0] / 9.80665
        temperature = struct.unpack('<f', message[29:33])[0]
        pressure = struct.unpack('<f', message[33:37])[0]

        sea_level_ref = 101.325  # kPa
        if pressure <= 0:
            altitude = float('nan')
        else:
            altitude = 145366.45 * (1 - math.pow(pressure / sea_level_ref, 0.190284))

        return IMUData(
            yaw=yaw, pitch=pitch, roll=roll,
            a_x=a_x, a_y=a_y, a_z=a_z,
            temperature=temperature,
            pressure=pressure,
            altitude=altitude
        )

    def readData(self):
        with self.lock:
            return self.currentData

    def __del__(self):
        if self.serialConnection and self.serialConnection.is_open:
            self.stop_thread.set()
            self.read_thread.join(timeout=1)
            self.serialConnection.close()
