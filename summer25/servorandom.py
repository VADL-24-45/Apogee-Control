from gpiozero import AngularServo
from time import sleep, time
from gpiozero.pins.pigpio import PiGPIOFactory
import random
factory = PiGPIOFactory()
servo = AngularServo(16, min_angle=0, max_angle=180, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory)

print("Run")

sleep(10)
print("Begin")
end_time = time() + 120
try:
    while time() < end_time:
        user_angle = random.uniform(0,180)
        servo.angle = user_angle
        print(f"Servo motor moved {servo.angle} degrees")
        sleep(0.25)
except KeyboardInterrupt:
    print("Program Stopped")