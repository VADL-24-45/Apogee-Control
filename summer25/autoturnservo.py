from gpiozero import AngularServo
from time import sleep, time
from gpiozero.pins.pigpio import PiGPIOFactory
import random
factory = PiGPIOFactory()
servo = AngularServo(16, min_angle=0, max_angle=180, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory)

print("Run")
servo.angle = 0
print(f"Servo motor moved {servo.angle} degrees")
sleep(0.5)
servo.angle = 45
print(f"Servo motor moved {servo.angle} degrees")
