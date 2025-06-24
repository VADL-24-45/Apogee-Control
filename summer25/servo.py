from gpiozero import AngularServo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory
factory = PiGPIOFactory()
servo = AngularServo(16, min_angle=0, max_angle=180, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory)
print("Run")

try:
    while True:
        user_input = input("Enter the numeric angle to turn servo ")
        try:
            user_angle = float(user_input)
            if 0 <= user_angle <= 180:
                servo.angle = user_angle
                print(f"Servo motor was moved {servo.angle} degrees ")
            else:
                print("Error: Numeric input not within limits")
        except ValueError:
            print("Error: Non-numeric input ")
except KeyboardInterrupt:
    print("Program Stopped")