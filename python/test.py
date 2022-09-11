from gpiozero import Motor, Button, AngularServo    #gpiozero zur Ansteuerung der GPIO-Pins
from gpiozero.pins.pigpio import PiGPIOFactory 
import time  
motor_1 = Motor(forward = 6,backward = 13)
factory = PiGPIOFactory()
servo = AngularServo(12,min_angle=-180,max_angle=180,min_pulse_width=5/10000,max_pulse_width=25/10000,frame_width=20/1000,pin_factory=factory)

while True:
    servo.angle = -80
    time.sleep(0.5)
    servo.angle = 0
    time.sleep(0.5)
