from gpiozero import Motor, Button, AngularServo    #gpiozero zur Ansteuerung der GPIO-Pins
from gpiozero.pins.pigpio import PiGPIOFactory 
import time  
motor_1 = Motor(forward = 13,backward = 6)
factory = PiGPIOFactory()
servo = AngularServo(23,min_angle=-180,max_angle=180,min_pulse_width=5/10000,max_pulse_width=25/10000,frame_width=20/1000,pin_factory=factory)

while True:
   motor_1.backward(1)
   time.sleep(2)
   motor_1.forward(1)
   time.sleep(2)
   servo.angle = 0
