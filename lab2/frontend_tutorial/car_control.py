import picar_4wd as fc
from picar_4wd import Ultrasonic, Pin, Servo, PWM
import time
import collections

SPEED = 70

def car_stop():
    fc.stop()

def go_foward():
    while True:
        fc.foward(SPEED)
        
def go_backward():
    while True:
        fc.backward(SPEED)
        
def turn_left():
    while True:
        fc.turn_left(SPEED)
        
def turn_right():
    while True:
        fc.turn_right(SPEED)