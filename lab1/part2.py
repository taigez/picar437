import numpy as np
import matplotlib.pyplot as plt
import picar_4wd as fc
from picar_4wd import Ultrasonic, Pin, Servo, PWM
import time
import collections

us = Ultrasonic(Pin('D8'), Pin('D9'))
servo = Servo(PWM("P0"), offset=0)
y_height = 100
x_width = 101
step = 2
curr_dir = step
curr_angle = -90
max_angle = 90
min_angle = -90

# map car coord to matrix coord
# ex: car starts at 0,0 but map to (50,80)
def to_x(x):
    return (int)(x+50)

def to_y(y):
    return (int)(99-y)

def scan():
    global curr_dir, curr_angle
    output = []
    for i in range((int)(180/step) + 1):
        distance = fc.get_distance_at(curr_angle)
        output.append((curr_angle, distance))
        curr_angle += curr_dir
        if curr_angle >= max_angle:
            curr_dir = -step
            curr_angle = max_angle - (curr_angle - max_angle)
        elif curr_angle <= min_angle:
            curr_dir = step
            curr_angle = min_angle + (min_angle - curr_angle)
    return output
            
def process(output):
    points = []
    for item in output:
        angle = item[0]
        distance = item[1]
        if distance <= 0:
            continue
        elif angle < 0:
            x = distance*np.sin(-angle * np.pi/180)
            y = distance*np.cos(-angle * np.pi/180)
            points.append((np.round(x), np.round(y)))
            
        else:
            x = distance*np.sin(angle * np.pi/180)
            y = distance*np.cos(angle * np.pi/180)
            points.append((-np.round(x), np.round(y)))
            
    return points
    
    
    
def to_color(x, y, A):
    if x < -50 or x > 50 or y < 0 or y > 99:
        return A
    else:
        A[to_y(y)][to_x(x)] = 200
        return A

A = np.zeros((y_height, x_width))
A[to_y(0)][to_x(0)] = 100

points = process(scan())
for point in points:
    x = point[0]
    y = point[1]
    A = to_color(x,y,A)

plt.imshow(A, interpolation='none')
plt.show()
