import numpy as np
import matplotlib.pyplot as plt
import picar_4wd as fc
from picar_4wd import Ultrasonic, Pin, Servo, PWM
import time
import collections
import opencv
import astar
import part1

us = Ultrasonic(Pin('D8'), Pin('D9'))
servo = Servo(PWM("P0"), offset=0)
y_height = 100
x_width = 101
step = 2
curr_dir = step
curr_angle = -90
max_angle = 90
min_angle = -90

CAR_COLOR = 100
OBJECT_COLOR = 200
PATH_COLOR = 75

# map car coord to matrix coord
# ex: car starts at 0,0 but map to (50,99)
def to_x(x):
    return (int)(x+50)

def to_y(y):
    return (int)(99-y)

# map matrix coord to car coord
# ex: (50,99) to (0,0)
def from_x(x):
    return 50-x

def from_y(y):
    return 99-y

# return a list of tuples with (current angle in real world, angle)
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

# return list of (x,y) values rounded to nearest int
def process(output):
    points = []
    for item in output:
        angle = item[0]
        distance = item[1]
        if distance <= 0:
            continue
        elif angle < 0:
            x = np.round(distance*np.sin(-angle * np.pi/180))
            y = np.round(distance*np.cos(-angle * np.pi/180))
            for i in range(-1,2):
                for j in range(-1,2):
                    points.append((x+i,y+j))
            
            
            
        else:
            x = -np.round(distance*np.sin(angle * np.pi/180))
            y = np.round(distance*np.cos(angle * np.pi/180))
            for i in range(-1,2):
                for j in range(-1,2):
                    points.append((x+i,y+j))
            
    return points
    
    
    
def to_color(x, y, A):
    if x < -50 or x > 50 or y < 0 or y > 99:
        return A
    else:
        A[to_y(y)][to_x(x)] = OBJECT_COLOR
        return A


# returns an 2d array that maps nearby obstacle with respect to the car
def get_plot():
    A = np.zeros((y_height, x_width))
    A[to_y(0)][to_x(0)] = CAR_COLOR
    points = process(scan())
    for point in points:
        x = point[0]
        y = point[1]
        A = to_color(x,y,A)
    plt.imshow(A, interpolation='none')
    plt.show()
    return A

def to_path(x, y, A):
    if x < 0 or x > 100 or y < 0 or y > 99:
        return A
    else:
        A[y][x] = PATH_COLOR
        return A


def find_path(endx, endy):
    maze = get_plot()
    
    start = (from_x(0), from_y(0))
    end = ((from_x(endx), from_y(endy)))
    

    path = astar.astar_pathfinding(maze, start, end)
    print(path)

    for step in path:
        for i in range(-1,2):
                for j in range(-1,2):
                    maze = to_path(step[0]+i, step[1]+j, maze)
      
      
    plt.imshow(maze, interpolation='none')
    plt.show()
    return path
    

def process_path(path, count):

    for i in range(min(len(path) - 1, count)):
        curr_x = path[i][0]
        curr_y = path[i][1]
        nxt_x = path[i+1][0]
        nxt_y = path[i+1][1]
        
        if curr_y > nxt_y:
            print('forward')
            fc.forward(20)
            time.sleep(.2)
        elif curr_y < nxt_y:
            print('backward')
            fc.backward(20)
            time.sleep(.2)
        elif curr_x > nxt_x:
            print('left')
            part1.right_90()
            fc.forward(20)
            time.sleep(.2)
        elif curr_x < nxt_x:
            print('right')
            part1.left_90()
            fc.forward(20)
            time.sleep(.2)
    
    fc.stop()
        
def main():    
    path = find_path(0,99)
    process_path(path, 100)


# for name in opencv.run('efficientdet_lite0.tflite', 0, 640, 480, 2, False):
#     # 'stop sign'
#     # 'person'
#     print(name)
    

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        fc.stop()
        exit(0)
    

