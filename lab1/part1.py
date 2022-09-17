import picar_4wd as fc
from picar_4wd import Ultrasonic, Pin, Servo, PWM
import time
import collections

us = Ultrasonic(Pin('D8'), Pin('D9'))
servo = Servo(PWM("P0"), offset=0)

speed = 35
first_threshold = 50
sec_threshold = 30

def move_back():
    fc.backward(100)
    time.sleep(.2)

def left_90():
    fc.turn_left(100)
    time.sleep(.75)
    fc.stop()
    
def left_45():
    fc.turn_left(100)
    time.sleep(.40)
    fc.stop()
    
def right_90():
    fc.turn_right(100)
    time.sleep(.75)
    fc.stop()
    
def right_45():
    fc.turn_right(100)
    time.sleep(.40)
    fc.stop()
    
def stop():
    fc.stop()
    


def test2():
    global speed
    while True:
        scan_list = fc.scan_step(50)
        if not scan_list:
            continue

        tmp = scan_list[3:7]
        avg = sum(tmp)/4
        print(tmp)
        print(avg)
        
        if avg >= 2:
            fc.forward(speed)
        elif avg < 2 and avg >= 1.5:
            speed = 25
            fc.forward(speed)
        else:
            speed = 0
            fc.stop()
            
            left = fc.get_status_at(45, first_threshold, sec_threshold)
            right = fc.get_status_at(-45, first_threshold, sec_threshold)
            
            #left side is better
            if left > right:
                move_back()
                left_45()
                time.sleep(.2)
                speed = 35
            else:
                move_back()
                right_45()
                time.sleep(.2)
                speed = 35
            
        
        
        
if __name__ == '__main__':
    try:
        test2()
    except KeyboardInterrupt:
        fc.stop()
        exit(0)
