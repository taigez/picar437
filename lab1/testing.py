import picar_4wd as fc
from picar_4wd import Ultrasonic, Pin, Servo, PWM
import time
import collections

us = Ultrasonic(Pin('D8'), Pin('D9'))
servo = Servo(PWM("P0"), offset=0)

def run():
    
    start = fc.get_distance_at(0)
    print(start)
    fc.forward(10)
    time.sleep(0.1)
    end = fc.get_distance_at(0)
    print(end)
    fc.stop()
    print(start - end)

if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt:
        fc.stop()
        exit(0)