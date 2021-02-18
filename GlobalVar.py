import RPi.GPIO as GPIO  # 引入函数库
import time
import cv2 as cv

VehSpd = 100

LaneTrackingThreadRunningFlag = False


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

def print_hex(bytes):
    print('0x', end="")
    j = 0
    for i in bytes:
        j += 1
        # print(j, end=": ")
        print(("{0:02X}".format(i)), end=" ")
    print()

def SleepSec(sec):
    for i in range(sec):
        time.sleep(1)


def SleepMs(ms):
    for i in range(ms):
        time.sleep(0.001)


PWMFreq = 100


