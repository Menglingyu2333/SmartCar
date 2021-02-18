import GlobalVar as Global
import WheelCtrl as Wheel
import Lidar
import threading
import multiprocessing
import camera
import cv2 as cv
import RPi.GPIO as GPIO
def Init():
    Wheel.Init()

def testFunc(que):
    i = 0
    while 1:
        i += 1
        print("testFunc, i = %d" % i)
        # que.put(i)
        # a = que.get()
        # print(a)
        Global.SleepSec(1)

def main():

    Init()
    # Lidar.LidarCtrl()
    # LidarThread = threading.Thread(target=Lidar.LidarCtrl(), kwargs={})
    # LidarThread.start()

    # LaneTrackingThread = threading.Thread(target=camera.LaneTracking, kwargs={})
    # LaneTrackingThread.start()
    camera.LaneTracking()
    while True:
        # Wheel.GoBackward(100)
        # Global.SleepSec(1)
        # Wheel.GoForward(100)
        # Global.SleepSec(1)
        # Wheel.SharplyTurnLeft(600)
        print("main process")
        Global.SleepSec(2)


if __name__ == "__main__":
    main()
