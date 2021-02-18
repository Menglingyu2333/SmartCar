
class ReadFrameThread(Process):
    def __init__(self, target, args):
        super(ReadFrameThread, self).__init__()
        self.target = target
        self.args = args

    def run(self):
        while Global.LaneTrackingThreadRunningFlag:
            self.result = self.target(self.args)
            Global.SleepMs(10)

    def get_result(self):
        try:
            return self.result
        except Exception:
            return None

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
        que.put(i)
        # a = que.get()
        # print(a)
        Global.SleepSec(1)

def main():

    Init()
    # LidarThread = threading.Thread(target=Lidar.LidarCtrl(), kwargs={})
    # LidarThread.start()

    # LaneTrackingThread = threading.Thread(target=camera.LaneTracking, kwargs={})
    # LaneTrackingThread.start()
    manager = multiprocessing.Manager()
    que = manager.Queue()
    testThread = multiprocessing.Process(target=testFunc, args=(que,))
    testThread.start()
    while True:
        print("size = %d" % que.qsize())
        for i in list(range(0, que.qsize() - 1)):
            que.get()
        val = que.get()

        print(val)
        print("main process")
        Global.SleepSec(2)

if __name__ == "__main__":
    main()
