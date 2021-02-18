import numpy as np
import cv2 as cv
import GlobalVar as Global
import WheelCtrl as Wheel
import threading
import time
import multiprocessing
import copy

# preliminary attempt at lane following system
# largely derived from: https://medium.com/pharos-production/
# road-lane-recognition-with-opencv-and-ios-a892a3ab635c

# Settings
# debug = False
debug = True
ImageWidth = 320
ImageHigh = 240
# ImageMaskArray = [[(ImageWidth // 5), (ImageHigh // 8)*7], [(ImageWidth // 5), ImageHigh // 3],
#                   [ImageWidth // 5*4, ImageHigh // 3], [ImageWidth // 5*4, (ImageHigh // 8)*7]]
ImageMaskArray = [[(ImageWidth // 5), ImageHigh], [(ImageWidth // 5), ImageHigh // 3],
                  [ImageWidth // 5*4, ImageHigh // 3], [ImageWidth // 5*4, ImageHigh]]
# DownLeft  UpLeft  UpRight  DownRight
RGB2BitImageThreshold = 45
BlurCore = (11, 11)
HoughRhoResolution = 1
HoughThetaResolution = np.pi / 180
HoughLength = 30
CannyMinValue = 30
CannyMaxValue = 150
ThetaLeftBegin = np.pi / 20
ThetaLeftEnd = np.pi / 3
ThetaRightBegin = np.pi / 3 * 2
ThetaRightEnd = np.pi / 20 * 19
L_Cross_Min = -100
L_Cross_Max = 100
R_Cross_Min = 220
R_Cross_Max = 420
LaneCfmThreshold = 2
VehStat = 0
VehStat_Forward = 1
VehStat_Backward = 2
VehStat_TurnLeft = 3
VehStat_TurnRight = 4


def ReadFrameFunc(FrameQueue):
    cap = cv.VideoCapture(-1)
    if cap is not False:
        while True:
            # ReadFrameProcessLock.acquire()
            StartTime = time.time()
            # print("Read Frame process start time = %.5f" % (time.time()))
            ret, frame = cap.read()
            # print(frame)
            FrameQueue.put(frame)
            # print(FrameQueue.size())
            # ReadFrameProcessLock.release()
            Global.SleepMs(20)
            # print("Read Frame process elapsed time = %.5f" % (time.time() - StartTime))
    cap.release()


def LaneTracking():
    global VehStat
    manager = multiprocessing.Manager()
    FrameQueue = manager.Queue()
    ReadFrameProcess = multiprocessing.Process(target=ReadFrameFunc, args=(FrameQueue,))
    ReadFrameProcess.start()

    LaneDetectTimer_L = 0
    LaneDetectTimer_R = 0

    while True:
        # StartTime = time.time()
        # ReadFrameProcessLock.acquire()
        while FrameQueue.empty():
            pass
        print("FrameQueue.qsize = %d" % FrameQueue.qsize())
        for i in list(range(0, FrameQueue.qsize())):
            FrameQueue.get()
        while FrameQueue.empty():
            pass
        frame = FrameQueue.get()
        # print(frame)
        # print("main process ret = %x" % ret)
        # print(frame)
        # ReadFrameProcessLock.release()

        if frame is None:
            continue
        # print("Read Frame time elapsed = %.5f" % (time.time() - StartTime))
        StartProcessTime = time.time()
        ThetaLeft, RhoLeft, ThetaRight, RhoRight = LaneDetector(frame)
        # print("Process Frame time elapsed = %.5f" % (time.time() - StartProcessTime))

        if ThetaLeft is None:
            if LaneDetectTimer_L > 0:
                LaneDetectTimer_L -= 1
                # print("LaneDetectTimer_L -= 1")
        else:
            if LaneDetectTimer_L < LaneCfmThreshold:
                LaneDetectTimer_L += 1
                # print("LaneDetectTimer_L += 1, LaneDetectTimer_L = %d" % LaneDetectTimer_L)

        if ThetaRight is None:
            if LaneDetectTimer_R > 0:
                LaneDetectTimer_R -= 1
        else:
            if LaneDetectTimer_R < LaneCfmThreshold:
                LaneDetectTimer_R += 1
        if debug:
            print("LaneDetectTimer: L  %d, R  %d" % (LaneDetectTimer_L, LaneDetectTimer_R))
        if LaneDetectTimer_L <= 0 and LaneDetectTimer_R <= 0:
            if VehStat is VehStat_Forward or VehStat_Backward:
                # Global.LaneTrackingThreadRunningFlag = False
                Wheel.GoBackwardWithLimit(Global.VehSpd, 200)
                VehStat = VehStat_Backward
            elif VehStat == VehStat_TurnLeft:
                Wheel.TurnLeft(100, 0, 200)
            elif VehStat == VehStat_TurnRight:
                Wheel.TurnRight(100, 0, 200)
        elif LaneDetectTimer_L <= 0 and LaneDetectTimer_R >= LaneCfmThreshold:
            Wheel.TurnLeft(100, 0, 200)
            VehStat = VehStat_TurnLeft
        elif LaneDetectTimer_R <= 0 and LaneDetectTimer_L >= LaneCfmThreshold:
            # print("turn")
            Wheel.TurnRight(100, 0, 200)
            VehStat = VehStat_TurnRight
        elif LaneDetectTimer_R >= LaneCfmThreshold and LaneDetectTimer_L >= LaneCfmThreshold:
            Wheel.GoForwardWithLimit(Global.VehSpd, 150)
            VehStat = VehStat_Forward

        # print("Whole time elapsed = %.5f" % (time.time() - StartProcessTime))

    cv.destroyAllWindows()


def LaneDetector(snip):
    # identify filename of video to be analyzed
    # loop through until entire video file is played
    # while cap.isOpened():

    # read video frame & show on screen
    # ret, frame = cap.read()
    # print(frame)
    ThetaLeftFinal = RhoLeftFinal = ThetaRightFinal = RhoRightFinal = None
    # cv.imshow("Original Scene", frame)
    # snip section of video frame of interest & show on screen
    # snip = frame[0:640, 0:480]
    # snip = frame
    # cv.imshow("Snip", snip)
    # cv.waitKey(10)

    # create polygon (trapezoid) mask to select region of interest
    mask = np.zeros((snip.shape[0], snip.shape[1]), dtype="uint8")
    pts = np.array(ImageMaskArray, dtype=np.int32)
    cv.fillConvexPoly(mask, pts, 180)
    # cv.imshow("Mask", mask)

    # apply mask and show masked image on screen
    masked = cv.bitwise_and(snip, snip, mask=mask)
    cv.imshow("Region of Interest", masked)

    # convert to grayscale then black/white to binary image
    frame = cv.cvtColor(masked, cv.COLOR_BGR2GRAY)
    frame = cv.threshold(frame, RGB2BitImageThreshold, 255, cv.THRESH_BINARY)[1]
    if debug:
        cv.imshow("Black/White", frame)

    # blur image to help with edge detection
    blurred = cv.GaussianBlur(frame, BlurCore, 0)
    # cv.imshow("Blurred", blurred)

    # identify edges & show on screen
    edged = cv.Canny(blurred, CannyMinValue, CannyMaxValue)
    if debug:
        cv.imshow("Edged", edged)

    # perform full Hough Transform to identify lane lines
    lines = cv.HoughLines(edged, HoughRhoResolution, HoughThetaResolution, HoughLength)
    # print(lines)
    # define arrays for left and right lanes
    rho_left = []
    theta_left = []
    rho_right = []
    theta_right = []

    # ensure cv.HoughLines found at least one line
    if lines is not None:

        # loop through all of the lines found by cv.HoughLines
        for i in range(0, len(lines)):

            # evaluate each row of cv.HoughLines output 'lines'
            for rho, theta in lines[i]:

                # collect left lanes
                if ThetaLeftBegin < theta < ThetaLeftEnd:
                    rho_left.append(rho)
                    theta_left.append(theta)

                    # plot all lane lines for DEMO PURPOSES ONLY
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    x1 = int(x0 + 400 * (-b))
                    y1 = int(y0 + 400 * a)
                    x2 = int(x0 - 600 * (-b))
                    y2 = int(y0 - 600 * a)

                    cv.line(snip, (x1, y1), (x2, y2), (0, 0, 255), 1)

                # collect right lanes
                if ThetaRightBegin < theta < ThetaRightEnd:
                    rho_right.append(rho)
                    theta_right.append(theta)

                    # plot all lane lines for DEMO PURPOSES ONLY
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    x1 = int(x0 + 400 * (-b))
                    y1 = int(y0 + 400 * a)
                    x2 = int(x0 - 600 * (-b))
                    y2 = int(y0 - 600 * a)

                    cv.line(snip, (x1, y1), (x2, y2), (0, 0, 255), 1)

    # statistics to identify median lane dimensions
    left_rho = np.median(rho_left)
    left_theta = np.median(theta_left)
    right_rho = np.median(rho_right)
    right_theta = np.median(theta_right)

    Offset_X = ImageWidth // 8
    Offset_Y = ImageHigh // 2
    mat_translation = np.float32([[1, 0, Offset_X], [0, 1, Offset_Y]])  # 变换矩阵：设置平移变换所需的计算矩阵：2行3列
    # [[1,0,20],[0,1,50]]   表示平移变换：其中20表示水平方向上的平移距离，50表示竖直方向上的平移距离。
    snip = cv.warpAffine(snip, mat_translation, ((ImageWidth + Offset_X), (ImageHigh + Offset_Y)))  # 变换函数

    # plot median lane on top of scene snip
    if left_theta > ThetaLeftBegin:
        a = np.cos(left_theta)
        b = np.sin(left_theta)
        # x0_l = a * left_rho
        # y0_l = b * left_rho
        x0_l = int(a * left_rho)
        y0_l = int(b * left_rho)
        L_Cross = x0_l - (y0_l * (ImageHigh - y0_l)) / x0_l
        if L_Cross_Min < L_Cross < L_Cross_Max:
            ThetaLeftFinal = left_theta
            RhoLeftFinal = left_rho
            if debug:
                x0_l += Offset_X
                y0_l += Offset_Y
                offset1 = 800
                offset2 = 800
                x1_l = int(x0_l - offset1 * (-b))
                y1_l = int(y0_l - offset1 * a)
                x2_l = int(x0_l + offset2 * (-b))
                y2_l = int(y0_l + offset2 * a)

                cv.line(snip, (x1_l, y1_l), (x2_l, y2_l), (0, 255, 0), 6)
                cv.line(snip, (Offset_X, Offset_Y), (x0_l, y0_l), (0, 255, 0), 2)
                text = "(%d, %d, %.2f deg, %d)" % (x0_l - Offset_X, y0_l - Offset_Y, np.rad2deg(left_theta), L_Cross)
                org = (x0_l - 100, y0_l + 10)
                fontFace = cv.FONT_HERSHEY_COMPLEX
                fontScale = 0.5
                thickness = 1
                lineType = 8
                bottomLeftOrigin = False
                # bottomLeftOrigin = True
                cv.putText(snip, text, org, fontFace, fontScale, (0, 0, 255), thickness, lineType, bottomLeftOrigin)

    if right_theta > ThetaRightBegin:
        a = np.cos(right_theta)
        b = np.sin(right_theta)

        x0_r = int(a * right_rho)
        y0_r = int(b * right_rho)
        R_Cross = (right_rho - ImageHigh * b) / a
        if R_Cross_Min < R_Cross < R_Cross_Max:
            ThetaRightFinal = right_theta
            RhoRightFinal = right_rho
            if debug:
                # print("r = %f, theta = %f, cos = %f sin = %f" % (right_rho, np.rad2deg(right_theta), a, b))
                x0_r += Offset_X
                y0_r += Offset_Y

                offset1 = 800
                offset2 = 800
                x1_r = int(x0_r - offset1 * (-b))
                y1_r = int(y0_r - offset1 * a)
                x2_r = int(x0_r + offset2 * (-b))
                y2_r = int(y0_r + offset2 * a)

                cv.line(snip, (x1_r, y1_r), (x2_r, y2_r), (255, 0, 0), 6)
                cv.line(snip, (Offset_X, Offset_Y), (x0_r, y0_r), (255, 0, 0), 2)
                text = "(%d, %d, %.2f deg, %d)" % (x0_r - Offset_X, y0_r - Offset_Y, np.rad2deg(right_theta), R_Cross)
                org = (x0_r - 100, y0_r)
                fontFace = cv.FONT_HERSHEY_COMPLEX
                fontScale = 0.5
                thickness = 1
                lineType = 8
                bottomLeftOrigin = False
                cv.putText(snip, text, org, fontFace, fontScale, (0, 0, 255), thickness, lineType, bottomLeftOrigin)

    if debug:
        # overlay semi-transparent lane outline on original
        if ThetaLeftFinal is not None and ThetaRightFinal is not None:
            pts = np.array([[x1_l, y1_l], [x2_l, y2_l], [x1_r, y1_r], [x2_r, y2_r]], dtype=np.int32)

            # (1) create a copy of the original:
            overlay = snip.copy()
            # (2) draw shapes:
            cv.fillConvexPoly(overlay, pts, (0, 255, 0))
            # (3) blend with the original:
            opacity = 0.4
            cv.addWeighted(overlay, opacity, snip, 1 - opacity, 0, snip)

        # cv.imshow("Lined", snip)
        cv.line(snip, (Offset_X, 0), (Offset_X, (ImageHigh + Offset_Y)), (0, 0, 255), 2)
        cv.line(snip, (0, Offset_Y), ((ImageWidth + Offset_X), Offset_Y), (0, 0, 255), 2)

        cv.imshow("Coordinate", snip)

    # perform probablistic Hough Transform to identify lane lines
    # lines = cv.HoughLinesP(edged, 1, np.pi / 180, 20, 2, 1)
    # for x in range(0, len(lines)):
    #     for x1, y1, x2, y2 in lines[x]:
    #         cv.line(snip, (x1, y1), (x2, y2), (0, 0, 255), 2)

    # press the q key to break out of video
    if debug:
        cv.waitKey(1)
    # cv.imshow("Coordinate", snip)
    # cv.waitKey(1)
    return ThetaLeftFinal, RhoLeftFinal, ThetaRightFinal, RhoRightFinal
    # clear everything once finished
    # cap.release()
    # cv.destroyAllWindows()

# def DebugShowLaneImage(image):

def TennisDetect():
    x_pos = 0  # initialize the tennis's position
    y_pos = 0
    radius = 0
    img_out = copy.copy(img)
    img = img[self.cut_edge:479, :, :]
    img = cv2.blur(img, (5, 5))  # denoising
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # rgb to HSV

    manager = multiprocessing.Manager()
    FrameQueue = manager.Queue()
    ReadFrameProcess = multiprocessing.Process(target=ReadFrameFunc, args=(FrameQueue,))
    ReadFrameProcess.start()

    while True:
        # StartTime = time.time()
        while FrameQueue.empty():
            pass
        print("FrameQueue.qsize = %d" % FrameQueue.qsize())
        for i in list(range(0, FrameQueue.qsize())):
            FrameQueue.get()
        while FrameQueue.empty():
            pass
        frame = FrameQueue.get()
        # print(frame)
        # print("main process ret = %x" % ret)
        # print(frame)
        # ReadFrameProcessLock.release()

        if frame is None:
            continue
        # print("Read Frame time elapsed = %.5f" % (time.time() - StartTime))
        StartProcessTime = time.time()
        ThetaLeft, RhoLeft, ThetaRight, RhoRight = LaneDetector(frame)
        # print("Process Frame time elapsed = %.5f" % (time.time() - StartProcessTime))
