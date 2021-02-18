import GlobalVar as Global
import RPi.GPIO as GPIO
import numpy as np
import serial
import struct
import cv2 as cv
import numpy as np
import math
import WheelCtrl as Wheel
import threading

LidarMotoCtrlPin = 32

FrameStartID = b'\xAA\x00'
FrameType = b'\x01\x61'
FrameTypeOffset = 3
FrameMeasureInfoID = b'\xAD'
FrameHealthInfoID = b'\xAE'
FrameInfoIDOffset = 5
FrameLenOffset = 2
DataLen = 0
DataLenOffset = 7
RotateSpeed = 0.00
ZeroOffset = 0.00
FrameStartAngle = 0.00
FramePointNum = 0
FramePoint = [[0 for i in range(2)] for i in range(100)]
CirclePointNum = 0
CirclePoint = [[0 for i in range(2)] for i in range(480)]
FirstFrameOfOneCircle = True
CircleToothNum = 0
CircleToothNumMax = 16
CircleDataValid = False

GPIO.setup(LidarMotoCtrlPin, GPIO.OUT)
LidarMotoPwm = GPIO.PWM(LidarMotoCtrlPin, Global.PWMFreq)

ser = serial.Serial("/dev/ttyAMA0", 115200, timeout=1)

Angle_Forward = 20
StopDist = 300
StartDist = 700
StartAngleThresh = 40

def Init():
    global Speed
    # Lidar motor control pin
    Global.GPIO.output(LidarMotoCtrlPin, 0)
    LidarMotoPwm.start(100)

    if ser.isOpen() is True:
        print('Open serial success!')
    ser.flushInput()

    Wheel.GoForward(Global.VehSpd)


def DrawPicture(tempCirclePoint, tempCirclePointNum):
    img = np.zeros((800, 800), np.uint8)  # 生成一个空灰度图像
    point_color = 255  # gray 单通道灰度图
    thickness = 2
    ptLeftTop = (390, 380)
    ptRightBottom = (410, 420)
    cv.rectangle(img, ptLeftTop, ptRightBottom, point_color, thickness)
    point_size = 1
    thickness = 1
    for i in list(range(0, tempCirclePointNum)):
        point = (400 - int(((math.sin(math.radians(tempCirclePoint[i][1]))) * tempCirclePoint[i][0]) / 20),
                 400 - int(((math.cos(math.radians(tempCirclePoint[i][1]))) * tempCirclePoint[i][0]) / 20))
        # print(point)
        cv.circle(img, point, point_size, point_color, thickness, 8)
    cv.namedWindow("image")
    cv.imshow('image', img)
    cv.waitKey(1)


def CalcChksum(Bytes):
    Chksum = 0
    for byte in Bytes:
        Chksum += byte
    return Chksum


def FrameFormatChk(Frame):
    # Global.print_hex(Frame)
    # print(Frame[FrameTypeOffset])
    # print(Frame[3:5])
    # Global.print_hex(Frame)
    if Frame[FrameTypeOffset:FrameTypeOffset + 2] != FrameType:
        return False
    if Frame[FrameLenOffset] != (Frame[DataLenOffset] + 8):
        return False
    # print(Frame[-2:])
    # print(CalcChksum(Frame[:-2]))
    # print(struct.unpack('>H', (Frame[-2:]))[0])
    temp = struct.unpack('>H', (Frame[-2:]))
    if temp[0] != CalcChksum(Frame[:-2]):
        return False
    return True


def FrameCombine():
    global FramePointNum, FramePoint, CirclePointNum, CirclePoint, \
        FirstFrameOfOneCircle, CircleToothNum, CircleToothNumMax, CircleDataValid

    if FirstFrameOfOneCircle:
        if FramePoint[0][1] < 0.0001:
            for i in list(range(0, FramePointNum)):
                CirclePoint[i][0] = FramePoint[i][0]
                CirclePoint[i][1] = FramePoint[i][1]
                CirclePointNum += 1
            FirstFrameOfOneCircle = False
            CircleToothNum = 1
    else:
        if FramePoint[0][1] < 0.0001:
            for i in list(range(0, FramePointNum)):
                CirclePoint[i][0] = FramePoint[i][0]
                CirclePoint[i][1] = FramePoint[i][1]
                CirclePointNum += 1
            CircleToothNum += 1

        elif FramePoint[0][1] < CirclePoint[CirclePointNum][1]:
            CirclePointNum = 0
            FirstFrameOfOneCircle = True
            CircleToothNum = 0
        else:
            for i in list(range(0, FramePointNum)):
                CirclePoint[CirclePointNum + i][0] = FramePoint[i][0]
                CirclePoint[CirclePointNum + i][1] = FramePoint[i][1]
            CirclePointNum += FramePointNum

            CircleToothNum += 1
            if CircleToothNum >= CircleToothNumMax:
                CircleDataValid = True
                # for i in list(range(0, CirclePointNum)):
                #     print('CirclePoint %d distance = %f, angle = %f' % (i, CirclePoint[i][0], CirclePoint[i][1]))

                DrawPictureThread = threading.Thread(target=DrawPicture,
                                                     kwargs={'tempCirclePoint': CirclePoint,
                                                             'tempCirclePointNum': CirclePointNum})
                DrawPictureThread.start()


def FrameAnalysis(Frame):
    global DataLen, RotateSpeed, ZeroOffset, FrameStartAngle, FramePointNum, FramePoint
    DataOffset = 8
    if Frame[FrameInfoIDOffset].to_bytes(length=1, byteorder='big', signed=False) == FrameMeasureInfoID:
        # Global.print_hex(Frame)
        DataLen = Frame[DataLenOffset]
        # print('DataLen = %d' % DataLen)

        RotateSpeed = round(Frame[DataOffset] * 0.05, 2)
        # print('RotateSpeed = %f' % RotateSpeed)
        DataOffset += 1

        ZeroOffset = round(struct.unpack('>H', Frame[DataOffset:DataOffset + 2])[0] * 0.01, 2)
        # print('ZeroOffset = %f' % ZeroOffset)
        DataOffset += 2

        FrameStartAngle = round(struct.unpack('>H', Frame[DataOffset:DataOffset + 2])[0] * 0.01, 2)
        # print('FrameStartAngle = %f' % FrameStartAngle)
        DataOffset += 2

        FramePointNum = int((DataLen - DataOffset + 5) / 3)
        # print('FramePointNum = %d' % FramePointNum)
        AngleResolution = 22.5 / FramePointNum
        # print('AngleResolution = %f' % AngleResolution)

        for i in list(range(0, FramePointNum)):
            FramePoint[i][0] = round(
                struct.unpack('>H', Frame[(DataOffset + i * 3 + 1):(DataOffset + i * 3 + 1 + 2)])[0] * 0.25,
                2)  # distance
            FramePoint[i][1] = round(FrameStartAngle + i * AngleResolution, 2)  # angle
            # print('FramePoint %d distance = %f, angle = %f' % (i, FramePoint[i][0], FramePoint[i][1]))

        FrameCombine()
        return True

    elif Frame[FrameInfoIDOffset] == FrameHealthInfoID:
        return True
    else:
        return False


def ReceiveData():
    global CircleDataValid, CirclePointNum, FirstFrameOfOneCircle, CircleToothNum
    CirclePointNum = 0
    FirstFrameOfOneCircle = True
    CircleToothNum = 0
    ser.flushInput()
    while not CircleDataValid:
        rev = ser.read_until(FrameStartID, 256)
        if rev is None:
            print('Open serial = ', ser.isOpen())
            # raise SerialException(
            #             'device reports readiness to read but returned no data '
            #             '(device disconnected or multiple access on port?)')

        if rev[-2:] == FrameStartID:
            # Global.print_hex(rev)
            rev += ser.read(1)
            FrameLen = rev[-1]
            # print(FrameLen)
            rev += ser.read(FrameLen - 1)
            # Global.print_hex(rev)
            # print(FrameFormatChk(rev))
            FrameAnalysis(rev)
        else:
            ser.flushInput()


# def LidarDecision():
#     global CirclePoint, CirclePointNum, Speed, Angle_Forward, StopDist, StartDist, StartAngleThresh
#     StartAngle = 0
#     StartPointNo = 0
#
#     for i in list(range(0, 255)):
#         # print('i=%d' % i)
#         if (CirclePoint[i][0] <= StopDist and CirclePoint[i][0] != 0) \
#                 or (CirclePoint[CirclePointNum - i - 1][0] <= StopDist and
#                     CirclePoint[CirclePointNum - i - 1][0] != 0):
#             print('Stop: i = %d, +dist = %f, -dist = %f, CirclePointNum = %d' %
#                   (i, CirclePoint[i][0],
#                    CirclePoint[CirclePointNum - i - 1][0], CirclePointNum), )
#             Wheel.Stop()
#             for j in list(range(i, 255)):
#                 if CirclePoint[j][0] >= StartDist or CirclePoint[j][0] == 0:
#                     if StartAngle == 0:
#                         StartAngle = CirclePoint[j][1]
#                         print('StartAngle = %f' % StartAngle)
#                     else:
#                         if CirclePoint[j][1] - StartAngle >= StartAngleThresh:
#                             StartPointNo = j
#                             StartAngle = 0
#                             print('FinishAngle = %f, StartPointNo = %d'
#                                   % (CirclePoint[j][1], StartPointNo))
#                             break
#                 else:
#                     StartAngle = 0
#
#             if CirclePoint[StartPointNo][1] <= 180:
#                 Wheel.TurnRight(CirclePoint[StartPointNo][1])
#             else:
#                 Wheel.TurnRight(360 - CirclePoint[StartPointNo][1])
#             break
#         else:
#             if CirclePoint[i][1] >= Angle_Forward:
#                 Wheel.GoForward(Speed)
#                 break

def LidarDecision():
    global CirclePoint, CirclePointNum, Angle_Forward, StopDist, StartDist, StartAngleThresh
    StartAngle_R = 0
    StartAngle_L = 0
    StartPointNo_R = 0
    StartPointNo_L = 0

    for i in list(range(0, 255)):
        # print('i=%d' % i)
        if (CirclePoint[i][0] <= StopDist and CirclePoint[i][0] != 0) \
                or (CirclePoint[CirclePointNum - i - 1][0] <= StopDist and
                    CirclePoint[CirclePointNum - i - 1][0] != 0):
            print('Stop: i = %d, +dist = %f, -dist = %f, CirclePointNum = %d' %
                  (i, CirclePoint[i][0],
                   CirclePoint[CirclePointNum - i - 1][0], CirclePointNum), )
            Wheel.Stop()
            for j in list(range(i, 255)):
                if CirclePoint[j][0] >= StartDist or CirclePoint[j][0] == 0:
                    if StartAngle_R == 0:
                        StartAngle_R = CirclePoint[j][1]
                        print('StartAngle_R = %f' % StartAngle_R)
                    else:
                        if CirclePoint[j][1] - StartAngle_R >= StartAngleThresh:
                            StartPointNo_R = j
                            print('FinishAngle = %f, StartPointNo = %d'
                                  % (CirclePoint[j][1], StartPointNo_R))
                            break
                elif (CirclePoint[CirclePointNum - j - 1][0] >= StartDist
                      or CirclePoint[CirclePointNum - j - 1][0] == 0):
                    if StartAngle_L == 0:
                        StartAngle_L = CirclePoint[CirclePointNum - j - 1][1]
                        print('StartAngle_L = %f' % StartAngle_L)
                    else:
                        if StartAngle_L - CirclePoint[CirclePointNum - j - 1][1] >= StartAngleThresh:
                            StartPointNo_L = CirclePointNum - j - 1
                            print('FinishAngle = %f, StartPointNo = %d'
                                  % (CirclePoint[CirclePointNum - j - 1][1], StartPointNo_L))
                            break
                else:
                    StartAngle_R = 0
                    StartAngle_L = 0

            if StartPointNo_R != 0:
                Wheel.SharplyTurnRight(CirclePoint[StartPointNo_R][1] - StartAngleThresh/2)
            elif StartPointNo_L != 0:
                Wheel.SharplyTurnLeft(360-(CirclePoint[StartPointNo_L][1] + StartAngleThresh/2))
            break
        else:
            if CirclePoint[i][1] >= Angle_Forward:
                Wheel.GoForward(Global.VehSpd)
                break

def LidarCtrl():
    global CirclePoint, CirclePointNum, CircleDataValid
    Init()
    while True:
        ReceiveData()
        # print(CirclePoint[0][0])
        LidarDecision()
        CircleDataValid = False
