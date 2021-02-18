import RPi.GPIO as GPIO  # 引入函数库
import time
import GlobalVar as Global

FLWheelCtrlPinA = 29
FLWheelCtrlPinB = 31
FRWheelCtrlPinA = 37
FRWheelCtrlPinB = 36
RLWheelCtrlPinA = 33
RLWheelCtrlPinB = 35
RRWheelCtrlPinA = 38
RRWheelCtrlPinB = 40

GPIO.setup(FLWheelCtrlPinA, GPIO.OUT)
GPIO.setup(FRWheelCtrlPinA, GPIO.OUT)
GPIO.setup(RLWheelCtrlPinA, GPIO.OUT)
GPIO.setup(RRWheelCtrlPinA, GPIO.OUT)
GPIO.setup(FLWheelCtrlPinB, GPIO.OUT)
GPIO.setup(FRWheelCtrlPinB, GPIO.OUT)
GPIO.setup(RLWheelCtrlPinB, GPIO.OUT)
GPIO.setup(RRWheelCtrlPinB, GPIO.OUT)
Wheel_FL_A = GPIO.PWM(FLWheelCtrlPinA, Global.PWMFreq)
Wheel_FR_A = GPIO.PWM(FRWheelCtrlPinA, Global.PWMFreq)
Wheel_RL_A = GPIO.PWM(RLWheelCtrlPinA, Global.PWMFreq)
Wheel_RR_A = GPIO.PWM(RRWheelCtrlPinA, Global.PWMFreq)
Wheel_FL_B = GPIO.PWM(FLWheelCtrlPinB, Global.PWMFreq)
Wheel_FR_B = GPIO.PWM(FRWheelCtrlPinB, Global.PWMFreq)
Wheel_RL_B = GPIO.PWM(RLWheelCtrlPinB, Global.PWMFreq)
Wheel_RR_B = GPIO.PWM(RRWheelCtrlPinB, Global.PWMFreq)


def Init():
    # wheel Control PWM

    GPIO.output(FLWheelCtrlPinA, 0)
    Wheel_FL_A.start(0)
    GPIO.output(FRWheelCtrlPinA, 0)
    Wheel_FR_A.start(0)
    GPIO.output(RLWheelCtrlPinA, 0)
    Wheel_RL_A.start(0)
    GPIO.output(RRWheelCtrlPinA, 0)
    Wheel_RR_A.start(0)
    GPIO.output(FLWheelCtrlPinB, 0)
    Wheel_FL_B.start(0)
    GPIO.output(FRWheelCtrlPinB, 0)
    Wheel_FR_B.start(0)
    GPIO.output(RLWheelCtrlPinB, 0)
    Wheel_RL_B.start(0)
    GPIO.output(RRWheelCtrlPinB, 0)
    Wheel_RR_B.start(0)


def Stop():
    Wheel_FL_A.ChangeDutyCycle(0)
    Wheel_FL_B.ChangeDutyCycle(0)
    Wheel_FR_A.ChangeDutyCycle(0)
    Wheel_FR_B.ChangeDutyCycle(0)
    Wheel_RL_A.ChangeDutyCycle(0)
    Wheel_RL_B.ChangeDutyCycle(0)
    Wheel_RR_A.ChangeDutyCycle(0)
    Wheel_RR_B.ChangeDutyCycle(0)


def GoForward(Speed):
    if Speed < 80:
        GoForward(100)
        Global.SleepMs(100)
    Wheel_FL_A.ChangeDutyCycle(Speed)
    Wheel_FL_B.ChangeDutyCycle(0)
    Wheel_FR_A.ChangeDutyCycle(Speed)
    Wheel_FR_B.ChangeDutyCycle(0)
    Wheel_RL_A.ChangeDutyCycle(Speed)
    Wheel_RL_B.ChangeDutyCycle(0)
    Wheel_RR_A.ChangeDutyCycle(Speed)
    Wheel_RR_B.ChangeDutyCycle(0)
    # Global.SleepMs(100)

def GoForwardWithLimit(Speed, SleepTime):
    if Speed < 80:
        GoForward(100)
        Global.SleepMs(100)
    Wheel_FL_A.ChangeDutyCycle(Speed)
    Wheel_FL_B.ChangeDutyCycle(0)
    Wheel_FR_A.ChangeDutyCycle(Speed)
    Wheel_FR_B.ChangeDutyCycle(0)
    Wheel_RL_A.ChangeDutyCycle(Speed)
    Wheel_RL_B.ChangeDutyCycle(0)
    Wheel_RR_A.ChangeDutyCycle(Speed)
    Wheel_RR_B.ChangeDutyCycle(0)
    Global.SleepMs(SleepTime)
    Stop()

def TurnRight(speed, factor, sleepTime):
    Wheel_FL_A.ChangeDutyCycle(speed)
    Wheel_FL_B.ChangeDutyCycle(0)
    Wheel_FR_A.ChangeDutyCycle(speed * factor)
    Wheel_FR_B.ChangeDutyCycle(0)
    Wheel_RL_A.ChangeDutyCycle(speed)
    Wheel_RL_B.ChangeDutyCycle(0)
    Wheel_RR_A.ChangeDutyCycle(speed * factor)
    Wheel_RR_B.ChangeDutyCycle(0)
    Global.SleepMs(sleepTime)
    Stop()

def TurnLeft(speed, factor, sleepTime):
    Wheel_FL_A.ChangeDutyCycle(speed * factor)
    Wheel_FL_B.ChangeDutyCycle(0)
    Wheel_FR_A.ChangeDutyCycle(speed)
    Wheel_FR_B.ChangeDutyCycle(0)
    Wheel_RL_A.ChangeDutyCycle(speed * factor)
    Wheel_RL_B.ChangeDutyCycle(0)
    Wheel_RR_A.ChangeDutyCycle(speed)
    Wheel_RR_B.ChangeDutyCycle(0)
    Global.SleepMs(sleepTime)
    Stop()

def SharplyTurnRight(angle):
    Stop()
    Global.SleepMs(100)
    Wheel_FL_A.ChangeDutyCycle(100)
    Wheel_FL_B.ChangeDutyCycle(0)
    Wheel_FR_A.ChangeDutyCycle(0)
    Wheel_FR_B.ChangeDutyCycle(100)
    Wheel_RL_A.ChangeDutyCycle(100)
    Wheel_RL_B.ChangeDutyCycle(0)
    Wheel_RR_A.ChangeDutyCycle(0)
    Wheel_RR_B.ChangeDutyCycle(100)
    Global.SleepMs(int((angle/30)*200))
    Stop()

def SharplyTurnLeft(angle):
    Stop()
    Global.SleepMs(100)
    Wheel_FL_A.ChangeDutyCycle(0)
    Wheel_FL_B.ChangeDutyCycle(100)
    Wheel_FR_A.ChangeDutyCycle(100)
    Wheel_FR_B.ChangeDutyCycle(0)
    Wheel_RL_A.ChangeDutyCycle(0)
    Wheel_RL_B.ChangeDutyCycle(100)
    Wheel_RR_A.ChangeDutyCycle(100)
    Wheel_RR_B.ChangeDutyCycle(0)
    Global.SleepMs(int((angle/30)*200))
    Stop()


def GoBackward(Speed):
    if Speed < 80:
        GoBackward(80)
    else:
        Stop()
        Global.SleepMs(100)
    Wheel_FL_A.ChangeDutyCycle(0)
    Wheel_FL_B.ChangeDutyCycle(Speed)
    Wheel_FR_A.ChangeDutyCycle(0)
    Wheel_FR_B.ChangeDutyCycle(Speed)
    Wheel_RL_A.ChangeDutyCycle(0)
    Wheel_RL_B.ChangeDutyCycle(Speed)
    Wheel_RR_A.ChangeDutyCycle(0)
    Wheel_RR_B.ChangeDutyCycle(Speed)
    # Global.SleepMs(100)


def GoBackwardWithLimit(Speed, SleepTime):
    if Speed < 80:
        GoBackward(80)
    else:
        Stop()
        Global.SleepMs(100)
    Wheel_FL_A.ChangeDutyCycle(0)
    Wheel_FL_B.ChangeDutyCycle(Speed)
    Wheel_FR_A.ChangeDutyCycle(0)
    Wheel_FR_B.ChangeDutyCycle(Speed)
    Wheel_RL_A.ChangeDutyCycle(0)
    Wheel_RL_B.ChangeDutyCycle(Speed)
    Wheel_RR_A.ChangeDutyCycle(0)
    Wheel_RR_B.ChangeDutyCycle(Speed)
    Global.SleepMs(SleepTime)
    Stop()

