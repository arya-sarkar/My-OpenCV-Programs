import RPi.GPIO as GPIO
from time import sleep
from threading import Thread
import cv2 as cv
import numpy as np

class WebcamVideoStream:
    def _init_(self,src=0):
        self.stream = cv.videoCapture(src)
        (self.grabbed,self.frame) = self.stream.read() ## Webcam Start
        self.stopped = False

    def start(self):
        Thread(target=self.update, args = ()).start() ## Starting a New Thread (File), and also keeps on updating in background
        return self

    def update(self):
        while True:
            if self.stopped:
                return
            (self.grabbed,self.frame) = self.stream.read()

    def read(self):
        while True:
            if self.stopped:
                return
            (self.grabbed,self.frame) = self.stream.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True

GPIO.setmode(GPIO.BOARD)

# Assign GPIO Pins for both Motors #

lm_ena=33 ## Check the """ """ at end
lm_pos=35
lm_neg=37

rm_ena=36 ## Check the """ """ at end
rm_pos=38
rm_neg=40

# Set Pins in the Output Mode #

GPIO.setup(lm_ena,GPIO.OUT)
GPIO.setup(lm_pos,GPIO.OUT)
GPIO.setup(lm_neg,GPIO.OUT)

GPIO.setup(rm_ena,GPIO.OUT)
GPIO.setup(rm_pos,GPIO.OUT)
GPIO.setup(rm_neg,GPIO.OUT)

# MOVEMENT OF ROBOTS #

def moveRobot(d):
    if(d=='f'):
        print("FORWARD")

        ## LEFT MOTOR FORWARD ##

        GPIO.output(lm_ena,GPIO.HIGH)
        GPIO.output(lm_pos,GPIO.HIGH)
        GPIO.output(lm_neg,GPIO.LOW)

        ## LEFT MOTOR FORWARD ##

        ## RIGHT MOTOR FORWARD ##

        GPIO.output(rm_ena,GPIO.HIGH)
        GPIO.output(rm_pos,GPIO.HIGH)
        GPIO.output(rm_neg,GPIO.LOW)

        ## RIGHT MOTOR FORWARD ##

    if(d=='b'):
        print("BACKWARD")

        ## LEFT MOTOR BACKWARD ##

        GPIO.output(lm_ena,GPIO.HIGH)
        GPIO.output(lm_pos,GPIO.LOW)
        GPIO.output(lm_neg,GPIO.HIGH)

        ## LEFT MOTOR BACKWARD ##

        ## RIGHT MOTOR BACKWARD ##

        GPIO.output(rm_ena,GPIO.HIGH)
        GPIO.output(rm_pos,GPIO.LOW)
        GPIO.output(rm_neg,GPIO.HIGH)

        ## RIGHT MOTOR BACKWARD ##

    if(d=='r'):
        print("RIGHT")

        ## LEFT MOTOR RIGHT ##

        GPIO.output(lm_ena,GPIO.HIGH)
        GPIO.output(lm_pos,GPIO.HIGH)
        GPIO.output(lm_neg,GPIO.LOW)

        ## LEFT MOTOR RIGHT ##

        ## RIGHT MOTOR RIGHT ##

        GPIO.output(rm_ena,GPIO.HIGH)
        GPIO.output(rm_pos,GPIO.LOW)
        GPIO.output(rm_neg,GPIO.HIGH)

        ## RIGHT MOTOR RIGHT ##

    if(d=='l'):
        print("LEFT")

        ## LEFT MOTOR LEFT ##

        GPIO.output(lm_ena,GPIO.HIGH)
        GPIO.output(lm_pos,GPIO.LOW)
        GPIO.output(lm_neg,GPIO.HIGH)

        ## LEFT MOTOR LEFT ##

        ## RIGHT MOTOR LEFT ##

        GPIO.output(rm_ena,GPIO.HIGH)
        GPIO.output(rm_pos,GPIO.HIGH)
        GPIO.output(rm_neg,GPIO.LOW)

        ## RIGHT MOTOR LEFT ##

    if(d=='s'):#
        print("STOP")

        ## LEFT MOTOR STOP ##

        GPIO.output(lm_ena,GPIO.HIGH)
        GPIO.output(lm_pos,GPIO.LOW)
        GPIO.output(lm_neg,GPIO.LOW)

        ## LEFT MOTOR STOP ##

        ## RIGHT MOTOR STOP ##

        GPIO.output(rm_ena,GPIO.HIGH)
        GPIO.output(rm_pos,GPIO.LOW)
        GPIO.output(rm_neg,GPIO.LOW)

        ## RIGHT MOTOR STOP ##

cam = WebcamVideoStream(src=0).start()

while (True):
    frame = cam.read()

    key = cv.waitKey(10)

    if key==ord('w'):
        moveRobot('f')

    if key==ord('a'):
        moveRobot('l')

    if key==ord('s'):
        moveRobot('b')

    if key==ord('d'):
        moveRobot('r')

    if key==32:
        moveRobot('s')
        # I have to mention 32 to stop program

    if key==27:
        break

    cv.imshow('frame',frame)

cam.stop()
cv.destroyAllWindows()

GPIO.cleanup()

"""

EN_A & EN_B Pin in Motor Driver:

1. Used to determine whether the concerned motor is active or not. It overrides the MA1, MA2, MB1 or MB2 pins' signals, if set to LOW, and disables motor.
2. Can be used to control speed by feeding it a PWM (Pulse Width Modulation) signal from your Raspberry Pi. This lets you vary the motor speed.

"""

