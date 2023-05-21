#! /usr/bin/env python

from tf import TransformBroadcaster, transformations
import rospy
from rospy import Time
import math
import numpy as np

from tkinter import *
from tkinter import ttk

import time

import RPi.GPIO as GPIO

OUT_PIN=11
PULSE_FREQ=50

#GPIO.setmode(GPIO.BOARD)



import sys
import time
import RPi.GPIO as GPIO

mode=GPIO.getmode()

#GPIO.cleanup()

left_forward=26
left_backward=20
right_forward=19
right_backward=16
sleeptime=1
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(3,GPIO.OUT)
GPIO.setup(left_forward, GPIO.OUT)
GPIO.setup(left_backward, GPIO.OUT)
GPIO.setup(right_forward, GPIO.OUT)
GPIO.setup(right_backward, GPIO.OUT)


def forward(x):
     GPIO.output(left_forward, GPIO.HIGH)
     GPIO.output(right_forward, GPIO.HIGH)
     print("Moving Forward")
     time.sleep(1)
     GPIO.output(left_forward, GPIO.LOW)
     GPIO.output(right_forward, GPIO.LOW)

def reverse(x):
     GPIO.output(left_backward, GPIO.HIGH)
     GPIO.output(right_backward, GPIO.HIGH)
     print("Moving Backward")
     time.sleep(1)
     GPIO.output(left_backward, GPIO.LOW)
     GPIO.output(right_backward, GPIO.LOW)


def left(x):
     GPIO.output(left_forward, GPIO.HIGH)
     print("Moving left")
     time.sleep(6.5)
     GPIO.output(left_forward, GPIO.LOW)

def right(x):
     GPIO.output(right_forward, GPIO.HIGH)
     print("Moving right")
     time.sleep(6.5)
     GPIO.output(right_forward,GPIO.LOW)
     
def stop_car():
	GPIO.output(right_forward,GPIO.LOW)
	GPIO.output(left_forward,GPIO.LOW)
	GPIO.output(right_backward,GPIO.LOW)
	GPIO.output(left_backward,GPIO.LOW)

def getRadian(degree):
    pi = 3.14159265359;
    return (degree * (pi / 180))

def rotatePnt(point, radians, origin):
    x, y = point
    ox, oy = origin
    qx = ox + math.cos(radians) * (x - ox) -  math.sin(radians) * (y - oy)
    qy = oy + math.sin(radians) * (x - ox) + math.cos(radians) * (y - oy)
    return qx, qy

def on_clickF(event):
    global alpha,x,y,theta
    rad=getRadian(alpha)
    x-=0.3*math.cos(rad)
    y+=0.3*math.sin(rad)
    forward(5)
    print("Forward\n")

def on_clickB(event):
    global alpha,x,y
    rad=getRadian(alpha)
    x+=0.3*math.cos(rad)
    y-=0.3*math.sin(rad)
    reverse(5)
    print("Backward\n")

def on_clickL(event):
    global flag1,alpha,x,y
    alpha-=90
    rad=getRadian(alpha)
    points=rotatePnt((x,y),rad,((x-(0.34)*math.cos(getRadian(alpha+90))),(y-(0.34)*math.sin(getRadian(alpha+90)))))
    x=points[0]
    y=points[1]
    left(5)
    print("Left\n")

def on_clickR(event):
    global flag2,alpha,x,y
    alpha+=90
    rad=getRadian(alpha)
    points=rotatePnt((x,y),rad,((x-(0.34)*math.cos(getRadian(alpha+90))),(y-(0.34)*math.sin(getRadian(alpha+90)))))
    x=points[0]
    y=points[1]
    right(5)
    print("Right\n")

def run_code():
    global alpha,translation,x,y,flag,angle,rotation,servo1
    if not rospy.is_shutdown():

        theta=math.radians(alpha)
        translation=(x,y,0.0)

        if(angle>450):
            flag=1
        elif(angle<270):
            flag=0
        if(flag==0):
            angle+=1
        else:
            angle-=1
        angle=0
        servo1.ChangeDutyCycle(2+(0.277)*((angle-270)/5))
        rotation = transformations.quaternion_from_euler(getRadian(0),getRadian(0),getRadian(alpha))
        b.sendTransform(translation, rotation, Time.now(),'/Axes', 'base_frame' )
        translation1=(0,0,-0.1)
        rotation1 = transformations.quaternion_from_euler(getRadian(0),getRadian(angle),getRadian(0))
        b.sendTransform(translation1, rotation1, Time.now(),'/base_frame', 'laser_frame' )
        win.after(10,run_code)

win=Tk()
alpha=0
rospy.init_node('base_link_to_laser4')
b = TransformBroadcaster()
translation = (0.0, 0.0, 0.0)
angle=270
rotation = transformations.quaternion_from_euler(0, 0, getRadian(angle))
flag=0
x, y = 0.0,0.0
# win.geometry("750x350")
# fb=ttk.Button(win, text= "Forward", command=lambda: on_clickF())
# fb.pack()
# bb=ttk.Button(win, text= "Backward", command=lambda: on_clickB())
# bb.pack()
# lb=ttk.Button(win, text= "Left", command=lambda: on_clickL())
# lb.pack()
# rb=ttk.Button(win, text= "Right", command=lambda: on_clickR())
# rb.pack()
frame = Frame(win, width=100, height=10)
T=Label(win,text="Believe me This is your JOYSTICK!!!")
T.config(font=("Courier",14))
T.pack()
frame.bind('<Up>', on_clickF)
frame.bind('<Down>', on_clickB)
frame.bind('<Left>', on_clickL)
frame.bind('<Right>', on_clickR)
frame.bind('<Tab>', stop_car)
frame.focus_set()
frame.pack()
servo1=GPIO.PWM(3,PULSE_FREQ)
servo1.start(0)
win.after(500,run_code)
win.mainloop()
servo1.stop()
GPIO.cleanup()
