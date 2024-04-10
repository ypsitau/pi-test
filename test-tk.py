#!/usr/bin/env python
import tkinter as tk
import RPi.GPIO as GPIO
import time

OFFSET_DUTY = 1        # define pulse offset of servo
SERVO_MIN_DUTY = 2.5 + OFFSET_DUTY     # define pulse duty cycle for minimum angle of servo
SERVO_MAX_DUTY = 12.5 + OFFSET_DUTY    # define pulse duty cycle for maximum angle of servo

pinsServo = [11, 13, 15, 12]

GPIO.setmode(GPIO.BOARD)         # use PHYSICAL GPIO Numbering
pwmsServo = []
for pin in pinsServo:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)
    pwm = GPIO.PWM(pin, 50)     # set Frequence to 50Hz
    pwmsServo.append(pwm)

for pwm in pwmsServo:
    pwm.start(0)                     # Set initial Duty Cycle to 0

def OnScale_Servo(value, pwm):
    num = int(value)
    pwm.ChangeDutyCycle(SERVO_MIN_DUTY + num * (SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 100)

def main():
    root = tk.Tk()
    root.title('Robot Hand Manipulator')
    frame = tk.Frame(root, width = 300, height = 300)
    frame.pack_propagate(False)
    ctrl = tk.Scale(frame, orient = tk.HORIZONTAL, from_ = 70, to = 40, command = lambda value: OnScale_Servo(value, pwmsServo[3]))
    ctrl.set(60)
    ctrl.pack(side = tk.TOP, fill = tk.X)
    ctrl = tk.Scale(frame, orient = tk.HORIZONTAL, from_ = 100, to = 0, command = lambda value: OnScale_Servo(value, pwmsServo[0]))
    ctrl.set(50)
    ctrl.pack(side = tk.BOTTOM, fill = tk.X)
    ctrl = tk.Scale(frame, orient = tk.VERTICAL, from_ = 60, to = 0, command = lambda value: OnScale_Servo(value, pwmsServo[1]))
    ctrl.set(30)
    ctrl.pack(side = tk.LEFT, fill = tk.Y)
    ctrl = tk.Scale(frame, orient = tk.VERTICAL, from_ = 30, to = 0, command = lambda value: OnScale_Servo(value, pwmsServo[2]))
    ctrl.set(0)
    ctrl.pack(side = tk.RIGHT, fill = tk.Y)
    frame.pack()
    root.mainloop()

main()
for pwm in pwmsServo:
    pwm.stop()
GPIO.cleanup()
