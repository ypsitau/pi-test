#!/usr/bin/env python
import RPi.GPIO as GPIO
import time

pinLED = 11
pinButton = 12
GPIO.setmode(GPIO.BOARD)
GPIO.setup(pinLED, GPIO.OUT)
GPIO.setup(pinButton, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.output(pinLED, GPIO.LOW)
pwmLED = GPIO.PWM(pinLED, 500)
pwmLED.start(10)
while True:
    for dutyCycle in range(0, 101, 1):
        pwmLED.ChangeDutyCycle(dutyCycle)
        time.sleep(.01)
    for dutyCycle in range(100, -1, -1):
        pwmLED.ChangeDutyCycle(dutyCycle)
        time.sleep(.01)
#pwmLED.ChangeDutyCycle(50)
#while True:
#    if GPIO.input(pinButton) == GPIO.LOW:
#        GPIO.output(pinLED, GPIO.HIGH)
#    else:
#        GPIO.output(pinLED, GPIO.LOW)
