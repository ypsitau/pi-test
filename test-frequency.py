#!/usr/bin/env python

import RPi.GPIO as GPIO
import time

pinOut = 11

GPIO.setmode(GPIO.BOARD)
GPIO.setup(pinOut, GPIO.OUT)
try:
	while True:
		GPIO.output(pinOut, GPIO.HIGH)
		GPIO.output(pinOut, GPIO.HIGH)
		GPIO.output(pinOut, GPIO.HIGH)
		GPIO.output(pinOut, GPIO.HIGH)
		#time.sleep(0.00001)
		GPIO.output(pinOut, GPIO.LOW)
		GPIO.output(pinOut, GPIO.LOW)
		GPIO.output(pinOut, GPIO.LOW)
		GPIO.output(pinOut, GPIO.LOW)
		#time.sleep(0.00001)
except KeyboardInterrupt:
    GPIO.cleanup()
