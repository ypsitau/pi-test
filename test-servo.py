#!/usr/bin/env python3
########################################################################
# Filename    : Sweep.py
# Description : Servo sweep
# Author      : www.freenove.com
# modification: 2019/12/27
########################################################################
import smbus
import RPi.GPIO as GPIO
import time

OFFSET_DUTY = 1        # define pulse offset of servo
SERVO_MIN_DUTY = 2.5 + OFFSET_DUTY     # define pulse duty cycle for minimum angle of servo
#SERVO_MAX_DUTY = 12.5 + OFFSET_DUTY    # define pulse duty cycle for maximum angle of servo
SERVO_MAX_DUTY = 12.5 + OFFSET_DUTY    # define pulse duty cycle for maximum angle of servo
servoPin = 12

class ADCDevice(object):
    def __init__(self):
        self.cmd = 0
        self.address = 0
        self.bus=smbus.SMBus(1)
        # print("ADCDevice init")
        
    def detectI2C(self,addr):
        try:
            self.bus.write_byte(addr,0)
            print("Found device in address 0x%x"%(addr))
            return True
        except:
            print("Not found device in address 0x%x"%(addr))
            return False
            
    def close(self):
        self.bus.close()

class PCF8591(ADCDevice):
    def __init__(self):
        super(PCF8591, self).__init__()
        self.cmd = 0x40     # The default command for PCF8591 is 0x40.
        self.address = 0x48 # 0x48 is the default i2c address for PCF8591 Module.
        
    def analogRead(self, chn): # PCF8591 has 4 ADC input pins, chn:0,1,2,3
        value = self.bus.read_byte_data(self.address, self.cmd+chn)
        value = self.bus.read_byte_data(self.address, self.cmd+chn)
        return value
    
    def analogWrite(self,value): # write DAC value
        self.bus.write_byte_data(address,cmd,value)	

class ADS7830(ADCDevice):
    def __init__(self):
        super(ADS7830, self).__init__()
        self.cmd = 0x84
        self.address = 0x4b # 0x4b is the default i2c address for ADS7830 Module.   
        
    def analogRead(self, chn): # ADS7830 has 8 ADC input pins, chn:0,1,2,3,4,5,6,7
        value = self.bus.read_byte_data(self.address, self.cmd|(((chn<<2 | chn>>1)&0x07)<<4))
        return value

def setup():
    global p
    GPIO.setmode(GPIO.BOARD)         # use PHYSICAL GPIO Numbering
    GPIO.setup(servoPin, GPIO.OUT)   # Set servoPin to OUTPUT mode
    GPIO.output(servoPin, GPIO.LOW)  # Make servoPin output LOW level

    p = GPIO.PWM(servoPin, 50)     # set Frequence to 50Hz
    p.start(0)                     # Set initial Duty Cycle to 0
    
def loop():
    adc = ADS7830()
    while True:
        value = adc.analogRead(0)
        p.ChangeDutyCycle(SERVO_MIN_DUTY + value * (SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 255)
        time.sleep(0.1)

def destroy():
    p.stop()
    GPIO.cleanup()

if __name__ == '__main__':     # Program entrance
    print ('Program is starting...')
    setup()
    try:
        loop()
    except KeyboardInterrupt:  # Press ctrl-c to end the program.
        destroy()
