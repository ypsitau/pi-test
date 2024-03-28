#!/usr/bin/env python
import time
import smbus
import RPi.GPIO as GPIO

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

pinLEDRed = 11
pinLEDGreen = 13
pinLEDBlue = 15

adc = None
pwmLEDRed = None
pwmLEDGreen = None
pwmLEDBlue = None

def setup():
    global adc
    global pwmLEDRed
    global pwmLEDGreen
    global pwmLEDBlue
    adc = ADCDevice() # Define an ADCDevice class object
    if(adc.detectI2C(0x48)): # Detect the pcf8591.
        adc = PCF8591()
    elif(adc.detectI2C(0x4b)): # Detect the ads7830
        adc = ADS7830()
    else:
        print("No correct I2C address found, \n"
        "Please use command 'i2cdetect -y 1' to check the I2C address! \n"
        "Program Exit. \n");
        exit(-1)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pinLEDRed, GPIO.OUT)
    GPIO.setup(pinLEDGreen, GPIO.OUT)
    GPIO.setup(pinLEDBlue, GPIO.OUT)
    GPIO.output(pinLEDRed, GPIO.LOW)
    GPIO.output(pinLEDGreen, GPIO.LOW)
    GPIO.output(pinLEDBlue, GPIO.LOW)
    pwmLEDRed = GPIO.PWM(pinLEDRed, 500)
    pwmLEDGreen = GPIO.PWM(pinLEDGreen, 500)
    pwmLEDBlue = GPIO.PWM(pinLEDBlue, 500)
    pwmLEDRed.start(0)
    pwmLEDGreen.start(0)
    pwmLEDBlue.start(0)
        
def loop():
    while True:
        values = []
        for i in range(3):
            values.append(adc.analogRead(i))
        #print ('ADC Value : %d, Voltage : %.2f'%(value,voltage))
        print('%03d %03d %03d' % (values[0], values[1], values[2]))
        pwmLEDRed.ChangeDutyCycle(100 - values[0] * 100 / 255)
        pwmLEDGreen.ChangeDutyCycle(100 - values[1] * 100 / 255)
        pwmLEDBlue.ChangeDutyCycle(100 - values[2] * 100 / 255)
        time.sleep(0.1)

def destroy():
    adc.close()
    
if __name__ == '__main__':   # Program entrance
    print ('Program is starting ... ')
    try:
        setup()
        loop()
    except KeyboardInterrupt: # Press ctrl-c to end the program.
        destroy()
