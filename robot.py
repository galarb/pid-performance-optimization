'''
    lego cable pinout:
        green   - 3v
        red     - GND
        blue    - encoderin1
        yellow  - encoderin2
        black/white - motor terminals  - connect to the ZK-5AD motor driver!        
          
'''
import motordriver
import machine
import utime
from time import sleep
from ili9341 import Display, color565
from xglcd_font import XglcdFont
import sdcard
import os
import math
from bmphandle import read_bmp_file, display_bmp, read_bmp_in_chunks

encoder1_pin = 14 
encoder2_pin = 12  
in1_pin = 0       
in2_pin = 4
wheel_size = 65    # wheel size (in mm)


# SPI and display setup
spi = machine.SPI(1, baudrate=30000000, sck=machine.Pin(18), mosi=machine.Pin(23), miso=machine.Pin(19))
dc = machine.Pin(21)
cs = machine.Pin(5)  # Screen CS
rst = machine.Pin(22)
sd_cs = machine.Pin(15)
display = Display(spi, cs=cs, dc=dc, rst=rst, rotation=90, mirror=False)

# Clear the display
display.clear()

# Load font for displaying text
espresso_dolce = XglcdFont('fonts/EspressoDolce18x24.c', 18, 24)
arcade = XglcdFont('fonts/ArcadePix9x11.c', 9, 11)

# Initialize SD card (optional, can be removed if not needed)
try:
    sd = sdcard.SDCard(spi, sd_cs)
    os.mount(sd, '/sd')
    print("Files on SD card:", os.listdir('/sd'))
    read_bmp_in_chunks('/sd/gal.bmp', display, x_offset=85, y_offset=260)

except OSError as e:
    print("Error initializing SD card:", e)
    
robot = motordriver.motordriver(encoder1_pin, encoder2_pin, in1_pin, in2_pin, wheel_size, display, arcade)
robot.reset_iterations()
#print('displacement = ', robot.gomm(100, 240*3)) #distance, number of iterations
robot.godegreesp(360, 3*240, 2, 0, 0, color565(0, 255, 0)) # distance, number of iterations, Kp, Ki, Kd
robot.godegreesp(720, 3*240, 5, 1, 0, color565(255, 0, 0)) # distance, number of iterations, Kp, Ki, Kd
robot.godegreesp(1040, 3*240, 2, 3, 1, color565(255, 80, 255))
robot.motgo(0)
# try: godegrees(angle, times) and godegreesp(angle, times, kp, ki, kd)
