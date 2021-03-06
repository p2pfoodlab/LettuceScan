import serial
import time
import numpy as np

def clamped(value, minval, maxval, scale):
   return int(scale*np.clip(float(value), minval, maxval))

def init(port="/dev/ttyUSB0",homing=False):
   s = serial.Serial(port,115200)
   s.write("\r\n\r\n")
   time.sleep(2)
   s.flushInput()

   if homing: home(s)
   s.write("G90 \n")
   grbl_out=s.readline()
   print ' : ' + grbl_out.strip()
   s.write("G21 \n")
   grbl_out=s.readline()
   print ' : ' + grbl_out.strip()
   return s

def home(s):
   send_cmd("$H \n",s)
   #send_cmd("g28 \n",s)
   send_cmd("g92 x0 y0 z0 \n",s)

def send_cmd(cmd,s):
    print cmd
    s.write(cmd)
    grbl_out=s.readline()
    print ' : ' + grbl_out.strip()
    return grbl_out

