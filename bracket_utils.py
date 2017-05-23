import serial
import time
import numpy as np

modes={'SINGLE': 1, 'DOUBLE': 2, 'INTERLEAVE': 3, "MICROSTEP": 4}

def init(port="/dev/ttyACM0", mode=2, speed=50):
   s = serial.Serial(port,9600)
   s.write("s%s;"%speed)
   #s.write("m%s;"%modes[mode])
   return s

def init_test(port="/dev/ttyACM0", params={}):
   s = serial.Serial(port,9600)
   #TODO: As for now only single character commands are allowed we use q,r for pan stepper settings and u, v for tilt (should be changed)
   s.write("q%s;"%params['speed']['pan'])
   s.write("u%s;"%params['speed']['tilt'])
   s.write("r%s;"%modes[params['mode']['pan']])
   s.write("v%s;"%modes[params['mode']['tilt']])
   return s

def send_cmd(cmd, port):
    s.write(cmd)
    #res=s.readline()
    return res
