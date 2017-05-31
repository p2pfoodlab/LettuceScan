import DepthSense as DS
from flask import Flask, render_template, Response,request
import requests
import time
import cv2
import os
import numpy as np
import relator
import urllib

app=Flask(__name__)
ds=DS.initDepthSense()
#imdir="/home/pi/scan/test"
imdir="/run/mjpg"

meas=[]


@app.route('/scan3d/RO/<cmd>')
def getMuv(cmd):
   req=urllib.urlopen("http://0.0.0.0:9000/?action=snapshot")
   arr = np.asarray(bytearray(req.read()), dtype=np.uint8)
   img = cv2.imdecode(arr,-1)
   cur_data=[cmd]
   ds=relator.getMuv(img)
   for d in ds: cur_data.append(d)
   return meas.append(cur_data)


@app.route('/scan3d/svg/<fname>')
def svg(fname,erase=True):
   np.savetxt("%s.dat"%fname,meas)
   if erase: meas=[]

@app.route('/scan3d/<ims>/<i>')
def getframe(ims, i):
   if (ims=='c' or ims=='all'): 
      im=DS.getColourMap()
      cv2.imwrite("%s/color_%s.png"%(imdir,i), im)   
   if (ims=='all'): 
      im=DS.getConfidenceMap()
      cv2.imwrite("%s/confidence_%s.png"%(imdir,i), im)   
   if (ims=='all'): 
      im=DS.getDepthColouredMap()
      cv2.imwrite("%s/DepthColored_%s.png"%(imdir,i), im)   
   if (ims=='all'): 
      im=DS.getDepthMap()
      cv2.imwrite("%s/depth_%s.png"%(imdir,i), im)   
   if (ims=='all'): 
      im=DS.getGreyScaleMap()
      cv2.imwrite("%s/gscale_%s.png"%(imdir,i), im)   
   if (ims=='all'): 
      im=DS.getSyncMap()
      np.save("%s/sync_%s"%(imdir,i), im)   
   if (ims=='all'): 
      im=DS.getUVMap()
      np.save("%s/uv_%s"%(imdir,i), im)   
   if (ims=='all'): 
      im=DS.getVertices()
      np.save("%s/vert_%s"%(imdir,i), im)   
   return "OK"

      
if __name__=='__main__':
   app.run(host='0.0.0.0',debug=False)
   #ds=DS.initDepthSense()
             
