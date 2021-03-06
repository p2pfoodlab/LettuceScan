from flask import Flask, request, jsonify, render_template, abort
from flask import redirect, url_for, send_file
import time
import serial
import json
import numpy as np
import cnc_utils as cu
import bracket_utils as bu
import depthsense_utils as DSu

params=json.load(open("params/default.json",'r'))
#app = Flask(__name__, template_folder=params["tempdir"])
app = Flask(__name__)
ds = DSu.DS.initDepthSense()
s_cnc=cu.init()
s_bracket=bu.init()

@app.route('/')
def index():
    return redirect('static/index.html')

@app.route('/moveto', methods=['POST'])
def updatePose():
   pose=[]
   for coord in ['x','y','z','pan','tilt']: pos.append(cu.clamped(request.form[coord], params['lims'][coord], params['scale'][coord]))           
   cu.send_cmd("g0x%sy%sz-%s"%(pose[0],pos[1],pos[2]),s_cnc)
   bu.send_cmd("p%;"%pose[3],s_bracket)
   bu.send_cmd("t%;"%pose[4],s_bracket)
   return

@app.route('/grab')
def grab():
    DS.grab(params["imdir"],"./")
    return          

@app.route('/rgb.png')
def rgb():
   return send_file(params["imdir"]+"/rgb.png", mimetype='image/png')

@app.route('/depth.png')
def depth():
   return send_file(params["imdir"]+"/depth.png", mimetype='image/png')
              
if __name__ == '__main__':
   app.run(debug=False,host='0.0.0.0')
