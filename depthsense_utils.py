import DepthSense as DS
import cv2
import numpy as np

def getframe(imdir, sfx):
   im=DS.getColourMap()
   cv2.imwrite("%s/rgb%s.png"%(imdir,sfx), im)
   #im=DS.getConfidenceMap()
   #cv2.imwrite("%s/confidence%s.png"%(imdir,sfx), im)
   #im=DS.getDepthColouredMap()
   #cv2.imwrite("%s/DepthColored%s.png"%(imdir,sfx), im)
   im=DS.getDepthMap()
   cv2.imwrite("%s/depth%s.png"%(imdir,sfx), im)
   #im=DS.getGreyScaleMap()
   #cv2.imwrite("%s/gscale%s.png"%(imdir,sfx), im)
   im=DS.getSyncMap()
   np.save("%s/sync%s"%(imdir,sfx), im)
   im=DS.getUVMap()
   np.save("%s/uv%s"%(imdir,sfx), im)
   im=DS.getVertices()
   np.save("%s/vert%s"%(imdir,sfx), im)
   return "OK"
