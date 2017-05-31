from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as pl
import numpy as np
import scipy.linalg
import pypcd as pcd

vert=np.load("/home/kodda/Dropbox/p2pflab/data/17.03.17/N100_c/vert_5.npy").reshape([320*240,3]).clip(-500,500)
fig = pl.figure()
ax = fig.add_subplot(111, projection='3d')
#ax._axis3don = False

idx=np.where((vert[:,0]*.8+vert[:,1]*0.85+242.78-vert[:,2])<-10)[0]

ax.scatter(vert[idx,0],vert[idx,1],vert[idx,2], marker='.',c="r",alpha=.1)


A = np.c_[vert[idx,0], vert[idx,1], np.ones(vert[idx,:].shape[0])]
C,_,_,_ = scipy.linalg.lstsq(A, vert[idx,2])    # coefficients

X,Y = np.meshgrid(np.arange(-500.0, 500.0, 5), np.arange(-500.0, 500.0, 5))
XX = X.flatten()
YY = Y.flatten()

Z = C[0]*X + C[1]*Y + C[2]
Z2 = np.dot(np.c_[XX, YY, np.ones(XX.shape)], C).reshape(X.shape)
ax.scatter(X,Y,Z2, marker='.',c="g",alpha=.1)

