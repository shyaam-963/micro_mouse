import numpy as np

scale = 0.25

p1 = np.array([1,1])*scale
p2 = np.array([-1,1])*scale
p3 = np.array([-1,-1])*scale
p4 = np.array([1,-1])*scale

p = [p1,p2,p3,p4]
data = np.asarray(p)
np.save('points.npy',data)