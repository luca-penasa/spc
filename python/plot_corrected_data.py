#!/usr/bin/env python

from __future__ import division
import sys
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

def gau(x, b):
    return np.exp(- ( x**2 / (2*b**2)) )
# x = np.linspace(-10,10,1000)
#
# y = gau(x, 4)

for arg in sys.argv:
    print arg

fname = sys.argv[1]

print("FILENAME: " + fname)



data = np.loadtxt(fname, skiprows = 1)
d, a, i,nn, i_std, er, pred_i = data.T[[0,1,2,3,4,5,-1]]


#er -= np.min(er)
#er/= np.max(er)

figure()
plt.scatter(d, i, c = np.log(er * 1/i_std + 1), s = 50)

figure()
plt.scatter(d, i, c= nn < 20 , s = 50)


show()