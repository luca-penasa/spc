#!/usr/bin/env python

import sys
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt




# plt.xkcd()

for arg in sys.argv:
    print arg

fname = sys.argv[1]

print("FILENAME: " + fname)



data = np.loadtxt(fname, skiprows = 1)
d = data.T[0]
a = data.T[1]
i = data.T[2]


fig = plt.figure()

ax = fig.add_subplot(111, projection='3d')
ax.scatter(d[::5],a[::5],i[::5])

show()