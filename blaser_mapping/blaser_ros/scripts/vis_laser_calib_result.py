#!/usr/bin/env python
# coding: utf-8

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import math
import sys

def draw_frame(ax, R, t):
    axis_len = 0.05
    origin = t.flatten()
    end_x = origin + axis_len * R[:, 0]
    end_y = origin + axis_len * R[:, 1]
    end_z = origin + axis_len * R[:, 2]
    ax.plot([origin[0], end_x[0]], [origin[1], end_x[1]], [origin[2], end_x[2]],
            color='red', linewidth=2)
    ax.plot([origin[0], end_y[0]], [origin[1], end_y[1]], [origin[2], end_y[2]],
            color='green', linewidth=2)
    ax.plot([origin[0], end_z[0]], [origin[1], end_z[1]], [origin[2], end_z[2]],
            color='blue', linewidth=2)


# read plane parameters and laser points
result_file = open(sys.argv[1])
raw_data = result_file.readlines()
plane_params = np.fromstring(raw_data[1].strip(), dtype=float, sep=',')
print(plane_params)
raw_data = raw_data[3:]
pts = np.zeros((0, 3))
while len(raw_data) != 0 and raw_data[0][:2] == "lp":
    pts = np.append(pts, np.fromstring(raw_data[0][4:].strip(), dtype=float, sep=',').reshape(1,3), axis=0)
    raw_data = raw_data[1:]

# generate plane
min_x = np.min(pts[:, 0])
max_x = np.max(pts[:, 0])
min_y = np.min(pts[:, 1])
max_y = np.max(pts[:, 1])
min_z = np.min(pts[:, 2])
max_z = np.max(pts[:, 2])
pl_xx, pl_yy = np.meshgrid(np.linspace(min_x - 0.1, max_x + 0.1),
                           np.linspace(min_y - 0.1, max_y + 0.1))
pl_z = (-plane_params[0] * pl_xx - plane_params[1] * pl_yy - plane_params[3]) / plane_params[2]


# plot points and plane
fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')
ax.plot_surface(pl_xx, pl_yy, pl_z, alpha=0.2, color='green')
ax.scatter(pts[:,0], pts[:,1], pts[:,2], s=0.3, alpha=0.2, color='red')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_title('Laser plane calibration visualization')

#code to have equal axis
max_range = np.array([max_x-min_x, max_y - min_y, max_z - min_z]).max() / 2.0 + 0.1
mid_x = (max_x+min_x) * 0.5
mid_y = (max_y+min_y) * 0.5
mid_z = (max_z+min_z) * 0.5
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

# draw axes
draw_frame(ax, np.eye(3), np.zeros((3,1)))

# read laser extrinsics
if len(raw_data) != 0:
    Rcl = np.zeros((3,3))
    Rcl[0] = np.fromstring(raw_data[1].strip(), dtype=float, sep=' ')
    Rcl[1] = np.fromstring(raw_data[2].strip(), dtype=float, sep=' ')
    Rcl[2] = np.fromstring(raw_data[3].strip(), dtype=float, sep=' ')
    raw_data = raw_data[4:]
    tcl = np.fromstring(raw_data[1].strip(), dtype=float, sep=' ').reshape(3,1)
    raw_data = raw_data[2:]
    lb_l = np.fromstring(raw_data[1].strip(), dtype=float, sep=' ')
    lb_l = np.append(lb_l, 0.).reshape(3,1)
    rb_l = np.fromstring(raw_data[3].strip(), dtype=float, sep=' ')
    rb_l = np.append(rb_l, 0.).reshape(3,1)
    raw_data = raw_data[4:]
    assert(len(raw_data) == 0)

    # draw laser extrinsics
    draw_frame(ax, Rcl, tcl)
    lb_c = Rcl.dot(0.5 * lb_l) + tcl
    rb_c = Rcl.dot(0.5 * rb_l) + tcl
    ax.plot([tcl[0][0], lb_c[0][0]], [tcl[1][0], lb_c[1][0]], [tcl[2][0], lb_c[2][0]],
	    color='c', linewidth=1.3)
    ax.plot([tcl[0][0], rb_c[0][0]], [tcl[1][0], rb_c[1][0]], [tcl[2][0], rb_c[2][0]],
	    color='c', linewidth=1.3)

plt.show()
# todo number is right, maybe drawing is wrong? frame!

# In[ ]:




