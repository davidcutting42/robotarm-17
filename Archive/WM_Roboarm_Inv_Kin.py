# Robot Arm Controller Code
# (C) 2016-2017 David Cutting
# https://github.com/Phylliade/ikpy/wiki/Inverse-Kinematics
# https://github.com/Phylliade/ikpy/wiki
# https://github.com/Phylliade/ikpy/blob/master/tutorials/ikpy/getting_started.md

import ikpy
import numpy as np
from ikpy import plot_utils
import time
from math import *
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

# Create the kinematic definition object from the URDF file located in this folder.
kin_def = ikpy.chain.Chain.from_urdf_file("WM_Roboarm_Inv_Kin_Def.urdf")

#Variables for target point
targetx = 100
targety = 200
targetz = 50

#print("The angles of each joints are : ", kin_def.inverse_kinematics([[1, 0, 0, 100],
 #                                                                     [0, 1, 0, 100],
  #                                                                    [0, 0, 1, 50],
   #                                                                   [0, 0, 0, 1]]))
#

kin_def.plot(kin_def.inverse_kinematics([
    [1, 0, 0, 100],
    [0, 1, 0, 100],
    [0, 0, 1, 50],
    [0, 0, 0, 1]
    ]), ax)
matplotlib.pyplot.show()
