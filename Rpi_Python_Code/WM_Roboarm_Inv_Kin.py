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

# Create the kinematic definition object from the URDF file located in this folder.
kin_def = ikpy.chain.Chain.from_urdf_file("WM_Roboarm_Inv_Kin_Def.urdf")

#Variables for target point
targetx = 100
targety = 200
targetz = 50

try:
    while True:
        # Inverse kinematics calculation
        #target_vector = [targetx, targety, targetz]
        #target_frame = np.eye(4)
        #target_frame[:3, 3] = target_vector

        print("The angles of each joints are : ", kin_def.inverse_kinematics([1, 0, 0, 2], [0, 1, 0, 2], [0, 0, 1, 2], [0, 0, 0, 1]))

        # Inverse kinematics checking
        #real_frame = kin_def.forward_kinematics(kin_def.inverse_kinematics(target_frame))
        #print("Computed position vector : %s, original position vector : %s" % (real_frame[:3, 3], target_frame[:3, 3]))
except KeyboardInterrupt:
        print "Whoa there"
