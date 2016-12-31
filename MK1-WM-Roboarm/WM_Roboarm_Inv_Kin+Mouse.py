# Robot Arm Controller Code

# (C) 2016-2017 David Cutting

# https://github.com/Phylliade/ikpy/wiki/Inverse-Kinematics
# https://github.com/Phylliade/ikpy/wiki
# https://github.com/Phylliade/ikpy/blob/master/tutorials/ikpy/getting_started.md

# If there is a matplotlib error, uncomment the next line, and comment the line below it.
# %matplotlib inline
%matplotlib notebook

import matplotlib.pyplot as plt
import ikpy
import numpy as np
from ikpy import plot_utils
import time
from spnav import *
from math import *
import minimalmodbus

# Open the 3D mouse instrunment
spnav_open()

# Open the modbus communication channel with the arduino device
arduino = minimalmodbus.Instrument('/dev/ttyACM0', 1) # port name, slave address (in decimal)

# Create the kinematic definition object from the URDF file located in this folder.
kin_def = ikpy.chain.Chain.from_urdf_file("WM_Roboarm_Inv_Kin_Def.URDF")

#Variables for target point
targetx = 0
targety = 0
targetz = 0

time.sleep(2)

try:
    while True:
        Shoulder_Pot_Degrees = arduino.read_register(2, 1)
        Elbow_Pot_Degrees = arduino.read_register(3, 1)
        Shoulder_Pot_Raw = arduino.read_register(4, 0)
        Elbow_Pot_Raw = arduino.read_register(5, 0)

        #Convert Degrees to Radians
        Shoulder_Pot_Radians = radians(Shoulder_Pot_Degrees)
        Elbow_Pot_Radians = radians(Elbow_Pot_Degrees)

        #Determine if there is change in the mouse
        event = spnav_poll_event()

        if (event != None):
            if (event.ev_type == SPNAV_EVENT_MOTION):
                #Aquiring and parsing out translation and rotation variables from the mouse
                tran = event.translation
                rot = event.rotation
                x = tran[0]
                y = tran[1]
                z = tran[2]
                a = rot[0]
                b = rot[1]
                c = rot[2]

                #Scale mouse input to correct size
                targety += (y/mouse_scal)
                targetz += (z/mouse_scal)
				
                # Inverse kinematics calculation
                target_vector = [ 0.1, -0.2, 0.1]
                target_frame = np.eye(4)
                target_frame[:3, 3] = target_vector
                print("The angles of each joints are : ", kin_def.inverse_kinematics(target_frame))

                # Inverse kinematics checking
                real_frame = kin_def.forward_kinematics(kin_def.inverse_kinematics(target_frame))
                print("Computed position vector : %s, original position vector : %s" % (real_frame[:3, 3], target_frame[:3, 3]))

                ### Plot the inverse kinematics result (uncomment the next 4 lines to enable)
                ax = plot_utils.init_3d_figure()
                my_chain.plot(my_chain.inverse_kinematics(target_frame), ax, target=target_vector)
                plt.xlim(-0.1, 0.1)
                plt.ylim(-0.1, 0.1)
except KeyboardInterrupt:
        print "Whoa there"
		
arduino.close()
spnav_close() 
