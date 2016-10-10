#setup section

#Time Library Import
import time

#Cartesian Space to Joint Space Libraries
from math import *
import numpy as np

#Arm Calculation Code Import
import Arm

wmroboarm = Arm.Arm3Link(L = np.array([255,255,1]))
                
#Scale mouse input to correct size
targety = 400
targetz = 200

Shoulder_Current = 50
Elbow_Current = -50
      
#Plug Variables into function and store output as angles variable
wmroboarm.q = [radians(Shoulder_Current), radians(Elbow_Current), 0]
angles = wmroboarm.inv_kin(xy = (targety, targetz))
       
#Find angles to set arm at
Shoulder_Set_Angle = (degrees(angles[0]))
Elbow_Set_Angle = (degrees(angles[1]))

#Print angles

print Shoulder_Set_Angle
print Elbow_Set_Angle

