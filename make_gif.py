import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from arm_kinematics import kin_2dof_solver

# sample parameters used to generate gif
L1 = 0.35, 
L2= 0.30
w1 = 1.0
w2 =-2.0         
wdot1 = 0.05
wdot2 = 0.0
t_stop = 6.0                

frame_step = 3
out_path = "figures/demo.gif"

