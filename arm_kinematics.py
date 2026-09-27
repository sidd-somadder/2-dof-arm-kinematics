import numpy as np
import matplotlib as plt




def kin_2dof_solver(L1, L2, w1, w2, wdot1, wdot2, t_stop=10.0, dt=0.02):
    '''
    Uses known geometric & initial kinematic parameters to solve for the wrist velocity and acceleration over time

    Inputs:
    * L1, L2: Fixed arm lengths L1 (upper arm) and L2 (forearm)
    * w1, w2: Known initial joint angular velocities at shoulder pin (w1) and elbow joint (w2) where w2 is relative to w1
    * wdot1, wdot2: Known joint angular accelerations where wdot2 is relative to wdot1, assumed constant
    Note:
    * time of computation is pre-defined in function call but may be changed (t_stop)
    * time step is predefined (0.02 s) may be changed.

    Outputs:
    * The following arrays, where N is the number of time steps:
        * t: time array, size N
        * theta1, theta2: absolute joint angles from the global x-axis in radians, size N
        (initial conditions fixed at theta1 = 30 deg, theta2 = 60 deg)
        * p_elbow, p_wrist: elbow and wrist positions in the global frame, size N x 2
        * v_wrist: wrist velocity in global x and y components, size N x 2
        * a_wrist: wrist acceleration in global x and y components, size N x 2
    '''

    # Note, problem uses initial conditions shoulder angle = 30 deg. from positive x axis, 
    # and elbow angle 60 from negative x-axis 
    thetaS0 = np.radians(30)
    theta20 = np.radians(60)
    thetaE0 = np.pi - theta20

    step_ct = round(t_stop / dt) + 1 # should be 501 in standard run, +1 to account for [0, t_stop] inclusive

    t = np.linspace(0, t_stop, step_ct)
    print(t)
 
    w1_t = w1 + wdot1*t
    w2_t = w2 + wdot2*t
    w_Wt = w1_t + w2_t
    wdot_Wt = wdot1 + wdot2; # constant value

    thetaS = thetaS0 + w1 * t + 0.5 * wdot1 * t**2
    thetaE = thetaE0 + (w_Wt) * t + 0.5 * (wdot_Wt) * t**2    

    # define unit vectors for rotating shoulder and elbow frames
    # see derivations.pdf for setup using shoulder/elbow coordinate frames, this transforms back to global i,j 
    i_s = np.column_stack((np.cos(thetaS), np.sin(thetaS)))
    i_e = np.column_stack((np.cos(thetaE), np.sin(thetaE)))
    j_s = np.column_stack((-np.sin(thetaS), np.cos(thetaS)))
    j_e = np.column_stack((-np.sin(thetaE), np.cos(thetaE)))

    return {"t": t, "thetaS": thetaS, "thetaE": thetaE}




