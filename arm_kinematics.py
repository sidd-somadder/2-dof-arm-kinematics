import numpy as np

def kin_2dof_solver(L1, L2, w1, w2, wdot1, wdot2, t_stop=10.0, dt=0.02):
    '''
    Uses known geometric & initial kinematic parameters to solve for the wrist velocity and acceleration over time

    Inputs:
    * L1, L2: Fixed arm lengths L1 (upper arm) and L2 (forearm) (meters)
    * w1, w2: Known initial joint angular velocities at shoulder pin (w1) and elbow joint (w2) where w2 is relative to w1 (rad/s)
    * wdot1, wdot2: Known joint angular accelerations where wdot2 is relative to wdot1, assumed constant (rad/s^2)
    Note:
    * time of computation is pre-defined in function call but may be changed (t_stop) (seconds)
    * time step is predefined (0.02 s) may be changed.

    Outputs:
    * The dictionary with the following arrays, where N is the number of time steps:
        * t: time array, size N (seconds)
        * thetaS, thetaE: absolute upper arm and forearm angles from the global 
        +x axis in radians, size N. Initial conditions fixed at thetaS0 = 30 deg,
        thetaE0 = 120 deg (forearm 60 deg from the -x axis). (radians)
        * p_elbow, p_wrist: elbow and wrist positions in the global frame, size N x 2 (meters)
        * v_wrist: wrist velocity in global x and y components, size N x 2 (m/s)
        * a_wrist: wrist acceleration in global x and y components, size N x 2 (m/s^2)
    '''

    # Note, problem uses initial conditions shoulder angle = 30 deg. from positive x axis, 
    # and elbow angle 60 from negative x-axis 
    thetaS0 = np.radians(30)
    theta20 = np.radians(60)
    thetaE0 = np.pi - theta20

    step_ct = round(t_stop / dt) + 1 # should be 501 in standard run, +1 to account for [0, t_stop] inclusive

    t = np.linspace(0, t_stop, step_ct)
 
    w1_t = w1 + wdot1*t
    w2_t = w2 + wdot2*t
    w_Wt = w1_t + w2_t
    wdot_Wt = wdot1 + wdot2 # constant value

    thetaS = thetaS0 + w1 * t + 0.5 * wdot1 * t**2
    thetaE = thetaE0 + (w1+w2) * t + 0.5 * (wdot_Wt) * t**2    

    # define unit vectors for rotating shoulder and elbow frames
    # see derivations.pdf for setup using shoulder/elbow coordinate frames, this transforms back to global i,j 
    i_s = np.column_stack((np.cos(thetaS), np.sin(thetaS)))
    i_e = np.column_stack((np.cos(thetaE), np.sin(thetaE)))
    j_s = np.column_stack((-np.sin(thetaS), np.cos(thetaS)))
    j_e = np.column_stack((-np.sin(thetaE), np.cos(thetaE)))

    # define relative position vectors
    R_EtS = L1 * i_s
    R_WtE = L2 * i_e

    # position vectors of the wrist and elbow
    p_wrist = R_EtS + R_WtE
    p_elbow = R_EtS

    # wrist velocity and acceleration; see derivations.pdf for expression
    vel_wrist = L1 * w1_t[:, None] * j_s + L2 * w_Wt[:, None] * j_e
    accel_wrist = (L1 * wdot1 * j_s - L1 * (w1_t[:,None])**2 * i_s 
                   + L2 * wdot_Wt * j_e - L2 * (w_Wt[:,None])**2 * i_e) 

    return {"t": t, "thetaS": thetaS, "thetaE": thetaE,
        "p_elbow": p_elbow, "p_wrist": p_wrist,
        "v_wrist": vel_wrist, "a_wrist": accel_wrist}




