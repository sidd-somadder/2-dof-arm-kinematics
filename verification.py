import numpy as np
from arm_kinematics import kin_2dof_solver

def rigid_rotation_test():
    print(r"Rigid Rotation test: w2 = wdot1 = wdot2 = 0; tolerance = 1e-10")
    tolerance = 1e-10;
    # arbitrary L1, L2, w1 values
    L1 = 0.35
    L2 = 0.30
    w1 = 1.5
    results = kin_2dof_solver(L1,L2,w1,0.0,0.0,0.0,t_stop=5.0)

    # extract solved wrist position, velocity, and acceleration; convert from vectors to magnitude arrays
    p = results["p_wrist"]
    r = np.linalg.norm(p, axis=1)
    v = results["v_wrist"]
    v_mag = np.linalg.norm(v, axis=1)
    a = results["a_wrist"]
    a_mag = np.linalg.norm(a, axis=1)

    print(f"initial conditions: |r| = {r[0]:.3f}; |v| = {v_mag[0]:.3f}; |a| = {a_mag[0]:.3f}")
    # analytical solution: r = constant, |v| = Rw1, |a| = w^2R
    r_error = np.max(np.abs(r-r[0]));
    v_error = np.max(np.abs(v_mag-r*w1));
    a_error = np.max(np.abs(a_mag-r*w1**2));

    errors = [r_error, v_error, a_error]
    test = []
    for e in range(len(errors)): 
        if errors[e] <= tolerance:
            test.append("PASS")
        else:
            test.append("FAIL")

    print(f"|r| = constant : {test[0]}, max r_error = {errors[0]:}")
    print(f"|v| = rw : {test[1]}, max v_error = {errors[1]}")
    print(f"|a| = w^2r : {test[2]}, max a_error = {errors[2]}")

def fixed_shoulder_angle_test():
    print(r"Fixed Shoulder Angle test: w1 = wdot1 = 0; tolerance = 1e-10")
    tolerance = 1e-10;
    # arbitrary L1, L2, w1 values
    L1 = 0.35
    L2 = 0.30
    w2 = 1.5
    wdot2 = 0.3
    results = kin_2dof_solver(L1,L2,0.0,w2,0.0,wdot2,t_stop=5.0)
    t = results["t"]
    w2_t = w2 + wdot2*t;

    # extract solved elbow position, wrist velocity/acceleration; convert from vectors to magnitude arrays
    p_e = results["p_elbow"]
    v = results["v_wrist"]
    v_mag = np.linalg.norm(v, axis=1)
    a = results["a_wrist"]
    a_mag = np.linalg.norm(a, axis=1)
    
    # analytical solution: r_elbow stationary, |v|(t) = L2 * w2_t, |a|(t) = sqrt((L2*w2_t^2)^2+(L2*wdot2)^2)
    elbow_error = np.max(np.linalg.norm(p_e - p_e[0], axis=1));
    v_error = np.max(np.abs(v_mag-L2*np.abs(w2_t)));
    a_error = np.max(np.abs(a_mag-np.sqrt((L2*(w2_t**2))**2+(L2*wdot2)**2)));
    
    errors = [elbow_error, v_error, a_error]
    test = []
    for e in range(len(errors)): 
        if errors[e] <= tolerance:
            test.append("PASS")
        else:
            test.append("FAIL")
    
    print(f"stationary elbow : {test[0]}, max elbow position error = {errors[0]}")
    print(f"|v|(t) = L2*w2_t : {test[1]}, max v_error = {errors[1]}")
    print(f"|a|(t) = sqrt((L2*w2_t^2)^2+(L2*wdot2)^2) : {test[2]}, max a_error = {errors[2]}")

def finite_difference_test():
    print(r"Finite difference test: all inputs nonzero; tolerance = 1e-4")
    tolerance = 1e-4
    results = kin_2dof_solver(L1=0.35, L2= 0.30, w1=2, w2=-3, wdot1=0.2, wdot2=0.45, t_stop=5.0,dt=0.0005)
    t = results["t"]
    p = results["p_wrist"]
    v = results["v_wrist"]
    v_mag = np.linalg.norm(v, axis=1)
    a = results["a_wrist"]
    a_mag = np.linalg.norm(a, axis=1)

    v_numerical = np.gradient(p, t, axis=0)
    a_numerical = np.gradient(v, t, axis=0)
    s = slice(3,-3)

    v_error = np.max(np.linalg.norm(v_numerical[s]-v[s],axis=1))
    a_error = np.max(np.linalg.norm(a_numerical[s]-a[s],axis=1))

    errors = [v_error, a_error]
    test = []
    for e in range(len(errors)): 
        if errors[e] <= tolerance:
            test.append("PASS")
        else:
                test.append("FAIL")

    print(f"velocity : {test[0]}, max v_error = {errors[0]}")
    print(f"acceleration : {test[1]}, max a_error = {errors[1]}")    

rigid_rotation_test()
print("------------------------")
fixed_shoulder_angle_test()
print("------------------------")
finite_difference_test()


    


