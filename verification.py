import numpy as np
from arm_kinematics import kin_2dof_solver

def rigid_rotation_test():
    print(r"Rigid Rotation test: w2 = wdot1 = wdot2 = 0; tolerance = 1e-10")
    tolerance = 1e-10;
    L1 = 0.30
    L2 = 0.35
    w1 = 1.5
    results = kin_2dof_solver(L1,L2,w1,0.0,0.0,0.0,t_stop=5.0)

    p = results["p_wrist"]
    r = np.linalg.norm(p, axis=1)
    v = results["v_wrist"]
    v_mag = np.linalg.norm(v, axis=1)
    a = results["a_wrist"]
    a_mag = np.linalg.norm(a, axis=1)

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

    print(f"|r| = constant : {test[0]}, max r_error = {errors[0]}")
    print(f"|v| = rw : {test[1]}, max a_error = {errors[1]}")
    print(f"|a| = w^2r : {test[2]}, max a_error = {errors[2]}")

rigid_rotation_test()
print("------------------------")


    


