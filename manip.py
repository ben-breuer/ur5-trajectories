import numpy as np
import roboticstoolbox as rtb

robot = rtb.models.DH.UR5()
v_soll = np.array([0.1, 0, 0, 0, 0, 0])

print(" q3 [°]   w        max|q̇| [°/s]")
for q3 in [80, 40, 20, 10, 5, 1, 0]:
    q = np.deg2rad([20, -60, q3, -110, -90, 30])
    w = robot.manipulability(q)
    if w > 1e-9:
        qd = np.rad2deg(np.linalg.solve(robot.jacob0(q), v_soll))
        print(f"{q3:6d}   {w:.5f}   {np.max(np.abs(qd)):8.1f}")
    else:
        print(f"{q3:6d}   {w:.5f}   singulär")