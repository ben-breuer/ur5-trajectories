import numpy as np
import roboticstoolbox as rtb

np.set_printoptions(suppress=True, precision=2)
robot = rtb.models.DH.UR5()

v_soll = np.array([0.1, 0, 0, 0, 0, 0])

stellungen = {
    "normal (q5=-90)":  [20, -60, 80, -110, -90, 30],
    "nahe q5=0 (q5=5)": [20, -60, 80, -110,   5, 30],
    "q5 = 0":           [20, -60, 80, -110,   0, 30],
    "nahe q3=0 (q3=5)": [20, -60,  5, -110, -90, 30],
    "q3 = 0":           [20, -60,  0, -110, -90, 30],
}

for name, q_deg in stellungen.items():
    J = robot.jacob0(np.deg2rad(q_deg))
    rang = np.linalg.matrix_rank(J)
    det = np.linalg.det(J)
    print(f"{name:18s}  Rang: {rang}   det: {det:9.2e}")
    if rang == 6:
        qd = np.rad2deg(np.linalg.solve(J, v_soll))
        print(f"{'':18s}  q̇ [°/s]: {qd}")
