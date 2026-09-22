import numpy as np
import roboticstoolbox as rtb

np.set_printoptions(suppress=True, precision=3)
robot = rtb.models.DH.UR5()

q = np.deg2rad([20, -60, 80, -110, -90, 30])
J = robot.jacob0(q)
print("Form:", J.shape)
print("J (Zeilen: vx vy vz wx wy wz | Spalten: Gelenk 1..6):\n", J)

dq = np.deg2rad([0.1, -0.2, 0.1, 0.05, 0.1, -0.1])   # kleine Gelenkänderung

dp_echt = robot.fkine(q + dq).t - robot.fkine(q).t   # FK zweimal, Differenz
dp_jacobi = J[:3] @ dq                               # nur obere 3 Zeilen = v-Teil

print("\nΔp echt   [mm]:", dp_echt * 1000)
print("Δp Jacobi [mm]:", dp_jacobi * 1000)

v_soll = np.array([0.1, 0, 0, 0, 0, 0])      # 0.1 m/s in x, keine Drehung
qd = np.linalg.solve(J, v_soll)              # löst J · qd = v_soll

print("\nbenötigte Gelenkgeschwindigkeiten [°/s]:", np.rad2deg(qd))
print("Probe J · qd:", J @ qd)