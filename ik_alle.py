import numpy as np
import roboticstoolbox as rtb

np.set_printoptions(suppress=True, precision=1)
robot = rtb.models.DH.UR5()

q_original = np.deg2rad([20, -60, 80, -110, -90, 30])
T_ziel = robot.fkine(q_original)

loesungen = []
for seed in range(50):
    sol = robot.ikine_LM(T_ziel, tol=1e-12, seed=seed)
    if not sol.success:
        continue
    q_deg = np.rad2deg(np.arctan2(np.sin(sol.q), np.cos(sol.q)))
    neu = all(not np.allclose(q_deg, alt, atol=0.1) for alt in loesungen)
    if neu:
        loesungen.append(q_deg)

print("Anzahl verschiedener Lösungen:", len(loesungen))
for i, q in enumerate(loesungen, 1):
    print(i, q)

q_aktuell = np.deg2rad([10, -50, 70, -100, -80, 20])
sol = robot.ikine_LM(T_ziel, q0=q_aktuell, tol=1e-12)
print("\nStart nahe am Original ->", np.rad2deg(sol.q))

q_aktuell = np.deg2rad([0, -30, -60, 100, -80, 20])
sol = robot.ikine_LM(T_ziel, q0=q_aktuell, tol=1e-12)
print("Start mit q3 = -60      ->", np.rad2deg(sol.q))