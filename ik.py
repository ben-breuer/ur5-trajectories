import numpy as np
import roboticstoolbox as rtb

np.set_printoptions(suppress=True, precision=2)
robot = rtb.models.DH.UR5()

# 1. Zielpose erzeugen: aus bekannten Winkeln per FK
q_original = np.deg2rad([20, -60, 80, -110, -90, 30])
T_ziel = robot.fkine(q_original)
print("Zielposition:", T_ziel.t)

# 2. IK: nur mit der Pose, ohne die Winkel zu kennen
sol = robot.ikine_LM(T_ziel)
print(sol)
print("q original [°]:", np.rad2deg(q_original))
print("q gefunden [°]:", np.rad2deg(sol.q))

# 3. Probe: FK der gefundenen Lösung, gleiche Pose?
T_check = robot.fkine(sol.q)
fehler_mm = np.linalg.norm(T_check.t - T_ziel.t) * 1000
print(f"Positionsfehler: {fehler_mm:.3f} mm")