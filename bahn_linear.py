import numpy as np
import roboticstoolbox as rtb

np.set_printoptions(suppress=True, precision=2)
robot = rtb.models.DH.UR5()

q0 = np.deg2rad([20, -60, 80, -110, -90, 30])
q1 = np.deg2rad([-40, -90, 60, -60, -90, 120])

T = 2.0
t = np.linspace(0, T, 100)

T0 = robot.fkine(q0)
T1 = robot.fkine(q1)

s = rtb.quintic(0, 1, t).s        # dasselbe Zeitgesetz wie jtraj
Ts = rtb.ctraj(T0, T1, s=s)       # 100 Posen exakt auf der Geraden

q_lin = np.zeros((len(t), 6))
q_vor = q0
for i, Ti in enumerate(Ts):
    sol = robot.ikine_LM(Ti, q0=q_vor, tol=1e-12)
    if not sol.success:
        print(f"IK gescheitert bei i = {i}")
        break
    q_lin[i] = sol.q
    q_vor = sol.q                 # Kette: Lösung wird Startwert des nächsten

P = robot.fkine(q_lin).t
p0, p1 = P[0], P[-1]
sehne = np.linalg.norm(p1 - p0)
u = (p1 - p0) / sehne
r = P - p0
d = np.linalg.norm(r - np.outer(r @ u, u), axis=1)
weg = np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1))

print(f"Luftlinie: {sehne*1000:.1f} mm   gefahren: {weg*1000:.1f} mm")
print(f"max. Abweichung von der Geraden: {d.max()*1000:.4f} mm")

sprung = np.rad2deg(np.max(np.abs(np.diff(q_lin, axis=0))))
print(f"größter Winkelsprung Punkt zu Punkt: {sprung:.2f}°")
print("q am Ziel [°]:", np.rad2deg(q_lin[-1]))
print("q1 erwartet [°]:", np.rad2deg(q1))