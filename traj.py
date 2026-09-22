import numpy as np
import roboticstoolbox as rtb

np.set_printoptions(suppress=True, precision=2)
robot = rtb.models.DH.UR5()

q0 = np.deg2rad([20, -60, 80, -110, -90, 30])
q1 = np.deg2rad([-40, -90, 60, -60, -90, 120])

T = 2.0                       # Fahrzeit in Sekunden
t = np.linspace(0, T, 100)    # 100 Stützstellen
tg = rtb.jtraj(q0, q1, t)

print("Form q:", tg.q.shape, " qd:", tg.qd.shape)
print("Start [°]:", np.rad2deg(tg.q[0]))
print("Ziel  [°]:", np.rad2deg(tg.q[-1]))
print("q̇ Start [°/s]:", np.rad2deg(tg.qd[0]))
print("q̇ Ziel  [°/s]:", np.rad2deg(tg.qd[-1]))

qd_max = np.rad2deg(np.max(np.abs(tg.qd), axis=0))
vorhersage = np.rad2deg(1.875 * np.abs(q1 - q0) / T)
print("\nmax |q̇| gemessen   [°/s]:", qd_max)
print("max |q̇| Vorhersage [°/s]:", vorhersage)

tg.plot(block=True)