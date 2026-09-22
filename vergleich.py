import time
import numpy as np
import roboticstoolbox as rtb

robot = rtb.models.DH.UR5()
q0 = np.deg2rad([20, -60, 80, -110, -90, 30])
q1 = np.deg2rad([-40, -90, 60, -60, -90, 120])
T = 2.0
t = np.linspace(0, T, 100)

tic = time.perf_counter()
tg = rtb.jtraj(q0, q1, t)
zeit_j = time.perf_counter() - tic
q_j, qd_j = tg.q, tg.qd

tic = time.perf_counter()
s = rtb.quintic(0, 1, t).s
Ts = rtb.ctraj(robot.fkine(q0), robot.fkine(q1), s=s)
q_l = np.zeros((len(t), 6))
q_vor = q0
for i, Ti in enumerate(Ts):
    sol = robot.ikine_LM(Ti, q0=q_vor, tol=1e-12)
    q_l[i] = sol.q
    q_vor = sol.q
zeit_l = time.perf_counter() - tic
qd_l = np.gradient(q_l, t, axis=0)

def bahnlaenge(q):
    P = robot.fkine(q).t
    return np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1)) * 1000

print(f"Bahnlänge    MoveJ: {bahnlaenge(q_j):8.1f} mm    MoveL: {bahnlaenge(q_l):8.1f} mm")
print(f"Rechenzeit   MoveJ: {zeit_j*1000:8.2f} ms    MoveL: {zeit_l*1000:8.2f} ms")
print(f"             Faktor {zeit_l/zeit_j:.0f}x\n")

print("Gelenk    MoveJ [°/s]    MoveL [°/s]")
for i in range(6):
    a = np.rad2deg(np.abs(qd_j[:, i]).max())
    b = np.rad2deg(np.abs(qd_l[:, i]).max())
    print(f"   {i+1}         {a:8.1f}       {b:8.1f}")

zeile, spalte = np.unravel_index(np.abs(qd_l).argmax(), qd_l.shape)
print(f"\nMoveJ-Spitze: {np.rad2deg(np.abs(qd_j).max()):.1f} °/s")
print(f"MoveL-Spitze: {np.rad2deg(np.abs(qd_l).max()):.1f} °/s "
      f"an Gelenk {spalte+1} bei t = {t[zeile]:.2f} s (τ = {t[zeile]/T:.2f})")

def gelenkweg(q):
    return np.rad2deg(np.sum(np.abs(np.diff(q, axis=0)), axis=0))

print("\nabgefahrener Gelenkweg [°]")
print("  MoveJ:", np.round(gelenkweg(q_j), 1), " Summe:", round(gelenkweg(q_j).sum(), 1))
print("  MoveL:", np.round(gelenkweg(q_l), 1), " Summe:", round(gelenkweg(q_l).sum(), 1))