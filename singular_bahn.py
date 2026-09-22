import numpy as np
import roboticstoolbox as rtb

robot = rtb.models.DH.UR5()

qa = np.deg2rad([20, -60, 80, -110, -25, 30])
qb = np.deg2rad([20, -60, 80, -110,  25, 30])   # nur q5 dreht, durch die Null

T = 2.0
n = 100
t = np.linspace(0, T, n)

tgj = rtb.jtraj(qa, qb, t)
print("MoveJ  max |q̇| [°/s]:", np.round(np.rad2deg(np.abs(tgj.qd).max(axis=0)), 1))

s = rtb.quintic(0, 1, t).s
Ts = rtb.ctraj(robot.fkine(qa), robot.fkine(qb), s=s)

q = np.zeros((n, 6))
qv = qa
fehler = 0
for i, Ti in enumerate(Ts):
    sol = robot.ikine_LM(Ti, q0=qv, tol=1e-12)
    if not sol.success:
        fehler += 1
    q[i] = sol.q
    qv = sol.q

qd = np.rad2deg(np.gradient(q, t, axis=0))
w = np.array([robot.manipulability(qq) for qq in q])

print("MoveL  max |q̇| [°/s]:", np.round(np.abs(qd).max(axis=0), 1))
print(f"nicht konvergierte IK-Aufrufe: {fehler}")
sehne = np.linalg.norm(robot.fkine(qb).t - robot.fkine(qa).t) * 1000
print(f"Luftlinie Start→Ziel: {sehne:.1f} mm")

print("\n   τ        w        q5 [°]      q̇4 [°/s]     q̇6 [°/s]")
for i in range(0, n, 10):
    print(f"{t[i]/T:6.2f}  {w[i]:9.5f}  {np.rad2deg(q[i,4]):8.2f}  {qd[i,3]:12.1f}  {qd[i,5]:12.1f}")

k = w.argmin()
print(f"\nw minimal: {w[k]:.5f} bei τ = {t[k]/T:.3f}, q5 = {np.rad2deg(q[k,4]):.2f}°")
print(f"Spitze: {np.abs(qd).max():.0f} °/s = {np.abs(qd).max()/180:.1f}-fache Gelenkgrenze")
print(f"q4 Bereich [°]: {np.rad2deg(q[:,3]).min():.1f} … {np.rad2deg(q[:,3]).max():.1f}")
print(f"q6 Bereich [°]: {np.rad2deg(q[:,5]).min():.1f} … {np.rad2deg(q[:,5]).max():.1f}")