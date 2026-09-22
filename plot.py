import numpy as np
import matplotlib.pyplot as plt
import roboticstoolbox as rtb

robot = rtb.models.DH.UR5()
q0 = np.deg2rad([20, -60, 80, -110, -90, 30])
q1 = np.deg2rad([-40, -90, 60, -60, -90, 120])
T, n = 2.0, 100
t = np.linspace(0, T, n)

tg = rtb.jtraj(q0, q1, t)
q_j, qd_j = tg.q, np.rad2deg(tg.qd)

s = rtb.quintic(0, 1, t).s
Ts = rtb.ctraj(robot.fkine(q0), robot.fkine(q1), s=s)
q_l = np.zeros((n, 6))
qv = q0
for i, Ti in enumerate(Ts):
    sol = robot.ikine_LM(Ti, q0=qv, tol=1e-12)
    q_l[i] = sol.q
    qv = sol.q
qd_l = np.rad2deg(np.gradient(q_l, t, axis=0))

Pj = robot.fkine(q_j).t
Pl = robot.fkine(q_l).t

def abweichung(P):
    u = (P[-1] - P[0]) / np.linalg.norm(P[-1] - P[0])
    r = P - P[0]
    return np.linalg.norm(r - np.outer(r @ u, u), axis=1) * 1000

fig = plt.figure(figsize=(15, 5))

ax = fig.add_subplot(1, 3, 1, projection="3d")
ax.plot(*Pj.T, lw=2, label="MoveJ")
ax.plot(*Pl.T, lw=2, label="MoveL")
ax.scatter(*Pj[0], c="k", s=40)
ax.scatter(*Pj[-1], c="k", s=40)
ax.set_aspect("equal")
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
ax.set_title("Flange path")
ax.legend()

ax = fig.add_subplot(1, 3, 2)
ax.plot(t / T, abweichung(Pj), lw=2, label="MoveJ")
ax.plot(t / T, abweichung(Pl), lw=2, label="MoveL")
ax.set_xlabel("τ"); ax.set_ylabel("Deviation from straight line [mm]")
ax.set_title("Path deviation"); ax.grid(alpha=.3); ax.legend()

ax = fig.add_subplot(1, 3, 3)
for i in range(6):
    linie = ax.plot(t, qd_j[:, i], lw=1.5, label=f"Joint {i+1}")
    ax.plot(t, qd_l[:, i], lw=1.5, ls="--", color=linie[0].get_color())
ax.set_xlabel("t [s]"); ax.set_ylabel("q̇ [°/s]")
ax.set_title("Joint velocities (solid MoveJ, dashed MoveL)")
ax.grid(alpha=.3); ax.legend(fontsize=8, ncol=2)

fig.tight_layout()
fig.savefig("movej_vs_movel.png", dpi=150)
plt.show()
