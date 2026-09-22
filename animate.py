import numpy as np
import roboticstoolbox as rtb

robot = rtb.models.DH.UR5()
q0 = np.deg2rad([20, -60, 80, -110, -90, 30])
q1 = np.deg2rad([-40, -90, 60, -60, -90, 120])
T, n = 2.0, 100
t = np.linspace(0, T, n)

q_j = rtb.jtraj(q0, q1, t).q

s = rtb.quintic(0, 1, t).s
Ts = rtb.ctraj(robot.fkine(q0), robot.fkine(q1), s=s)
q_l = np.zeros((n, 6))
qv = q0
for i, Ti in enumerate(Ts):
    sol = robot.ikine_LM(Ti, q0=qv, tol=1e-12)
    q_l[i] = sol.q
    qv = sol.q

zyklus = np.vstack([q_j, q_l[::-1]])        # hin MoveJ, zurück MoveL

Pj = robot.fkine(q_j).t
Pl = robot.fkine(q_l).t

env = robot.plot(q0, backend="pyplot", block=False,
                 limits=[-0.9, 0.9, -0.9, 0.9, -0.2, 1.1])
env.ax.plot(*Pj.T, lw=1.5, color="tab:blue", label="MoveJ")
env.ax.plot(*Pl.T, lw=1.5, color="tab:orange", label="MoveL")
env.ax.view_init(elev=25, azim=-60)
env.ax.legend()

for q in zyklus:
    robot.q = q
    env.step(T / n)

env.hold()