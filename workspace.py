import numpy as np
import roboticstoolbox as rtb
import matplotlib.pyplot as plt

robot = rtb.models.DH.UR5()

rng = np.random.default_rng(0)
N = 20000
Q = rng.uniform(-np.pi, np.pi, size=(N, 6))    # N zufällige Stellungen

P = robot.fkine(Q).t                           # (N, 3) Flanschpositionen
w = robot.manipulability(Q)                    # (N,) Manipulierbarkeit
r = np.hypot(P[:, 0], P[:, 1])                 # Abstand von der Basisachse

fig, ax = plt.subplots(1, 2, figsize=(11, 5))

ax[0].scatter(P[:, 0], P[:, 1], c=w, s=1, cmap="viridis")
ax[0].set_aspect("equal")
ax[0].set_title("Draufsicht (x–y)")
ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]")

sc = ax[1].scatter(r, P[:, 2], c=w, s=1, cmap="viridis")
ax[1].set_aspect("equal")
ax[1].set_title("Querschnitt (r–z)")
ax[1].set_xlabel("r [m]"); ax[1].set_ylabel("z [m]")
fig.colorbar(sc, ax=ax[1], label="Manipulierbarkeit w")

plt.tight_layout()
plt.show()