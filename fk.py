import numpy as np
import roboticstoolbox as rtb

np.set_printoptions(suppress=True, precision=4)
robot = rtb.models.DH.UR5()

konfigs = {
    "q = 0":   [0, 0, 0, 0, 0, 0],
    "(A) Kerze": [0, -90, 0, -90, 0, 0],
    "(B) Ellbogen 90°": [0, -90, 90, 0, 0, 0],
    "(C)": [0, -30, -60, 90, 0, 0],
    "(D)": [0, -45, 45, 0, 0, 0],
}

for name, q_deg in konfigs.items():
    q = np.deg2rad(q_deg)
    T = robot.fkine(q)
    print(f"{name:18s}  Position: {T.t}   RPY [°]: {T.rpy(unit='deg')}")

q_D = np.deg2rad(konfigs["(D)"])
import matplotlib.pyplot as plt

env = robot.plot(q_D, block=False)
env.ax.view_init(elev=0, azim=-90)
env.hold()