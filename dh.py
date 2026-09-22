import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

np.set_printoptions(suppress=True, precision=4)

robot = rtb.models.DH.UR5()
L = robot.links[1]          # Gelenk 2 (Python zählt ab 0)
print("d, a, alpha:", L.d, L.a, L.alpha)

q = np.deg2rad(30)
A_toolbox = L.A(q)

A_hand = SE3.Rz(q) * SE3.Tz(L.d) * SE3.Tx(L.a) * SE3.Rx(L.alpha)

print("Toolbox:\n", A_toolbox.A)
print("Von Hand:\n", A_hand.A)
print("gleich?", np.allclose(A_toolbox.A, A_hand.A))

L0 = robot.links[0]
A1 = L0.A(0)
print("\nGelenk 1, q=0:\n", A1.A)
print("neue x-Achse:", A1.A[:3, 0])
print("neue y-Achse:", A1.A[:3, 1])
print("neue z-Achse:", A1.A[:3, 2])

def kette(robot, q):
    T = SE3()                        # Start: Einheitsmatrix = Basis
    for link, qi in zip(robot.links, q):
        T = T * link.A(qi)           # von links nach rechts anhängen
    return T

q = np.zeros(6)
T_kette = kette(robot, q)
print("\nKette, q = 0:\n", T_kette.A)
print("gleich wie fkine?", np.allclose(T_kette.A, robot.fkine(q).A))