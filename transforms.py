import numpy as np

def rot_z(theta):
    """Rotationsmatrix um die Z-Achse (3x3)."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1],
    ])

def homogeneous(R, t):
    """Baut eine 4x4 homogene Transformationsmatrix aus Rotation R und Verschiebung t."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

# Punkt im lokalen Koordinatensystem
p_local = np.array([1.0, 0.0, 0.0, 1.0])  # letzte 1 = homogene Koordinate

# Transformation: 90 Grad um Z drehen, dann um (2, 0, 0) verschieben
R = rot_z(np.radians(90))
t = np.array([2.0, 0.0, 0.0])
T = homogeneous(R, t)

p_welt = T @ p_local

print("Rotationsmatrix R:\n", R)
print("Transformation T:\n", T)
print("Punkt lokal:", p_local)
print("Punkt im Weltkoordinatensystem:", p_welt)

from spatialmath import SE3

T_sm = SE3.Tx(2) * SE3.Rz(90, 'deg')
print("spatialmath T:\n", T_sm)
print("gleich wie mein T?", np.allclose(T_sm.A, T))

T_falsch = SE3.Rz(90, 'deg') * SE3.Tx(2)
print("andere Reihenfolge, Translation:", T_falsch.t)

print("R^T @ R:\n", np.round(R.T @ R, 6))
print("det(R):", np.linalg.det(R))

T_inv = np.eye(4)
T_inv[:3, :3] = R.T
T_inv[:3, 3] = -R.T @ T[:3, 3]

p_welt = T @ p_local
print("zurück nach lokal:", T_inv @ p_welt)
print("gleich wie np.linalg.inv?", np.allclose(T_inv, np.linalg.inv(T)))
print("gleich wie spatialmath inv?", np.allclose(T_inv, T_sm.inv().A))