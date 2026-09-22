import numpy as np
import roboticstoolbox as rtb

q0 = np.deg2rad([20, -60, 80, -110, -90, 30])
q1 = np.deg2rad([-40, -90, 60, -60, -90, 120])
T = 2.0
dq = q1 - q0

for n in [100, 400, 1600]:
    t = np.linspace(0, T, n)

    quint = rtb.quintic(0, 1, t)
    trap = rtb.trapezoidal(0, 1, t)

    q_q = q0 + np.outer(quint.s, dq)
    q_t = q0 + np.outer(trap.s, dq)

    def kennwerte(q, t):
        qd = np.gradient(q, t, axis=0)
        qdd = np.gradient(qd, t, axis=0)
        j = np.gradient(qdd, t, axis=0)
        return (np.rad2deg(np.abs(qd).max()),
                np.rad2deg(np.abs(qdd).max()),
                np.rad2deg(np.abs(j).max()))

    vq, aq, jq = kennwerte(q_q, t)
    vt, at, jt = kennwerte(q_t, t)

    print(f"n = {n}")
    print(f"  quintisch    v {vq:7.1f} °/s   a {aq:7.1f} °/s²   j {jq:10.1f} °/s³")
    print(f"  trapezförmig v {vt:7.1f} °/s   a {at:7.1f} °/s²   j {jt:10.1f} °/s³")