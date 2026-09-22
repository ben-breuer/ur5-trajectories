import numpy as np
import roboticstoolbox as rtb

np.set_printoptions(suppress=True, precision=4)
robot = rtb.models.DH.UR5()

q0 = np.deg2rad([20, -60, 80, -110, -90, 30])
q1 = np.deg2rad([-40, -90, 60, -60, -90, 120])

T = 2.0
t = np.linspace(0, T, 100)
tg = rtb.jtraj(q0, q1, t)

P = robot.fkine(tg.q).t          # (100, 3) — FK auf die ganze Bahn
p0, p1 = P[0], P[-1]

sehne = np.linalg.norm(p1 - p0)
weg = np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1))
print(f"Luftlinie Start→Ziel: {sehne*1000:8.1f} mm")
print(f"tatsächlich gefahren: {weg*1000:8.1f} mm")
print(f"Umweg:                {(weg-sehne)*1000:8.1f} mm  ({100*(weg/sehne-1):.1f} %)")

u = (p1 - p0) / sehne            # Einheitsvektor entlang der Geraden
r = P - p0                       # alle Punkte relativ zum Start
laengs = r @ u                   # Projektion auf die Gerade, (100,)
quer = r - np.outer(laengs, u)   # Restanteil, senkrecht dazu
d = np.linalg.norm(quer, axis=1)

i = np.argmax(d)
print(f"\nmax. Abweichung: {d[i]*1000:.1f} mm bei t = {t[i]:.2f} s (τ = {t[i]/T:.2f})")
print("Abweichung [mm] alle 10 Schritte:", np.round(d[::10]*1000, 1))