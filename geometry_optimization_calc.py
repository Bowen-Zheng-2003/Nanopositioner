import numpy as np
from scipy.integrate import quad

E = 1.6e3   # MPa (Young's Modulus)
G = 0.5     # MPa (Shear Modulus)
d1 = 4.8    # microns (stroke of piezo actuator)
F = 69.0    # Arbitrary force

best_Ra = -np.inf
best_params = {}

for l2 in range(1, 11):
    for l3 in range(1, 11):
        for R in np.arange(0, 1.05, 0.05):
            for e in range(1, 11):
                for m in np.arange(0, 4.5, 0.5):

                    # Critical check: prevent division by zero, near-zero, and negative denominator cases
                    if m <= 2 * R:
                        continue

                    def safe_denominator(theta):
                        denom = m - 2 * R * np.cos(theta)
                        # Skip near-singular evaluations
                        if abs(denom) < 1e-8:
                            return None
                        return denom

                    def f1(theta):
                        denom = safe_denominator(theta)
                        if denom is None:
                            return 0
                        return (np.sin(theta)**2 * np.cos(theta)) / denom**3

                    def f2(theta):
                        denom = safe_denominator(theta)
                        if denom is None:
                            return 0
                        return np.cos(theta) / denom**3

                    def f3(theta):
                        denom = safe_denominator(theta)
                        if denom is None:
                            return 0
                        return np.cos(theta) / denom

                    try:
                        I1, _ = quad(f1, -np.pi/2, np.pi/2, epsrel=1e-6, epsabs=1e-8)
                        I2, _ = quad(f2, -np.pi/2, np.pi/2, epsrel=1e-6, epsabs=1e-8)
                        I3, _ = quad(f3, -np.pi/2, np.pi/2, epsrel=1e-6, epsabs=1e-8)
                    except Exception:
                        continue

                    gamma_z = (12 * F * R * (l3 + R)) / (E * e) * I2
                    Delta_x = (12 * F * R**3) / (E * e) * I1 + (12 * F * R**2 * (l3 + R)) / (E * e) * I2 + (F * R) / (G * e) * I3

                    cx1 = (l2 + l3) * gamma_z
                    bx1 = l3 * gamma_z
                    cx = cx1 + Delta_x
                    bx = bx1 + Delta_x

                    if bx == 0:
                        continue

                    Ra = cx / bx
                    if Ra > best_Ra:
                        best_Ra = Ra
                        best_params = {'l2': l2, 'l3': l3, 'R': R, 'e': e, 'm': m}

print(f"Highest Amplification Ratio Ra = {best_Ra:.3f}")
print("Parameter values for highest amplification:")
print(best_params)
