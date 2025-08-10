import numpy as np
from scipy.integrate import quad

#### THIS CONFIGURATION GIVES AMP RATIO OF 4.54!
# Parameters (from Table 1)
l2 = 10    # mm (second half of lever arm)
l3 = 1    # mm (first half of lever arm)
R = 1.1     # mm (radius of inner flexure arm)
e = 6.35    # mm (thickness) - doesn't matter
m = 2.25    # mm (width of inner flexure arm)
E = 1.6e3   # MPa (Young's Modulus)
G = 0.5     # MPa (Shear Modulus)
d1 = 4.8    # microns (stroke of piezo actuator)

F = 69.0     # Arbitrary force (will cancel out)

# Define the integrand functions
def f1(theta):
    return (np.sin(theta)**2 * np.cos(theta)) / (m - 2 * R * np.cos(theta))**3

def f2(theta):
    return np.cos(theta) / (m - 2 * R * np.cos(theta))**3

def f3(theta):
    return np.cos(theta) / (m - 2 * R * np.cos(theta))

# Numerical integration over [-pi/2, pi/2]
I1, _ = quad(f1, -np.pi/2, np.pi/2, epsrel=1e-10, epsabs=1e-12)
I2, _ = quad(f2, -np.pi/2, np.pi/2, epsrel=1e-10, epsabs=1e-12)
I3, _ = quad(f3, -np.pi/2, np.pi/2, epsrel=1e-10, epsabs=1e-12)

# Angular deformation (gamma_z)
gamma_z = (12 * F * R * (l3 + R)) / (E * e) * I2

# Linear deformation (Delta_x)
Delta_x = (12 * F * R**3) / (E * e) * I1 + (12 * F * R**2 * (l3 + R)) / (E * e) * I2 + (F * R) / (G * e) * I3

# Coordinates and displacements
cx1 = (l2 + l3) * gamma_z
bx1 = l3 * gamma_z
cx = cx1 + Delta_x
bx = bx1 + Delta_x

# Amplification ratio
Ra = cx / bx
d2 = d1 * Ra

# Display result
print(f"Amplification Ratio Ra = {Ra:.3f}")
print(f"Output Displacement d2 = {d2:.3f} microns")
