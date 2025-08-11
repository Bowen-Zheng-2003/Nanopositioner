def bending_stiffness(E, b, h, L):
    return (3*E*b*h**3)/(12*L**3)

## 1st part of flexure's stiffness
E_steel = 200e9   # Pa (Young's modulus)
b_steel = 0.00635 # m (Thickness of the plate)
h_steel = 0.0006  # m (Height of the plate)
L_steel = 0.02    # m (Length of the plate)
k_b_steel = round(bending_stiffness(E_steel, b_steel, h_steel, L_steel), 1)

E_resin = 1.6e9   # Pa (Young's modulus)
b_resin = 0.00635 # m (Thickness of the plate)
h_resin = 0.003   # m (Height of the plate)
L_resin = 0.02    # m (Length of the plate)
k_b_resin = round(bending_stiffness(E_resin, b_resin, h_resin, L_resin), 1)

print("Flexural stiffness of steel flexure is " + str(k_b_steel) + " N/m")
print("Flexural stiffness of resin flexure is " + str(k_b_resin) + " N/m")

## Contact stiffness
import math
def contact_stiffness(L, v, E, F, R):
    return (L/((2*(1-v*v)/E)*math.log((2*(1-v*v)/E)*L**3/(F*R))))

force = 1       # N
radius_steel = 0.00075 # m
L_s = 0.00635    # m (length of contact)
v_s = 0.3        # Poisson's ratio
E_s = 200e9      # Pa (Young's modulus)
k_c_steel = round(abs(contact_stiffness(L_s, v_s, E_s, force, radius_steel)), 1)

radius_resin = 0.0002 # m
L_r = 0.01    # m (length of contact)
v_r = 0.3        # Poisson's ratio
E_r = 1.6e9      # Pa (Young's modulus)
k_c_resin = round(abs(contact_stiffness(L_r, v_r, E_r, 10, radius_resin)), 1)

print("Contact stiffness of steel on steel is " + str(k_c_steel) + " N/m")
print("Contact stiffness of resin on resin is " + str(k_c_resin) + " N/m")





