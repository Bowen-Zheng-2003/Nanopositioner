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
h_resin = 0.003  # m (Height of the plate)
L_resin = 0.02   # m (Length of the plate)
k_b_resin = round(bending_stiffness(E_resin, b_resin, h_resin, L_resin), 1)

print("Flexural stiffness of steel flexure is " + str(k_b_steel) + " N/m")
print("Flexural stiffness of resin flexure is " + str(k_b_resin) + " N/m")

## 2nd part of flexure's stiffness
E_steel1 = 200e9   # Pa (Young's modulus)
b_steel1 = 0.00635 # m (Thickness of the plate)
h_steel1 = 0.004   # m (Height of the plate)
L_steel1 = 0.0077  # m (Length of the plate)
k_b_steel1 = round(bending_stiffness(E_steel1, b_steel1, h_steel1, L_steel1), 1)

E_resin1 = 1.6e9   # Pa (Young's modulus)
b_resin1 = 0.00635 # m (Thickness of the plate)
h_resin1 = 0.0092  # m (Height of the plate)
L_resin1 = 0.0077    # m (Length of the plate)
k_b_resin1 = round(bending_stiffness(E_resin1, b_resin1, h_resin1, L_resin1), 1)

print("Flexural stiffness of steel flexure's 2nd part is " + str(k_b_steel1) + " N/m")
print("Flexural stiffness of resin flexure's 2nd part is " + str(k_b_resin1) + " N/m")
