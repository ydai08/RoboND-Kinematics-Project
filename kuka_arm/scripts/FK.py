# For Kuka KR210 robot -
# calculate individual transformation matrices about each joint
# generate generalized homogenous transform between base_link and gripper_link

import numpy as np
from sympy import symbols, cos, sin, atan2, pi, simplify, sqrt
from sympy.matrices import Matrix

# Define DH param symbols
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
# Joint angle symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #thetas

# Modified DH params
s = {alpha0: 0,     a0: 0,      d1: 0.75,
     alpha1: -pi/2, a1: 0.35,   d2: 0,      q2: q2-pi/2,
     alpha2: 0,     a2: 1.25,   d3: 0,
     alpha3: -pi/2, a3: -0.054, d4: 1.5,
     alpha4: pi/2,  a4: 0,      d5: 0,
     alpha5: -pi/2, a5: 0,      d6: 0,
     alpha6: 0,     a6: 0,      d7: 0.303,  q7: 0}

# Define generic placeholder symbols
alpha, a, d, q = symbols('alpha, a, d, q')

# Define Modified DH Transformation matrix
def dh_transform(jointparam):
    # Generic DH homogenous transform
    T = Matrix([[cos(q),            -sin(q),            0,              a],
                [sin(q)*cos(alpha), cos(q)*cos(alpha),  -sin(alpha),    -sin(alpha)*d],
                [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),     cos(alpha)*d],
                [0,                 0,                  0,              1]])

    # Sub in each joint parameter set
    Tjoint = T.subs(jointparam)
    Tjoint = simplify(Tjoint.subs(s))
    return Tjoint

# Create individual transformation matrices
T0_1 = dh_transform({alpha: alpha0, a: a0, d: d1, q: q1})
T1_2 = dh_transform({alpha: alpha1, a: a1, d: d2, q: q2})
T2_3 = dh_transform({alpha: alpha2, a: a2, d: d3, q: q3})
T3_4 = dh_transform({alpha: alpha3, a: a3, d: d4, q: q4})
T4_5 = dh_transform({alpha: alpha4, a: a4, d: d5, q: q5})
T5_6 = dh_transform({alpha: alpha5, a: a5, d: d6, q: q6})
T6_G = dh_transform({alpha: alpha6, a: a6, d: d7, q: q7})

# Calculate transform between fixed base and gripper
T0_G = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G)
#T0_6 = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6)
T3_6 = simplify(T3_4 * T4_5 * T5_6)

# Rotate gripper from DH to URDF coordinates
rot_z = Matrix([[cos(pi),   -sin(pi),   0,          0],
                [sin(pi),   cos(pi),    0,          0],
                [0,         0,          1,          0],
                [0,         0,          0,          1]])
rot_y = Matrix([[cos(-pi/2),    0,      sin(-pi/2),     0],
                [0,             1,      0,              0],
                [-sin(-pi/2),   0,      cos(-pi/2),     0],
                [0,             0,      0,              1]])
DH_URDF = simplify(rot_z * rot_y)
print(DH_URDF)

# Transform gripper to URDF coordinates
T0_G_URDF = simplify(T0_G * DH_URDF)
T3_6_URDF = simplify(T3_6 * DH_URDF)

# Calculate with values to compare against sim output
print(T3_6_URDF)
#print(T0_G_URDF.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
#print(T0_6_URDF.evalf(subs={q1: 0, q2: 0.396, q3: 0, q4: 0, q5: 0, q6: 0}))

'''
###########
# GRIPPER POSE FROM SIM
# Direct transform without using DH frames

# Define gripper pose symbols
x, y, z, roll, pitch, yaw = symbols('x, y, z, roll, pitch, yaw')

# Rotation matrices
r_z = Matrix([[cos(yaw),    -sin(yaw),  0],
              [sin(yaw),    cos(yaw),   0],
              [0,           0,          1]])
r_y = Matrix([[cos(pitch),  0,          sin(pitch)],
              [0,           1,          0],
              [-sin(pitch), 0,          cos(pitch)]])
r_x = Matrix([[1,       0,          0],
              [0,       cos(roll),  -sin(roll)],
              [0,       sin(roll),  cos(roll)]])

# 3X3 Z-Y-X rotation
r_tot = simplify(r_z * r_y * r_x)
# 3X1 translation
translate = Matrix([x, y, z])
# 1X4 homogenous transform last row
id_row = Matrix([0, 0, 0, 1]).T

# Join submatrices of homogenous transform
T_direct = r_tot.row_join(translate).col_join(id_row)
print(T_direct.evalf(subs={x:2.374, y:0, z:1.2, roll:0, pitch:.396, yaw:0}))
'''
