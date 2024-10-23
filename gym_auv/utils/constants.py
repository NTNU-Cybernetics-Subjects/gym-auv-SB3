import numpy as np



# ---- THE SIM MODEL WITH OTTER PARAM CALCULATIONS USING BB VALUES --- #
# ---- WORKING ---- #
# ---- With Otter params ---- #

# NB!!: Thruster and lever arms in _init_py must be changed when using this:
# 'thrusters_max_forward': 55.21
# 'thrusters_max_backwards': 27.56
# 'lever_arm_left_propeller': -0.285, 
# 'lever_arm_right_propeller': 0.285, 
# and set them back to 1 and 0.285 when going back to SIM model

# Main data
g   = 9.81         # acceleration of gravity (m/s^2)
rho = 1025;         # density of water
L = 1.195;            # length (m)
B = 0.910;           # beam/width (m)
m = 15.0;           # mass (kg)
x_g = 0.2            # CG for hull in x-axis only (m)
R66 = 0.25 * L      # radii of gyrations (m) 0.25 comes from "Principles of Yacht Design" by Larsson, Eliasson, and Orych or "Principles of Naval Architecture" (Society of Naval Architects and Marine Engineers, edited by Lewis
T_sway = 1;         # time constant in sway (s)
T_yaw = 1;          # time constant in yaw (s)
Umax = 6 * 0.5144   # 6 knots maximum forward speed (m/s)

# Data for one pontoon
B_pont = 0.170   # beam/width of one pontoon (m)
y_pont = 0.327  # distance from centerline to waterline area center (m)
Cw_pont = 0.75  # waterline area coefficient (-) typically between 0.65 and 0.85 - "Principles of Naval Architecture" by Lewis, E.V.
Cb_pont = 0.4   # block coefficient, computed from m = 55 kg typically between 0.3 to 0.60 -  "Basic Ship Theory" series by Rawson and Tupper

# Inertia dyadic, volume displacement and draft
nabla = m/rho   # volume
T = nabla / (2 * Cb_pont * B_pont * L)  # draft
Ig_CG = m * R66 # only hull in the CG
I_z = Ig_CG - m*x_g ** 2  # hull in the CO, only in z as we use #DOF
# NB: have placed CG on the x-axis so it will only have arm in the z and y rotations, 
# and we will only have rotations in z as we have 3DOF


# Hydrodynamic added mass (best practice)
X_udot = - (2.7 * rho * nabla ** (5/3) ) / ((L ** 2) * m)
Y_vdot = - 1.5 * m
N_rdot = - 1.7 * I_z

M =  np.array([[m - X_udot, 0, 0],
    [0, m - Y_vdot, m*x_g],
    [0, m*x_g, I_z - N_rdot]]
)   
M_inv = np.linalg.inv(M)

# Linear damping terms
X_u = - 24.4 * g /Umax
Y_v = - M[1,1] / T_sway
N_r = - M[2,2] / T_yaw
Y_r = -0.1  # Just a guess, making the sim more realistic when driving
N_v = -0.1  # Just a guess, making the sim more realistic when driving

M =  np.array([[m - X_udot, 0, 0],
    [0, m - Y_vdot, m*x_g],
    [0, m*x_g, I_z - N_rdot]]
)   
M_inv = np.linalg.inv(M)


def N(nu):
    u = nu[0]
    v = nu[1]
    r = nu[2]
    N = np.array([
        [-X_u, 0, 0],
        [0, -Y_v, m*u - Y_r],
        [0, - N_v, m*x_g*u-N_r]
    ])  
    return N





# # ---- ORIGINAL SIM MODEL ------ #
# m = 23.8
# x_g = 0.046
# I_z = 1.760
# X_udot = -1.697
# Y_vdot = -22.5
# Y_rdot = 0.0
# N_rdot = -6.58
# N_vdot = 0.0
# X_u = -2.0
# Y_v = -7.0
# Y_r = -0.1
# N_v = -0.1
# N_r = -0.5
# X_uu = -1.32742
# Y_vv = -80
# Y_rr = 0.3
# N_vv = -1.5
# N_rr = -9.1
# Y_uvb = -0.5*1000*np.pi*1.24*(0.15/2)**2
# Y_uvf = -1000*3*0.0064
# Y_urf = -0.4*Y_uvf
# N_uvb = (-0.65*1.08 + 0.4)*Y_uvb
# N_uvf = -0.4*Y_uvf
# N_urf = -0.4*Y_urf
# Y_uudr = 19.2
# N_uudr = -0.4*Y_uudr

# MAX_SPEED = 2

# M =  np.array([[m - X_udot, 0, 0],
#     [0, m - Y_vdot, m*x_g - Y_rdot],
#     [0, m*x_g - N_vdot, I_z - N_rdot]]
# )   
# M_inv = np.linalg.inv(M)
# def D(nu):
#     r = nu[2]
#     D =  np.array([
#     [2.0, 0, 0],
#     [0, 7.0, 0],
#     [0, 0, 1.422*r]
#     ])
#     return D

# def C(nu):
#     u = nu[0]
#     v = nu[1]
#     r = nu[2]
#     C = np.array([
#         [0, 0, Y_vdot*v + m*x_g*r],
#         [0, 0, -X_udot*u],
#         [-Y_vdot*v - m*x_g*r, -25.8*u, 0]
#     ])  
#     return C

# def N(nu):
#     u = nu[0]
#     v = nu[1]
#     r = nu[2]
#     N = np.array([
#         [-X_u, 0, 0],
#         [0, -Y_v, m*u - Y_r],
#         [0, -N_v, m*x_g*u-N_r]
#     ])  
#     return N



# ---- LINEAR FOSSEN MODEL ---- #
# ---- with SysIded params ---- #
# m = 15
# x_g = 0.1
# I_z = 3.87
# g = 9.81
# U_max = 3
# U_cruise = U_max/5
# T_sway = 1
# T_yaw = 1
# X_udot = 8.2116
# Y_vdot = 14.846
# Y_rdot = -10.959
# N_rdot = -1295.5

# MAX_SPEED = 2

# M =  np.array([[m - X_udot, 0, 0],
#     [0, m - Y_vdot, m*x_g - Y_rdot],
#     [0, m*x_g - Y_rdot, I_z - N_rdot]]
# )   
# M_inv = np.linalg.inv(M)

# X_u = - 24.4 * g/U_max
# Y_v = - M[1,1]/T_sway
# N_r = - M[2,2]/T_sway
# Y_r = -0.1
# N_v = -0.1

# # Setting both C_rb, C_a and D in the same N
# N = np.array([
#     [-X_u, 0, 0],
#     [0, -Y_v, m*U_cruise - X_udot*U_cruise],
#     [0, (X_udot - Y_vdot)*U_cruise, (m*x_g - Y_rdot)*U_cruise-N_r]
# ])

# # ---- THE SIM MODEL WITH OTTER PARAMS --- #
# # ---- WORKING WITH ODE45 ----
# # ---- With Otter params ---- #

# # NB!!: Thruster and lever arms in _init_py must be changed when using this:
# # 'thrusters_max_forward': 119.6
# # 'thrusters_max_backwards': 66.69
# # 'lever_arm_left_propeller': -0.395, 
# # 'lever_arm_right_propeller': 0.395, 
# # and set them back to 1 and 0.285 when going back to SIM model

# # Main data
# g   = 9.81         # acceleration of gravity (m/s^2)
# rho = 1025;         # density of water
# L = 2.0;            # length (m)
# B = 1.08;           # beam (m)
# m = 55.0;           # mass (kg)
# x_g = 0.2            # CG for hull in x-axis only (m)
# R66 = 0.25 * L      # radii of gyrations (m)
# T_sway = 1;         # time constant in sway (s)
# T_yaw = 1;          # time constant in yaw (s)
# Umax = 6 * 0.5144   # 6 knots maximum forward speed (m/s)

# # Data for one pontoon
# B_pont = 0.25   # beam of one pontoon (m)
# y_pont = 0.395  # distance from centerline to waterline area center (m)
# Cw_pont = 0.75  # waterline area coefficient (-)
# Cb_pont = 0.4   # block coefficient, computed from m = 55 kg

# # Inertia dyadic, volume displacement and draft
# nabla = m/rho   # volume
# T = nabla / (2 * Cb_pont * B_pont * L)  # draft
# Ig_CG = m * R66 # only hull in the CG
# I_z = Ig_CG - m*x_g ** 2  # hull in the CO, only in z as we use #DOF
# # NB: have placed CG on the x-axis so it will only have arm in the z and y rotations, 
# # and we will only have rotations in z as we have 3DOF

# # Calculated max thrusts from Otter data including lever arms
# l1 = -y_pont    # lever arm left propeller
# l2 = y_pont     # lever arm right propeller
# T_max_forward = 119.6   # Max thrust forward
# T_max_backwards = 66.69 # Max thrust backwards

# # Hydrodynamic added mass (best practice)
# X_udot = - (2.7 * rho * nabla ** (5/3) ) / ((L ** 2) * m)
# Y_vdot = - 1.5 * m
# N_rdot = - 1.7 * I_z

# M =  np.array([[m - X_udot, 0, 0],
#     [0, m - Y_vdot, m*x_g],
#     [0, m*x_g, I_z - N_rdot]]
# )   
# M_inv = np.linalg.inv(M)

# # Linear damping terms
# X_u = - 24.4 * g /Umax
# Y_v = - M[1,1] / T_sway
# N_r = - M[2,2] / T_yaw

# M =  np.array([[m - X_udot, 0, 0],
#     [0, m - Y_vdot, m*x_g],
#     [0, m*x_g, I_z - N_rdot]]
# )   
# M_inv = np.linalg.inv(M)

# def N(nu):
#     u = nu[0]
#     v = nu[1]
#     r = nu[2]
#     N = np.array([
#         [-X_u, 0, 0],
#         [0, -Y_v, m*u],
#         [0, 0, m*x_g*u-N_r]
#     ])  
#     return N