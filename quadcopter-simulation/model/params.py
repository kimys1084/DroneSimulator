import numpy as np


control_frequency = 200 #hz
dt = 1.0 / control_frequency

g = 9.81 # m/s/s



#controller 1 drone length 0.1
'''
mass = 0.3 # kg

I = np.array([(0.00025, 0, 2.55e-6),
              (0, 0.000232, 0),
              (2.55e-6, 0, 0.0003738)]);

mass = 2.723
I = np.array([(0.05442735, 0, 0),
              (0, 0.05741943, 0),
              (0, 0, 0.08800943)]);
'''
#controller 3 ascending technology hummingbird quadrotor
mass = 0.5 # kg
I = np.array([(0.0025, 0, 0),
              (0, 0.00232, 0),
              (0, 0, 0.003738)]);
invI = np.linalg.inv(I)
arm_length = 0.2 # meter
height = 0.05
maxF = 2.0 * mass * g
minF = 0.0#maxF / 20.0
L = arm_length
H = height
km = 1.5e-9
kf = 6.11e-8
r = km / kf

#  [ F  ]         [ F1 ] 
#  | M1 |  = A *  | F2 | 
#  | M2 |         | F3 |
#  [ M3 ]         [ F4 ]

#  F = F1 + F2 + F3 + F4
# M1 = L*F2 - L*F4
# M2 = -L*F1 + L*F3
# M3 = r*F1 -r*F2 + r*F3 -r*F4
A = np.array([[ 1,  1,  1,  1],
              [ 0,  L,  0, -L],
              [-L,  0,  L,  0],
              [ r, -r,  r, -r]])

invA = np.linalg.inv(A)

#quadrotor body design
body_frame = np.array([(L, 0, 0, 1),
                       (0, L, 0, 1),
                       (-L, 0, 0, 1),
                       (0, -L, 0, 1),
                       (0, 0, 0, 1),
                       (0, 0, H, 1)])

