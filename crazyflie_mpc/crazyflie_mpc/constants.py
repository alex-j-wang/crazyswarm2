import numpy as np

MASS = 0.03              # kg
IXX = 1.43e-5            # kg*m^2
IYY = 1.43e-5            # kg*m^2
IZZ = 2.89e-5            # kg*m^2
ARM_LENGTH = 0.046       # m
ROTOR_SPEED_MIN = 0      # rad/s
ROTOR_SPEED_MAX = 2500   # rad/s
K_THRUST = 2.3e-08       # N/(rad/s)**2
K_DRAG = 7.8e-11         # Nm/(rad/s)**2

INERTIA = np.diag(np.array([IXX, IYY, IZZ]))    # kg*m^2
g = 9.81                                        # m/s^2
WEIGHT = MASS * g                               # N

GEO_ROLLPITCH_KP = 10
GEO_ROLLPITCH_KD = 2 * 1.0 * np.sqrt(GEO_ROLLPITCH_KP)
GEO_YAW_KP = 50
GEO_YAW_KD = 2 * 1.15 * np.sqrt(GEO_YAW_KP)
ATT_KP_MAT = np.diag(np.array([GEO_ROLLPITCH_KP, GEO_ROLLPITCH_KP, GEO_YAW_KP]))
ATT_KD_MAT = np.diag(np.array([GEO_ROLLPITCH_KD, GEO_ROLLPITCH_KD, GEO_YAW_KD]))
k = K_DRAG / K_THRUST
CTRL_FORCES_MAP = np.array([
    [1, 1, 1, 1],
    [0, ARM_LENGTH, 0, -ARM_LENGTH],
    [-ARM_LENGTH, 0, ARM_LENGTH, 0],
    [k, -k, k, -k]
])
FORCES_CTRL_MAP = np.linalg.inv(CTRL_FORCES_MAP)
TRIM_MOTOR_SPD = 1790.0
