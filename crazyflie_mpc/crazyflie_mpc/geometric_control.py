import constants

import numpy as np
from scipy.spatial.transform import Rotation
from tf_transformations import euler_from_matrix

class GeometriControl(object):
    def __init__(self):
        self.pos_kp = 2.5
        self.pos_kd = 2 * 1.0 * np.sqrt(self.pos_kp)
        self.pos_ki = 0.25
        self.posz_kp = 4.0
        self.posz_kd = 2.4
        self.posz_ki = 1.5
        self.pos_kp_mat = np.diag(np.array([self.pos_kp, self.pos_kp, self.posz_kp]))
        self.pos_kd_mat = np.diag(np.array([self.pos_kd, self.pos_kd, self.posz_kd]))
        self.pos_ki_mat = np.diag(np.array([self.pos_ki, self.pos_ki, self.posz_ki]))

        self.pos_error_integral = np.zeros(3)
        self.pos_integral_limit = 1.0        
        self.last_time = None

        trim_force = constants.K_THRUST * np.square(constants.TRIM_MOTOR_SPD)
        self.forces_old = np.repeat(trim_force, 4)

    def update(self, t, state, flat_output):
        pos = state['x']
        vel = state['v']
        quats = state['q']
        rates = state['w']
        pos_des = flat_output['x']
        vel_des = flat_output['x_dot']
        yaw_des = flat_output['yaw']

        # Position controller
        pos_error = pos - pos_des
        vel_error = vel - vel_des
        
        if self.last_time is not None:
            dt = t - self.last_time
            self.pos_error_integral += pos_error * dt
            self.pos_error_integral = np.clip(self.pos_error_integral, -self.pos_integral_limit, self.pos_integral_limit)
        self.last_time = t
        
        r_ddot_des = -(self.pos_kp_mat @ pos_error) - (self.pos_kd_mat @ vel_error) - (self.pos_ki_mat @ self.pos_error_integral)
            
        # Geometric nonlinear controller
        r = Rotation.from_quat(quats)
        rot_mat = r.as_matrix()
        f_des = constants.MASS * r_ddot_des + np.array([0, 0, constants.WEIGHT])
        f_des = np.squeeze(f_des)  # Need this line if using MPC to compute r_ddot_des
        b3 = rot_mat @ np.array([0, 0, 1])
        b3_des = f_des / np.linalg.norm(f_des)
        a_psi = np.array([np.cos(yaw_des), np.sin(yaw_des), 0])
        b2_des = np.cross(b3_des, a_psi) / np.linalg.norm(np.cross(b3_des, a_psi))
        rot_des = np.array([[np.cross(b2_des, b3_des)], [b2_des], [b3_des]]).T
        rot_des = np.squeeze(rot_des)
        euler = euler_from_matrix(rot_des)
        
        err_mat = 0.5 * (rot_des.T @ rot_mat - rot_mat.T @ rot_des)
        err_vec = np.array([-err_mat[1, 2], err_mat[0, 2], -err_mat[0, 1]])

        u1 = np.array([b3 @ f_des])
        u2 = constants.INERTIA @ (-constants.ATT_KP_MAT @ err_vec - constants.ATT_KD_MAT @ rates)

        # Get motor speed commands
        forces = constants.FORCES_CTRL_MAP @ np.concatenate((u1, u2))
        forces[forces < 0] = np.square(self.forces_old[forces < 0]) * constants.K_THRUST
        cmd_motor_speeds = np.sqrt(forces / constants.K_THRUST)
        self.forces_old = forces

        # Software limits for motor speeds
        cmd_motor_speeds = np.clip(cmd_motor_speeds, constants.ROTOR_SPEED_MIN, constants.ROTOR_SPEED_MAX)

        # Not used in simulation, for analysis only
        forces_limited = constants.K_THRUST * np.square(cmd_motor_speeds)
        ctrl_limited = constants.CTRL_FORCES_MAP @ forces_limited
        cmd_thrust = ctrl_limited[0]
        cmd_moment = ctrl_limited[1:]
        r = Rotation.from_matrix(rot_des)
        cmd_quat = r.as_quat()

        control_input = {
            'euler': euler,
            'cmd_thrust': u1,
            'cmd_motor_speeds': cmd_motor_speeds,
            'cmd_moment': cmd_moment,
            'cmd_quat': cmd_quat,
            'r_ddot_des': r_ddot_des
        }
        return control_input
