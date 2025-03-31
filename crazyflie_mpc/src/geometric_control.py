import numpy as np
from scipy.spatial.transform import Rotation
from tf.transformations import euler_from_matrix

class GeometriControl(object):
    def __init__(self):
        # Quadrotor physical parameters.
        self.mass = 0.03  # quad_params['mass'] # kg
        self.Ixx = 1.43e-5  # quad_params['Ixx']  # kg*m^2
        self.Iyy = 1.43e-5  # quad_params['Iyy']  # kg*m^2
        self.Izz = 2.89e-5  # quad_params['Izz']  # kg*m^2
        self.arm_length = 0.046  # quad_params['arm_length'] # meters
        self.rotor_speed_min = 0  # quad_params['rotor_speed_min'] # rad/s
        self.rotor_speed_max = 2500  # quad_params['rotor_speed_max'] # rad/s
        self.k_thrust = 2.3e-08  # quad_params['k_thrust'] # N/(rad/s)**2
        self.k_drag = 7.8e-11  # quad_params['k_drag']   # Nm/(rad/s)**2

        # You may define any additional constants you like including control gains.
        self.inertia        = np.diag(np.array([self.Ixx, self.Iyy, self.Izz])) # kg*m^2
        self.g              = 9.81 # m/s^2

        # STUDENT CODE HERE
        self.pos_kp = 2.0
        self.pos_kd = 2 * 1.0 * np.sqrt(self.pos_kp)
        self.posz_kp = 4.0  # was 4
        self.posz_kd = 2.4  # was 2.4
        self.pos_kp_mat = np.diag(np.array([self.pos_kp, self.pos_kp, self.posz_kp]))
        self.pos_kd_mat = np.diag(np.array([self.pos_kd, self.pos_kd, self.posz_kd]))
        self.att_rollpitch_kp = 9
        self.att_rollpitch_kd = 2 * 1.0 * np.sqrt(self.att_rollpitch_kp)
        self.att_yaw_kp = 20
        self.att_yaw_kd = 2 * 1.15 * np.sqrt(self.att_yaw_kp)
        self.geo_rollpitch_kp = 10
        self.geo_rollpitch_kd = 2 * 1.0 * np.sqrt(self.geo_rollpitch_kp)
        self.geo_yaw_kp = 50
        self.geo_yaw_kd = 2 * 1.15 * np.sqrt(self.geo_yaw_kp)
        self.att_kp_mat = np.diag(np.array([self.geo_rollpitch_kp, self.geo_rollpitch_kp, self.geo_yaw_kp]))
        self.att_kd_mat = np.diag(np.array([self.geo_rollpitch_kd, self.geo_rollpitch_kd, self.geo_yaw_kd]))

        k                       = self.k_drag / self.k_thrust  # 0.003391304347826087
        self.ctrl_forces_map    = np.array([[1, 1, 1, 1],
                                            [0, self.arm_length, 0, -self.arm_length],
                                            [-self.arm_length, 0, self.arm_length, 0],  # 0.046
                                            [k, -k, k, -k]])
        self.forces_ctrl_map    = np.linalg.inv(self.ctrl_forces_map)
        self.trim_motor_spd     = 1790.0
        trim_force              = self.k_thrust * np.square(self.trim_motor_spd)
        self.forces_old         = np.array([trim_force, trim_force, trim_force, trim_force])

        # self.duration       = 5.5
        # time                = np.arange(0, self.duration, 1 / 100)
        # self.u_perturb      = chirp(time, f0=10, f1=0.01, t1=self.duration, method='linear')
        # self.cnt            = 0
        # self.reset_flag     = True

    def update(self, t, state, flat_output):
        pos         = state['x']
        vel         = state['v']
        quats       = state['q']
        rates       = state['w']
        pos_des     = flat_output['x']
        vel_des     = flat_output['x_dot']
        yaw_des     = 0.0 #flat_output['yaw']

        # Get rotation matrix, in quaternions
        r           = Rotation.from_quat(quats)
        rot_mat     = r.as_matrix()

        t11         = rot_mat[0, 0]
        t12         = rot_mat[0, 1]
        t13         = rot_mat[0, 2]
        t21         = rot_mat[1, 0]
        t22         = rot_mat[1, 1]
        t23         = rot_mat[1, 2]
        t31         = rot_mat[2, 0]
        t32         = rot_mat[2, 1]
        t33         = rot_mat[2, 2]

        # Get Euler angles from rotation matrix
        phi         = np.arcsin(t32)
        tet         = np.arctan2(-t31 / np.cos(phi), t33 / np.cos(phi))
        psi         = np.arctan2(-t12 / np.cos(phi), t22 / np.cos(phi))

        # Position controller
        r_ddot_des  = -(self.pos_kd_mat @ (vel - vel_des)) - (self.pos_kp_mat @ (pos - pos_des))

            
            
        # Geometric nonlinear controller
        f_des       = self.mass * r_ddot_des + np.array([0, 0, self.mass * self.g])
        f_des       = np.squeeze(f_des) # Need this line if using MPC to compute r_ddot_des
        b3          = rot_mat @ np.array([0, 0, 1])
        b3_des      = f_des / np.linalg.norm(f_des)
        a_psi       = np.array([np.cos(yaw_des), np.sin(yaw_des), 0])
        b2_des      = np.cross(b3_des, a_psi) / np.linalg.norm(np.cross(b3_des, a_psi))
        rot_des     = np.array([[np.cross(b2_des, b3_des)], [b2_des], [b3_des]]).T
        rot_des     = np.squeeze(rot_des)
        err_mat     = 0.5 * (rot_des.T @ rot_mat - rot_mat.T @ rot_des)
        err_vec     = np.array([-err_mat[1, 2], err_mat[0, 2], -err_mat[0, 1]])
        euler       = euler_from_matrix(rot_des) # euler angles from rotation matrix

        u1          = np.array([b3 @ f_des])
        u2          = self.inertia @ (-self.att_kp_mat @ err_vec - self.att_kd_mat @ rates)

        # if 1.0 <= t <= 6.0:
        #     u2[0] = u2[0] + self.u_perturb[self.cnt] * 0.005
        #     self.cnt += 1
        #
        # if t >= 6.5 and t <= 11.5:
        #     u2[1] = u2[1] + self.u_perturb[self.cnt] * 0.005
        #     self.cnt += 1
        #
        # if t >= 12.0 and t <= 17.0:
        #     u2[2] = u2[2] + self.u_perturb[self.cnt]*0.0005
        #     self.cnt += 1
        #
        # if t >= 17.5 and t <= 22.5:
        #     u1 = u1 + self.u_perturb[self.cnt]*0.15
        #     self.cnt += 1

        # if 6.0 < t < 6.5 or 11.5 < t < 12.0 or 17.0 < t < 17.5 or 22.5 < t < 28:
        #     self.cnt = 0

        # Making sure that yaw error stays within [-pi,pi)
        #yaw_err = psi - yaw_des
        #while yaw_err >= np.radians(180.0):
        #    yaw_err -= np.radians(360.0)
        #while yaw_err < -np.radians(180.0):
        #    yaw_err += np.radians(360.0)

        # Linear backstepping controller
        #yaw_mat     = np.array([[np.cos(yaw_des), np.sin(yaw_des)], [np.sin(yaw_des), -np.cos(yaw_des)]])
        #r_ddot_norm = np.array([r_ddot_des[0], r_ddot_des[1]])
        #angles_des  = (yaw_mat @ r_ddot_norm) / self.g
        #roll_ctrl   = -self.att_rollpitch_kp * (phi - angles_des[1]) - self.att_rollpitch_kd * rates[0]
        #pitch_ctrl  = -self.att_rollpitch_kp * (tet - angles_des[0]) - self.att_rollpitch_kd * rates[1]
        #yaw_ctrl    = -self.att_yaw_kp * yaw_err - self.att_yaw_kd * (rates[2] - 0.0)
        #att_ctrl    = np.array([roll_ctrl, pitch_ctrl, yaw_ctrl])

        #u1       = np.array([(r_ddot_des[2] + self.g) * self.mass])
        #u2       = self.inertia @ att_ctrl

        # Get motor speed commands
        forces = self.forces_ctrl_map @ np.concatenate((u1, u2))
        # Protect against invalid force and motor speed commands (set them to previous motor speeds)
        forces[forces < 0]  = np.square(self.forces_old[forces < 0]) * self.k_thrust
        cmd_motor_speeds    = np.sqrt(forces / self.k_thrust)

        # Software limits for motor speeds
        cmd_motor_speeds    = np.clip(cmd_motor_speeds, self.rotor_speed_min, self.rotor_speed_max)

        # Not used in simulation, for analysis only
        forces_limited  = self.k_thrust * np.square(cmd_motor_speeds)
        ctrl_limited    = self.ctrl_forces_map @ forces_limited
        cmd_thrust      = ctrl_limited[0]
        cmd_moment      = ctrl_limited[1:]
        r               = Rotation.from_matrix(rot_des)
        cmd_q           = r.as_quat()

        self.forces_old     = forces
        control_input = {'euler': euler,
                         'cmd_thrust' : u1,
                         'cmd_motor_speeds':cmd_motor_speeds,
                         'cmd_moment':cmd_moment,
                         'cmd_quat':cmd_q,
                         'r_ddot_des':r_ddot_des}
        return control_input
