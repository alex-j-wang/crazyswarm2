from casadi import *
from scipy.spatial.transform import Rotation
from tf_transformations import euler_from_matrix
import torch
from NODE.NODE import *

class HybridControl(object):
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

        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz]))  # kg*m^2
        self.g = 9.81  # m/s^2
        
        self.geo_rollpitch_kp = 10
        self.geo_rollpitch_kd = 2 * 1.0 * np.sqrt(self.geo_rollpitch_kp)
        self.geo_yaw_kp = 50
        self.geo_yaw_kd = 2 * 1.15 * np.sqrt(self.geo_yaw_kp)
        self.att_kp_mat = np.diag(np.array([self.geo_rollpitch_kp, self.geo_rollpitch_kp, self.geo_yaw_kp]))
        self.att_kd_mat = np.diag(np.array([self.geo_rollpitch_kd, self.geo_rollpitch_kd, self.geo_yaw_kd]))
        k = self.k_drag / self.k_thrust
        self.ctrl_forces_map = np.array([[1, 1, 1, 1],
                                         [0, self.arm_length, 0, -self.arm_length],
                                         [-self.arm_length, 0, self.arm_length, 0],  # 0.046
                                         [k, -k, k, -k]])
        self.forces_ctrl_map = np.linalg.inv(self.ctrl_forces_map)
        self.trim_motor_spd = 1790.0
        trim_force = self.k_thrust * np.square(self.trim_motor_spd)
        self.forces_old = np.array([trim_force, trim_force, trim_force, trim_force])

        inv_inertia = np.linalg.inv(self.inertia)

        self.num_states = 6
        self.num_inputs = 3
        x = MX.sym('x', self.num_states, 1)
        u = MX.sym('u', self.num_inputs, 1)
        
        # These settings are for the kinematic model
        sampling_rate   = 0.06
        self.N_ctrl     = 10  # Control horizon (in number of timesteps)

        # Kinematic model (knowledge)
        xdot            = vertcat(x[3], x[4], x[5])
        xdotdot         = u  # Notice that there are no gravity term here
        ode             = vertcat(xdot, xdotdot)
        
        # loading neural network parameters
        ode_torch = torch.load("/home/tom/catkin_ws/src/crazyflie_ros/crazyflie_mpc/src/knode_models/rigid_1layer_2traj.pth", map_location=torch.device('cpu'))['ode_train']
        param_ls = []
        for _, layer in ode_torch.func.state_dict().items():
            param_ls.append(layer.detach().cpu().numpy())
        ode_nn = vertcat(x, u)
        # unrolling the nn to build a function
        for i in range(int(len(param_ls) / 2) - 1):
            # hidden layers
            ode_nn = tanh(mtimes(param_ls[i * 2], ode_nn)) + param_ls[i * 2 + 1]
        ode_nn = mtimes(param_ls[-2], ode_nn) + param_ls[-1]  # output layer
        
        ode_hybrid = ode + 1*ode_nn  # summing knowledge and nn 
        f               = Function('f', [x, u], [ode_hybrid])

        dae = {'x': x, 'p': u, 'ode': f(x, u)}
        options = dict(tf=sampling_rate, simplify=True, number_of_finite_elements=4)
        intg = integrator('intg', 'rk', dae, options)
        res = intg(x0=x, p=u)
        x_next = res['xf']
        self.Dynamics = Function('F', [x, u], [x_next])

        self.downsample_cnt = 0

    def update(self, t, state, flat_output):
        # State information
        pos     = state['x']
        vel     = state['v']
        quats   = state['q']
        rates   = state['w']
        pos_des = flat_output['x']
        vel_des = flat_output['x_dot']
        yaw_des = flat_output['yaw']

        # MPC
        if self.downsample_cnt % 50 == 0: # This assumes update() to be called at 200Hz
            opti = casadi.Opti()
            x = opti.variable(self.num_states, self.N_ctrl + 1)  # States
            u = opti.variable(self.num_inputs, self.N_ctrl)  # Control input
            p = opti.parameter(self.num_states, 1)  # Parameters

            state_des = vertcat(pos_des, vel_des)
            umax = np.array([15, 15, 15])
            # opti.minimize(1.0 * sumsqr(x[0:3, :] - pos_des) + 0.05 * sumsqr(x[3:, :] - vel_des) + 0.007 * sumsqr(u))
            opti.minimize(1. * sumsqr(x[0:2, :] - pos_des[0:2]) + \
                          1.2 * sumsqr(x[2, :] - pos_des[2]) + \
                          0.25 * sumsqr(x[3:, :] - vel_des) + 0.05 * sumsqr(u))

            for k in range(self.N_ctrl):
                opti.subject_to(x[:, k + 1] == self.Dynamics(x[:, k], u[:, k]))  # Dynamics constraints

            opti.subject_to(x[:, 0] == p)  # Initial condition constraints
            # opti.subject_to(x[:, self.N_ctrl] == state_des)  # Terminal constraints
            # opti.subject_to(opti.bounded(-umax, u[:, :], umax))

            # Specifying the solver and setting options
            p_opts = dict(print_time=False)
            s_opts = dict(print_level=0)
            opti.solver("ipopt", p_opts, s_opts)

            MPC_ctrl = opti.to_function('M', [p], [u[:, 0]])
            self.r_ddot_des = MPC_ctrl(vertcat(pos, vel))
        self.downsample_cnt += 1

        # Position controller
        # Geometric nonlinear controller
        r = Rotation.from_quat(quats)
        rot_mat = r.as_matrix()
        f_des = self.mass * self.r_ddot_des + np.array([0, 0, self.mass * self.g])
        f_des = np.squeeze(f_des)  # Need this line if using MPC to compute r_ddot_des
        b3 = rot_mat @ np.array([0, 0, 1])
        b3_des = f_des / np.linalg.norm(f_des)
        a_psi = np.array([np.cos(yaw_des), np.sin(yaw_des), 0])
        b2_des = np.cross(b3_des, a_psi) / np.linalg.norm(np.cross(b3_des, a_psi))
        rot_des = np.array([[np.cross(b2_des, b3_des)], [b2_des], [b3_des]]).T
        rot_des = np.squeeze(rot_des)
        euler = euler_from_matrix(rot_des) # euler angles from rotation matrix

        err_mat = 0.5 * (rot_des.T @ rot_mat - rot_mat.T @ rot_des)
        err_vec = np.array([-err_mat[1, 2], err_mat[0, 2], -err_mat[0, 1]])

        u1 = np.array([b3 @ f_des])
        u2 = self.inertia @ (-self.att_kp_mat @ err_vec - self.att_kd_mat @ rates)

        # Get motor speed commands
        forces = self.forces_ctrl_map @ np.concatenate((u1, u2))
        forces[forces < 0] = np.square(self.forces_old[forces < 0]) * self.k_thrust
        cmd_motor_speeds = np.sqrt(forces / self.k_thrust)
        self.forces_old = forces

        # Software limits for motor speeds
        cmd_motor_speeds = np.clip(cmd_motor_speeds, self.rotor_speed_min, self.rotor_speed_max)

        # Not used in simulation, for analysis only
        forces_limited = self.k_thrust * np.square(cmd_motor_speeds)
        ctrl_limited = self.ctrl_forces_map @ forces_limited
        cmd_thrust = ctrl_limited[0]
        cmd_moment = ctrl_limited[1:]
        r = Rotation.from_matrix(rot_des)
        cmd_quat = r.as_quat()

        control_input = {'euler': euler,
                         'cmd_thrust': u1,
                         'cmd_motor_speeds': cmd_motor_speeds,
                         'cmd_moment': cmd_moment,
                         'cmd_quat': cmd_quat,
                         'r_ddot_des': self.r_ddot_des}
        return control_input
