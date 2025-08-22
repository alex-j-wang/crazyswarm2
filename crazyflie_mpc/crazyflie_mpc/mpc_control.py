import constants

from casadi import *
from scipy.spatial.transform import Rotation
from tf_transformations import euler_from_matrix

class MPControl(object):
    def __init__(self, control_frequency):
        SAMPLE_PERIOD = 0.02
        HORIZON = 10
        NUM_STATES = 6
        NUM_INPUTS = 3

        x = MX.sym('x', NUM_STATES)
        u = MX.sym('u', NUM_INPUTS)
        xdot = x[3:6]
        xdotdot = u

        ode = vertcat(xdot, xdotdot)
        dae = dict(x=x, p=u, ode=ode)
        options = dict(simplify=True, number_of_finite_elements=4)
        intg = integrator('intg', 'rk', dae, 0, SAMPLE_PERIOD, options)
        res = intg(x0=x, p=u)
        x_next = res['xf']
        Dynamics = Function('F', [x, u], [x_next])

        opti = Opti()
        p = opti.parameter(NUM_STATES, 1)           # Initial state
        pos_des = opti.parameter(3, 1)              # Desired position
        vel_des = opti.parameter(3, 1)              # Desired velocity
        x = opti.variable(NUM_STATES, HORIZON + 1)  # States over horizon
        u = opti.variable(NUM_INPUTS, HORIZON)      # Control over horizon
        
        cost = (
            1.0 * sumsqr(x[0:2, :] - pos_des[0:2]) +
            1.2 * sumsqr(x[2, :] - pos_des[2]) +
            0.25 * sumsqr(x[3:, :] - vel_des) +
            0.05 * sumsqr(u)
        )
        opti.minimize(cost)

        opti.subject_to(x[:, 0] == p)
        for k in range(HORIZON):
            opti.subject_to(x[:, k + 1] == Dynamics(x[:, k], u[:, k]))

        p_opts = dict(print_time=False)
        s_opts = dict(print_level=0)
        opti.solver("ipopt", p_opts, s_opts)

        self.MPC = opti.to_function('M', [p, pos_des, vel_des], [u[:, 0]])
        self.control_frequency = control_frequency
        self.downsample_cnt = 0

        trim_force = constants.K_THRUST * np.square(constants.TRIM_MOTOR_SPD)
        self.forces_old = np.repeat(trim_force, 4)

    def update(self, t, state, flat_output):
        # State information
        pos = state['x']
        vel = state['v']
        quats = state['q']
        rates = state['w']
        pos_des = flat_output['x']
        vel_des = flat_output['x_dot']
        yaw_des = flat_output['yaw']

        # MPC
        if self.downsample_cnt % (self.control_frequency // 4) == 0:
            p = vertcat(pos, vel)
            self.r_ddot_des = self.MPC(p, pos_des, vel_des)
        self.downsample_cnt += 1

        # Geometric nonlinear controller
        r = Rotation.from_quat(quats)
        rot_mat = r.as_matrix()
        f_des = constants.MASS * self.r_ddot_des + np.array([0, 0, constants.WEIGHT])
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
            'r_ddot_des': self.r_ddot_des
        }
        return control_input
