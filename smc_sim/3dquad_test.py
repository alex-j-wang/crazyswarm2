#3d quad simulation setup
#states 3D: [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot]
# [position, roll, pitch, yaw, velocities, angular rates]

#control inputs 3D: [thrust, roll_torque, pitch_torque, yaw_torque]
#goal: have 2d trajs, while doing circle and fig8 (y=0 and yaw=0)

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Quad3D:
    def __init__(self):
        self.m = 0.5  # kg
        self.Ixx = 0.2  # kg*m^2
        self.Iyy = 0.2  # kg*m^2  
        self.Izz = 0.4  # kg*m^2
        self.g = 9.81
        self.l = 0.25
        
        self.disturbance_amplitude = 0.0
        self.disturbance_freq = 0.5
        
    def dynamics(self, state, u, t):
        x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot = state
        u1, u2, u3, u4 = u
        
        if t > 20:
            self.disturbance_amplitude = 0  # N
        else:
            self.disturbance_amplitude = 0.0
            
        dist_x = 0
        dist_y = 0  
        dist_z = self.disturbance_amplitude
        dist_phi = 0.05 * np.sin(self.disturbance_freq * t * 1.2)
        dist_theta = 0.1 * np.sin(self.disturbance_freq * t * 1.5)
        dist_psi = 0.02 * np.sin(self.disturbance_freq * t * 0.8)
        
        #rot matrix form body to world
        cphi = np.cos(phi)
        sphi = np.sin(phi)
        ctheta = np.cos(theta)
        stheta = np.sin(theta)
        cpsi = np.cos(psi)
        spsi = np.sin(psi)
        
        # translational dyn
        x_ddot = u1/self.m * (cpsi*stheta*cphi + spsi*sphi) + dist_x
        y_ddot = u1/self.m * (spsi*stheta*cphi - cpsi*sphi) + dist_y
        z_ddot = u1/self.m * (ctheta*cphi) - self.g + dist_z
        
        # rot dyn (small angle approx for coupling terms)
        phi_ddot = u2/self.Ixx + dist_phi
        theta_ddot = u3/self.Iyy + dist_theta  
        psi_ddot = u4/self.Izz + dist_psi
        
        return np.array([x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot, 
                        x_ddot, y_ddot, z_ddot, phi_ddot, theta_ddot, psi_ddot])

class SMC3D:
    def __init__(self, quad):
        self.quad = quad
        self.lambda_x = 1.5
        self.lambda_y = 1.5
        self.lambda_z = 2.0
        self.lambda_phi = 4.0
        self.lambda_theta = 4.0
        self.lambda_psi = 3.0
        
        self.k_x = 2.0
        self.k_y = 2.0
        self.k_z = 6.0
        self.k_phi = 6.0
        self.k_theta = 8.0
        self.k_psi = 4.0
        
        self.phi_x = 0.1
        self.phi_y = 0.1
        self.phi_z = 0.1
        self.phi_phi = 0.15
        self.phi_theta = 0.21
        self.phi_psi = 0.15
        
    def control(self, state, desired, t):
        x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot = state
        x_d, y_d, z_d, phi_d, theta_d, psi_d, x_d_dot, y_d_dot, z_d_dot, phi_d_dot, theta_d_dot, psi_d_dot = desired
        
        # err definition
        e_x = x - x_d
        e_y = y - y_d
        e_z = z - z_d
        e_x_dot = x_dot - x_d_dot
        e_y_dot = y_dot - y_d_dot
        e_z_dot = z_dot - z_d_dot
        
        #pos sliding surfs
        s_x = e_x_dot + self.lambda_x * e_x
        s_y = e_y_dot + self.lambda_y * e_y
        s_z = e_z_dot + self.lambda_z * e_z
        
        #finding attitude des from pos ctrl
        if abs(s_x) > self.phi_x:
            theta_cmd = -self.k_x * np.sign(s_x) / self.quad.g
        else:
            theta_cmd = -self.k_x * (s_x / self.phi_x) / self.quad.g
            
        if abs(s_y) > self.phi_y:
            phi_cmd = self.k_y * np.sign(s_y) / self.quad.g
        else:
            phi_cmd = self.k_y * (s_y / self.phi_y) / self.quad.g
            
        psi_cmd = psi_d  #maintain yaw des
        
        #clip attitude cmds within limit
        theta_cmd = np.clip(theta_cmd, -np.pi/6, np.pi/6)
        phi_cmd = np.clip(phi_cmd, -np.pi/6, np.pi/6)
        
        # attitude error calc
        e_phi = phi - phi_cmd
        e_theta = theta - theta_cmd
        e_psi = psi - psi_cmd
        e_phi_dot = phi_dot
        e_theta_dot = theta_dot
        e_psi_dot = psi_dot
        
        #attitude slideing surfs 
        s_phi = e_phi_dot + self.lambda_phi * e_phi
        s_theta = e_theta_dot + self.lambda_theta * e_theta
        s_psi = e_psi_dot + self.lambda_psi * e_psi
        
        #ctrl inputs, thrust (u1)
        if abs(s_z) > self.phi_z:
            u1_sw = -self.k_z * np.sign(s_z)
        else:
            u1_sw = -self.k_z * (s_z / self.phi_z)
            
        u1_eq = self.quad.m * (self.quad.g - self.lambda_z * e_z_dot) / (np.cos(phi)*np.cos(theta))
        u1 = u1_eq + u1_sw
        
        #roll torque, u2
        if abs(s_phi) > self.phi_phi:
            u2_sw = -self.k_phi * np.sign(s_phi)
        else:
            u2_sw = -self.k_phi * (s_phi / self.phi_phi)
        u2_eq = -self.quad.Ixx * self.lambda_phi * e_phi_dot
        u2 = u2_eq + u2_sw
        
        #pitch torque (u3)
        if abs(s_theta) > self.phi_theta:
            u3_sw = -self.k_theta * np.sign(s_theta)
        else:
            u3_sw = -self.k_theta * (s_theta / self.phi_theta)
        u3_eq = -self.quad.Iyy * self.lambda_theta * e_theta_dot
        u3 = u3_eq + u3_sw
        
        # yaw torque (u4)
        if abs(s_psi) > self.phi_psi:
            u4_sw = -self.k_psi * np.sign(s_psi)
        else:
            u4_sw = -self.k_psi * (s_psi / self.phi_psi)
        u4_eq = -self.quad.Izz * self.lambda_psi * e_psi_dot
        u4 = u4_eq + u4_sw
        # limit to prevent saturation
        u1 = np.clip(u1, 1.0, 20.0)
        u2 = np.clip(u2, -2.0, 2.0)
        u3 = np.clip(u3, -3.0, 3.0)
        u4 = np.clip(u4, -1.0, 1.0)
        
        return np.array([u1, u2, u3, u4]), [s_x, s_y, s_z, s_phi, s_theta, s_psi]

class PID3D:
    def __init__(self, quad):
        self.quad = quad
        
        #pos gains
        self.kp_x = 1.2
        self.ki_x = 0.1
        self.kd_x = 1.8
        self.kp_y = 1.2
        self.ki_y = 0.1
        self.kd_y = 1.8
        self.kp_z = 4.0
        self.ki_z = 0.8
        self.kd_z = 2.5
        
        # attitude gains
        self.kp_phi = 6.0
        self.ki_phi = 0.8
        self.kd_phi = 2.0
        self.kp_theta = 8.0
        self.ki_theta = 1.0
        self.kd_theta = 3.0
        self.kp_psi = 4.0
        self.ki_psi = 0.5
        self.kd_psi = 1.5
        
        # integral terms
        self.int_x = 0
        self.int_y = 0
        self.int_z = 0
        self.int_phi = 0
        self.int_theta = 0
        self.int_psi = 0
        
        # previous errors
        self.prev_e_x = 0
        self.prev_e_y = 0
        self.prev_e_z = 0
        self.prev_e_phi = 0
        self.prev_e_theta = 0
        self.prev_e_psi = 0
        
        self.int_limit = 5.0
        
    def control(self, state, desired, dt):
        x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot = state
        x_d, y_d, z_d, phi_d, theta_d, psi_d, x_d_dot, y_d_dot, z_d_dot, phi_d_dot, theta_d_dot, psi_d_dot = desired
        
        #pos errs
        e_x = x_d - x
        e_y = y_d - y
        e_z = z_d - z
        
        #integral errors
        self.int_x += e_x * dt
        self.int_y += e_y * dt
        self.int_z += e_z * dt
        
        # Clipping integrals
        self.int_x = np.clip(self.int_x, -self.int_limit, self.int_limit)
        self.int_y = np.clip(self.int_y, -self.int_limit, self.int_limit)
        self.int_z = np.clip(self.int_z, -self.int_limit, self.int_limit)
        der_x = (e_x - self.prev_e_x) / dt if dt > 0 else 0
        der_y = (e_y - self.prev_e_y) / dt if dt > 0 else 0
        der_z = (e_z - self.prev_e_z) / dt if dt > 0 else 0
        
        self.prev_e_x = e_x
        self.prev_e_y = e_y
        self.prev_e_z = e_z
        
        #attitude des from pos ctrl
        F_x = self.kp_x * e_x + self.ki_x * self.int_x + self.kd_x * der_x
        F_y = self.kp_y * e_y + self.ki_y * self.int_y + self.kd_y * der_y
        F_z = self.quad.g + self.kp_z * e_z + self.ki_z * self.int_z + self.kd_z * der_z
        
        theta_desired = np.arctan2(F_x, F_z)
        phi_desired = np.arctan2(-F_y, F_z)
        psi_desired = psi_d
        
        theta_desired = np.clip(theta_desired, -np.pi/6, np.pi/6)
        phi_desired = np.clip(phi_desired, -np.pi/6, np.pi/6)
        
        #attitude errors
        e_phi = phi_desired - phi
        e_theta = theta_desired - theta
        e_psi = psi_desired - psi
        
        #update attitude integrals
        self.int_phi += e_phi * dt
        self.int_theta += e_theta * dt
        self.int_psi += e_psi * dt
        
        self.int_phi = np.clip(self.int_phi, -self.int_limit, self.int_limit)
        self.int_theta = np.clip(self.int_theta, -self.int_limit, self.int_limit)
        self.int_psi = np.clip(self.int_psi, -self.int_limit, self.int_limit)
        
        #attitude derivatives
        der_phi = (e_phi - self.prev_e_phi) / dt if dt > 0 else 0
        der_theta = (e_theta - self.prev_e_theta) / dt if dt > 0 else 0
        der_psi = (e_psi - self.prev_e_psi) / dt if dt > 0 else 0
        
        self.prev_e_phi = e_phi
        self.prev_e_theta = e_theta
        self.prev_e_psi = e_psi
        
        #control output calcs
        u1 = self.quad.m * F_z / (np.cos(phi)*np.cos(theta))
        u2 = self.kp_phi * e_phi + self.ki_phi * self.int_phi + self.kd_phi * der_phi
        u3 = self.kp_theta * e_theta + self.ki_theta * self.int_theta + self.kd_theta * der_theta
        u4 = self.kp_psi * e_psi + self.ki_psi * self.int_psi + self.kd_psi * der_psi
        
        #clipping to prevent saturation, lim as SMC
        u1 = np.clip(u1, 1.0, 20.0)
        u2 = np.clip(u2, -2.0, 2.0)
        u3 = np.clip(u3, -3.0, 3.0)
        u4 = np.clip(u4, -1.0, 1.0)
        
        return np.array([u1, u2, u3, u4])

def generate_traj_3d(t):
    circle_radius = 2.0
    circle_freq = 0.1
    
    #xz plane circ traj
    x_circle = circle_radius * np.cos(2 * np.pi * circle_freq * t)
    y_circle = np.zeros_like(t)  # Stay in x-z plane
    z_circle = circle_radius * np.sin(2 * np.pi * circle_freq * t)
    x_dot_circle = -circle_radius * 2 * np.pi * circle_freq * np.sin(2 * np.pi * circle_freq * t)
    y_dot_circle = np.zeros_like(t)
    z_dot_circle = circle_radius * 2 * np.pi * circle_freq * np.cos(2 * np.pi * circle_freq * t)
    
    #xz plane fig8 traj
    fig8_amp_x = 2.5
    fig8_amp_z = 1.5
    fig8_freq = 0.05
    
    x_fig8 = fig8_amp_x * np.sin(2 * np.pi * fig8_freq * t)
    y_fig8 = np.zeros_like(t)  # Stay in x-z plane
    z_fig8 = fig8_amp_z * np.sin(4 * np.pi * fig8_freq * t)
    x_dot_fig8 = fig8_amp_x * 2 * np.pi * fig8_freq * np.cos(2 * np.pi * fig8_freq * t)
    y_dot_fig8 = np.zeros_like(t)
    z_dot_fig8 = fig8_amp_z * 4 * np.pi * fig8_freq * np.cos(4 * np.pi * fig8_freq * t)
    
    #setting no des rotation
    phi_traj = np.zeros_like(t)
    theta_traj = np.zeros_like(t)
    psi_traj = np.zeros_like(t)
    phi_dot_traj = np.zeros_like(t)
    theta_dot_traj = np.zeros_like(t)
    psi_dot_traj = np.zeros_like(t)
    
    circle_traj = np.array([x_circle, y_circle, z_circle, phi_traj, theta_traj, psi_traj,
                           x_dot_circle, y_dot_circle, z_dot_circle, phi_dot_traj, theta_dot_traj, psi_dot_traj])
    fig8_traj = np.array([x_fig8, y_fig8, z_fig8, phi_traj, theta_traj, psi_traj,
                         x_dot_fig8, y_dot_fig8, z_dot_fig8, phi_dot_traj, theta_dot_traj, psi_dot_traj])
    
    return circle_traj, fig8_traj

def rk4_step(dynamics_func, state, u, t, dt):
    k1 = dynamics_func(state, u, t)
    k2 = dynamics_func(state + 0.5 * dt * k1, u, t + 0.5 * dt)
    k3 = dynamics_func(state + 0.5 * dt * k2, u, t + 0.5 * dt)
    k4 = dynamics_func(state + dt * k3, u, t + dt)
    
    return state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

def sim_controller_3d(controller, trajectory, controller_type, dt=0.01, t_end=50):
    t = np.arange(0, t_end, dt)
    n_steps = len(t)
    print(f"{controller_type}: {n_steps} steps")

    quad = Quad3D()
    
    state = np.zeros((12, n_steps))
    state[:, 0] = trajectory[:, 0]
    
    controls = np.zeros((4, n_steps))
    if controller_type == 'SMC':
        sliding_surfaces = np.zeros((6, n_steps))
    
    for i in range(n_steps - 1):
        current_state = state[:, i]
        desired_state = trajectory[:, i]
        
        if controller_type == 'SMC':
            u, s = controller.control(current_state, desired_state, t[i])
            sliding_surfaces[:, i] = s
        else:
            u = controller.control(current_state, desired_state, dt)
        
        controls[:, i] = u
        
        state[:, i+1] = rk4_step(quad.dynamics, current_state, u, t[i], dt)
    
    if controller_type == 'SMC':
        return t, state, controls, sliding_surfaces
    else:
        return t, state, controls

def calc_perf_metrics_3d(t, state, reference):
    x_error = state[0,:] - reference[0,:]
    y_error = state[1,:] - reference[1,:]
    z_error = state[2,:] - reference[2,:]
    phi_error = state[3,:] - reference[3,:]
    theta_error = state[4,:] - reference[4,:]
    psi_error = state[5,:] - reference[5,:]
    
    rmse_x = np.sqrt(np.mean(x_error**2))
    rmse_y = np.sqrt(np.mean(y_error**2))
    rmse_z = np.sqrt(np.mean(z_error**2))
    rmse_total = np.sqrt(rmse_x**2 + rmse_y**2 + rmse_z**2)
    
    max_error_x = np.max(np.abs(x_error))
    max_error_y = np.max(np.abs(y_error))
    max_error_z = np.max(np.abs(z_error))
    max_error_phi = np.max(np.abs(phi_error))
    max_error_theta = np.max(np.abs(theta_error))
    max_error_psi = np.max(np.abs(psi_error))
    
    return {
        'rmse_x': rmse_x,
        'rmse_y': rmse_y,
        'rmse_z': rmse_z,
        'rmse_total': rmse_total,
        'max_error_x': max_error_x,
        'max_error_y': max_error_y,
        'max_error_z': max_error_z,
        'max_error_phi': max_error_phi,
        'max_error_theta': max_error_theta,
        'max_error_psi': max_error_psi
    }

def plot_3d_results(results_smc, results_pid, trajectories, traj_name):
    t_smc, state_smc, controls_smc, sliding_smc = results_smc
    t_pid, state_pid, controls_pid = results_pid
    circle_traj, fig8_traj = trajectories
    
    if traj_name == 'circle':
        ref_traj = circle_traj
    else:
        ref_traj = fig8_traj
    
    fig = plt.figure(figsize=(20, 15))
    
    #traj plot
    ax1 = fig.add_subplot(2, 4, 1, projection='3d')
    ax1.plot(state_smc[0,:], state_smc[1,:], state_smc[2,:], 'b-', linewidth=2, label='SMC')
    ax1.plot(state_pid[0,:], state_pid[1,:], state_pid[2,:], 'r--', linewidth=2, label='PID')
    ax1.plot(ref_traj[0,:], ref_traj[1,:], ref_traj[2,:], 'k:', linewidth=3, label='Reference')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title(f'3D {traj_name.title()} Trajectory')
    ax1.legend()
    
    #x pos
    ax2 = fig.add_subplot(2, 4, 2)
    ax2.plot(t_smc, state_smc[0,:], 'b-', linewidth=2, label='SMC')
    ax2.plot(t_pid, state_pid[0,:], 'r--', linewidth=2, label='PID')
    ax2.plot(t_smc, ref_traj[0,:], 'k:', linewidth=2, label='Reference')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('X Position (m)')
    ax2.set_title('X Tracking')
    ax2.legend()
    ax2.grid(True)
    
    #y pos, ensure this is close to 0 in result
    ax3 = fig.add_subplot(2, 4, 3)
    ax3.plot(t_smc, state_smc[1,:], 'b-', linewidth=2, label='SMC')
    ax3.plot(t_pid, state_pid[1,:], 'r--', linewidth=2, label='PID')
    ax3.plot(t_smc, ref_traj[1,:], 'k:', linewidth=2, label='Reference')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Y Position (m)')
    ax3.set_title('Y Tracking')
    ax3.legend()
    ax3.grid(True)
    
    #z pos
    ax4 = fig.add_subplot(2, 4, 4)
    ax4.plot(t_smc, state_smc[2,:], 'b-', linewidth=2, label='SMC')
    ax4.plot(t_pid, state_pid[2,:], 'r--', linewidth=2, label='PID')
    ax4.plot(t_smc, ref_traj[2,:], 'k:', linewidth=2, label='Reference')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Z Position (m)')
    ax4.set_title('Z Tracking')
    ax4.legend()
    ax4.grid(True)
    
    #attitude angles 
    ax5 = fig.add_subplot(2, 4, 5)
    ax5.plot(t_smc, state_smc[3,:]*180/np.pi, 'b-', linewidth=2, label='SMC φ')
    ax5.plot(t_pid, state_pid[3,:]*180/np.pi, 'r--', linewidth=2, label='PID φ')
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Roll Angle (deg)')
    ax5.set_title('Roll Angle')
    ax5.legend()
    ax5.grid(True)
    
    ax6 = fig.add_subplot(2, 4, 6)
    ax6.plot(t_smc, state_smc[4,:]*180/np.pi, 'b-', linewidth=2, label='SMC θ')
    ax6.plot(t_pid, state_pid[4,:]*180/np.pi, 'r--', linewidth=2, label='PID θ')
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Pitch Angle (deg)')
    ax6.set_title('Pitch Angle')
    ax6.legend()
    ax6.grid(True)
    
    ax7 = fig.add_subplot(2, 4, 7)
    ax7.plot(t_smc, state_smc[5,:]*180/np.pi, 'b-', linewidth=2, label='SMC ψ')
    ax7.plot(t_pid, state_pid[5,:]*180/np.pi, 'r--', linewidth=2, label='PID ψ')
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('Yaw Angle (deg)')
    ax7.set_title('Yaw Angle (should be 0)')
    ax7.legend()
    ax7.grid(True)
    
    #ctrl inputs 
    ax8 = fig.add_subplot(2, 4, 8)
    ax8.plot(t_smc, controls_smc[0,:], 'b-', linewidth=2, label='SMC Thrust')
    ax8.plot(t_pid, controls_pid[0,:], 'r--', linewidth=2, label='PID Thrust')
    ax8.set_xlabel('Time (s)')
    ax8.set_ylabel('Thrust (N)')
    ax8.set_title('Thrust Control')
    ax8.legend()
    ax8.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    quad = Quad3D()
    smc_controller = SMC3D(quad)
    pid_controller = PID3D(quad)
    
    t = np.arange(0, 50, 0.01)
    circle_traj, fig8_traj = generate_traj_3d(t)
    
    print("3D Circle trajectory simulation")
    results_smc_circle = sim_controller_3d(smc_controller, circle_traj, 'SMC')
    pid_controller = PID3D(quad)
    results_pid_circle = sim_controller_3d(pid_controller, circle_traj, 'PID')
    
    print("3D Figure-8 trajectory simulation")  
    smc_controller = SMC3D(quad)
    results_smc_fig8 = sim_controller_3d(smc_controller, fig8_traj, 'SMC')
    pid_controller = PID3D(quad)
    results_pid_fig8 = sim_controller_3d(pid_controller, fig8_traj, 'PID')
    
    plot_3d_results(results_smc_circle, results_pid_circle, (circle_traj, fig8_traj), 'circle')
    plot_3d_results(results_smc_fig8, results_pid_fig8, (circle_traj, fig8_traj), 'figure-8')
    
    print("\n3D Performance Metrics")
    print("\nCircle Trajectory:")
    metrics_smc_circle = calc_perf_metrics_3d(results_smc_circle[0], results_smc_circle[1], circle_traj)
    metrics_pid_circle = calc_perf_metrics_3d(results_pid_circle[0], results_pid_circle[1], circle_traj)
    
    print(f"SMC - RMSE: {metrics_smc_circle['rmse_total']:.4f}m, Max ErrX: {metrics_smc_circle['max_error_x']:.3f}m, Max ErrY: {metrics_smc_circle['max_error_y']:.3f}m, Max ErrZ: {metrics_smc_circle['max_error_z']:.3f}m")
    print(f"PID - RMSE: {metrics_pid_circle['rmse_total']:.4f}m, Max ErrX: {metrics_pid_circle['max_error_x']:.3f}m, Max ErrY: {metrics_pid_circle['max_error_y']:.3f}m, Max ErrZ: {metrics_pid_circle['max_error_z']:.3f}m")
    
    print("\nFigure-8 Trajectory:")
    metrics_smc_fig8 = calc_perf_metrics_3d(results_smc_fig8[0], results_smc_fig8[1], fig8_traj)
    metrics_pid_fig8 = calc_perf_metrics_3d(results_pid_fig8[0], results_pid_fig8[1], fig8_traj)
    
    print(f"SMC - RMSE: {metrics_smc_fig8['rmse_total']:.4f}m, Max ErrX: {metrics_smc_fig8['max_error_x']:.3f}m, Max ErrY: {metrics_smc_fig8['max_error_y']:.3f}m, Max ErrZ: {metrics_smc_fig8['max_error_z']:.3f}m")
    print(f"PID - RMSE: {metrics_pid_fig8['rmse_total']:.4f}m, Max ErrX: {metrics_pid_fig8['max_error_x']:.3f}m, Max ErrY: {metrics_pid_fig8['max_error_y']:.3f}m, Max ErrZ: {metrics_pid_fig8['max_error_z']:.3f}m")
    
    print(f"\nY-axis deviation (should be ~0):")
    print(f"SMC Circle Max Y Error: {metrics_smc_circle['max_error_y']:.4f}m")
    print(f"PID Circle Max Y Error: {metrics_pid_circle['max_error_y']:.4f}m")