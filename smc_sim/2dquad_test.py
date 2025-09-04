import numpy as np
import matplotlib.pyplot as plt

class Quad2D:
    def __init__(self):
        self.m = 0.5 #kg
        self.I = 0.02
        self.g = 9.81
        self.l = 0.25
        
        #mini dist
        self.disturbance_amplitude = 0.0
        self.disturbance_freq = 0.5
        
    def dynamics(self, state, u, t):
        x, z, theta, x_dot, z_dot, theta_dot = state
        u1, u2 = u        
        #sin and cos dist
        if t>20:
            self.disturbance_amplitude = 5
        else: 
            self.disturbance_amplitude = 0.0
        # dist_x = self.disturbance_amplitude * np.sin(self.disturbance_freq * t)
        # dist_z = self.disturbance_amplitude * np.cos(self.disturbance_freq * t * 1.1)
        dist_x = 0
        dist_z = self.disturbance_amplitude
        dist_theta = 0.1 * np.sin(self.disturbance_freq * t * 1.5)
        
        #from euler 
        x_ddot = (u1/self.m) * np.sin(theta) + dist_x
        z_ddot = (u1/self.m) * np.cos(theta) - self.g + dist_z
        theta_ddot = u2/self.I + dist_theta
        
        return np.array([x_dot, z_dot, theta_dot, x_ddot, z_ddot, theta_ddot])

class SMC:
    def __init__(self, quad):
        self.quad = quad
        self.lambda_x = 1.5
        self.lambda_z = 2.0
        self.lambda_theta = 4.0
        self.k_x = 2.0
        self.k_z = 6.0
        self.k_theta = 8.0
        self.phi = 0.1
        
    def control(self, state, desired, t):
        x, z, theta, x_dot, z_dot, theta_dot = state
        x_d, z_d, theta_d, x_d_dot, z_d_dot, theta_d_dot = desired
        
        #error calcs
        e_x = x - x_d
        e_z = z - z_d
        e_x_dot = x_dot - x_d_dot
        e_z_dot = z_dot - z_d_dot
        
        #settting up sliding surfs
        s_x = e_x_dot + self.lambda_x * e_x
        s_z = e_z_dot + self.lambda_z * e_z
        
        if abs(s_x) > self.phi:
            theta_cmd = -self.k_x * np.sign(s_x) / self.quad.g
        else:
            theta_cmd = -self.k_x * (s_x / self.phi) / self.quad.g
        
        theta_cmd = np.clip(theta_cmd, -np.pi/6, np.pi/6)
        
        e_theta = theta - theta_cmd
        e_theta_dot = theta_dot
        s_theta = e_theta_dot + self.lambda_theta * e_theta      
          
        if abs(s_z) > self.phi:
            u1_sw = -self.k_z * np.sign(s_z)
        else:
            u1_sw = -self.k_z * (s_z / self.phi)
            
        if abs(s_theta) > self.phi:
            u2_sw = -self.k_theta * np.sign(s_theta)
        else:
            u2_sw = -self.k_theta * (s_theta / self.phi)
        
        u1_eq = self.quad.m * (self.quad.g - self.lambda_z * e_z_dot) / np.cos(theta)
        u1 = u1_eq + u1_sw
        
        u2_eq = -self.quad.I * self.lambda_theta * e_theta_dot
        u2 = u2_eq + u2_sw
        
        u1 = np.clip(u1, 1.0, 20.0)
        u2 = np.clip(u2, -3.0, 3.0)
        
        return np.array([u1, u2]), [s_x, s_z, s_theta]

class PID:
    def __init__(self, quad):
        self.quad = quad
        
        self.kp_x = 1.2
        self.ki_x = 0.1
        self.kd_x = 1.8
        self.kp_z = 4.0
        self.ki_z = 0.8
        self.kd_z = 2.5
        self.kp_theta = 8.0
        self.ki_theta = 1.0
        self.kd_theta = 3.0
        
        self.int_x = 0
        self.int_z = 0
        self.int_theta = 0
        self.prev_e_x = 0
        self.prev_e_z = 0
        self.prev_e_theta = 0
        
        self.int_limit = 5.0
        
    def control(self, state, desired, dt):
        x, z, theta, x_dot, z_dot, theta_dot = state
        x_d, z_d, theta_d, x_d_dot, z_d_dot, theta_d_dot = desired
        
        #err cals
        e_x = x_d - x
        e_z = z_d - z
        
        self.int_x += e_x * dt
        self.int_z += e_z * dt
        
        self.int_x = np.clip(self.int_x, -self.int_limit, self.int_limit)
        self.int_z = np.clip(self.int_z, -self.int_limit, self.int_limit)
        
        der_x = (e_x - self.prev_e_x) / dt if dt>0 else 0
        der_z = (e_z - self.prev_e_z) / dt if dt > 0 else 0
        
        self.prev_e_x = e_x
        self.prev_e_z = e_z
        
        theta_desired = np.arctan2(self.kp_x * e_x + self.ki_x * self.int_x + self.kd_x * der_x, self.quad.g + self.kp_z * e_z + self.ki_z * self.int_z + self.kd_z * der_z)
        theta_desired = np.clip(theta_desired, -np.pi/4, np.pi/4)
        
        e_theta = theta_desired - theta
        self.int_theta += e_theta * dt
        self.int_theta = np.clip(self.int_theta, -self.int_limit, self.int_limit)
        
        der_theta = (e_theta - self.prev_e_theta) / dt if dt > 0 else 0
        self.prev_e_theta = e_theta
        
        thrust_vertical = self.quad.m * (self.quad.g + self.kp_z * e_z + self.ki_z * self.int_z + self.kd_z * der_z)
        u1 = thrust_vertical / (np.cos(theta))
        u2 = self.kp_theta * e_theta + self.ki_theta * self.int_theta + self.kd_theta * der_theta
        
        u1 = np.clip(u1, 1.0, 20.0)
        u2 = np.clip(u2, -3.0, 3.0)
        
        return np.array([u1, u2])

def rk4_step(dynamics_func, state, u, t, dt):
    k1 = dynamics_func(state, u, t)
    k2 = dynamics_func(state + 0.5 * dt * k1, u, t + 0.5 * dt)
    k3 = dynamics_func(state + 0.5 * dt * k2, u, t + 0.5 * dt)
    k4 = dynamics_func(state + dt * k3, u, t + dt)
    
    return state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

def generate_traj(t):
    circle_radius = 2.0
    circle_freq = 0.05
    
    #circ traj
    x_circle = circle_radius * np.cos(2 * np.pi * circle_freq * t)
    z_circle = circle_radius * np.sin(2 * np.pi * circle_freq * t)
    x_dot_circle = -circle_radius * 2 * np.pi * circle_freq * np.sin(2 * np.pi * circle_freq * t)
    z_dot_circle = circle_radius * 2 * np.pi * circle_freq * np.cos(2 * np.pi * circle_freq * t)
    
    fig8_amp_x = 2.5
    fig8_amp_z = 1.5
    fig8_freq = 0.25
    
    #fig8 traj
    x_fig8 = fig8_amp_x * np.sin(2 * np.pi * fig8_freq * t)
    z_fig8 =  fig8_amp_z * np.sin(4 * np.pi * fig8_freq * t)
    x_dot_fig8 = fig8_amp_x * 2 * np.pi * fig8_freq * np.cos(2 * np.pi * fig8_freq * t)
    z_dot_fig8 = fig8_amp_z * 4 * np.pi * fig8_freq * np.cos(4 * np.pi * fig8_freq * t)
    
    circle_traj = np.array([x_circle, z_circle, np.zeros_like(t), x_dot_circle, z_dot_circle, np.zeros_like(t)])
    fig8_traj = np.array([x_fig8, z_fig8, np.zeros_like(t), x_dot_fig8, z_dot_fig8, np.zeros_like(t)])
    
    return circle_traj, fig8_traj

def sim_controller(controller, trajectory, controller_type, dt=0.01, t_end=50):
    t = np.arange(0, t_end, dt)
    n_steps = len(t)
    print(n_steps)

    quad = Quad2D()
    
    state = np.zeros((6, n_steps))
    state[:, 0] = [2, 0, 0, 0, 0, 0]
    
    controls = np.zeros((2, n_steps))
    if controller_type == 'SMC':
        sliding_surfaces = np.zeros((3, n_steps))
    
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

def plot_comparison(results_smc, results_pid, trajectories, traj_name):
    t_smc, state_smc, controls_smc, sliding_smc = results_smc
    t_pid, state_pid, controls_pid = results_pid
    circle_traj, fig8_traj = trajectories
    
    if traj_name == 'circle':
        ref_traj = circle_traj
    else:
        ref_traj = fig8_traj
    
    fig, axes = plt.subplots(3, 3, figsize=(18, 15))
    fig.suptitle(f'{traj_name.title()} Trajectory', fontsize=16)
    
    axes[0,0].plot(state_smc[0,:], state_smc[1,:], 'b-', linewidth=2, label='SMC')
    axes[0,0].plot(state_pid[0,:], state_pid[1,:], 'r--', linewidth=2, label='PID')
    axes[0,0].plot(ref_traj[0,:], ref_traj[1,:], 'k:', linewidth=3, label='Reference')
    axes[0,0].set_xlabel('X Pos (m)')
    axes[0,0].set_ylabel('Z Pos(m)')
    axes[0,0].set_title('2D Traj')
    axes[0,0].legend()
    axes[0,0].grid(True)
    axes[0,0].axis('equal')
    
    axes[0,1].plot(t_smc, state_smc[0,:], 'b-', linewidth=2, label='SMC')
    axes[0,1].plot(t_pid, state_pid[0,:], 'r--', linewidth=2, label='PID')
    axes[0,1].plot(t_smc, ref_traj[0,:], 'k:', linewidth=2, label='Reference')
    axes[0,1].set_xlabel('Time (s)')
    axes[0,1].set_ylabel('X Pos (m)')
    axes[0,1].set_title('X Pos Tracking')
    axes[0,1].legend()
    axes[0,1].grid(True)
    
    axes[0,2].plot(t_smc, state_smc[1,:], 'b-', linewidth=2, label='SMC')
    axes[0,2].plot(t_pid, state_pid[1,:], 'r--', linewidth=2, label='PID')
    axes[0,2].plot(t_smc, ref_traj[1,:], 'k:', linewidth=2, label='Reference')
    axes[0,2].set_xlabel('Time (s)')
    axes[0,2].set_ylabel('Z Pos (m)')
    axes[0,2].set_title('Z Pos Tracking')
    axes[0,2].legend()
    axes[0,2].grid(True)
    
    axes[1,0].plot(t_smc, state_smc[2,:], 'b-', linewidth=2, label='SMC')
    axes[1,0].plot(t_pid, state_pid[2,:], 'r--', linewidth=2, label='PID')
    axes[1,0].set_xlabel('Time (s)')
    axes[1,0].set_ylabel('Pitch Ang (rad)')
    axes[1,0].set_title('Pitch Ang')
    axes[1,0].legend()
    axes[1,0].grid(True)
    
    axes[1,1].plot(t_smc, controls_smc[0,:], 'b-', linewidth=2, label='SMC Thrust')
    axes[1,1].plot(t_pid, controls_pid[0,:], 'r--', linewidth=2, label='PID Thrust')
    axes[1,1].set_xlabel('Time (s)')
    axes[1,1].set_ylabel('Thrust (N)')
    axes[1,1].set_title('Thrust Ctrl')
    axes[1,1].legend()
    axes[1,1].grid(True)
    
    axes[1,2].plot(t_smc, controls_smc[1,:], 'b-', linewidth=2, label='SMC Torque')
    axes[1,2].plot(t_pid, controls_pid[1,:], 'r--', linewidth=2, label='PID Torque')
    axes[1,2].set_xlabel('Time (s)')
    axes[1,2].set_ylabel('Torque (N⋅m)')
    axes[1,2].set_title('Torque Ctrl')
    axes[1,2].legend()
    axes[1,2].grid(True)
    
    error_x_smc = state_smc[0,:] - ref_traj[0,:]
    error_z_smc = state_smc[1,:] - ref_traj[1,:]
    error_x_pid = state_pid[0,:] - ref_traj[0,:]
    error_z_pid = state_pid[1,:] - ref_traj[1,:]
    
    axes[2,0].plot(t_smc, error_x_smc, 'b-', linewidth=2, label='SMC')
    axes[2,0].plot(t_pid, error_x_pid, 'r--', linewidth=2, label='PID')
    axes[2,0].set_xlabel('Time (s)')
    axes[2,0].set_ylabel('X Err (m)')
    axes[2,0].set_title('X Pos Err')
    axes[2,0].legend()
    axes[2,0].grid(True)
    
    axes[2,1].plot(t_smc, error_z_smc, 'b-', linewidth=2, label='SMC')
    axes[2,1].plot(t_pid, error_z_pid, 'r--', linewidth=2, label='PID')
    axes[2,1].set_xlabel('Time (s)')
    axes[2,1].set_ylabel('Z Err (m)')
    axes[2,1].set_title('Z Pos Err')
    axes[2,1].legend()
    axes[2,1].grid(True)
    
    axes[2,2].plot(t_smc, sliding_smc[0,:], 'b-', linewidth=2, label='s_x')
    axes[2,2].plot(t_smc, sliding_smc[1,:], 'g-', linewidth=2, label='s_z')
    # axes[2,2].plot(t_smc, sliding_smc[2,:], 'purple', linewidth=2, label='s_θ')
    axes[2,2].axhline(y=0, color='r', linestyle='--', alpha=0.7)
    axes[2,2].set_xlabel('Time (s)')
    axes[2,2].set_ylabel('Sliding Surf')
    axes[2,2].set_title('SMC Sliding Surfs')
    axes[2,2].legend()
    axes[2,2].grid(True)
    
    plt.tight_layout()
    plt.show()

def calc_perf_metrics(t, state, reference):
    x_error = state[0,:] - reference[0,:]
    z_error = state[1,:] - reference[1,:]
    
    rmse_x = np.sqrt(np.mean(x_error**2))
    rmse_z = np.sqrt(np.mean(z_error**2))
    rmse_total = np.sqrt(rmse_x**2 + rmse_z**2)
    
    max_error_x = np.max(np.abs(x_error))
    max_error_z = np.max(np.abs(z_error))
    
    settling_time_x = None
    settling_time_z = None
    tolerance = 0.1
    
    for i in range(len(t)):
        if np.all(np.abs(x_error[i:]) < tolerance):
            settling_time_x = t[i]
            break
    
    for i in range(len(t)):
        if np.all(np.abs(z_error[i:]) < tolerance):
            settling_time_z = t[i]
            break
    
    return {
        'rmse_x': rmse_x,
        'rmse_z': rmse_z, 
        'rmse_total': rmse_total,
        'max_error_x': max_error_x,
        'max_error_z': max_error_z,
        'settling_time_x': settling_time_x,
        'settling_time_z': settling_time_z
    }

if __name__ == "__main__":
    quad = Quad2D()
    smc_controller = SMC(quad)
    pid_controller = PID(quad)
    
    t = np.arange(0, 50, 0.01)
    circle_traj, fig8_traj = generate_traj(t)
    
    print("circ trajectory sim")
    results_smc_circle = sim_controller(smc_controller, circle_traj, 'SMC')
    pid_controller = PID(quad)
    results_pid_circle = sim_controller(pid_controller, circle_traj, 'PID')
    
    print("figure-8 traj sim")
    smc_controller = SMC(quad)
    results_smc_fig8 = sim_controller(smc_controller, fig8_traj, 'SMC')
    pid_controller = PID(quad)
    results_pid_fig8 = sim_controller(pid_controller, fig8_traj, 'PID')
    
    plot_comparison(results_smc_circle, results_pid_circle, (circle_traj, fig8_traj), 'circle')
    plot_comparison(results_smc_fig8, results_pid_fig8, (circle_traj, fig8_traj), 'figure-8')
    
    print("\nperf metrics")
    print("\ncirc Traj:")
    metrics_smc_circle = calc_perf_metrics(results_smc_circle[0], results_smc_circle[1], circle_traj)
    metrics_pid_circle = calc_perf_metrics(results_pid_circle[0], results_pid_circle[1], circle_traj)
    
    print(f"SMC - RMSE: {metrics_smc_circle['rmse_total']:.4f}m, Max ErrX: {metrics_smc_circle['max_error_x']:.3f}m, Max Err Z: {metrics_smc_circle['max_error_z']:.3f}m")
    print(f"PID - RMSE: {metrics_pid_circle['rmse_total']:.4f}m, Max Err X: {metrics_pid_circle['max_error_x']:.3f}m, Max Err Z: {metrics_pid_circle['max_error_z']:.3f}m")
    
    print("\nFigure-8 Traj:")
    metrics_smc_fig8 = calc_perf_metrics(results_smc_fig8[0], results_smc_fig8[1], fig8_traj)
    metrics_pid_fig8 = calc_perf_metrics(results_pid_fig8[0], results_pid_fig8[1], fig8_traj)
    
    print(f"SMC - RMSE: {metrics_smc_fig8['rmse_total']:.4f}m, Max Err X: {metrics_smc_fig8['max_error_x']:.3f}m, Max Err Z: {metrics_smc_fig8['max_error_z']:.3f}m")
    print(f"PID - RMSE: {metrics_pid_fig8['rmse_total']:.4f}m, Max Err X: {metrics_pid_fig8['max_error_x']:.3f}m, Max Err Z: {metrics_pid_fig8['max_error_z']:.3f}m")