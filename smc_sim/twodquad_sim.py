import numpy as np
import matplotlib.pyplot as plt

class Quad2D_Controller:
    def __init__(self):
        # Crazyflie 2.1 parameters
        self.m = 0.027    # mass (kg)
        self.I = 1.4e-5   # moment of inertia (kg*m^2)
        self.l = 0.046    # arm length (m)
        self.g = 9.81     # gravity (m/s^2)
        
        # Control limits
        self.max_thrust = 4 * 0.016  # Max total thrust
        self.hover_thrust = self.m * self.g
        self.u1_min = self.hover_thrust * 0.5
        self.u1_max = self.hover_thrust * 2.0
        self.u2_max = 5e-6  # Max torque (reduced for stability)
        self.theta_max = np.pi/8  # Max 22.5 degrees tilt (reduced)
        
        # Conservative controller gains for stability
        # Position control gains (reduced for stability)
        self.kp_pos = 0.8
        self.ki_pos = 0.02
        self.kd_pos = 0.4
        
        # Attitude control gains (well-tuned PID baseline)
        self.kp_att = 2.5
        self.ki_att = 0.1
        self.kd_att = 0.6
        
        # Conservative SMC parameters 
        self.lambda_smc = 2.0  # Sliding surface slope (reduced)
        self.k_smc = 0.3       # Switching gain (much reduced)
        self.phi_smc = 0.1     # Boundary layer thickness (reduced)
        
        # Control type
        self.use_smc = False
        self.controller_name = "PID"
        
        # State variables for integral control
        self.reset_controller_states()
        
        # Moderate disturbance parameters
        self.dist_force_amp = 0.02    # 20 mN force disturbances (reduced)
        self.dist_torque_amp = 2e-6   # 2 μN⋅m torque disturbances (reduced)
        self.dist_freq = [1.0, 1.5, 0.7]  # Different frequencies
        
    def reset_controller_states(self):
        """Reset all controller internal states"""
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_theta = 0.0
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_theta = 0.0
        
    def saturate(self, value, min_val, max_val):
        """Saturate value between limits"""
        return np.clip(value, min_val, max_val)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return ((angle + np.pi) % (2*np.pi)) - np.pi
    
    def dynamics(self, state, u1, u2, disturbances):
        """2D quadrotor dynamics with disturbances"""
        x, x_dot, y, y_dot, theta, theta_dot = state
        fx_dist, fy_dist, tau_dist = disturbances
        
        # Dynamics equations
        x_ddot = (u1 * np.sin(theta) + fx_dist) / self.m
        y_ddot = (u1 * np.cos(theta) - self.m * self.g + fy_dist) / self.m
        theta_ddot = (u2 + tau_dist) / self.I
        
        return np.array([x_dot, x_ddot, y_dot, y_ddot, theta_dot, theta_ddot])
    
    def position_controller(self, state, dt, x_ref, y_ref):
        """PID position controller -> desired thrust and attitude"""
        x, x_dot, y, y_dot, theta, theta_dot = state
        
        # Position errors
        e_x = x_ref - x
        e_y = y_ref - y
        
        # Update integrals with anti-windup
        self.integral_x += e_x * dt
        self.integral_y += e_y * dt
        self.integral_x = self.saturate(self.integral_x, -1.0, 1.0)
        self.integral_y = self.saturate(self.integral_y, -1.0, 1.0)
        
        # Derivatives
        de_x = (e_x - self.prev_error_x) / dt if dt > 0 else 0
        de_y = (e_y - self.prev_error_y) / dt if dt > 0 else 0
        
        # PID outputs (desired accelerations)
        ax_des = self.kp_pos * e_x + self.ki_pos * self.integral_x + self.kd_pos * de_x
        ay_des = self.kp_pos * e_y + self.ki_pos * self.integral_y + self.kd_pos * de_y
        
        # Limit accelerations (more conservative)
        ax_des = self.saturate(ax_des, -3.0, 3.0)
        ay_des = self.saturate(ay_des, -3.0, 3.0)
        
        # Convert to thrust and desired attitude
        ay_total = ay_des + self.g
        
        # Desired thrust
        u1_des = self.m * np.sqrt(ax_des**2 + ay_total**2)
        u1_des = self.saturate(u1_des, self.u1_min, self.u1_max)
        
        # Desired attitude
        if abs(ay_total) > 0.1:
            theta_des = np.arctan2(ax_des, ay_total)
            theta_des = self.saturate(theta_des, -self.theta_max, self.theta_max)
        else:
            theta_des = 0.0
        
        # Store errors
        self.prev_error_x = e_x
        self.prev_error_y = e_y
        
        return u1_des, theta_des
    
    def attitude_controller_pid(self, state, dt, theta_ref):
        """Pure PID attitude controller"""
        x, x_dot, y, y_dot, theta, theta_dot = state
        
        # Attitude error
        e_theta = self.normalize_angle(theta_ref - theta)
        
        # Update integral with anti-windup
        self.integral_theta += e_theta * dt
        self.integral_theta = self.saturate(self.integral_theta, -0.1, 0.1)
        
        # Derivative
        de_theta = (e_theta - self.prev_error_theta) / dt if dt > 0 else 0
        
        # PID control
        u2 = (self.kp_att * e_theta + 
              self.ki_att * self.integral_theta + 
              self.kd_att * de_theta)
        
        u2 = self.saturate(u2, -self.u2_max, self.u2_max)
        
        # Store error
        self.prev_error_theta = e_theta
        
        return u2, 0.0  # Return 0 for sliding surface (not used)
    
    def attitude_controller_smc(self, state, dt, theta_ref):
        """SMC attitude controller with proper design"""
        x, x_dot, y, y_dot, theta, theta_dot = state
        
        # Attitude tracking errors
        e_theta = self.normalize_angle(theta_ref - theta)
        e_theta_dot = -theta_dot  # Desired angular velocity is 0
        
        # Sliding surface: s = ė + λe
        s = e_theta_dot + self.lambda_smc * e_theta
        
        # SMC control design
        # Equivalent control: maintains sliding motion once on surface
        de_theta = (e_theta - self.prev_error_theta) / dt if dt > 0 else 0
        u2_eq = -self.I * self.lambda_smc * de_theta  # Feedforward term
        
        # Switching control: ensures reaching the sliding surface
        if abs(s) > self.phi_smc:
            u2_sw = -self.k_smc * np.sign(s)
        else:
            # Boundary layer implementation to reduce chattering
            u2_sw = -self.k_smc * (s / self.phi_smc)
        
        # Total control (keep SMC contribution moderate)
        u2_smc_total = u2_eq + u2_sw
        
        # Add small PID component for better performance
        u2_pid = (0.5 * self.kp_att * e_theta + 
                  0.2 * self.ki_att * self.integral_theta + 
                  0.3 * self.kd_att * de_theta)
        
        # Update integral
        self.integral_theta += e_theta * dt
        self.integral_theta = self.saturate(self.integral_theta, -0.1, 0.1)
        
        # Combine SMC and PID
        u2 = u2_pid + 0.3 * u2_smc_total  # Scale down SMC contribution
        u2 = self.saturate(u2, -self.u2_max, self.u2_max)
        
        # Store error
        self.prev_error_theta = e_theta
        
        return u2, s
    
    def control_system(self, state, dt, x_ref, y_ref):
        """Complete control system"""
        # Outer loop: position control
        u1, theta_ref = self.position_controller(state, dt, x_ref, y_ref)
        
        # Inner loop: attitude control
        if self.use_smc:
            u2, sliding_surface = self.attitude_controller_smc(state, dt, theta_ref)
        else:
            u2, sliding_surface = self.attitude_controller_pid(state, dt, theta_ref)
        
        return u1, u2, theta_ref, sliding_surface
    
    def generate_disturbances(self, t):
        """Generate realistic time-varying disturbances"""
        # Multiple frequency components for realistic disturbances
        fx = (self.dist_force_amp * 
              (0.6 * np.sin(self.dist_freq[0] * t) + 
               0.3 * np.sin(self.dist_freq[1] * t + np.pi/3) +
               0.1 * np.sin(self.dist_freq[2] * t + np.pi/6)))
        
        fy = (self.dist_force_amp * 0.8 * 
              (0.5 * np.cos(self.dist_freq[0] * t + np.pi/4) + 
               0.4 * np.cos(self.dist_freq[1] * t) +
               0.1 * np.cos(self.dist_freq[2] * t + np.pi/2)))
        
        tau = (self.dist_torque_amp * 
               (0.7 * np.sin(2 * self.dist_freq[0] * t + np.pi/6) +
                0.3 * np.sin(self.dist_freq[1] * t + np.pi/3)))
        
        return np.array([fx, fy, tau])
    
    def rk4_step(self, state, dt, u1, u2, disturbances):
        """4th order Runge-Kutta integration"""
        k1 = self.dynamics(state, u1, u2, disturbances)
        k2 = self.dynamics(state + 0.5*dt*k1, u1, u2, disturbances)
        k3 = self.dynamics(state + 0.5*dt*k2, u1, u2, disturbances)
        k4 = self.dynamics(state + dt*k3, u1, u2, disturbances)
        return state + dt*(k1 + 2*k2 + 2*k3 + k4)/6
    
    def simulate(self, t_end=12.0, dt=0.01, x_target=0.2, y_target=0.3, 
                 include_disturbances=True):
        """Run simulation"""
        t = np.arange(0, t_end, dt)
        n_steps = len(t)
        
        # Initialize arrays
        states = np.zeros((n_steps, 6))
        controls = np.zeros((n_steps, 2))
        references = np.zeros(n_steps)
        sliding_surfaces = np.zeros(n_steps)
        disturbances = np.zeros((n_steps, 3))
        
        # Initial conditions - start with small offset
        states[0] = [-0.05, 0.0, -0.05, 0.0, 0.02, 0.0]  # More conservative start
        
        # Reset controller
        self.reset_controller_states()
        
        print(f"Simulating {self.controller_name} controller...")
        print(f"Target: ({x_target:.1f}, {y_target:.1f}) m")
        print(f"Disturbances: {'Enabled' if include_disturbances else 'Disabled'}")
        
        # Simulation loop
        for i in range(n_steps - 1):
            # Generate disturbances
            if include_disturbances:
                disturbances[i] = self.generate_disturbances(t[i])
            else:
                disturbances[i] = np.zeros(3)
            
            # Control system
            u1, u2, theta_ref, s = self.control_system(states[i], dt, x_target, y_target)
            
            controls[i] = [u1, u2]
            references[i] = theta_ref
            sliding_surfaces[i] = s
            
            # Integrate dynamics
            states[i+1] = self.rk4_step(states[i], dt, u1, u2, disturbances[i])
            
            # Safety check with more lenient bounds
            if (abs(states[i+1, 0]) > 1.5 or abs(states[i+1, 2]) > 1.5 or 
                abs(states[i+1, 4]) > np.pi/3):
                print(f"Simulation stopped at t={t[i]:.2f}s - safety bounds exceeded")
                # Pad arrays to maintain consistent size
                t = t[:i+2]
                states = states[:i+2]
                controls = controls[:i+2] 
                references = references[:i+2]
                sliding_surfaces = sliding_surfaces[:i+2]
                disturbances = disturbances[:i+2]
                break
                
        return t, states, controls, references, sliding_surfaces, disturbances

def compare_controllers():
    """Compare PID vs SMC performance"""
    
    # Simulation parameters
    sim_time = 12.0
    target_x, target_y = 0.2, 0.3
    
    print("=" * 60)
    print("COMPARING PID vs SMC CONTROL")
    print("=" * 60)
    
    # Test 1: PID only
    print("\n--- Testing PID Controller ---")
    quad_pid = Quad2D_Controller()
    quad_pid.use_smc = False
    quad_pid.controller_name = "PID"
    
    t_pid, states_pid, controls_pid, refs_pid, sliding_pid, dist_pid = quad_pid.simulate(
        t_end=sim_time, x_target=target_x, y_target=target_y, include_disturbances=True)
    
    # Calculate performance metrics for PID
    pos_error_pid = np.sqrt((states_pid[:, 0] - target_x)**2 + (states_pid[:, 2] - target_y)**2)
    final_error_pid = pos_error_pid[-1]
    # Use last 20% of simulation for steady-state analysis
    steady_state_idx = max(1, int(0.8 * len(pos_error_pid)))
    steady_state_error_pid = np.mean(pos_error_pid[steady_state_idx:])
    max_attitude_pid = np.max(np.abs(states_pid[:, 4]))
    control_effort_pid = np.mean(np.abs(controls_pid[:, 1]))
    
    print(f"PID Results:")
    print(f"  Simulation time: {t_pid[-1]:.2f}s")
    print(f"  Final position error: {final_error_pid:.4f} m")
    print(f"  Steady-state error: {steady_state_error_pid:.4f} m")
    print(f"  Max attitude: {np.degrees(max_attitude_pid):.1f}°")
    print(f"  Avg control effort: {control_effort_pid*1e6:.2f} μN⋅m")
    
    # Test 2: SMC
    print("\n--- Testing SMC Controller ---")
    quad_smc = Quad2D_Controller()
    quad_smc.use_smc = True
    quad_smc.controller_name = "SMC"
    
    t_smc, states_smc, controls_smc, refs_smc, sliding_smc, dist_smc = quad_smc.simulate(
        t_end=sim_time, x_target=target_x, y_target=target_y, include_disturbances=True)
    
    # Calculate performance metrics for SMC
    pos_error_smc = np.sqrt((states_smc[:, 0] - target_x)**2 + (states_smc[:, 2] - target_y)**2)
    final_error_smc = pos_error_smc[-1]
    steady_state_idx = max(1, int(0.8 * len(pos_error_smc)))
    steady_state_error_smc = np.mean(pos_error_smc[steady_state_idx:])
    max_attitude_smc = np.max(np.abs(states_smc[:, 4]))
    control_effort_smc = np.mean(np.abs(controls_smc[:, 1]))
    
    print(f"SMC Results:")
    print(f"  Simulation time: {t_smc[-1]:.2f}s")
    print(f"  Final position error: {final_error_smc:.4f} m")
    print(f"  Steady-state error: {steady_state_error_smc:.4f} m")
    print(f"  Max attitude: {np.degrees(max_attitude_smc):.1f}°")
    print(f"  Avg control effort: {control_effort_smc*1e6:.2f} μN⋅m")
    
    # Performance comparison
    print(f"\n--- Performance Comparison ---")
    if steady_state_error_pid > 0:
        improvement = ((steady_state_error_pid - steady_state_error_smc)/steady_state_error_pid)*100
        print(f"Position tracking improvement: {improvement:.1f}%")
    if control_effort_pid > 0:
        effort_change = ((control_effort_smc - control_effort_pid)/control_effort_pid)*100
        print(f"Control effort change: {effort_change:.1f}%")
    
    # Plotting - handle different array sizes
    plot_comparison(t_pid, t_smc, states_pid, states_smc, controls_pid, controls_smc, 
                   sliding_smc, dist_pid, target_x, target_y, pos_error_pid, pos_error_smc)

def plot_comparison(t_pid, t_smc, states_pid, states_smc, controls_pid, controls_smc, 
                   sliding_smc, disturbances, target_x, target_y, error_pid, error_smc):
    """Plot comparison results - handles different array lengths"""
    
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle('PID vs SMC Control Comparison', fontsize=16, fontweight='bold')
    
    # Position tracking
    axes[0,0].plot(t_pid, states_pid[:,0], 'b-', linewidth=2, label='PID')
    axes[0,0].plot(t_smc, states_smc[:,0], 'r-', linewidth=2, label='SMC')
    axes[0,0].axhline(y=target_x, color='k', linestyle='--', alpha=0.7, label='Target')
    axes[0,0].set_xlabel('Time (s)')
    axes[0,0].set_ylabel('X Position (m)')
    axes[0,0].set_title('X Position Tracking')
    axes[0,0].grid(True, alpha=0.3)
    axes[0,0].legend()
    
    axes[0,1].plot(t_pid, states_pid[:,2], 'b-', linewidth=2, label='PID')
    axes[0,1].plot(t_smc, states_smc[:,2], 'r-', linewidth=2, label='SMC')
    axes[0,1].axhline(y=target_y, color='k', linestyle='--', alpha=0.7, label='Target')
    axes[0,1].set_xlabel('Time (s)')
    axes[0,1].set_ylabel('Y Position (m)')
    axes[0,1].set_title('Y Position Tracking')
    axes[0,1].grid(True, alpha=0.3)
    axes[0,1].legend()
    
    # Attitude and control
    axes[1,0].plot(t_pid, np.degrees(states_pid[:,4]), 'b-', linewidth=2, label='PID')
    axes[1,0].plot(t_smc, np.degrees(states_smc[:,4]), 'r-', linewidth=2, label='SMC')
    axes[1,0].set_xlabel('Time (s)')
    axes[1,0].set_ylabel('Attitude (degrees)')
    axes[1,0].set_title('Attitude Response')
    axes[1,0].grid(True, alpha=0.3)
    axes[1,0].legend()
    
    axes[1,1].plot(t_pid, controls_pid[:,1]*1e6, 'b-', linewidth=2, label='PID')
    axes[1,1].plot(t_smc, controls_smc[:,1]*1e6, 'r-', linewidth=2, label='SMC')
    axes[1,1].set_xlabel('Time (s)')
    axes[1,1].set_ylabel('Torque (μN⋅m)')
    axes[1,1].set_title('Control Effort')
    axes[1,1].grid(True, alpha=0.3)
    axes[1,1].legend()
    
    # Error comparison and sliding surface
    axes[2,0].plot(t_pid, error_pid, 'b-', linewidth=2, label='PID Error')
    axes[2,0].plot(t_smc, error_smc, 'r-', linewidth=2, label='SMC Error')
    axes[2,0].set_xlabel('Time (s)')
    axes[2,0].set_ylabel('Position Error (m)')
    axes[2,0].set_title('Position Error Comparison')
    axes[2,0].grid(True, alpha=0.3)
    axes[2,0].legend()
    
    axes[2,1].plot(t_smc, sliding_smc, 'r-', linewidth=2, label='Sliding Surface')
    axes[2,1].axhline(y=0, color='k', linestyle='--', alpha=0.7)
    axes[2,1].axhline(y=0.1, color='gray', linestyle=':', alpha=0.7, label='Boundary ±0.1')
    axes[2,1].axhline(y=-0.1, color='gray', linestyle=':', alpha=0.7)
    axes[2,1].set_xlabel('Time (s)')
    axes[2,1].set_ylabel('Sliding Surface')
    axes[2,1].set_title('SMC Sliding Surface')
    axes[2,1].grid(True, alpha=0.3)
    axes[2,1].legend()
    
    plt.tight_layout()
    plt.show()
    
    # Trajectory comparison
    plt.figure(figsize=(10, 6))
    plt.plot(states_pid[:,0], states_pid[:,2], 'b-', linewidth=2, label='PID Trajectory', alpha=0.8)
    plt.plot(states_smc[:,0], states_smc[:,2], 'r-', linewidth=2, label='SMC Trajectory', alpha=0.8)
    
    # Start and end points
    plt.plot(states_pid[0,0], states_pid[0,2], 'go', markersize=8, label='Start')
    plt.plot(target_x, target_y, 'k*', markersize=15, label='Target')
    plt.plot(states_pid[-1,0], states_pid[-1,2], 'bo', markersize=8, label='PID Final')
    plt.plot(states_smc[-1,0], states_smc[-1,2], 'ro', markersize=8, label='SMC Final')
    
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('2D Trajectory Comparison')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.axis('equal')
    plt.show()

# Test with disturbance scenarios
def test_disturbance_scenarios():
    """Test both controllers under different disturbance levels"""
    print("\n" + "="*60)
    print("DISTURBANCE ROBUSTNESS TEST")
    print("="*60)
    
    disturbance_levels = [0.01, 0.02, 0.04]  # Different disturbance amplitudes
    
    for dist_level in disturbance_levels:
        print(f"\n--- Disturbance Level: {dist_level*1000:.0f} mN ---")
        
        # Test both controllers
        for use_smc in [False, True]:
            controller_name = "SMC" if use_smc else "PID"
            
            quad = Quad2D_Controller()
            quad.use_smc = use_smc
            quad.controller_name = controller_name
            quad.dist_force_amp = dist_level
            quad.dist_torque_amp = dist_level * 1e-4
            
            t, states, _, _, _, _ = quad.simulate(t_end=8.0, x_target=0.2, y_target=0.3)
            
            # Calculate final error
            pos_error = np.sqrt((states[:, 0] - 0.2)**2 + (states[:, 2] - 0.3)**2)
            final_error = pos_error[-1]
            
            print(f"  {controller_name}: Final error = {final_error:.4f} m, Sim time = {t[-1]:.1f}s")

# Run the comparison
if __name__ == "__main__":
    compare_controllers()
    test_disturbance_scenarios()