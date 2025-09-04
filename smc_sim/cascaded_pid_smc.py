import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

class PIDController:
    """Well-tuned PID Controller (your baseline)"""
    def __init__(self, kp, ki, kd, dt, output_limits=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.output_limits = output_limits
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.first_call = True
        
    def update(self, setpoint, measurement):
        error = setpoint - measurement
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with windup protection
        self.integral += error * self.dt
        if self.output_limits:
            max_integral = self.output_limits[1] / (self.ki + 1e-10)
            min_integral = self.output_limits[0] / (self.ki + 1e-10)
            self.integral = np.clip(self.integral, min_integral, max_integral)
        i_term = self.ki * self.integral
        
        # Derivative term
        if self.first_call:
            d_term = 0.0
            self.first_call = False
        else:
            d_term = self.kd * (error - self.prev_error) / self.dt
        
        # Total output
        output = p_term + i_term + d_term
        if self.output_limits:
            output = np.clip(output, self.output_limits[0], self.output_limits[1])
        
        self.prev_error = error
        return output

class SlidingModeController:
    """Simple SMC for robustness - FIXED PARAMETERS"""
    def __init__(self, lambda_param=3.0, eta=0.5, phi=0.1):  # INCREASED eta from 0.15 to 0.5
        self.lambda_param = lambda_param  # Sliding surface slope
        self.eta = eta                   # Switching gain (MAIN FIX!)
        self.phi = phi                   # Boundary layer thickness
        
    def sliding_surface(self, error, error_dot):
        """Define sliding surface s = error_dot + lambda * error"""
        return error_dot + self.lambda_param * error
    
    def switching_function(self, s):
        """Smooth switching function"""
        if abs(s) <= self.phi:
            return s / self.phi
        else:
            return np.sign(s)
    
    def control_law(self, s, uncertainty_bound=1.0):
        """Simple SMC control law"""
        return -self.eta * uncertainty_bound * self.switching_function(s)

class Quadrotor2D:
    """2D Quadrotor with uncertainties and disturbances"""
    def __init__(self, mass=0.5, length=0.15, inertia=0.0075, g=9.81):
        # Nominal parameters
        self.mass_nominal = mass
        self.length_nominal = length  
        self.inertia_nominal = inertia
        self.g = g
        
        # Actual parameters (with uncertainty)
        self.mass = mass
        self.length = length
        self.inertia = inertia
        
    def set_uncertainty(self, mass_factor=1.0, inertia_factor=1.0):
        """Introduce parameter uncertainty"""
        self.mass = self.mass_nominal * mass_factor
        self.inertia = self.inertia_nominal * inertia_factor
        
    def dynamics(self, state, t, thrust, torque, disturbance=None):
        """
        State: [x, z, theta, x_dot, z_dot, theta_dot]
        Inputs: thrust (N), torque (N⋅m)
        """
        x, z, theta, x_dot, z_dot, theta_dot = state
        
        # Add disturbances if provided
        if disturbance is not None:
            thrust += disturbance[0]
            torque += disturbance[1]
        
        # Dynamics equations (with actual parameters)
        x_ddot = -(thrust/self.mass) * np.sin(theta)
        z_ddot = (thrust/self.mass) * np.cos(theta) - self.g
        theta_ddot = torque / self.inertia
        
        return [x_dot, z_dot, theta_dot, x_ddot, z_ddot, theta_ddot]

class SimpleRobustController:
    """Simple PID + SMC: PID stabilizes, SMC adds robustness"""
    def __init__(self, dt):
        self.dt = dt
        
        # IMPROVED PID TUNING (main fix for tracking performance)
        self.x_pid = PIDController(
            kp=3.5, ki=0.3, kd=2.2, dt=dt,  # Increased gains
            output_limits=(-0.4, 0.4)      # Increased limits
        )
        
        self.z_pid = PIDController(
            kp=8.5, ki=1.0, kd=3.5, dt=dt,  # Increased gains
            output_limits=(1.0, 12.0)        # Increased upper limit
        )
        
        self.theta_pid = PIDController(
            kp=12.0, ki=0.8, kd=1.8, dt=dt,  # Increased gains
            output_limits=(-8.0, 8.0)        # Increased limits
        )
        
        self.theta_dot_pid = PIDController(
            kp=0.18, ki=0.02, kd=0.008, dt=dt,  # Increased gains
            output_limits=(-1.0, 1.0)         # Increased limits
        )
        
        # LESS CONSERVATIVE SMC (main fix for robustness)
        self.x_smc = SlidingModeController(lambda_param=2.0, eta=0.6, phi=0.15)     # eta: 0.1 -> 0.8
        self.z_smc = SlidingModeController(lambda_param=1.5, eta=0.8, phi=0.15)     # eta: 0.15 -> 1.0  
        self.theta_smc = SlidingModeController(lambda_param=2.5, eta=0.25, phi=0.1)  # eta: 0.05 -> 0.3
        
        # Simple derivative calculation with filtering
        self.prev_state = None
        self.prev_x_dot = 0.0
        self.prev_z_dot = 0.0
        self.prev_theta_dot = 0.0
        
    def simple_filter(self, new_value, old_value, alpha=0.7):
        """Simple exponential filter to reduce derivative noise"""
        return alpha * new_value + (1 - alpha) * old_value
        
    def control(self, state, setpoint, enable_smc=True):
        x, z, theta, x_dot, z_dot, theta_dot = state
        
        # Filter velocities to reduce noise
        x_dot = self.simple_filter(x_dot, self.prev_x_dot)
        z_dot = self.simple_filter(z_dot, self.prev_z_dot)  
        theta_dot = self.simple_filter(theta_dot, self.prev_theta_dot)
        
        self.prev_x_dot = x_dot
        self.prev_z_dot = z_dot
        self.prev_theta_dot = theta_dot
        
        # === STEP 1: PID CONTROL (for stabilization) ===
        theta_desired = -self.x_pid.update(setpoint[0], x)
        thrust_pid = self.z_pid.update(setpoint[1], z)
        theta_dot_desired = self.theta_pid.update(theta_desired, theta)
        torque_pid = self.theta_dot_pid.update(theta_dot_desired, theta_dot)
        
        # === STEP 2: SMC ROBUSTNESS (for disturbance rejection) ===
        if enable_smc:
            # X position robustness
            x_error = setpoint[0] - x
            s_x = self.x_smc.sliding_surface(x_error, -x_dot)
            theta_smc_correction = self.x_smc.control_law(s_x, uncertainty_bound=1.0)
            theta_desired += theta_smc_correction
            
            # Z position robustness  
            z_error = setpoint[1] - z
            s_z = self.z_smc.sliding_surface(z_error, -z_dot)
            thrust_smc_correction = self.z_smc.control_law(s_z, uncertainty_bound=2.5)
            thrust_total = thrust_pid + thrust_smc_correction
            
            # Attitude robustness
            theta_error = theta_desired - theta
            s_theta = self.theta_smc.sliding_surface(theta_error, -theta_dot)
            torque_smc_correction = self.theta_smc.control_law(s_theta, uncertainty_bound=0.8)
            torque_total = torque_pid + torque_smc_correction
            
        else:
            # PID only (baseline)
            thrust_total = thrust_pid
            torque_total = torque_pid
        
        # Final safety limits
        thrust_total = np.clip(thrust_total, 0.5, 15.0)
        torque_total = np.clip(torque_total, -2.0, 2.0)
        
        self.prev_state = state.copy()
        
        return thrust_total, torque_total, theta_desired

def generate_disturbance(t, disturbance_type='wind'):
    """Generate realistic disturbances"""
    if disturbance_type == 'wind':
        # Continuous wind disturbance
        wind_force = 0.8 * np.sin(0.5 * t) + 0.4 * np.sin(1.2 * t)
        wind_torque = 0.2 * np.sin(0.8 * t)
        return [wind_force, wind_torque]
    elif disturbance_type == 'gust':
        # Wind gusts at specific times
        if 3.0 <= t <= 3.5:
            return [3.0, 0.3]  
        elif 6.0 <= t <= 6.3:
            return [-2.5, -0.25] 
        elif 9.0 <= t <= 9.2:
            return [2.0, 0.4] 
        else:
            return [0.0, 0.0]
    else:
        return [0.0, 0.0]

def run_simple_improved_simulation():
    """Run simple improved simulation"""
    
    dt = 0.01
    t_end = 12.0
    steps = int(t_end / dt)
    time_vec = np.linspace(0, t_end, steps)
    
    # Initialize quadrotors with uncertainty
    quad_pid = Quadrotor2D()
    quad_robust = Quadrotor2D()
    
    # SIGNIFICANT uncertainty to test robustness
    quad_pid.set_uncertainty(mass_factor=1.6, inertia_factor=0.6)    # 60% mass up, 40% inertia down
    quad_robust.set_uncertainty(mass_factor=1.6, inertia_factor=0.6)
    
    # Controllers
    controller_pid = SimpleRobustController(dt)
    controller_robust = SimpleRobustController(dt)
    
    # Initial states
    state_pid = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    state_robust = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    # Storage
    states_pid = np.zeros((steps, 6))
    states_robust = np.zeros((steps, 6))
    controls_pid = np.zeros((steps, 2))
    controls_robust = np.zeros((steps, 2))
    references = np.zeros((steps, 2))
    disturbances = np.zeros((steps, 2))
    
    print("Running Simple Improved Control Comparison...")
    print("Key fixes: Better PID tuning + Less conservative SMC")
    
    for i in range(steps):
        t = time_vec[i]
        
        # Reference trajectory
        if t < 2.0:
            ref = [0.0, 1.0]
        elif t < 5.0:
            ref = [2.5, 1.0]  
        elif t < 8.0:
            ref = [2.5, 2.5]
        else:
            ref = [0.5 * np.sin(0.4 * t), 1.5 + 0.5 * np.cos(0.3 * t)]
        
        # Generate disturbance
        disturbance = generate_disturbance(t, 'gust')  # Try 'wind' for continuous
        
        # Control - PID only (baseline)
        thrust_pid, torque_pid, _ = controller_pid.control(state_pid, ref, enable_smc=False)
        
        # Control - PID + SMC (robust)
        thrust_robust, torque_robust, _ = controller_robust.control(state_robust, ref, enable_smc=True)
        
        # Simulate dynamics
        state_pid_new = odeint(quad_pid.dynamics, state_pid, [t, t+dt], 
                              args=(thrust_pid, torque_pid, disturbance))
        state_pid = state_pid_new[-1]
        
        state_robust_new = odeint(quad_robust.dynamics, state_robust, [t, t+dt],
                                 args=(thrust_robust, torque_robust, disturbance))
        state_robust = state_robust_new[-1]
        
        # Store data
        states_pid[i] = state_pid
        states_robust[i] = state_robust
        controls_pid[i] = [thrust_pid, torque_pid]
        controls_robust[i] = [thrust_robust, torque_robust]
        references[i] = ref
        disturbances[i] = disturbance
        
        # Progress
        if i % (steps//10) == 0:
            print(f"t={t:.1f}s - PID: x={state_pid[0]:.2f}, z={state_pid[1]:.2f}")
            print(f"      - SMC: x={state_robust[0]:.2f}, z={state_robust[1]:.2f}")
    
    return time_vec, states_pid, states_robust, controls_pid, controls_robust, references, disturbances

# Run simulation with simple fixes
time_vec, states_pid, states_robust, controls_pid, controls_robust, refs, dist = run_simple_improved_simulation()

# Calculate performance metrics
pid_error_x = np.mean(np.abs(refs[:, 0] - states_pid[:, 0]))
pid_error_z = np.mean(np.abs(refs[:, 1] - states_pid[:, 1]))
robust_error_x = np.mean(np.abs(refs[:, 0] - states_robust[:, 0]))
robust_error_z = np.mean(np.abs(refs[:, 1] - states_robust[:, 1]))

print(f"\n{'='*70}")
print("SIMPLE PERFORMANCE IMPROVEMENT")
print(f"{'='*70}")
print(f"Mean Absolute Error (60% mass + 40% inertia uncertainty):")
print(f"  PID Only:")
print(f"    X position: {pid_error_x:.3f} m")
print(f"    Z position: {pid_error_z:.3f} m")
print(f"  PID + SMC:")
print(f"    X position: {robust_error_x:.3f} m") 
print(f"    Z position: {robust_error_z:.3f} m")
print(f"  Improvement:")
print(f"    X: {((pid_error_x - robust_error_x)/pid_error_x*100):.1f}%")
print(f"    Z: {((pid_error_z - robust_error_z)/pid_error_z*100):.1f}%")

# Simple, clear plotting
fig, axes = plt.subplots(2, 3, figsize=(15, 10))
fig.suptitle('Simple PID+SMC Improvement: Better Tuning + Less Conservative SMC', fontsize=14)

# Position tracking
axes[0,0].plot(time_vec, states_pid[:,0], 'b-', linewidth=2, label='PID Only')
axes[0,0].plot(time_vec, states_robust[:,0], 'r-', linewidth=2, label='PID + SMC')
axes[0,0].plot(time_vec, refs[:,0], 'k--', linewidth=1, alpha=0.7, label='Reference')
axes[0,0].set_ylabel('X Position (m)')
axes[0,0].legend()
axes[0,0].grid(True)
axes[0,0].set_title('X Position Tracking')

axes[0,1].plot(time_vec, states_pid[:,1], 'b-', linewidth=2, label='PID Only')
axes[0,1].plot(time_vec, states_robust[:,1], 'r-', linewidth=2, label='PID + SMC')
axes[0,1].plot(time_vec, refs[:,1], 'k--', linewidth=1, alpha=0.7, label='Reference')
axes[0,1].set_ylabel('Z Position (m)')
axes[0,1].legend()
axes[0,1].grid(True)
axes[0,1].set_title('Z Position Tracking')

# Attitude
axes[0,2].plot(time_vec, np.degrees(states_pid[:,2]), 'b-', linewidth=2, label='PID Only')
axes[0,2].plot(time_vec, np.degrees(states_robust[:,2]), 'r-', linewidth=2, label='PID + SMC')
axes[0,2].set_ylabel('Pitch Angle (deg)')
axes[0,2].legend()
axes[0,2].grid(True)
axes[0,2].set_title('Attitude Response')

# Tracking errors
error_pid_x = refs[:,0] - states_pid[:,0]
error_robust_x = refs[:,0] - states_robust[:,0]
error_pid_z = refs[:,1] - states_pid[:,1]
error_robust_z = refs[:,1] - states_robust[:,1]

axes[1,0].plot(time_vec, error_pid_x, 'b-', linewidth=2, label='PID X Error')
axes[1,0].plot(time_vec, error_robust_x, 'r-', linewidth=2, label='SMC X Error')
axes[1,0].axhline(y=0, color='k', linestyle=':', alpha=0.5)
axes[1,0].set_ylabel('X Error (m)')
axes[1,0].set_xlabel('Time (s)')
axes[1,0].legend()
axes[1,0].grid(True)
axes[1,0].set_title('X Position Error')

axes[1,1].plot(time_vec, error_pid_z, 'b-', linewidth=2, label='PID Z Error')
axes[1,1].plot(time_vec, error_robust_z, 'r-', linewidth=2, label='SMC Z Error')
axes[1,1].axhline(y=0, color='k', linestyle=':', alpha=0.5)
axes[1,1].set_ylabel('Z Error (m)')
axes[1,1].set_xlabel('Time (s)')
axes[1,1].legend()
axes[1,1].grid(True)
axes[1,1].set_title('Z Position Error')

# SMC benefit (what SMC adds to PID)
thrust_benefit = controls_robust[:,0] - controls_pid[:,0]
torque_benefit = controls_robust[:,1] - controls_pid[:,1]

axes[1,2].plot(time_vec, thrust_benefit, 'g-', linewidth=2, label='Thrust SMC Addition')
axes[1,2].plot(time_vec, torque_benefit, 'm-', linewidth=2, label='Torque SMC Addition')
axes[1,2].plot(time_vec, dist[:,0], 'orange', linewidth=1, alpha=0.7, label='Force Disturbance')
axes[1,2].set_ylabel('Control Addition')
axes[1,2].set_xlabel('Time (s)')
axes[1,2].legend()
axes[1,2].grid(True)
axes[1,2].set_title('SMC Robustness Benefit')

plt.tight_layout()
plt.show()

# Trajectory comparison
plt.figure(figsize=(10, 8))
plt.plot(states_pid[:,0], states_pid[:,1], 'b-', linewidth=3, alpha=0.8, label='PID Only')
plt.plot(states_robust[:,0], states_robust[:,1], 'r-', linewidth=3, alpha=0.8, label='PID + SMC')
plt.plot(refs[:,0], refs[:,1], 'k--', linewidth=2, alpha=0.7, label='Reference')
plt.scatter(0, 0, color='green', s=100, marker='o', label='Start')
plt.xlabel('X Position (m)')
plt.ylabel('Z Position (m)')
plt.title('Simple Fix: PID Stabilizes, SMC Adds Robustness')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
