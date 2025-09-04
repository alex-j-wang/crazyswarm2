import numpy as np
import matplotlib.pyplot as plt

class CascadedPIDController:
    """
    Phase 1: Traditional cascaded PID control
    
    Architecture:
    Position PID (100 Hz) -> Attitude PID (500 Hz) -> Motors
    
    Goal: Establish solid baseline before adding SMC
    """
    
    def __init__(self):
        # Physical parameters
        self.m = 0.5
        self.g = 9.81
        self.I = 0.02
        self.b = 0.01
        
        # Position PID gains (outer loop, slower) - VERY CONSERVATIVE
        self.kp_pos = np.array([0.4, 1.0])
        self.ki_pos = np.array([0.0, 0.0])  # Much smaller integral gains
        self.kd_pos = np.array([1.0, 2.0])
        
        # Attitude PID gains (inner loop, faster)
        self.kp_att = 8.0
        self.ki_att = 0.5  # Reduced integral gain
        self.kd_att = 4.0
        
        # Control limits
        self.max_thrust = 10.0  # Reduced max thrust
        self.max_torque = 2.0
        self.max_tilt = np.radians(10)  # Reduced to 15 degrees
        
        self.reset_controller()
        
    def reset_controller(self):
        self.integral_pos = np.zeros(2)
        self.integral_att = 0.0
        self.prev_desired_pos = None  # Track reference changes
        
    def position_controller(self, pos, vel, desired_pos, desired_vel, desired_acc, dt):
        """Position PID controller (outer loop)"""
        
        # Reset integral if reference changed significantly  
        if self.prev_desired_pos is not None:
            pos_change = np.linalg.norm(desired_pos - self.prev_desired_pos)
            if pos_change > 0.5:  # Reference changed
                self.integral_pos = np.zeros(2)
                print(f"  Reference changed, resetting position integrals")
        self.prev_desired_pos = desired_pos.copy()
        
        # Tracking errors
        pos_error = pos - desired_pos
        vel_error = vel - desired_vel
        
        # Integral with anti-windup (smaller limits)
        self.integral_pos += pos_error * dt
        self.integral_pos = np.clip(self.integral_pos, -1.0, 1.0)
        
        # PID control law
        acc_cmd = (desired_acc + 
                  -self.kp_pos * pos_error + 
                  -self.ki_pos * self.integral_pos + 
                  -self.kd_pos * vel_error)
        
        # Convert acceleration commands to thrust and desired attitude
        u_x = self.m * acc_cmd[0]
        u_y = self.m * (acc_cmd[1] + self.g)  # Add gravity compensation
        
        # Thrust magnitude
        thrust = np.sqrt(u_x**2 + u_y**2)
        thrust = np.clip(thrust, 0.1, self.max_thrust)
        
        # Desired roll angle (2D case)
        if u_y > 0:
            desired_roll = np.arctan2(u_x, u_y)
        else:
            desired_roll = 0.0
            
        # Limit tilt angle for safety
        desired_roll = np.clip(desired_roll, -self.max_tilt, self.max_tilt)
        
        return thrust, desired_roll
        
    def attitude_controller(self, roll, roll_rate, desired_roll, dt):
        """Attitude PID controller (inner loop)"""
        
        # Tracking errors
        roll_error = roll - desired_roll
        roll_rate_error = roll_rate - 0.0  # Desired roll rate = 0
        
        # Integral with anti-windup
        self.integral_att += roll_error * dt
        self.integral_att = np.clip(self.integral_att, -1.0, 1.0)
        
        # PID control law - FIXED SIGNS!
        torque = (-self.kp_att * roll_error + 
                 -self.ki_att * self.integral_att + 
                 -self.kd_att * roll_rate_error)
        
        # Apply torque limits
        torque = np.clip(torque, -self.max_torque, self.max_torque)
        
        return torque
        
    def control_update(self, state, desired_pos, desired_vel, desired_acc, dt):
        """Main control update with debugging"""
        x, y, roll, x_dot, y_dot, roll_dot = state
        pos = np.array([x, y])
        vel = np.array([x_dot, y_dot])
        
        # Outer loop: Position control
        thrust, desired_roll = self.position_controller(
            pos, vel, desired_pos, desired_vel, desired_acc, dt)
        
        # Inner loop: Attitude control
        torque = self.attitude_controller(roll, roll_dot, desired_roll, dt)
        
        # Debug info every 50 steps (0.5 seconds)
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0
            
        if self._debug_counter % 50 == 0:
            pos_err = pos - desired_pos
            print(f"  Control debug: pos_err=({pos_err[0]:.2f},{pos_err[1]:.2f}), "
                  f"desired_roll={np.degrees(desired_roll):.1f}°, thrust={thrust:.2f}")
        
        return thrust, torque, desired_roll


def quadrotor_dynamics(state, t, controller, desired_traj_func, disturbance_func):
    """2D quadrotor dynamics - FIXED SIGNS"""
    x, y, roll, x_dot, y_dot, roll_dot = state
    
    # Get desired trajectory
    desired_pos, desired_vel, desired_acc = desired_traj_func(t)
    
    # Control update
    dt = 0.01
    thrust, torque, _ = controller.control_update(
        state, desired_pos, desired_vel, desired_acc, dt)
    
    # Use consistent limits with controller
    thrust = np.clip(thrust, 0.1, controller.max_thrust)
    torque = np.clip(torque, -controller.max_torque, controller.max_torque)
    
    # External disturbances
    disturbance = disturbance_func(t)
    
    # Quadrotor dynamics - CORRECTED SIGNS
    # Positive roll should give positive x acceleration
    x_ddot = (thrust / controller.m) * np.sin(roll) + disturbance[0] / controller.m
    y_ddot = (thrust / controller.m) * np.cos(roll) - controller.g + disturbance[1] / controller.m
    roll_ddot = torque / controller.I - controller.b * roll_dot / controller.I
    
    return np.array([x_dot, y_dot, roll_dot, x_ddot, y_ddot, roll_ddot])


def rk4_step(f, y, t, h, *args):
    """RK4 integration step"""
    k1 = h * f(y, t, *args)
    k2 = h * f(y + k1/2, t + h/2, *args)
    k3 = h * f(y + k2/2, t + h/2, *args)
    k4 = h * f(y + k3, t + h, *args)
    return y + (k1 + 2*k2 + 2*k3 + k4) / 6


def step_trajectory(t):
    """Step reference trajectory"""
    if t < 2:
        return np.array([0, 2]), np.array([0, 0]), np.array([0, 0])
    elif t < 4:
        return np.array([2, 2]), np.array([0, 0]), np.array([0, 0])
    elif t < 6:
        return np.array([2, 4]), np.array([0, 0]), np.array([0, 0])
    else:
        return np.array([0, 4]), np.array([0, 0]), np.array([0, 0])


def moderate_disturbance(t):
    """Very small disturbances for initial testing"""
    wind = 0.2 * np.array([np.sin(2*t), 0.2*np.cos(3*t)])  # Much smaller
    if 5 < t < 5.2:  # Small wind gust later in simulation
        wind += np.array([0.5, 0.3])
    return wind


# Simulation
print("Phase 1: Cascaded PID Controller Simulation")
print("Testing basic PID-PID architecture with VERY conservative gains...")
print("FIXED: Sign error in dynamics and reduced integral gains")

controller = CascadedPIDController()

# Print controller parameters
print(f"\nController Gains:")
print(f"Position: Kp={controller.kp_pos}, Ki={controller.ki_pos}, Kd={controller.kd_pos}")
print(f"Attitude: Kp={controller.kp_att}, Ki={controller.ki_att}, Kd={controller.kd_att}")
print(f"Max tilt: {np.degrees(controller.max_tilt):.1f}°, Max thrust: {controller.max_thrust}N")

# Simulation parameters
t_span = [0, 8]
dt = 0.01
t_array = np.arange(t_span[0], t_span[1] + dt, dt)
initial_state = [0, 0, 0, 0, 0, 0]

print(f"\nSimulation: {t_span[1]}s with dt={dt}s")
print(f"Desired trajectory: (0,2) -> (2,2) -> (2,4) -> (0,4)")
print(f"Step times: 0-2s, 2-4s, 4-6s, 6-8s")

# RK4 integration
y = np.array(initial_state, dtype=float)
states = np.zeros((len(t_array), 6))
control_data = np.zeros((len(t_array), 3))  # thrust, torque, desired_roll

states[0] = y

print(f"Starting simulation...")

for i in range(1, len(t_array)):
    y = rk4_step(quadrotor_dynamics, y, t_array[i-1], dt, 
                 controller, step_trajectory, moderate_disturbance)
    states[i] = y
    
    # Store control data
    desired_pos, desired_vel, desired_acc = step_trajectory(t_array[i])
    thrust, torque, desired_roll = controller.control_update(
        y, desired_pos, desired_vel, desired_acc, dt)
    control_data[i] = [thrust, torque, desired_roll]
    
    # Check for instability
    if i % 200 == 0:  # Print every 2 seconds
        roll_deg = np.degrees(y[2])
        desired_pos, _, _ = step_trajectory(t_array[i])
        print(f"t={t_array[i]:.1f}s: pos=({y[0]:.2f},{y[1]:.2f}), desired=({desired_pos[0]:.1f},{desired_pos[1]:.1f}), roll={roll_deg:.1f}°")
    
    if abs(y[2]) > np.radians(90):  # If roll > 90 degrees, something's wrong
        print(f"INSTABILITY DETECTED at t={t_array[i]:.2f}s: roll={np.degrees(y[2]):.1f}°")
        print(f"State: {y}")
        break

print(f"Simulation completed.")

# Extract results
x_traj = states[:, 0]
y_traj = states[:, 1]
roll_traj = states[:, 2]
x_dot_traj = states[:, 3]
y_dot_traj = states[:, 4]
roll_dot_traj = states[:, 5]

thrust_traj = control_data[:, 0]
torque_traj = control_data[:, 1]
desired_roll_traj = control_data[:, 2]

# Reference trajectory (same length as t_array)
ref_x, ref_y = [], []
for t in t_array:
    pos_ref, _, _ = step_trajectory(t)
    ref_x.append(pos_ref[0])
    ref_y.append(pos_ref[1])

ref_x = np.array(ref_x)
ref_y = np.array(ref_y)

# Ensure same lengths
if len(x_traj) != len(ref_x):
    min_len = min(len(x_traj), len(ref_x))
    x_traj = x_traj[:min_len]
    y_traj = y_traj[:min_len] 
    ref_x = ref_x[:min_len]
    ref_y = ref_y[:min_len]
    t_array = t_array[:min_len]

# Calculate performance metrics
x_error = x_traj - ref_x
y_error = y_traj - ref_y
rmse_x = np.sqrt(np.mean(x_error**2))
rmse_y = np.sqrt(np.mean(y_error**2))
max_roll = np.max(np.abs(np.degrees(roll_traj)))
max_thrust = np.max(thrust_traj)

# Plotting
fig, axes = plt.subplots(2, 2, figsize=(15, 10))
fig.suptitle('Phase 1: Cascaded PID Controller', fontsize=16)

# Position tracking
axes[0,0].plot(t_array, x_traj, 'b-', linewidth=2, label='Actual X')
axes[0,0].plot(t_array, ref_x, 'r--', linewidth=2, label='Desired X')
axes[0,0].plot(t_array, y_traj, 'g-', linewidth=2, label='Actual Y')
axes[0,0].plot(t_array, ref_y, 'm--', linewidth=2, label='Desired Y')
axes[0,0].set_xlabel('Time (s)')
axes[0,0].set_ylabel('Position (m)')
axes[0,0].set_title('Position Tracking')
axes[0,0].grid(True)
axes[0,0].legend()

# Velocity
axes[0,1].plot(t_array, x_dot_traj, 'b-', linewidth=2, label='X velocity')
axes[0,1].plot(t_array, y_dot_traj, 'g-', linewidth=2, label='Y velocity')
axes[0,1].set_xlabel('Time (s)')
axes[0,1].set_ylabel('Velocity (m/s)')
axes[0,1].set_title('Velocity')
axes[0,1].grid(True)
axes[0,1].legend()

# Control inputs
axes[1,0].plot(t_array, thrust_traj, 'purple', linewidth=2, label='Thrust')
axes[1,0].plot(t_array, torque_traj * 3, 'orange', linewidth=2, label='Torque × 3')
axes[1,0].set_xlabel('Time (s)')
axes[1,0].set_ylabel('Control Input')
axes[1,0].set_title('Control Inputs')
axes[1,0].grid(True)
axes[1,0].legend()

# Attitude tracking
axes[1,1].plot(t_array, np.degrees(roll_traj), 'b-', linewidth=2, label='Actual Roll')
axes[1,1].plot(t_array, np.degrees(desired_roll_traj), 'r--', linewidth=2, label='Desired Roll')
axes[1,1].set_xlabel('Time (s)')
axes[1,1].set_ylabel('Roll Angle (degrees)')
axes[1,1].set_title('Attitude Control')
axes[1,1].grid(True)
axes[1,1].legend()

plt.tight_layout()
plt.show()

# Performance summary
print(f"\nPhase 1 Performance Results:")
print(f"X Position RMSE: {rmse_x:.4f} m")
print(f"Y Position RMSE: {rmse_y:.4f} m")
print(f"Maximum Roll Angle: {max_roll:.2f}°")
print(f"Maximum Thrust: {max_thrust:.2f} N")

print(f"\nController Parameters:")
print(f"Position PID gains: Kp={controller.kp_pos}, Ki={controller.ki_pos}, Kd={controller.kd_pos}")
print(f"Attitude PID gains: Kp={controller.kp_att:.1f}, Ki={controller.ki_att:.1f}, Kd={controller.kd_att:.1f}")

print(f"\nNext Steps:")
print(f"1. Tune attitude controller first (most critical)")
print(f"2. Then tune position controller")
print(f"3. Test with different trajectories and disturbances")
print(f"4. Once stable, proceed to Phase 2 (add SMC to attitude loop)")