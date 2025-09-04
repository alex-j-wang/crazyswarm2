import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import time

class PIDController:
    """Simple PID Controller with basic protection"""
    def __init__(self, kp, ki, kd, dt, output_limits=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.output_limits = output_limits
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0
        self.first_call = True
        
    def update(self, setpoint, measurement):
        error = setpoint - measurement
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * self.dt
        # Simple integral windup protection
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
        
        # Apply output limits
        if self.output_limits:
            output = np.clip(output, self.output_limits[0], self.output_limits[1])
        
        self.prev_error = error
        self.prev_measurement = measurement
        
        return output

class Quadrotor2D:
    """2D Quadrotor dynamics"""
    def __init__(self, mass=0.5, length=0.15, inertia=0.0075, g=9.81):
        self.mass = mass
        self.length = length  
        self.inertia = inertia
        self.g = g
        
    def dynamics(self, state, t, thrust, torque):
        """
        State: [x, z, theta, x_dot, z_dot, theta_dot]
        Inputs: thrust (N), torque (N⋅m)
        """
        x, z, theta, x_dot, z_dot, theta_dot = state
        
        # Dynamics equations
        x_ddot = -(thrust/self.mass) * np.sin(theta)
        z_ddot = (thrust/self.mass) * np.cos(theta) - self.g
        theta_ddot = torque / self.inertia
        
        return [x_dot, z_dot, theta_dot, x_ddot, z_ddot, theta_ddot]

class SimpleController:
    """Simple cascaded controller with conservative gains"""
    def __init__(self, dt):
        self.dt = dt
        self.quad = Quadrotor2D()
        
        # VERY CONSERVATIVE GAINS - start small!
        self.x_controller = PIDController(
            kp=2, ki=0.1, kd=1.5, dt=dt, 
            output_limits=(-0.2, 0.2)  # Max ±11° pitch
        )
        
        self.z_controller = PIDController(
            kp=6.5, ki=0.5, kd=2.5, dt=dt,
            output_limits=(2.0, 8.0)  # Thrust limits
        )
        
        self.theta_controller = PIDController(
            kp=9.0, ki=0.5, kd=1.0, dt=dt,
            output_limits=(-5.0, 5.0)  # Angular velocity limits
        )
        
        self.theta_dot_controller = PIDController(
            kp=0.1, ki=0.01, kd=0.005, dt=dt,
            output_limits=(-0.5, 0.5)  # Torque limits
        )
        
    def control(self, state, setpoint):
        x, z, theta, x_dot, z_dot, theta_dot = state
        
        # Position to attitude
        theta_desired = -self.x_controller.update(setpoint[0], x)
        
        # Altitude control  
        thrust = self.z_controller.update(setpoint[1], z)
        
        # Attitude control
        theta_dot_desired = self.theta_controller.update(theta_desired, theta)
        
        # Angular rate control
        torque = self.theta_dot_controller.update(theta_dot_desired, theta_dot)
        
        return thrust, torque, theta_desired

def run_simulation():
    """Run simulation with debugging"""
    
    # Simulation setup
    dt = 0.01  # Larger time step for stability
    t_end = 10.0
    steps = int(t_end / dt)
    
    # Initialize
    controller = SimpleController(dt)
    state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Start at origin
    
    # Storage
    time_vec = np.linspace(0, t_end, steps)
    states = np.zeros((steps, 6))
    controls = np.zeros((steps, 2))
    references = np.zeros((steps, 2))
    theta_des = np.zeros(steps)
    
    # Check initial hover
    print("Testing initial hover...")
    hover_thrust = controller.quad.mass * controller.quad.g
    print(f"Hover thrust should be: {hover_thrust:.2f} N")
    
    # Main simulation loop
    for i in range(steps):
        t = time_vec[i]
        
        # Simple step reference
        if t < 3.0:
            ref = [0.0, 1.0]  # Hover at 1m
        elif t < 6.0:
            ref = [1.0, 1.0]  # Move to x=1m
        else:
            ref = [1.0, 1.5]  # Move to z=1.5m
            
        # Control
        thrust, torque, theta_desired = controller.control(state, ref)
        
        # Limit checking
        if abs(thrust) > 20 or abs(torque) > 2:
            print(f"WARNING: Large control at t={t:.2f}: thrust={thrust:.2f}, torque={torque:.3f}")
            thrust = np.clip(thrust, 0, 15)
            torque = np.clip(torque, -1, 1)
        
        # Integrate dynamics
        try:
            state_new = odeint(controller.quad.dynamics, state, [t, t+dt], args=(thrust, torque))
            state = state_new[-1]
            
            # Sanity check
            if np.any(np.abs(state) > 100):
                print(f"UNSTABLE at t={t:.2f}: state = {state}")
                break
                
        except Exception as e:
            print(f"Integration failed at t={t:.2f}: {e}")
            break
            
        # Store data
        states[i] = state
        controls[i] = [thrust, torque]
        references[i] = ref
        theta_des[i] = theta_desired
        
        # Progress
        if i % (steps//10) == 0:
            print(f"t={t:.1f}s: x={state[0]:.2f}, z={state[1]:.2f}, θ={np.degrees(state[2]):.1f}°")
    
    return time_vec, states, controls, references, theta_des

# Run simulation
print("Starting Conservative 2D Quadrotor Simulation")
print("=" * 50)

time_vec, states, controls, refs, theta_des = run_simulation()

# Analyze results
print("\n" + "=" * 50)
print("SIMULATION RESULTS")
print("=" * 50)

final_state = states[-1]
final_ref = refs[-1]

print(f"Final state: x={final_state[0]:.3f}m, z={final_state[1]:.3f}m, θ={np.degrees(final_state[2]):.1f}°")
print(f"Final reference: x={final_ref[0]:.1f}m, z={final_ref[1]:.1f}m")
print(f"Final errors: Δx={final_ref[0]-final_state[0]:.3f}m, Δz={final_ref[1]-final_state[1]:.3f}m")

# Check stability
max_pos = np.max(np.abs(states[:, 0:2]))
max_angle = np.max(np.abs(states[:, 2]))
print(f"Max position deviation: {max_pos:.2f}m")
print(f"Max attitude deviation: {np.degrees(max_angle):.1f}°")

if max_pos < 10 and max_angle < 1.0:  # Reasonable bounds
    print("✓ SIMULATION STABLE!")
else:
    print("✗ SIMULATION UNSTABLE!")

# Plot results
fig, axes = plt.subplots(3, 2, figsize=(14, 10))
fig.suptitle('2D Quadrotor - Debug Version', fontsize=16)

# Positions
axes[0,0].plot(time_vec, states[:,0], 'b-', linewidth=2, label='Actual')
axes[0,0].plot(time_vec, refs[:,0], 'r--', linewidth=2, label='Reference')
axes[0,0].set_ylabel('X Position (m)')
axes[0,0].legend()
axes[0,0].grid(True)
axes[0,0].set_title('X Position')

axes[0,1].plot(time_vec, states[:,1], 'b-', linewidth=2, label='Actual')
axes[0,1].plot(time_vec, refs[:,1], 'r--', linewidth=2, label='Reference')
axes[0,1].set_ylabel('Z Position (m)')
axes[0,1].legend()
axes[0,1].grid(True)
axes[0,1].set_title('Z Position')

# Attitude
axes[1,0].plot(time_vec, np.degrees(states[:,2]), 'b-', linewidth=2, label='Actual')
axes[1,0].plot(time_vec, np.degrees(theta_des), 'g--', linewidth=2, label='Desired')
axes[1,0].set_ylabel('Pitch Angle (deg)')
axes[1,0].legend()
axes[1,0].grid(True)
axes[1,0].set_title('Pitch Angle')

axes[1,1].plot(time_vec, np.degrees(states[:,5]), 'b-', linewidth=2)
axes[1,1].set_ylabel('Pitch Rate (deg/s)')
axes[1,1].grid(True)
axes[1,1].set_title('Pitch Rate')

# Controls
axes[2,0].plot(time_vec, controls[:,0], 'b-', linewidth=2)
axes[2,0].axhline(y=0.5*9.81, color='r', linestyle=':', alpha=0.7, label='Hover')
axes[2,0].set_ylabel('Thrust (N)')
axes[2,0].set_xlabel('Time (s)')
axes[2,0].legend()
axes[2,0].grid(True)
axes[2,0].set_title('Thrust Command')

axes[2,1].plot(time_vec, controls[:,1], 'b-', linewidth=2)
axes[2,1].set_ylabel('Torque (N⋅m)')
axes[2,1].set_xlabel('Time (s)')
axes[2,1].grid(True)
axes[2,1].set_title('Torque Command')

plt.tight_layout()
plt.show()

# Trajectory plot
plt.figure(figsize=(10, 6))
plt.plot(states[:,0], states[:,1], 'b-', linewidth=3, label='Actual trajectory')
plt.plot(refs[:,0], refs[:,1], 'r--', linewidth=2, label='Reference')
plt.scatter(states[0,0], states[0,1], color='green', s=100, label='Start')
plt.scatter(states[-1,0], states[-1,1], color='red', s=100, label='End')
plt.xlabel('X Position (m)')
plt.ylabel('Z Position (m)')
plt.title('2D Quadrotor Trajectory')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()

print("\nSimulation complete!")

# Diagnostic information
print("\n" + "=" * 50) 
print("DIAGNOSTIC INFO")
print("=" * 50)
print(f"Control range - Thrust: {np.min(controls[:,0]):.2f} to {np.max(controls[:,0]):.2f} N")
print(f"Control range - Torque: {np.min(controls[:,1]):.3f} to {np.max(controls[:,1]):.3f} N⋅m")
print(f"State range - Position: {np.min(states[:,0:2]):.2f} to {np.max(states[:,0:2]):.2f} m") 
print(f"State range - Attitude: {np.degrees(np.min(states[:,2])):.1f} to {np.degrees(np.max(states[:,2])):.1f} deg")

# Check for common issues
if np.any(np.isnan(states)) or np.any(np.isinf(states)):
    print("⚠️  WARNING: NaN or Inf values detected in simulation!")

if np.max(np.abs(controls)) > 50:
    print("⚠️  WARNING: Very large control inputs detected!")

if np.std(states[-100:, 0]) > 0.5:  # Check if still oscillating at end
    print("⚠️  WARNING: System may be oscillatory or unstable!")