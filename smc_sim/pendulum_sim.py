import numpy as np
import matplotlib.pyplot as plt

#Pendulum dyn
m = 1.0      
l = 1.0      
b = 0.1      
g = 9.81     

# SMC parameters
lambda_param = 5.0 
k = 15.0           
phi = 0.1          

# Disturbance parameters
disturbance_amplitude = 2.0
disturbance_freq = 5.0


def sliding_mode_control(theta, theta_dot, t):
    """Sliding mode controller with boundary layer"""
    # Desired trajectory (upright position)
    theta_d = np.pi  # desired angle
    theta_d_dot = 0  # desired angular velocity
    
    # Tracking errors
    e = theta - theta_d
    e_dot = theta_dot - theta_d_dot
    
    # Sliding surface
    s = e_dot + lambda_param * e
    
    # Equivalent control (Slotine notation)
    u_eq = (m * l**2 * 
           (lambda_param * e_dot + 
            (g/l) * np.sin(theta) + 
            (b/(m*l**2)) * theta_dot))
    
    # Switching control with boundary layer (Slotine)
    if abs(s) > phi:
        u_sw = -k * np.sign(s)
    else:
        u_sw = -k * (s / phi)
    
    return u_eq + u_sw


def pendulum_dynamics(theta, theta_dot, t):
    """Calculate pendulum acceleration with SMC control"""
    # External disturbance
    disturbance = disturbance_amplitude * np.sin(disturbance_freq * t)
    
    # Control input
    u = sliding_mode_control(theta, theta_dot, t)
    
    # Pendulum equation: 
    theta_ddot = (-(g/l) * np.sin(theta) 
                 - (b/(m*l**2)) * theta_dot 
                 + (1/(m*l**2)) * u 
                 + disturbance/(m*l**2))
    
    return theta_ddot


# Simulation setup
t_start = 0
t_end = 5
dt = 0.01
t = np.arange(t_start, t_end, dt)
n_steps = len(t)

# Initial conditions
theta = np.zeros(n_steps)
theta_dot = np.zeros(n_steps)

theta[0] = np.pi + 0.5     # start
theta_dot[0] = 0.5  # small initial velocity

print("Simulating pendulum swing-up with sliding mode control...")
print(f"Using Euler method with dt = {dt} s")

# Euler integration
for i in range(n_steps - 1):
    theta_ddot = pendulum_dynamics(theta[i], theta_dot[i], t[i])
    
    # Euler integration step
    theta_dot[i+1] = theta_dot[i] + theta_ddot * dt
    theta[i+1] = theta[i] + theta_dot[i] * dt


# control inputs and sliding surface for each time step
# This is just for plotting, didn't save in the above integration
# its just bad code structure on my side...

control_inputs = np.zeros(n_steps)
sliding_surface = np.zeros(n_steps)

for i in range(n_steps):
    control_inputs[i] = sliding_mode_control(theta[i], theta_dot[i], t[i])
    
    # Sliding surface value
    e = theta[i] - np.pi
    e_dot = theta_dot[i]
    sliding_surface[i] = e_dot + lambda_param * e


#plots
fig, axes = plt.subplots(2, 2, figsize=(15, 10))
fig.suptitle('Sliding Mode Control of Pendulum', fontsize=16)

# Angle plot
axes[0,0].plot(t, theta, 'b-', linewidth=2, label='Actual theta')
axes[0,0].axhline(y=np.pi, color='r', linestyle='--', linewidth=2, label='Target theta = pi')
axes[0,0].set_xlabel('Time (s)')
axes[0,0].set_ylabel('Angle theta (rad)')
axes[0,0].set_title('Pendulum angle')
axes[0,0].grid(True)
axes[0,0].legend()

# Angular velocity plot
axes[0,1].plot(t, theta_dot, 'g-', linewidth=2)
axes[0,1].axhline(y=0, color='r', linestyle='--', linewidth=2, label='Target theta = theta')
axes[0,1].set_xlabel('Time (s)')
axes[0,1].set_ylabel('Angular celocity')
axes[0,1].set_title('Angular velocity')
axes[0,1].grid(True)
axes[0,1].legend()

# Control input plot
axes[1,0].plot(t, control_inputs, 'purple', linewidth=2)
axes[1,0].set_xlabel('Time (s)')
axes[1,0].set_ylabel('Control Input')
axes[1,0].set_title('Control Torque')
axes[1,0].grid(True)

# Sliding surface plot
axes[1,1].plot(t, sliding_surface, 'orange', linewidth=2)
axes[1,1].axhline(y=0, color='r', linestyle='--', alpha=0.7)
axes[1,1].set_xlabel('Time (s)')
axes[1,1].set_ylabel('Sliding Surface s')
axes[1,1].set_title('Sliding Surface')
axes[1,1].grid(True)

plt.tight_layout()
plt.show()