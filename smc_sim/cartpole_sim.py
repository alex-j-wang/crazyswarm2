import numpy as np
import matplotlib.pyplot as plt

class CartpoleSMC:
    def __init__(self):
        # Cartpole params
        self.M = 1.0      # Cart mass (kg)
        self.m = 0.1      # Pole mass (kg)
        self.l = 0.5      # Pole length (m)
        self.g = 9.81     # Gravity (m/s^2)
        
        # SMC params
        self.lambda_param = 0.3 
        self.k = 8           
        self.phi = 0.65         
        
        # Disturbance params
        self.disturbance_amplitude = 2.0
        self.disturbance_freq = 5.0
    
    def dynamics(self, state, t, control_force, disturbances):
        """
        Cartpole dynamics in state space form
        state = [x, x_dot, theta, theta_dot]
        returns state_dot = [x_dot, x_ddot, theta_dot, theta_ddot]
        """
        x, x_dot, theta, theta_dot = state
        
        # Adding external disturbances
        f = control_force + disturbances
        
        #same denom
        denom = self.M + self.m * np.sin(theta)**2
        
        # Cart acceleration
        x_ddot = (f + self.m * np.sin(theta) * (self.l * theta_dot**2 - self.g * np.cos(theta))) / denom
        
        #pole angular acceleration
        theta_ddot = (
            -f * np.cos(theta) 
            - self.m * self.l * theta_dot**2 * np.sin(theta) * np.cos(theta) 
            + (self.M + self.m) * self.g * np.sin(theta)
        ) / (self.l * denom)
        
        return np.array([x_dot, x_ddot, theta_dot, theta_ddot])
    
    def rk4_int(self, state, dt, control_force, disturbances):
        """
        RK4 method:
        k1 = f(t, y)
        k2 = f(t + dt/2, y + dt*k1/2)
        k3 = f(t + dt/2, y + dt*k2/2)  
        k4 = f(t + dt, y + dt*k3)
        y_next = y + dt*(k1 + 2*k2 + 2*k3 + k4)/6
        """
        t = 0
        k1 = self.dynamics(state, t, control_force, disturbances)
        k2 = self.dynamics(state + 0.5*dt*k1, t + 0.5*dt, control_force, disturbances)
        k3 = self.dynamics(state + 0.5*dt*k2, t + 0.5*dt, control_force, disturbances)
        k4 = self.dynamics(state + dt*k3, t + dt, control_force, disturbances)
        state_next = state + dt*(k1 + 2*k2 + 2*k3 + k4)/6
        return state_next
    
    def sliding_mode_control(self, state, t):
        """
        Sliding mode controller for cartpole stabilization
        Control both cart position and pole angle
        """
        x, x_dot, theta, theta_dot = state
        
        #would like cart at origin and pole upright
        x_d = 0          # desired cart position
        x_d_dot = 0      # desired cart velocity
        theta_d = np.pi    # desired pole angle (upright)
        theta_d_dot = 0  # desired pole angular velocity
        
        # Tracking errors
        e_x = x - x_d
        e_x_dot = x_dot - x_d_dot
        e_theta = theta - theta_d
        e_theta_dot = theta_dot - theta_d_dot
    
        s = (e_x_dot + self.lambda_param * e_x) + (e_theta_dot + self.lambda_param * e_theta)
        
        #common denom from dynamics
        denom = self.M + self.m*np.sin(theta)**2

        f_coeff = 1/denom - np.cos(theta)/(self.l * denom)

        #term without f
        no_f_terms = (
            # From x_ddot
            (self.m * np.sin(theta) * (self.l * theta_dot**2 - self.g * np.cos(theta))) / denom +
            # From theta_ddot  
            (-self.m * self.l * theta_dot**2 * np.sin(theta) * np.cos(theta) + 
             (self.M + self.m) * self.g * np.sin(theta)) / (self.l * denom) +
            # lambda terms
            self.lambda_param * e_x_dot + self.lambda_param * e_theta_dot + self.g*e_theta
        )
        u_eq = -no_f_terms/f_coeff
        
        # Switching control with boundary layer
        if abs(s) > self.phi:
            u_sw = -self.k * np.sign(s)
        else:
            u_sw = -self.k * (s / self.phi)
        
        return u_eq + u_sw
    
    def simulate(self, t_end=10.0, dt=0.01):
        """Run simulation with sliding mode control"""
        t = np.arange(0, t_end, dt)
        n_steps = len(t)
        
        # State history
        states = np.zeros((n_steps, 4))  # [x, x_dot, theta, theta_dot]
        control_inputs = np.zeros(n_steps)
        sliding_surface = np.zeros(n_steps)
        disturbances = np.zeros(n_steps)
        
        # Initial conditions
        states[0] = [0.5, 0.0, np.pi + 0.1, 0.0]  #off center with perturbation
        
        print("Simulating cartpole with sliding mode control...")
        print(f"Using RK4 integration with dt = {dt} s")
        
        # Simulation loop
        for i in range(n_steps - 1):
            # External disturbance
            disturbance = self.disturbance_amplitude * np.sin(self.disturbance_freq * t[i])
            disturbances[i] = disturbance
            
            # Control input
            u = self.sliding_mode_control(states[i], t[i])
            control_inputs[i] = u
            
            # Calculate sliding surface for plotting
            x, x_dot, theta, theta_dot = states[i]
            e_x = x - 0.0
            e_x_dot = x_dot - 0.0
            e_theta = theta - np.pi
            e_theta_dot = theta_dot - 0.0
            
            w1, w2 = 1.0, 1.0
            sliding_surface[i] = (w1 * (e_x_dot + self.lambda_param * e_x) + 
                                w2 * (e_theta_dot + self.lambda_param * e_theta))
            
            # Integrate using RK4
            states[i+1] = self.rk4_int(states[i], dt, u, disturbance)
        
        # Store final values
        control_inputs[-1] = self.sliding_mode_control(states[-1], t[-1])
        disturbances[-1] = self.disturbance_amplitude * np.sin(self.disturbance_freq * t[-1])
        
        x, x_dot, theta, theta_dot = states[-1]
        e_x = x - 0.0
        e_x_dot = x_dot - 0.0
        e_theta = theta - np.pi
        e_theta_dot = theta_dot - 0.0
        w1, w2 = 1.0, 1.0
        sliding_surface[-1] = (w1 * (e_x_dot + self.lambda_param * e_x) + 
                             w2 * (e_theta_dot + self.lambda_param * e_theta))
        
        return t, states, control_inputs, sliding_surface, disturbances
    
    def plot_results(self, t, states, control_inputs, sliding_surface, disturbances):
        """Plot simulation results"""
        fig, axes = plt.subplots(3, 2, figsize=(15, 12))
        fig.suptitle('SMC Control of Cartpole', fontsize=16)
        
        # Cart position
        axes[0,0].plot(t, states[:,0], 'b-', linewidth=2, label='Actual x')
        axes[0,0].axhline(y=0, color='r', linestyle='--', linewidth=2, label='Target x = 0')
        axes[0,0].set_xlabel('Time (s)')
        axes[0,0].set_ylabel('Cart Pos (m)')
        axes[0,0].set_title('Cart Pos')
        axes[0,0].grid(True)
        axes[0,0].legend()
        
        # Cart velocity
        axes[0,1].plot(t, states[:,1], 'g-', linewidth=2, label='Actual x_dot')
        axes[0,1].axhline(y=0, color='r', linestyle='--', linewidth=2, label='Target x_dot = 0')
        axes[0,1].set_xlabel('Time (s)')
        axes[0,1].set_ylabel('Cart Vel (m/s)')
        axes[0,1].set_title('Cart Vel')
        axes[0,1].grid(True)
        axes[0,1].legend()
        
        # Pole angle
        axes[1,0].plot(t, states[:,2], 'b-', linewidth=2, label='Actual theta')
        axes[1,0].axhline(y=np.pi, color='r', linestyle='--', linewidth=2, label='Target theta = pi')
        axes[1,0].set_xlabel('Time (s)')
        axes[1,0].set_ylabel('Pole Ang (rad)')
        axes[1,0].set_title('Pole Ang')
        axes[1,0].grid(True)
        axes[1,0].legend()
        
        # Pole angular velocity
        axes[1,1].plot(t, states[:,3], 'g-', linewidth=2, label='Actual theta_dot')
        axes[1,1].axhline(y=0, color='r', linestyle='--', linewidth=2, label='Target theta_dot = 0')
        axes[1,1].set_xlabel('Time (s)')
        axes[1,1].set_ylabel('Pole Ang Vel (rad/s)')
        axes[1,1].set_title('Pole Ang Vel')
        axes[1,1].grid(True)
        axes[1,1].legend()
        
        # Control input
        axes[2,0].plot(t, control_inputs, 'purple', linewidth=2)
        axes[2,0].set_xlabel('Time (s)')
        axes[2,0].set_ylabel('Control Force (N)')
        axes[2,0].set_title('Control Force')
        axes[2,0].grid(True)
        
        # Sliding surface
        axes[2,1].plot(t, sliding_surface, 'orange', linewidth=2, label='Sliding surface')
        axes[2,1].plot(t, disturbances, 'red', linewidth=1, alpha=0.7, label='Disturbance')
        axes[2,1].axhline(y=0, color='black', linestyle='--', alpha=0.7)
        axes[2,1].set_xlabel('Time (s)')
        axes[2,1].set_ylabel('Value')
        axes[2,1].set_title('Sliding Surf & Disturbance')
        axes[2,1].grid(True)
        axes[2,1].legend()
        
        plt.tight_layout()
        plt.show()

# Run simulation
if __name__ == "__main__":
    cartpole = CartpoleSMC()
    t, states, control_inputs, sliding_surface, disturbances = cartpole.simulate(t_end=20.0, dt=0.01)
    cartpole.plot_results(t, states, control_inputs, sliding_surface, disturbances)
    
    # Print final state
    print(f"\nFinal state:")
    print(f"Cart position: {states[-1,0]:.4f} m")
    print(f"Cart velocity: {states[-1,1]:.4f} m/s")
    print(f"Pole angle: {states[-1,2]:.4f} rad ({np.degrees(states[-1,2]):.2f}°)")
    print(f"Pole angular velocity: {states[-1,3]:.4f} rad/s")