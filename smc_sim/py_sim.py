import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg

#sim for cartpole
class CartPole:
    def __init__(self):
        self.cart_mass = 1 #kg
        self.pole_mass = 0.1 #kg
        self.pole_length = 0.5 #m
        self.gravity = 9.81 #m/s^2

        #control params
        #sliding surf param
        self.lambda_x = 8
        self.lambda_theta = 15
        #switching gains
        self.k_d_x = 20
        self.k_d_theta = 50
        self.delta = 0.2 #bl thickness
        self.max_force = 100 #in N

    def solve_acc(self, theta, theta_dot, F, disturbances):
        mc, mp, l, g = self.cart_mass, self.pole_mass, self.pole_length, self.gravity

        #including disturbances
        d_f = disturbances.get('force', 0)
        d_tau = disturbances.get('torque', 0)
        mass_unc = disturbances.get('mass_unc', 1.0)

        #unc due to change in mass
        mc_eff = mc*mass_unc
        mp_eff = mp*mass_unc

        #defining sine and cos
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)

        #sys matrix A
        A = np.array(
            [
                [mc_eff+mp_eff, mp_eff*l*cos_theta], 
                [mp_eff*l*cos_theta, mp_eff*l**2]
            ]
        )
        b = np.array([
            F + mp_eff*l*sin_theta*theta_dot**2+d_f,
            mp_eff*g*l*sin_theta+d_tau
        ])  
        
        #check for singularity 
        det_A = np.linalg.det(A)
        if abs(det_A) < 1e-12:
            return 0, 0
        
        x_ddot, theta_ddot = linalg.inv(A) @ b
        return x_ddot, theta_ddot
    
    def dynamics(self, state, time, control_force, disturbances):
        """
        state: [x, x_dot, theta, theta_dot]
        """
        x, x_dot, theta, theta_dot = state

        #solving accs
        x_ddot, theta_ddot = self.solve_acc(theta, theta_dot, control_force, disturbances)
        return np.array([x_dot, x_ddot, theta_dot, theta_ddot])

        # #cart acceleration
        # cart_acc = (total_force + mp*l*sin_ang*ang_vel**2 - mp*l*cos_ang*ang_vel**2)/(mc + mp)
        # pole_acc = (-mp*g*l*sin_ang - vel**2*mp*l*cos_ang)/(mp*l**2)
        # return [vel, cart_acc, ang_vel, pole_acc]
    
    #numerical integration
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
        k2 = self.dynamics(state+0.5*dt*k1, t+0.5*dt, control_force, disturbances)
        k3 = self.dynamics(state+0.5*dt*k2, t+0.5*dt, control_force, disturbances)
        k4 = self.dynamics(state+dt*k3, t+dt, control_force, disturbances)
        state_next = state + dt*(k1 + 2*k2+ 2*k3 + k4)/6
        return state_next

    def SMC(self, state, x_ref = 0, theta_ref = 0): #initializing
        """
        defining sliding surface:
        s_x = lambda_x * e_x + e_x_dot
        s_theta = lambda_theta *e_theta + e_theta_dot
        """
    #SMC base class
    #sliding slope param: lambda
    #switching gain: k
    #boundary layer: delta 
        x, x_dot, theta, theta_dot = state

        #pos ctrl
        e_x = x_ref - x
        e_x_dot = -x_dot #assuming zero ref velocity
        s_x = self.lambda_x * e_x + e_x_dot

        #angle ctrl
        e_theta = theta_ref - theta
        e_theta_dot = -theta_dot
        s_theta = self.lambda_theta*e_theta + e_theta_dot

        u_x = self.k_d_x * np.tanh(s_x/self.delta) ## confirm if tanh the best
        u_theta = self.k_d_theta*np.tanh(s_theta/self.delta)

        total_control = 0.3*u_x + 1*u_theta #add weights if we need to prioritize one over another
        return np.clip(total_control, -self.max_force, self.max_force)

class DistGen:
    #disturbance generator
    @staticmethod
    def step_dist(time, start_time, duration, amp):
        if start_time <= time <= start_time+duration:
            return amp
        return 0
    
    @staticmethod
    def imp_dist(time, imp_time, amp, width = 0.1):
        if abs(time-imp_time) <= width/2:
            return amp
        return 0
    
    @staticmethod
    def sin_dist(time, amp, freq):
        return amp*np.sin(2*np.pi*freq*time)
    
    @staticmethod
    def rand_dist(time, amp, seed=42):
        np.random.seed(int(seed+time*1000)%1000)
        return amp*(2*np.random.random()-1)
    
def sim_cartpole_smc(integration_method='rk4'):
    system = CartPole()
    tspan = 15
    control_freq = 100 #Hz
    dt = 1/control_freq
    t = np.arange(0, tspan, dt)

    #intializing states: [x, x_dot, theta, theta_dot]
    init_state = np.array([0, 0, 0.17453, 0]) #10 deg tilt
    state = init_state.copy()

    #ref signals: cart at origin and pole upright
    x_ref = 0
    theta_ref = 0

    #storage arrays
    states = np.zeros((len(t), 4))
    controls = np.zeros(len(t))
    dist_data = np.zeros((len(t),3))

    for i, time_val in enumerate(t):
        states[i] = state

        disturbances = {
            'force': (DistGen.step_dist(time_val, 3, 2, -8)+
                      DistGen.sin_dist(time_val, 3, 0.8) if time_val > 12 else 0),
            'torque': DistGen.imp_dist(time_val, 8, 0.2, 0.2),
            'mass_unc': 1+0.5*DistGen.step_dist(time_val, 10, 5, 1)
        }
        control_force = system.SMC(state, x_ref, theta_ref)
        controls[i] = control_force
        dist_data[i] = [disturbances['force'], disturbances['torque'], disturbances['mass_unc']]

        #rk4
        if i<len(t)-1:
            state = system.rk4_int(state, dt, control_force, disturbances)

    sliding_surfaces = np.zeros((len(t),2))
    for i in range(len(t)):
        x, x_dot, theta, theta_dot = states[i]

        #pos sliding surf
        e_x = x_ref - x
        e_x_dot = -x_dot
        s_x = system.lambda_x*e_x+e_x_dot

        #angle sliding surf
        e_theta = theta_ref-theta
        e_theta_dot = -theta_dot
        s_theta = system.lambda_theta*e_theta+e_theta_dot

        sliding_surfaces[i] = [s_x, s_theta]

    out_pos_err = abs(states[-1, 0])
    out_ang_err = abs(states[-1, 2]*180/np.pi)

    print(f"cartpole SMC results:")
    print(f"  Final position error: {out_pos_err:.4f} m")
    print(f"  Final angle error: {out_ang_err:.2f}°")

    return t, states, controls, sliding_surfaces, dist_data

def plot_cartpole_results(t, states, controls, sliding_surfaces, dist_data):
    
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    
    # Cart position
    axes[0,0].plot(t, states[:, 0], 'b-', linewidth=2, label='Position')
    axes[0,0].axhline(y=0, color='r', linestyle='--', alpha=0.5, label='Reference')
    axes[0,0].set_ylabel('Cart Position [m]')
    axes[0,0].set_title('Position Control')
    axes[0,0].legend()
    axes[0,0].grid(True, alpha=0.3)
    
    # Pole angle
    axes[0,1].plot(t, states[:, 2] * 180/np.pi, 'g-', linewidth=2, label='Angle')
    axes[0,1].axhline(y=0, color='r', linestyle='--', alpha=0.5, label='Reference')
    axes[0,1].set_ylabel('Pole Angle (degrees)')
    axes[0,1].set_title('Angle Stabilization')
    axes[0,1].legend()
    axes[0,1].grid(True, alpha=0.3)
    
    # Control signal and disturbances
    axes[1,0].plot(t, controls, 'm-', linewidth=2, label='Control Force')
    axes[1,0].plot(t, dist_data[:, 0], 'r--', alpha=0.7, label='Force Disturbance')
    axes[1,0].set_ylabel('Force [N]')
    axes[1,0].set_title('Control & Disturbances')
    axes[1,0].legend()
    axes[1,0].grid(True, alpha=0.3)
    
    # Phase portrait
    axes[1,1].plot(states[:, 2] * 180/np.pi, states[:, 3] * 180/np.pi, 'b-', alpha=0.8)
    axes[1,1].plot(states[0, 2] * 180/np.pi, states[0, 3] * 180/np.pi, 'go', 
                   markersize=8, label='Start')
    axes[1,1].plot(states[-1, 2] * 180/np.pi, states[-1, 3] * 180/np.pi, 'ro', 
                   markersize=8, label='End')
    axes[1,1].set_xlabel('Pole Angle [deg]')
    axes[1,1].set_ylabel('Pole Angular Velocity [deg/s]')
    axes[1,1].set_title('Phase Portrait')
    axes[1,1].legend()
    axes[1,1].grid(True, alpha=0.3)
    
    # Sliding surfaces
    axes[2,0].plot(t, sliding_surfaces[:, 0], 'b-', linewidth=2, label='Position s_x')
    axes[2,0].axhline(y=0, color='k', linestyle='--', alpha=0.5)
    axes[2,0].set_xlabel('Time [s]')
    axes[2,0].set_ylabel('Sliding Variable s_x')
    axes[2,0].set_title('Position Sliding Surface')
    axes[2,0].legend()
    axes[2,0].grid(True, alpha=0.3)
    
    axes[2,1].plot(t, sliding_surfaces[:, 1], 'g-', linewidth=2, label='Angle s_θ')
    axes[2,1].axhline(y=0, color='k', linestyle='--', alpha=0.5)
    axes[2,1].set_xlabel('Time [s]')
    axes[2,1].set_ylabel('Sliding Variable s_theta')
    axes[2,1].set_title('Angle Sliding Surface')
    axes[2,1].legend()
    axes[2,1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    #run rk4
    t, states, controls, sliding_surfaces, dist_data = sim_cartpole_smc()
    
    #plto results
    plot_cartpole_results(t, states, controls, sliding_surfaces, dist_data)