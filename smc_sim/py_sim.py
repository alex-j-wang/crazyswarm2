class SMC:

#simulation for cartpole
class CartPole:
    def __init__(self):
        self.cart_mass = 1 #kg
        self.pole_mass = 0.1 #kg
        self.pole_length = 0.5 #m
        self.gravity = 9.81 #m/s^2
    def dynamics(self, state, time, control_force, disturbances):
        pos, vel, ang, ang_vel = state

        #including disturbances
        force_dist = disturbances.get('force', 0)
        wind_dist = disturbances.get('wind', 0)
        mass_unc = disturbances.get('mass', 1.0)

        #unc due to change in mass
        mc = self.cart_mass*mass_unc
        mp = self.pole_mass*mass_unc
        l = self.pole_length
        g = self.gravity

        #defining sine and cos
        sin_ang = np.sin(ang)
        cos_ang = np.cos(ang)

        #total force applied changes based on disturbances
        total_force = control_force + force_dist
        #torque due to wind
        wind_torque = wind_dist*l

        #cart acceleration
        cart_acc = (total_force + mp*l*sin_ang*ang_vel**2 - mp*l*cos_ang*ang_vel**2)/(mc + mp)
        pole_acc = (-mp*g*l*sin_ang - vel**2*mp*l*cos_ang)/(mp*l**2)
        return [vel, cart_acc, ang_vel, pole_acc]
    
    def control_law(self,state, position_ref = 0, angle_ref = 0) #setting reference x to be 0 and theta to be 0
        
#simulation for 2d quad

class QuadrD: