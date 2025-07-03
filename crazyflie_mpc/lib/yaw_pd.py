from tf_transformations import euler_from_quaternion
import numpy as np

class YawPD:
    def __init__(self):
        self.prev_yaw = None
        self.prev_t = None
        self.kp = 200.0
        self.kd = 20.0
        self.max_yaw_rate = 180

    def _wrap_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))
    
    def compute_control(self, t, quat, yaw_des):
        _, _, yaw = euler_from_quaternion(quat)
        p = self.kp * self._wrap_angle(yaw_des - yaw)
        d = 0.0
        if self.prev_t is not None:
            dt = t - self.prev_t
            d = -self.kd * self._wrap_angle(yaw - self.prev_yaw) / dt
        u_yaw = np.clip(p + d, -self.max_yaw_rate, self.max_yaw_rate)
        self.prev_yaw = yaw
        self.prev_t = t
        
        return u_yaw
