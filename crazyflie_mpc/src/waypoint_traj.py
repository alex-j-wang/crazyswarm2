import numpy as np
from numpy import linalg

def get_coefficients(x0,x1,t0,t1,dx0=0,ddx0=0,dx1=0,ddx1=0):
    L = linalg.inv(np.array([[t0**5, t0**4, t0**3, t0**2, t0, 1],
                            [5*t0**4, 4*t0**3, 3*t0**2, 2*t0, 1, 0],
                            [20*t0**3, 12*t0**2, 6*t0, 2, 0, 0],
                            [t1**5, t1**4, t1**3, t1**2, t1, 1],
                            [5*t1**4, 4*t1**3, 3*t1**2, 2*t1, 1, 0],
                            [20*t1**3, 12*t1**2, 6*t1, 2, 0, 0]]))

    coefficients = L @ np.array([x0, dx0, ddx0, x1, dx1, ddx1]).T
    return coefficients

def get_min_jerk(points, t):
    N   = t.shape[0]
    mat = np.zeros((N-1,6,3))
    for i in range(N-1):
        mat[i,:,0] = get_coefficients(points[i,0], points[i+1,0], t[i], t[i+1])
        mat[i,:,1] = get_coefficients(points[i,1], points[i+1,1], t[i], t[i+1])
        mat[i,:,2] = get_coefficients(points[i,2], points[i+1,2], t[i], t[i+1])
    return mat

class WaypointTraj(object):
    def __init__(self, points):
        self.points         = points
        self.desired_spd    = 0.5 # 0.75, 3.0

        num_pts     = self.points.shape[0]
        dist        = linalg.norm(np.diff(self.points, axis=0), axis=1)
        self.time   = np.zeros((num_pts,))
        for i, d in enumerate(dist):
            # v = 0.05 * np.log(d) + 0.25
            # if v < 0.01:
            #     v = 0.0
            v = 1.5
            self.time[i + 1] = self.time[i] + d/v

        self.coeff_mat = get_min_jerk(self.points, self.time)

    def update(self, t):
        x = np.zeros((3,))
        x_dot = np.zeros((3,))

        x_ddot = np.zeros((3,))
        x_dddot = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        num_segments = len(self.points) - 1
        if num_segments > 0:
            segment_dists = self.points[1:(num_segments + 1), :] - self.points[0:num_segments, :]
            norm_dists = np.linalg.norm(segment_dists, axis=1)
            unit_vec = segment_dists / norm_dists[:, None]
            segment_times = norm_dists / self.desired_spd
            start_times = np.cumsum(segment_times)

            if t < start_times[len(start_times) - 1]:
                idx = np.where(t <= start_times)[0]
                segment_num = idx[0]

                diff_time = t - start_times[segment_num]
                x_dot = self.desired_spd * unit_vec[segment_num, :]
                x = self.points[segment_num + 1, :] + x_dot * diff_time
            else:  # time exceeds expected time at last waypoint
                segment_num = num_segments - 1
                x_dot = np.zeros((3,))
                x = self.points[segment_num + 1, :]
        else:
            # segment_dist    = self.points[0, :] - self.init_pos
            # norm_dist       = np.linalg.norm(segment_dist)
            # unit_vec        = segment_dist / norm_dist
            # segment_time    = norm_dist / self.desired_spd
            # if t < segment_time:
            #     diff_time   = t - segment_time
            #     x_dot       = self.desired_spd * unit_vec
            #     x           = self.points + x_dot * diff_time
            # else:
            #     x_dot       = np.zeros((3,))
            #     x           = self.points

            x_dot = np.zeros((3,))
            x = self.points

        flat_output = {'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot, 'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output
