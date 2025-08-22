from pathlib import Path

import os
from ament_index_python.packages import get_package_share_directory
import yaml

import sympy as sp
import csv

SEGMENT_DURATION = 0.5 # seconds

def taylor_coeffs(t, expr, t0, order=7):
    coeffs = []
    for n in range(order + 1):
        deriv = expr.diff(t, n)
        value = deriv.subs(t, t0) / sp.factorial(n)
        coeffs.append(float(value))
    return coeffs

def to_csv(t, x, y, z, yaw, duration, filepath):
    num_segments = int(duration / SEGMENT_DURATION)
    
    rows = []
    for i in range(num_segments):
        t0 = i * SEGMENT_DURATION
        x_coeffs = taylor_coeffs(t, x, t0)
        y_coeffs = taylor_coeffs(t, y, t0)
        z_coeffs = taylor_coeffs(t, z, t0)
        yaw_coeffs = taylor_coeffs(t, yaw, t0)
        row = [SEGMENT_DURATION] + x_coeffs + y_coeffs + z_coeffs + yaw_coeffs
        rows.append(row)

    header = ["duration"] \
        + [f"x^{i}" for i in range(8)] \
        + [f"y^{i}" for i in range(8)] \
        + [f"z^{i}" for i in range(8)] \
        + [f"yaw^{i}" for i in range(8)]

    with open(filepath, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)

def generate_trajectories():
    traj_folder = os.path.join(
        Path(__file__).parent,
        'data',
        'multi_trajectory')

    for file_path in Path(traj_folder).glob('*'):
        os.remove(file_path)
    
    # TODO: move config to crazyflie_examples
    config_path = os.path.join(
        get_package_share_directory('crazyflie_mpc'),
        'config',
        'mpc.yaml')
    
    with open(config_path, 'r') as f:
        mpc = yaml.safe_load(f)
    speed = mpc['constants']['desired_speed']

    idx = 0
    for _, traj in mpc['trajectories'].items():
        t = sp.Symbol('t')
        match traj['type']:
            case 'circle':
                radius = traj['radius']
                center = traj['center']
                duration = 2 * sp.pi * radius / speed

                x = center[0] + radius * sp.cos(2 * sp.pi * t / duration)
                y = center[1] + radius * sp.sin(2 * sp.pi * t / duration)
                z = center[2] + 0 * t

            case 'linear':
                start = traj['start']
                end = traj['end']
                duration = sp.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2 + (end[2] - start[2])**2) / speed

                x = start[0] + (end[0] - start[0]) * t / duration
                y = start[1] + (end[1] - start[1]) * t / duration
                z = start[2] + (end[2] - start[2]) * t / duration
                
            case 'figure8':
                center = traj['center']
                scale = traj['scale']
                # TODO: calculate exact arc length
                duration = 4 * (scale[0] + scale[1]) / speed

                x = center[0] + scale[0] * sp.sin(2 * sp.pi * t / duration)
                y = center[1] + scale[1] * sp.sin(2 * sp.pi * t / duration) * sp.cos(2 * sp.pi * t / duration)
                z = center[2] + 0 * t
                
            case 'spiral':
                center = traj['center']
                radius_start = traj['radius_start']
                radius_end = traj['radius_end']
                height_start = traj['height_start']
                height_end = traj['height_end']
                revolutions = traj['revolutions']
                # TODO: calculate exact arc length
                duration = 2 * sp.pi * (radius_start + radius_end) / 2 * revolutions / speed

                x = center[0] + (radius_start + (radius_end - radius_start) * t / duration) * sp.cos(2 * sp.pi * revolutions * t / duration)
                y = center[1] + (radius_start + (radius_end - radius_start) * t / duration) * sp.sin(2 * sp.pi * revolutions * t / duration)
                z = height_start + (height_end - height_start) * t / duration
                
            case 'hover':
                x = traj['position'][0] + 0 * t
                y = traj['position'][1] + 0 * t
                z = traj['position'][2] + 0 * t
                duration = traj['duration']

            case _:
                raise Exception(f"Trajectory type '{traj.type}' invalid")
        
        yaw = 0 + 0 * t
        filepath = os.path.join(traj_folder, f'traj{idx}.csv')
        to_csv(t, x, y, z, yaw, duration, filepath)
        idx += 1
