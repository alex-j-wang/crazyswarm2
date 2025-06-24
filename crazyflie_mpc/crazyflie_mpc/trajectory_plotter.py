#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from threading import Lock

class Trajectory3DPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')

        self.declare_parameter(
            'cfnames',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description='Names of Crazyflies to track'
            )
        )

        self.cfnames = self.get_parameter('cfnames').get_parameter_value().string_array_value

        # Data storage and thread lock
        self.data_lock = Lock()
        self.data = {
            name: {
                'actual': [],
                'goal': []
            } for name in self.cfnames
        }

        # Subscriptions
        for name in self.cfnames:
            self.create_subscription(
                PoseStamped,
                f'/{name}/tf_pos',
                lambda msg, n=name: self.actual_callback(msg, n),
                10
            )
            self.create_subscription(
                TwistStamped,
                f'/{name}/goal',
                lambda msg, n=name: self.goal_callback(msg, n),
                10
            )

        # Matplotlib setup
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_title('Crazyflie 3D Trajectories')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')

        # Assign unique colors and line objects
        colormap = plt.get_cmap('tab10')
        self.lines = {}
        for i, name in enumerate(self.cfnames):
            color = colormap(i % 10)
            actual_line, = self.ax.plot([], [], [], linestyle='-', label=f'{name} actual', color=color)
            goal_line, = self.ax.plot([], [], [], linestyle=':', label=f'{name} goal', color=color)
            self.lines[name] = {
                'actual': actual_line,
                'goal': goal_line
            }

        self.ax.legend()
        self.anim = FuncAnimation(self.fig, self.update_plot, interval=100)
        plt.show()

    def actual_callback(self, msg: PoseStamped, cfname: str):
        with self.data_lock:
            pos = msg.pose.position
            self.data[cfname]['actual'].append((pos.x, pos.y, pos.z))

    def goal_callback(self, msg: TwistStamped, cfname: str):
        with self.data_lock:
            pos = msg.twist.linear
            self.data[cfname]['goal'].append((pos.x, pos.y, pos.z))

    def update_plot(self, _):
        with self.data_lock:
            for name in self.cfnames:
                # Plot actual path
                actual = self.data[name]['actual']
                if actual:
                    actual_np = np.array(actual)
                    self.lines[name]['actual'].set_data(actual_np[:, 0], actual_np[:, 1])
                    self.lines[name]['actual'].set_3d_properties(actual_np[:, 2])

                # Plot goal path
                goal = self.data[name]['goal']
                if goal:
                    goal_np = np.array(goal)
                    self.lines[name]['goal'].set_data(goal_np[:, 0], goal_np[:, 1])
                    self.lines[name]['goal'].set_3d_properties(goal_np[:, 2])

            self.ax.relim()
            self.ax.autoscale_view()

def main(args=None):
    rclpy.init(args=args)
    node = Trajectory3DPlotter()
    rclpy.spin(node)
    
if __name__ == '__main__':
    main()
