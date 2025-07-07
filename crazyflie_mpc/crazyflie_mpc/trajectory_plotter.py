#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import Int32

import matplotlib.pyplot as plt
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
                'goal': [],
                'deviation_time': [],
                'deviation': [],
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
        self.create_subscription(Int32, '/command/cmd_state', self.cmd_state_callback, 1) # Phase request


        self.traj_fig = plt.figure(figsize=(12, 5))
        self.traj_fig.subplots_adjust(wspace=0.5)
        
        # --- 3D Trajectory Figure ---
        self.traj_ax = self.traj_fig.add_subplot(121, projection='3d')
        self.traj_ax.set_title('Crazyflie 3D Trajectories')
        self.traj_ax.set_xlabel('X (m)')
        self.traj_ax.set_ylabel('Y (m)')
        self.traj_ax.set_zlabel('Z (m)')
        self.traj_ax.set_xlim(-2, 2)
        self.traj_ax.set_ylim(-2, 2)
        self.traj_ax.set_zlim(0, 2)

        colormap = plt.get_cmap('tab10')
        self.lines = {}
        for i, name in enumerate(self.cfnames):
            color = colormap(i % 10)
            actual_line, = self.traj_ax.plot([], [], [], linestyle='-', label=f'{name} actual', color=color)
            goal_line, = self.traj_ax.plot([], [], [], linestyle=':', label=f'{name} goal', color=color)
            self.lines[name] = {
                'actual': actual_line,
                'goal': goal_line,
                'color': color,
            }

        self.traj_ax.legend()

        # --- Deviation Figure ---
        self.err_ax = self.traj_fig.add_subplot(122)
        self.err_ax.set_title('Deviation from Goal vs Time')
        self.err_ax.set_xlabel('Time (s)')
        self.err_ax.set_ylabel('Deviation (m)')
        self.err_ax.grid(True)

        self.error_lines = {
            name: self.err_ax.plot([], [], label=name, color=self.lines[name]['color'])[0]
            for name in self.cfnames
        }

        self.err_ax.legend()

        self.create_timer(0.2, self.update_plot)

    def actual_callback(self, msg: PoseStamped, cfname: str):
        with self.data_lock:
            pos = msg.pose.position
            self.data[cfname]['actual'].append((pos.x, pos.y, pos.z))

            if self.data[cfname]['goal']:
                t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                gx, gy, gz = self.data[cfname]['goal'][-1]
                deviation = np.linalg.norm(np.array([pos.x - gx, pos.y - gy, pos.z - gz]))
                self.data[cfname]['deviation_time'].append(t)
                self.data[cfname]['deviation'].append(deviation)

    def goal_callback(self, msg: TwistStamped, cfname: str):
        with self.data_lock:
            pos = msg.twist.linear
            self.data[cfname]['goal'].append((pos.x, pos.y, pos.z))

    def cmd_state_callback(self, msg: Int32):
        """
        Callback function for state changes
        """
        if msg.data == 4:
            self.get_logger().info('Shutdown requested!')
            plt.savefig("trajectory_summary.svg")
            plt.ioff()

    def update_plot(self):
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

                # Update error plot
                if self.data[name]['deviation']:
                    t = np.array(self.data[name]['deviation_time'])
                    d = np.array(self.data[name]['deviation'])
                    self.error_lines[name].set_data(t, d)

            self.err_ax.relim()
            self.err_ax.autoscale_view()
            self.traj_fig.canvas.draw()
            self.traj_fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    plt.ion()
    node = Trajectory3DPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
