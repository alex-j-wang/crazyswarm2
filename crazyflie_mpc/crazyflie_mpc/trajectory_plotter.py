#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String, Int32
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5 import QtWidgets, QtCore
import threading
import sys
import time
from tf2_ros import TransformListener, Buffer

class TrajectoryPlotterNode(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')
        
        self.declare_parameter(
            'cfnames', 
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description='Names of Crazyflies to track'
            )
        )
        
        self.declare_parameter(
            'world_frame', 
            'world',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='World coordinate frame'
            )
        )
        
        self.declare_parameter(
            'plot_history_seconds', 
            10.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='How many seconds of trajectory history to display'
            )
        )
        
        self.declare_parameter(
            'update_frequency', 
            30.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Update frequency for the plot in Hz'
            )
        )

        # Get parameters
        self.cfnames = self.get_parameter('cfnames').get_parameter_value().string_array_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.history_seconds = self.get_parameter('plot_history_seconds').get_parameter_value().double_value
        self.update_frequency = self.get_parameter('update_frequency').get_parameter_value().double_value
        
        # Set up TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Set up data structures for each drone
        self.drone_data = {}
        self.markers_pub = self.create_publisher(MarkerArray, 'trajectory_markers', 10)
        
        for cf_name in self.cfnames:
            self.drone_data[cf_name] = {
                'actual_positions': [], 
                'goal_positions': [],
                'actual_timestamps': [],
                'goal_timestamps': [],
                'goal_sub': self.create_subscription(
                    TwistStamped,
                    f'/{cf_name}/goal', 
                    lambda msg: self.goal_callback(msg, cf_name), 
                    10
                )
            }
            
        # Setup plotting
        self.app = None
        self.plot_thread = threading.Thread(target=self.start_gui)
        self.plot_thread.daemon = True
        self.plot_thread.start()
        
        # Create timer for visualization markers
        self.marker_timer = self.create_timer(1.0 / self.update_frequency, self.publish_markers)
        
        self.get_logger().info(f"Trajectory plotter initialized for {len(self.cfnames)} Crazyflies")
        
    def start_gui(self):
        self.app = QtWidgets.QApplication(sys.argv)
        self.plot_window = TrajectoryPlotWindow(self)
        self.plot_window.show()
        self.app.exec_()
        
    def goal_callback(self, msg, cf_name):
        """
        Callback for goal position updates
        """
        now = self.get_clock().now().nanoseconds / 1e9
        pos = msg.twist.linear
        self.drone_data[cf_name]['goal_positions'].append((pos.x, pos.y, pos.z))
        self.drone_data[cf_name]['goal_timestamps'].append(now)
        
    def publish_markers(self):
        """
        Publish visualization markers for RViz
        """
        now = self.get_clock().now().nanoseconds / 1e9
        marker_array = MarkerArray()
        marker_id = 0
        
        for cf_name in self.cfnames:
            # Get the current position from TF
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    cf_name,
                    rclpy.time.Time()
                )
                pos = transform.transform.translation
                self.drone_data[cf_name]['actual_positions'].append((pos.x, pos.y, pos.z))
                self.drone_data[cf_name]['actual_timestamps'].append(now)
            except Exception as e:
                pass  # Skip if transform is not available
            
            # Clean up old data based on history_seconds
            for data_type in ['actual_positions', 'goal_positions', 'actual_timestamps', 'goal_timestamps']:
                data_list = self.drone_data[cf_name][data_type]
                timestamps = self.drone_data[cf_name][data_type.replace('positions', 'timestamps')]
                
                if len(timestamps) > 0:
                    cutoff_time = now - self.history_seconds
                    while timestamps and timestamps[0] < cutoff_time:
                        data_list.pop(0)
                        timestamps.pop(0)
            
            # Create actual trajectory marker (blue)
            if self.drone_data[cf_name]['actual_positions']:
                marker_id = self.add_trajectory_marker(
                    marker_array, 
                    self.drone_data[cf_name]['actual_positions'], 
                    cf_name, 
                    marker_id, 
                    (0.0, 0.0, 1.0, 1.0)  # Blue
                )
                
            # Create goal trajectory marker (green)
            if self.drone_data[cf_name]['goal_positions']:
                marker_id = self.add_trajectory_marker(
                    marker_array, 
                    self.drone_data[cf_name]['goal_positions'], 
                    cf_name + '_goal', 
                    marker_id, 
                    (0.0, 1.0, 0.0, 1.0)  # Green
                )
        
        # Publish markers
        if marker_array.markers:
            self.markers_pub.publish(marker_array)
    
    def add_trajectory_marker(self, marker_array, positions, namespace, marker_id, color_rgba):
        """
        Helper to add a line strip marker to the marker array
        """
        if not positions:
            return marker_id
            
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Set marker properties
        marker.scale.x = 0.03  # Line width
        
        # Set color
        marker.color.r = color_rgba[0]
        marker.color.g = color_rgba[1]
        marker.color.b = color_rgba[2]
        marker.color.a = color_rgba[3]
        
        # Add points
        for pos in positions:
            point = self.create_point(pos[0], pos[1], pos[2])
            marker.points.append(point)
            
        marker_array.markers.append(marker)
        return marker_id + 1
        
    @staticmethod
    def create_point(x, y, z):
        """Create a Point message"""
        from geometry_msgs.msg import Point
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

class TrajectoryPlotWindow(QtWidgets.QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Crazyflie Trajectory Plotter")
        
        # Create the main widget
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        
        # Create layout
        layout = QtWidgets.QVBoxLayout(self.central_widget)
        
        # Create figures for different views
        self.fig = Figure(figsize=(10, 8))
        
        # Create 3D plot
        self.ax_3d = self.fig.add_subplot(2, 2, 1, projection='3d')
        self.ax_3d.set_title('3D Trajectory')
        self.ax_3d.set_xlabel('X (m)')
        self.ax_3d.set_ylabel('Y (m)')
        self.ax_3d.set_zlabel('Z (m)')
        
        # Create 2D plots for each plane
        self.ax_xy = self.fig.add_subplot(2, 2, 2)
        self.ax_xy.set_title('XY Plane')
        self.ax_xy.set_xlabel('X (m)')
        self.ax_xy.set_ylabel('Y (m)')
        self.ax_xy.grid(True)
        
        self.ax_xz = self.fig.add_subplot(2, 2, 3)
        self.ax_xz.set_title('XZ Plane')
        self.ax_xz.set_xlabel('X (m)')
        self.ax_xz.set_ylabel('Z (m)')
        self.ax_xz.grid(True)
        
        self.ax_yz = self.fig.add_subplot(2, 2, 4)
        self.ax_yz.set_title('YZ Plane')
        self.ax_yz.set_xlabel('Y (m)')
        self.ax_yz.set_ylabel('Z (m)')
        self.ax_yz.grid(True)
        
        # Add the figure to the layout
        self.canvas = FigureCanvas(self.fig)
        layout.addWidget(self.canvas)
        
        # Add legend entries
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], color='b', lw=2, label='Actual'),
            Line2D([0], [0], color='g', lw=2, label='Goal')
        ]
        self.ax_3d.legend(handles=legend_elements, loc='upper right')
        
        # Set window size
        self.resize(1200, 800)
        
        # Timer for updating the plot
        self.update_timer = QtCore.QTimer(self)
        self.update_timer.timeout.connect(self.update_plot)
        self.update_timer.start(1000 / self.node.update_frequency)  # Convert Hz to ms
        
        # Colors for each drone
        self.drone_colors = {}
        colormap = plt.cm.tab10
        for i, name in enumerate(self.node.cfnames):
            color_actual = colormap(i % 10)
            color_goal = colormap((i + len(self.node.cfnames)) % 10)
            self.drone_colors[name] = {
                'actual': color_actual,
                'goal': color_goal
            }
    
    def update_plot(self):
        """Update all plots with new data"""
        # Clear previous plots
        self.ax_3d.clear()
        self.ax_xy.clear()
        self.ax_xz.clear()
        self.ax_yz.clear()
        
        # Reset titles and labels
        self.ax_3d.set_title('3D Trajectory')
        self.ax_3d.set_xlabel('X (m)')
        self.ax_3d.set_ylabel('Y (m)')
        self.ax_3d.set_zlabel('Z (m)')
        
        self.ax_xy.set_title('XY Plane')
        self.ax_xy.set_xlabel('X (m)')
        self.ax_xy.set_ylabel('Y (m)')
        self.ax_xy.grid(True)
        
        self.ax_xz.set_title('XZ Plane')
        self.ax_xz.set_xlabel('X (m)')
        self.ax_xz.set_ylabel('Z (m)')
        self.ax_xz.grid(True)
        
        self.ax_yz.set_title('YZ Plane')
        self.ax_yz.set_xlabel('Y (m)')
        self.ax_yz.set_ylabel('Z (m)')
        self.ax_yz.grid(True)
        
        # Plot data for each drone
        for i, cf_name in enumerate(self.node.cfnames):
            data = self.node.drone_data[cf_name]
            
            # Get positions
            actual_positions = data['actual_positions']
            goal_positions = data['goal_positions']
            
            if actual_positions:
                # Convert to numpy arrays for easier plotting
                actual_pos = np.array(actual_positions)
                
                # Plot actual trajectory
                self.ax_3d.plot(actual_pos[:, 0], actual_pos[:, 1], actual_pos[:, 2], 'b-', linewidth=2, label=f'{cf_name} Actual' if i == 0 else "")
                self.ax_xy.plot(actual_pos[:, 0], actual_pos[:, 1], 'b-', linewidth=2)
                self.ax_xz.plot(actual_pos[:, 0], actual_pos[:, 2], 'b-', linewidth=2)
                self.ax_yz.plot(actual_pos[:, 1], actual_pos[:, 2], 'b-', linewidth=2)
                
                # Plot current position with a marker
                self.ax_3d.scatter(actual_pos[-1, 0], actual_pos[-1, 1], actual_pos[-1, 2], color='b', s=50, marker='o')
                self.ax_xy.scatter(actual_pos[-1, 0], actual_pos[-1, 1], color='b', s=50, marker='o')
                self.ax_xz.scatter(actual_pos[-1, 0], actual_pos[-1, 2], color='b', s=50, marker='o')
                self.ax_yz.scatter(actual_pos[-1, 1], actual_pos[-1, 2], color='b', s=50, marker='o')
            
            if goal_positions:
                # Convert to numpy arrays for easier plotting
                goal_pos = np.array(goal_positions)
                
                # Plot goal trajectory
                self.ax_3d.plot(goal_pos[:, 0], goal_pos[:, 1], goal_pos[:, 2], 'g-', linewidth=2, label=f'{cf_name} Goal' if i == 0 else "")
                self.ax_xy.plot(goal_pos[:, 0], goal_pos[:, 1], 'g-', linewidth=2)
                self.ax_xz.plot(goal_pos[:, 0], goal_pos[:, 2], 'g-', linewidth=2)
                self.ax_yz.plot(goal_pos[:, 1], goal_pos[:, 2], 'g-', linewidth=2)
                
                # Plot current goal with a marker
                self.ax_3d.scatter(goal_pos[-1, 0], goal_pos[-1, 1], goal_pos[-1, 2], color='g', s=50, marker='x')
                self.ax_xy.scatter(goal_pos[-1, 0], goal_pos[-1, 1], color='g', s=50, marker='x')
                self.ax_xz.scatter(goal_pos[-1, 0], goal_pos[-1, 2], color='g', s=50, marker='x')
                self.ax_yz.scatter(goal_pos[-1, 1], goal_pos[-1, 2], color='g', s=50, marker='x')
            
            # Add text label for the drone
            if actual_positions:
                pos = actual_positions[-1]
                self.ax_3d.text(pos[0], pos[1], pos[2], f' {cf_name}', fontsize=9)
        
        # Add legend
        self.ax_3d.legend()
        
        # Set equal aspect ratio for 2D plots
        self.ax_xy.set_aspect('equal')
        self.ax_xz.set_aspect('equal')
        self.ax_yz.set_aspect('equal')
        
        # Refresh the canvas
        self.canvas.draw()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
