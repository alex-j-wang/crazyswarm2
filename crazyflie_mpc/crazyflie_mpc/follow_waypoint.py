#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Int32

import waypoint_traj as wt
from mpc_control import MPControl
from hybrid_control import HybridControl
from knode_control import KNODEControl
from geometric_control import GeometriControl
from gp_control import GPControl
from scipy.interpolate import interp1d

class MPCDemo(Node):
    def __init__(self):
        super().__init__('mpc_demo', automatically_declare_parameters_from_overrides=True)
        
        self.world_frame = self.get_parameter('world_frame').value
        self.frame = self.get_parameter('frame').value
        self.sim = self.get_parameter('sim').value
        
        self.controller_type = self.get_parameter('controller_type').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.get_logger().info(f'Running {self.controller_type} controller at {self.control_frequency} Hz')
        self.trajectory_type = self.get_parameter('type').value
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.initial_pos = None
        self.prev_pos = None
        self.prev_vel = np.zeros(3)
        self.angular_vel = np.zeros(3) # Updated by IMU subscriber
        # self.curr_pos = np.zeros(3)
        # self.target_pos = np.zeros(3)
        # self.curr_quat = np.zeros(4)
        
        # Subscribers and publishers
        self.est_vel_pub = self.create_publisher(TwistStamped, 'est_vel', 1) # Estimated velocity
        self.u_pub = self.create_publisher(TwistStamped, 'u_euler', 1) # ???
        self.cmd_stamped_pub = self.create_publisher(TwistStamped, 'cmd_vel_stamped', 1) # Timestamped velocity command
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_legacy', 1) # Velocity command for Crazyflie
        self.goal_pub = self.create_publisher(TwistStamped, 'goal', 1) # Target twist along trajectory
        self.tf_pub = self.create_publisher(PoseStamped, 'tf_pos', 1) # Position from tf
        
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10) # Onboard IMU data
        # self.target_sub = self.create_subscription(PoseStamped, '/vicon/crazy_target/pose', self.target_callback, 10) # TODO: set correctly
        # self.vicon_sub = self.create_subscription(PoseStamped, f'/vicon/{self.frame}/{self.frame}/pose', self.vicon_callback, 10) # TODO: set correctly
        
        self.state_sub = self.create_subscription(Int32, '/command/cmd_state', self.cmd_state_callback, 1) # Phase request
        self.ready_pub = self.create_publisher(String, '/command/cf_ready', 1) # Phase completion
        
        self.m_state = 0 # 0 = IDLE, 1 = TAKEOFF, 2 = TRAJECTORY, 3 = LANDING, 4 = SHUTDOWN
        self.aborted = False # Aborted due to high position
        self.ready_sent = False # Prevents duplicate sends
        
        self.trajectory_points = self.get_trajectory_points()
        self.traj = None # Set by state updates
        self.controller = None # Set by state updates
        
        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.prev_time = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(1.0 / self.control_frequency, self.timer_callback)
        
    def create_controller(self):
        """
        Creates the controller used for the main trajectory
        """
        match self.controller_type:
            case 'mpc':
                return MPControl(self.control_frequency)
            case 'hybrid':
                return HybridControl(self.control_frequency)
            case 'gp':
                return GPControl(self.control_frequency)
            case 'knode':
                return KNODEControl(self.control_frequency)
            case _:
                return GeometriControl()
                
    def get_trajectory_points(self):
        """
        Gets trajectory points based on trajectory type
        """
        match self.trajectory_type:
            case 'circle':
                radius = self.get_parameter('radius').value
                center = self.get_parameter('center').value
                
                t = np.linspace(0, 2*np.pi, 500)
                x = center[0] + radius * np.cos(t)
                y = center[1] + radius * np.sin(t)
                z = np.repeat(center[2], len(t))
                points = np.stack([x, y, z], axis=1)
                return points
                
            case 'waypoint':
                points = []
                idx = 0
                while True:
                    param_name = f'p{idx}'
                    try:
                        point = self.get_parameter(param_name).value
                    except rclpy.exceptions.ParameterNotDeclaredException:
                        break
                    points.append(point)
                    idx += 1
                return np.array(points)
                
            case 'linear':
                start = self.get_parameter('start').value
                end = self.get_parameter('end').value
                points = np.vstack([start, end])
                return points
                
            case 'figure8':
                center = self.get_parameter('center').value
                scale = self.get_parameter('scale').value
                
                t = np.linspace(0, 2*np.pi, 500)
                x = center[0] + scale[0] * np.sin(t)
                y = center[1] + scale[1] * np.sin(t) * np.cos(t)
                z = np.repeat(center[2], len(t))
                points = np.stack([x, y, z], axis=1)
                return points
                
            case 'spiral':
                center = self.get_parameter('center').value
                radius_start = self.get_parameter('radius_start').value
                radius_end = self.get_parameter('radius_end').value
                height_start = self.get_parameter('height_start').value
                height_end = self.get_parameter('height_end').value
                revolutions = self.get_parameter('revolutions').value
                
                t = np.linspace(0, 2*np.pi*revolutions, 500)
                radius = np.linspace(radius_start, radius_end, len(t))
                height = np.linspace(height_start, height_end, len(t))
                x = center[0] + radius * np.cos(t)
                y = center[1] + radius * np.sin(t)
                z = height
                points = np.stack([x, y, z], axis=1)
                return points
                
            case 'hover':
                position = self.get_parameter('position').value
                points = np.array([position])
                return points
                
            case 'tracking': # TODO
                self.get_logger().warn('Target tracking unavailable')
                points = np.array([self.initial_pos])
                return points
                
            case _:
                self.get_logger().warn(f"Trajectory type '{self.trajectory_type}' invalid")
                points = np.array([self.initial_pos])
                return points
            
    def imu_callback(self, data):
        """
        Callback function for IMU data (angular velocity)
        """
        imu_angular_vel = Vector3()
        imu_angular_vel = data.angular_velocity
        self.angular_vel[0] = imu_angular_vel.x
        self.angular_vel[1] = imu_angular_vel.y
        self.angular_vel[2] = imu_angular_vel.z
        
    # def vicon_callback(self, data):
    #     """
    #     Callback function for vicon positions
    #     """
    #     self.curr_pos[0] = data.pose.position.x
    #     self.curr_pos[1] = data.pose.position.y
    #     self.curr_pos[2] = data.pose.position.z
    #     self.curr_quat[0] = data.pose.orientation.x
    #     self.curr_quat[1] = data.pose.orientation.y
    #     self.curr_quat[2] = data.pose.orientation.z
    #     self.curr_quat[3] = data.pose.orientation.w
    
    # def target_callback(self, data):
    #     """
    #     Callback function for target position
    #     """
    #     self.target_pos[0] = data.pose.position.x
    #     self.target_pos[1] = data.pose.position.y
    #     self.target_pos[2] = data.pose.position.z
    
    def cmd_state_callback(self, msg: Int32):
        """
        Callback function for state changes
        """
        if msg.data == self.m_state:
            self.get_logger.warn('Already in state {msg.data}')

        match msg.data:
            case 1:
                self.get_logger().info('Takeoff requested!')
                setup_speed = self.get_parameter('setup_speed').value
                traj_start = self.trajectory_points[0]
                elevated_pos = np.array([self.initial_pos[0], self.initial_pos[1], traj_start[2]])
                self.traj = self.generate_traj(np.vstack([self.initial_pos, elevated_pos, traj_start]), setup_speed)
                self.controller = GeometriControl()
            case 2:
                self.get_logger().info('Trajectory requested!')
                desired_speed = self.get_parameter('desired_speed').value
                self.traj = self.generate_traj(self.trajectory_points, desired_speed)
                self.controller = self.create_controller()
            case 3:
                self.get_logger().info('Landing requested!')
                setup_speed = self.get_parameter('setup_speed').value
                traj_end = self.trajectory_points[-1]
                xf = self.get_parameter('x_final').value
                yf = self.get_parameter('y_final').value
                zf = self.get_parameter('z_final').value
                elevated_pos = np.array([xf, yf, traj_end[2]])
                final_pos = np.array([xf, yf, zf])
                self.traj = self.generate_traj(np.vstack([traj_end, elevated_pos, final_pos]), setup_speed)
                self.controller = GeometriControl()
            case 4:
                self.get_logger().info('Shutdown requested!')
            case _:
                self.get_logger().warn(f"State request '{msg.data}' invalid")
                return

        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.m_state = msg.data
        self.ready_sent = False
            
    def generate_traj(self, points, desired_speed):
        """
        Generates a trajectory object from waypoints
        """
        return wt.WaypointTraj(points, desired_speed)
        
    def timer_callback(self):
        """
        Timer callback for running MPC
        """
        curr_time = self.get_clock().now().nanoseconds / 1e9
        dt = curr_time - self.prev_time
        
        # Get position from tf
        if self.tf_buffer.can_transform(self.world_frame, self.frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=2.0)):
            transform = self.tf_buffer.lookup_transform(self.world_frame, self.frame, rclpy.time.Time())
        else:
            self.get_logger().fatal(f'Transform not available within timeout')
            return
            
        pos = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
        quat = np.array([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
        
        if self.initial_pos is None and np.abs(pos).sum() == 0 and not self.sim:
            self.get_logger().warn(f'Ignoring initial position {pos}. Is the server active?')
            return

        # Check for initial position reading
        if self.initial_pos is None:
            self.get_logger().info(f'Initial position {pos}')
            self.initial_pos = self.prev_pos = pos
            
        v = (pos - self.prev_pos) / dt
        v_est_sum = np.abs(v).sum()
        if v_est_sum < 1e-6:
            v = self.prev_vel
        v = np.clip(v, -0.7, 0.7)
        
        if self.m_state in (0, 4):
            msg = Twist()
            self.cmd_pub.publish(msg)
            if not self.ready_sent:
                self.ready_sent = True
                self.ready_pub.publish(String(data=self.frame))
            if self.m_state > 3:
                self.destroy_node()
                exit(0)
            return

        # Abort if above threshold
        if pos[2] >= self.get_parameter('z_max').value and not self.aborted:
            self.get_logger().fatal(f'Position {pos.round(5)} above threshold, aborting')
            self.aborted = True

        if self.aborted:
            msg = Twist()
            msg.linear.z = 30000. # TODO: land more gracefully
            self.cmd_pub.publish(msg)
            if pos[2] <= self.get_parameter('z_final').value:
                self.get_logger().warn('Abort complete, shutting down')
                self.m_state = 4
            return
        
        # if self.trajectory_type == 'tracking':
        #     interp_time = [1, 4]
        #     points = interp1d(interp_time, np.vstack([self.curr_pos, self.target_pos]), axis=0)([1, 2, 3, 4]) # Trajectory to target
        #     points[:, 2] += 0.35 # Altitude offset for safety
        #     self.traj = self.generate_traj(points)
        #     self.get_logger().debug(f'Target at {self.target_pos}')

        curr_state = {
            'x': pos,
            'v': v,
            'q': quat,
            'w': self.angular_vel
        }
        
        # Update controller
        flat = self.sanitize_trajectory_dic(self.traj.update(curr_time - self.t0))
        u = self.controller.update(curr_time, curr_state, flat)
        
        # Extract control values
        roll = float(u['euler'][0])
        pitch = float(u['euler'][1])
        yaw = float(u['euler'][2])
        assert(u['cmd_thrust'].size == 1)
        thrust = float(u['cmd_thrust'][0])
        r_ddot_des = u['r_ddot_des']
        
        # Create and publish command
        msg = Twist()
        msg.linear.x = np.clip(np.degrees(pitch), -10, 10) # Pitch
        msg.linear.y = np.clip(np.degrees(roll), -10, 10) # Roll
        msg.linear.z = self.map_u1(thrust) # Thrust
        msg.angular.z = np.degrees(0) # Yawrate (TODO: 0 for now)
        if not self.sim:
            self.cmd_pub.publish(msg)
        
        # Log data for debugging and visualization
        self.log_ros_info(roll, pitch, yaw, r_ddot_des, v, msg, flat, pos, quat, thrust)
        
        # Update state variables if valid data
        if v_est_sum > 1e-6:
            self.prev_vel = v
            self.prev_time = curr_time
            self.prev_pos = pos
            
        # Notify command if done
        if flat['done'] and not self.ready_sent:
            self.ready_sent = True
            self.ready_pub.publish(String(data=self.frame))
            
    def map_u1(self, u1):
        """
        Map control thrust (N) to cmd_vel thrust (PWM)
        """
        min_cmd = 0
        trim_cmd = 41000 # Hover thrust
        trim_u1 = 0.03 * 9.81 # Hover u1
        max_cmd = 60000. # Max thrust
        max_u1 = max_cmd / trim_cmd * trim_u1 # Max u1 (N)
        
        if u1 <= trim_u1:
            mapped_u1 = min_cmd + (trim_cmd - min_cmd) * u1 / trim_u1
        else:
            mapped_u1 = trim_cmd + (max_cmd - trim_cmd) * (u1 - trim_u1) / (max_u1 - trim_u1)
        
        return min(mapped_u1, max_cmd)
        
    def sanitize_trajectory_dic(self, trajectory_dic):
        """
        Return a sanitized version of the trajectory dictionary where all elements are numpy arrays
        """
        trajectory_dic['x'] = np.asarray(trajectory_dic['x'], float).ravel()
        trajectory_dic['x_dot'] = np.asarray(trajectory_dic['x_dot'], float).ravel()
        trajectory_dic['x_ddot'] = np.asarray(trajectory_dic['x_ddot'], float).ravel()
        trajectory_dic['x_dddot'] = np.asarray(trajectory_dic['x_dddot'], float).ravel()
        trajectory_dic['x_ddddot'] = np.asarray(trajectory_dic['x_ddddot'], float).ravel()
        return trajectory_dic

    def log_ros_info(self, roll, pitch, yaw, r_ddot_des, est_v, cmd_msg, flat, tf_pos, tf_quat, thrust):
        """
        Publish current status
        """
        # Controller outputs
        curr_log_time = self.get_clock().now().to_msg()
        u_msg = TwistStamped()
        u_msg.header.stamp = curr_log_time
        # Roll, pitch, and yaw are mapped to TwistStamped angular
        u_msg.twist.angular.x = roll
        u_msg.twist.angular.y = pitch
        u_msg.twist.angular.z = yaw
        # r_ddot_des is mapped to TwistStamped linear
        u_msg.twist.linear.x = float(r_ddot_des[0])
        u_msg.twist.linear.y = float(r_ddot_des[1])
        u_msg.twist.linear.z = float(r_ddot_des[2])
        
        # Estimated velocities
        est_v_msg = TwistStamped()
        est_v_msg.header.stamp = curr_log_time
        # Estimated velocities are mapped to TwistStamped linear
        est_v_msg.twist.linear.x = est_v[0]
        est_v_msg.twist.linear.y = est_v[1]
        est_v_msg.twist.linear.z = est_v[2]
        
        # Timestamped velocity command
        cmd_stamped_msg = TwistStamped()
        cmd_stamped_msg.header.stamp = curr_log_time
        cmd_stamped_msg.twist.linear.x = cmd_msg.linear.x
        cmd_stamped_msg.twist.linear.y = cmd_msg.linear.y
        cmd_stamped_msg.twist.linear.z = cmd_msg.linear.z
        cmd_stamped_msg.twist.angular.z = cmd_msg.angular.z
        cmd_stamped_msg.twist.angular.x = thrust
        
        # Target twist
        traj_msg = TwistStamped()
        traj_msg.header.stamp = curr_log_time
        traj_msg.twist.linear.x = flat['x'][0]
        traj_msg.twist.linear.y = flat['x'][1]
        traj_msg.twist.linear.z = flat['x'][2]
        traj_msg.twist.angular.x = flat['x_dot'][0]
        traj_msg.twist.angular.y = flat['x_dot'][1]
        traj_msg.twist.angular.z = flat['x_dot'][2]
        
        # Position from tf
        tf_pose_msg = PoseStamped()
        tf_pose_msg.header.stamp = curr_log_time
        tf_pose_msg.pose.position.x = tf_pos[0]
        tf_pose_msg.pose.position.y = tf_pos[1]
        tf_pose_msg.pose.position.z = tf_pos[2]
        tf_pose_msg.pose.orientation.x = tf_quat[0]
        tf_pose_msg.pose.orientation.y = tf_quat[1]
        tf_pose_msg.pose.orientation.z = tf_quat[2]
        tf_pose_msg.pose.orientation.w = tf_quat[3]
        
        # Publish messages
        self.u_pub.publish(u_msg)
        self.est_vel_pub.publish(est_v_msg)
        self.cmd_stamped_pub.publish(cmd_stamped_msg)
        self.goal_pub.publish(traj_msg)
        self.tf_pub.publish(tf_pose_msg)
        
def main(args=None):
    rclpy.init(args=args)
    mpc_demo = MPCDemo()
    rclpy.spin(mpc_demo)
    
if __name__ == '__main__':
    main()
