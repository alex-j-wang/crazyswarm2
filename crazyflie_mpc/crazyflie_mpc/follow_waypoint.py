#!/usr/bin/env python3

import numpy as np
import rclpy
import rclpy.duration
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from tf2_ros import TransformListener, Buffer
from std_msgs.msg import String, Int32

from scipy.spatial.transform import Rotation
import waypoint_traj as wt
from mpc_control import MPControl
from hybrid_control import HybridControl
from geometric_control import GeometriControl
from gp_control import GPControl
from scipy.interpolate import interp1d

class MPCDemo(Node):
    def __init__(self):
        super().__init__(
            'mpc_demo',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True # TODO: this could lead to some silent bugs
        )
        
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.frame = self.get_parameter('frame').get_parameter_value().string_value
        # quad_name = self.frame

        self.controller_type = self.get_parameter('controller_type').get_parameter_value().string_value   
        self.control_frequency = self.get_parameter('control_frequency').value

        self.trajectory_type = self.get_parameter('type').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # subscribers and publishers
        self.timer = self.create_timer(1.0 / self.control_frequency, self.timer_callback)
        self.angular_vel = np.zeros([3,])  # angular velocity updated by imu subscriber
        # self.curr_pos = np.zeros([3,])
        # self.target_pos = np.zeros([3,])
        # self.curr_quat = np.zeros([4,])
        self.est_vel_pub = self.create_publisher(TwistStamped, 'est_vel', 1)  # publishing estimated velocity
        self.u_pub = self.create_publisher(TwistStamped, 'u_euler', 1)  # publishing stamped 
        self.cmd_stamped_pub = self.create_publisher(TwistStamped, 'cmd_vel_stamped', 1)  # publishing time stamped cmd_vel
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10)  # subscribing imu
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_legacy', 1)  # publishing to cmd_vel to control crazyflie
        self.goal_pub = self.create_publisher(TwistStamped, 'goal', 1)  # publishing waypoints along the trajectory        
        # self.target_sub = self.create_subscription(PoseStamped, "/vicon/crazy_target/pose", self.target_callback, 10) # TODO: set correctly
        # self.vicon_sub = self.create_subscription(PoseStamped, f'/vicon/{quad_name}/{quad_name}/pose', self.vicon_callback, 10) 
        self.tf_pub = self.create_publisher(PoseStamped, 'tf_pos', 1)

        # command topics
        self.state_sub = self.create_subscription(Int32, '/command/cmd_state', self.cmd_state_callback, 1)  # subscribing to state changes
        self.ready_pub = self.create_publisher(String, '/command/cf_ready', 1)  # publishing state completion
        
        # controller and waypoint
        self.m_state = 0 # Idle: 0, TakingOff: 1, Automatic: 2, Landing: 3, Stopping: 4
        self.ready_sent = False  # Flag to check if ready message has been sent
        # self.m_thrust = 0
        # self.m_startZ = 0
        
        self.trajectory_points = self.get_trajectory_points()
        self.traj = None
        self.controller = None
        
        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.prev_time = self.get_clock().now().nanoseconds / 1e9
        self.initial_pos = None
        self.prev_pos = None
        self.prev_vel = np.zeros(3)
            
    def create_controller(self):
        match self.controller_type:
            case "mpc":
                return MPControl(self.control_frequency)
            case "hybrid":
                return HybridControl()
            case "gp":
                return GPControl()
            case _:
                return GeometriControl()
    
    def get_trajectory_points(self):
        """
        Get trajectory points based on the trajectory type
        """
        trajectory_type = self.trajectory_type

        if trajectory_type == "circle":
            radius = self.get_parameter("radius").value
            height = self.get_parameter("height").value
            center = self.get_parameter("center").value
            duration = self.get_parameter("duration").value
                
            t_plot = np.linspace(0, duration, num=500)
            x_traj = radius * np.cos(t_plot) + center[0]
            y_traj = radius * np.sin(t_plot) + center[1]
            z_traj = np.repeat(height, len(t_plot))
            points = np.stack((x_traj, y_traj, z_traj), axis=1)
            return points
            
        elif trajectory_type == "waypoint":
            points = []
            for point_index in range(self.get_parameter('npoints').value):
                param_name = f"point_{point_index}"
                point = self.get_parameter(param_name).value
                points.append(point)
            return np.array(points)
        
        elif trajectory_type == "linear":
            start = np.array(self.get_parameter("start").value)
            end = np.array(self.get_parameter("end").value)

            points = np.vstack((start, end))
            return points
            
        elif trajectory_type == "figure8":
            center = self.get_parameter("center").value
            scale = self.get_parameter("scale").value
            
            t = np.linspace(0, 2*np.pi, 500)
            x = center[0] + scale[0] * np.sin(t)
            y = center[1] + scale[1] * np.sin(t) * np.cos(t)
            z = np.ones_like(t) * center[2]
            points = np.stack([x, y, z], axis=1)
            return points
            
        elif trajectory_type == "spiral":
            center = self.get_parameter("center").value
            radius_start = self.get_parameter("radius_start").value
            radius_end = self.get_parameter("radius_end").value
            height_start = self.get_parameter("height_start").value
            height_end = self.get_parameter("height_end").value
            revolutions = self.get_parameter("revolutions").value
            duration = self.get_parameter("duration").value
                
            t = np.linspace(0, 2*np.pi*revolutions, 500)
            radius = np.linspace(radius_start, radius_end, len(t))
            height = np.linspace(height_start, height_end, len(t))
            x = center[0] + radius * np.cos(t)
            y = center[1] + radius * np.sin(t)
            z = height
            points = np.stack([x, y, z], axis=1)
            return points
            
        elif trajectory_type == "hover":
            position = self.get_parameter("position").value
            points = np.array([position])
            return points
        
        elif trajectory_type == "tracking": # TODO
            self.get_logger().fatal("Target tracking unavailable")
            return [[0, 0, 0]]
        
        else:
            self.get_logger().fatal(f"Trajectory type '{trajectory_type}' not recognized")
            return [[0, 0, 0]]

    def imu_callback(self, data):
        '''
        callback function for getting current angular velocity
        '''
        imu_angular_vel = Vector3()
        imu_angular_vel = data.angular_velocity
        self.angular_vel[0] = imu_angular_vel.x
        self.angular_vel[1] = imu_angular_vel.y
        self.angular_vel[2] = imu_angular_vel.z

    # def vicon_callback(self, data):
    #     '''
    #     callback function for getting vicon positions
    #     '''
    #     self.curr_pos[0] = data.pose.position.x
    #     self.curr_pos[1] = data.pose.position.y
    #     self.curr_pos[2] = data.pose.position.z
    #     self.curr_quat[0] = data.pose.orientation.x
    #     self.curr_quat[1] = data.pose.orientation.y
    #     self.curr_quat[2] = data.pose.orientation.z
    #     self.curr_quat[3] = data.pose.orientation.w


    # def target_callback(self, data):
    #     self.target_pos[0] = data.pose.position.x
    #     self.target_pos[1] = data.pose.position.y
    #     self.target_pos[2] = data.pose.position.z

    def cmd_state_callback(self, msg: Int32):
        '''
        callback function for state changes
        '''
        if msg.data != self.m_state:
            match msg.data:
                case 1:
                    self.get_logger().info("Takeoff requested!")
                    traj_start = self.trajectory_points[0]
                    elevated_pos = np.array([self.initial_pos[0], self.initial_pos[1], traj_start[2]])
                    self.traj = self.generate_traj(np.vstack((self.initial_pos, elevated_pos, traj_start)))
                    self.controller = GeometriControl()
                case 2:
                    self.get_logger().info("Trajectory requested!")
                    self.traj = self.generate_traj(self.trajectory_points)
                    self.controller = self.create_controller()
                case 3:
                    self.get_logger().info("Landing requested!")
                    traj_end = self.trajectory_points[-1]
                    elevated_pos = np.array([self.initial_pos[0], self.initial_pos[1], traj_end[2]])
                    final_pos = np.array([self.initial_pos[0], self.initial_pos[1], self.get_parameter('z_final').value])
                    self.traj = self.generate_traj(np.vstack((traj_end, elevated_pos, final_pos)))
                    self.controller = GeometriControl()
                case 4:
                    self.get_logger().info("Shutdown requested!")
                    msg = Twist()
                    self.cmd_pub.publish(msg)
                    rclpy.shutdown()
                case _:
                    self.get_logger().warn(f"Invalid state request {msg.data}")
            self.t0 = self.get_clock().now().nanoseconds / 1e9
            self.m_state = msg.data
            self.ready_sent = False

    def generate_traj(self, points):
        '''
        returns trajectory object generated from points
        '''
        return wt.WaypointTraj(points)
    
    def timer_callback(self):  # running MPC
        curr_time = self.get_clock().now().nanoseconds / 1e9
        dt = curr_time - self.prev_time
        # if self.trajectory_type == 'tracking':
        #     # For target tracking, create a trajectory between current position and target
        #     interp_time = [1,4]
        #     points = interp1d(interp_time, np.vstack([self.curr_pos, self.target_pos]), axis=0)([1,2,3,4])
        #     # Add altitude offset for safety
        #     points[:, 2] += 0.35
        #     # Generate new trajectory to target
        #     self.traj = self.generate_traj(points)
        #     self.get_logger().debug(f"Target tracking: moving to {self.target_pos}")

        # Get position from tf
        if self.tf_buffer.can_transform(self.world_frame, self.frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=2.0)):
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.frame,
                rclpy.time.Time())
        else:
            self.get_logger().warn(f"Transform not available within timeout")
            return

        pos = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
        quat = np.array([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])

        if self.initial_pos is None:
            self.get_logger().info(f"Initial position {pos}")
            self.initial_pos = self.prev_pos = pos
            self.ready_sent = True
            self.ready_pub.publish(String(data=self.frame))  # publish ready message
            
        v = (pos - self.prev_pos) / dt # velocity estimate
        v_est_sum = np.sum(np.abs(v))
        if v_est_sum < 1e-6:
            v = self.prev_vel
        v = np.clip(v, -0.7, 0.7)

        if self.m_state == 0:
            msg = Twist()
            self.cmd_pub.publish(msg)
            return
        
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
        msg.linear.x = np.clip(np.degrees(pitch), -10., 10.)  # pitch
        msg.linear.y = np.clip(np.degrees(roll), -10., 10.)  # roll
        msg.linear.z = self.map_u1(thrust)
        msg.angular.z = np.degrees(0.)  # hardcoding yawrate to be 0 for now
        self.cmd_pub.publish(msg) # publishing msg to the crazyflie

        # Log data for debugging and visualization
        self.log_ros_info(roll, pitch, yaw, r_ddot_des, v, msg, flat, pos, quat, thrust)
        
        # Update state variables for next iteration if we have valid data
        if v_est_sum > 1e-6:
            self.prev_vel = v
            self.prev_time = curr_time
            self.prev_pos = pos

        if flat['done'] and not self.ready_sent:
            self.ready_sent = True
            self.ready_pub.publish(String(data=self.frame))

    def map_u1(self, u1):  # mapping control thrust output to cmd_vel thrust
        # u1 ranges from -0.2 to 0.2
        trim_cmd = 53000 # was 43000
        min_cmd = 20000 # was 10000
        u1_trim = 0.327
        c = min_cmd
        m = (trim_cmd - min_cmd)/u1_trim
        mapped_u1 = u1*m + c
        if mapped_u1 > 60000:
            mapped_u1 = float(60000)
        return mapped_u1

    def sanitize_trajectory_dic(self, trajectory_dic):
        """
        Return a sanitized version of the trajectory dictionary where all of the elements are np arrays
        """
        trajectory_dic['x'] = np.asarray(trajectory_dic['x'], float).ravel()
        trajectory_dic['x_dot'] = np.asarray(trajectory_dic['x_dot'], float).ravel()
        trajectory_dic['x_ddot'] = np.asarray(trajectory_dic['x_ddot'], float).ravel()
        trajectory_dic['x_dddot'] = np.asarray(trajectory_dic['x_dddot'], float).ravel()
        trajectory_dic['x_ddddot'] = np.asarray(trajectory_dic['x_ddddot'], float).ravel()
        return trajectory_dic


    def log_ros_info(self, roll, pitch, yaw, r_ddot_des, est_v, cmd_msg, flat, tf_pos, tf_quat, thrust):
        '''
        logging information from this demo
        '''
        # logging controller outputs
        curr_log_time = self.get_clock().now().to_msg()
        u_msg = TwistStamped()
        u_msg.header.stamp = curr_log_time
        # roll, pitch, and yaw are mapped to TwistStamped angular
        u_msg.twist.angular.x = roll          
        u_msg.twist.angular.y = pitch
        u_msg.twist.angular.z = yaw
        # r_ddot_des is mapped to TwistStamped linear
        u_msg.twist.linear.x = float(r_ddot_des[0])         
        u_msg.twist.linear.y = float(r_ddot_des[1])
        u_msg.twist.linear.z = float(r_ddot_des[2])
        
        # logging estimate velocities
        est_v_msg = TwistStamped()
        est_v_msg.header.stamp = curr_log_time
        # estimated velocities are mapped to TwistStampedow()
        est_v_msg.twist.linear.x = est_v[0]  
        est_v_msg.twist.linear.y = est_v[1]
        est_v_msg.twist.linear.z = est_v[2]
        
        # logging time stamped cmd_vel
        cmd_stamped_msg = TwistStamped()
        cmd_stamped_msg.header.stamp = curr_log_time
        cmd_stamped_msg.twist.linear.x = cmd_msg.linear.x
        cmd_stamped_msg.twist.linear.y = cmd_msg.linear.y
        cmd_stamped_msg.twist.linear.z = cmd_msg.linear.z
        cmd_stamped_msg.twist.angular.z = cmd_msg.angular.z
        cmd_stamped_msg.twist.angular.x = thrust

        # logging waypoints
        traj_msg = TwistStamped()
        traj_msg.header.stamp = curr_log_time
        traj_msg.twist.linear.x = flat['x'][0]
        traj_msg.twist.linear.y = flat['x'][1]
        traj_msg.twist.linear.z = flat['x'][2]
        traj_msg.twist.angular.x = flat['x_dot'][0]
        traj_msg.twist.angular.y = flat['x_dot'][1]
        traj_msg.twist.angular.z = flat['x_dot'][2]

        # logging position from tf
        tf_pose_msg = PoseStamped()
        tf_pose_msg.header.stamp = curr_log_time
        tf_pose_msg.pose.position.x = tf_pos[0]
        tf_pose_msg.pose.position.y = tf_pos[1]
        tf_pose_msg.pose.position.z = tf_pos[2]
        tf_pose_msg.pose.orientation.x = tf_quat[0]
        tf_pose_msg.pose.orientation.y = tf_quat[1]
        tf_pose_msg.pose.orientation.z = tf_quat[2]
        tf_pose_msg.pose.orientation.w = tf_quat[3]


        # publishing the messages
        self.u_pub.publish(u_msg)
        self.est_vel_pub.publish(est_v_msg)
        self.cmd_stamped_pub.publish(cmd_stamped_msg)
        self.goal_pub.publish(traj_msg)
        self.tf_pub.publish(tf_pose_msg)

def main(args=None):
    rclpy.init(args=args)
    mpc_demo = MPCDemo()
    rclpy.spin(mpc_demo)
    mpc_demo.destroy_node()

if __name__ == '__main__':
    main()

