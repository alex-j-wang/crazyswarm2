#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from tf2_ros import TransformListener, Buffer
from std_msgs.msg import String

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
            automatically_declare_parameters_from_overrides=True
        ) # initializing node
        # self.m_serviceLand = self.create_service('land', , self.landingService)
        # self.m_serviceTakeoff = self.create_service('takeoff', , self.takeoffService)
        
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        quad_name = self.get_parameter('frame').get_parameter_value().string_value
        self.frame = quad_name
        self.trajectory_type = self.get_parameter('trajectory_type').get_parameter_value().string_value
        self.controller_type = self.get_parameter('controller_type').get_parameter_value().string_value
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # subscribers and publishers
        self.timer = self.create_timer(1.0/370.0, self.timer_callback)
        self.angular_vel = np.zeros([3,])  # angular velocity updated by imu subscriber
        self.curr_pos = np.zeros([3,])
        self.target_pos = np.zeros([3,])
        self.curr_quat = np.zeros([4,])
        self.est_vel_pub = self.create_publisher(TwistStamped, 'est_vel', 1)  # publishing estimated velocity
        self.u_pub = self.create_publisher(TwistStamped, 'u_euler', 1)  # publishing stamped 
        self.cmd_stamped_pub = self.create_publisher(TwistStamped, 'cmd_vel_stamped', 1)  # publishing time stamped cmd_vel
        self.imu_sub = self.create_subscription(Imu, '/crazyflie/imu', self.imu_callback, 10)  # subscribing imu
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 1)  # publishing to cmd_vel to control crazyflie
        self.goal_pub = self.create_publisher(TwistStamped, 'goal', 1)  # publishing waypoints along the trajectory        
        self.target_sub = self.create_subscription(PoseStamped, "/vicon/crazy_target/pose", self.target_callback, 10)
        self.vicon_sub = self.create_subscription(PoseStamped, "/vicon/" + quad_name + "/pose", self.vicon_callback, 10) 
        self.tf_pub = self.create_publisher(PoseStamped, 'tf_pos', 1)
        
        # controller and waypoint
        self.m_state = 0 # Idle: 0, Automatic: 1, TakingOff: 2, Landing: 3
        self.m_thrust = 0
        self.m_startZ = 0
        
        # Get trajectory points based on trajectory type
        points = self.get_trajectory_points()
        self.traj = self.generate_traj(points)  # trajectory
        
        ############# CONTROLLER ########### 
        self.controller = self.create_controller()
        
        self.initial_state = {'x': np.array([0, 0, 0]), # positions
                              'v': np.array([0, 0, 0]), # velocities
                              'q': np.array([0, 0, 0, 1]), # quaternion
                              'w': np.zeros(3,)} # angular vel
        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.prev_time = self.get_clock().now().nanoseconds / 1e9
        self.prev_pos = self.initial_state['x']
        self.prev_vel = np.zeros([3,])
        self.get_logger().info("=============== MPC Demo Initialized ===============")
        self.get_logger().info(f"Trajectory type: {self.trajectory_type}")
        self.get_logger().info(f"Controller type: {self.controller_type}")
    
    def create_controller(self):
        controller_type = self.controller_type
        if controller_type == "mpc":
            return MPControl()
        elif controller_type == "hybrid":
            return HybridControl()
        elif controller_type == "gp":
            return GPControl()
        else:
            return GeometriControl()
    
    def get_trajectory_points(self):
        """
        Get trajectory points based on the trajectory type
        """
        trajectory_type = self.trajectory_type

        if trajectory_type == "circle":
            radius = self.get_parameter("trajectories.circle.radius").value
            height = self.get_parameter("trajectories.circle.height").value
            center = self.get_parameter("trajectories.circle.center").value
            duration = self.get_parameter("trajectories.circle.duration").value
                
            t_plot = np.linspace(0, duration, num=500)
            x_traj = radius * np.cos(t_plot) + center[0]
            y_traj = radius * np.sin(t_plot) + center[1]
            z_traj = np.zeros((len(t_plot),)) + height
            points = np.stack((x_traj, y_traj, z_traj), axis=1)
            points[-1, 2] = 0.2  # End with lower height
            return points
            
        elif trajectory_type == "square":
            points = []
            point_index = 0
            
            while True:
                param_name = f"trajectories.square.point_{point_index}"
                try:
                    point = self.get_parameter(param_name).value
                    points.append(point)
                    point_index += 1
                except:
                    break
          
            return np.array(points)
        
        elif trajectory_type == "linear":
            start = self.get_parameter("trajectories.linear.start").value
            end = self.get_parameter("trajectories.linear.end").value
            steps = self.get_parameter("trajectories.linear.steps").value

            rl_pt_count = steps
            rl_x = np.ones([rl_pt_count*2-1, 1]) * start[0]
            rl_z = np.ones([rl_pt_count*2-1, 1]) * start[2]
            rl_y_to = np.linspace(start[1], end[1], rl_pt_count)
            rl_y_fro = np.linspace(rl_y_to[-2], start[1], rl_pt_count-1)
            rl_y = np.concatenate([rl_y_to, rl_y_fro], 0)
            points = np.concatenate([rl_x, rl_y.reshape([-1, 1]), rl_z], 1)
            points[-2, 2] = 0.4
            points[-1, 2] = 0.2
            return points
            
        elif trajectory_type == "figure8":
            center = self.get_parameter("trajectories.figure8.center").value
            scale = self.get_parameter("trajectories.figure8.scale").value
            duration = self.get_parameter("trajectories.figure8.duration").value
                
            t = np.linspace(0, 2*np.pi, 500)
            x = center[0] + scale[0] * np.sin(t)
            y = center[1] + scale[1] * np.sin(t) * np.cos(t)
            z = np.ones_like(t) * center[2]
            points = np.vstack([x, y, z]).T
            return points
            
        elif trajectory_type == "spiral":
            center = self.get_parameter("trajectories.spiral.center").value
            radius_start = self.get_parameter("trajectories.spiral.radius_start").value
            radius_end = self.get_parameter("trajectories.spiral.radius_end").value
            height_start = self.get_parameter("trajectories.spiral.height_start").value
            height_end = self.get_parameter("trajectories.spiral.height_end").value
            revolutions = self.get_parameter("trajectories.spiral.revolutions").value
            duration = self.get_parameter("trajectories.spiral.duration").value
                
            t = np.linspace(0, 2*np.pi*revolutions, 500)
            radius = np.linspace(radius_start, radius_end, len(t))
            height = np.linspace(height_start, height_end, len(t))
            x = center[0] + radius * np.cos(t)
            y = center[1] + radius * np.sin(t)
            z = height
            points = np.vstack([x, y, z]).T
            return points
            
        elif trajectory_type == "hover":
            position = self.get_parameter("trajectories.hover.position").value
            duration = self.get_parameter("trajectories.hover.duration").value
                
            # Generate simple hover path (just one point)
            points = np.array([position])
            return points
        
        else:
            # Default to circle if trajectory not recognized
            self.get_logger().warning(f"Trajectory type '{trajectory_type}' not recognized. Using circle.")
            radius = self.get_parameter("trajectories.circle.radius").value
            height = self.get_parameter("trajectories.circle.height").value
            center = self.get_parameter("trajectories.circle.center").value
            duration = self.get_parameter("trajectories.circle.duration").value
                
            t_plot = np.linspace(0, duration, num=500)
            x_traj = radius * np.cos(t_plot) + center[0]
            y_traj = radius * np.sin(t_plot) + center[1]
            z_traj = np.zeros((len(t_plot),)) + height
            points = np.stack((x_traj, y_traj, z_traj), axis=1)
            points[-1, 2] = 0.2  # End with lower height
            return points

    def timer_callback(self):
        # Check and update state machine
        if self.m_state == 0:
            self.idle()
        elif self.m_state == 3:
            self.land()
        elif self.m_state == 1:
            self.automatic(True)
        elif self.m_state == 2:
            self.takeoff()

    def imu_callback(self, data):
        '''
        callback function for getting current angular velocity
        '''
        imu_angular_vel = Vector3()
        imu_angular_vel = data.angular_velocity
        self.angular_vel[0] = imu_angular_vel.x
        self.angular_vel[1] = imu_angular_vel.y
        self.angular_vel[2] = imu_angular_vel.z

    def vicon_callback(self, data):
        '''
        callback function for getting vicon positions
        '''
        self.curr_pos[0] = data.pose.position.x
        self.curr_pos[1] = data.pose.position.y
        self.curr_pos[2] = data.pose.position.z
        self.curr_quat[0] = data.pose.orientation.x
        self.curr_quat[1] = data.pose.orientation.y
        self.curr_quat[2] = data.pose.orientation.z
        self.curr_quat[3] = data.pose.orientation.w


    def target_callback(self, data):
        self.target_pos[0] = data.pose.position.x
        self.target_pos[1] = data.pose.position.y
        self.target_pos[2] = data.pose.position.z


    def takeoff(self):  # TODO
        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.frame,
                rclpy.time.Time())
            if transform.transform.translation.z > 0 + 0.1: # when the quad has lifted off
                self.m_state = 1 # switch to automatic
        except:
            pass

    def landingService(self, req, res):  # TODO
        pass

    def generate_traj(self, points):
        '''
        returns trajectory object generated from points
        '''
        return wt.WaypointTraj(points) 
    
    def takeoffService(self, req, res):  # TODO
        self.get_logger().info("Takeoff requested!")
        m_state = 2  # set state to taking off
        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.frame,
                rclpy.time.Time())
            self.m_startZ = transform.transform.translation.z  # set z coor for start position
        except:
            pass

    
    def land(self):  # TODO
        self.get_logger().info("landing")
        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.frame,
                rclpy.time.Time())
            pos = transform.transform.translation
            if pos.z <= self.initial_state['x'][2] + 0.05:
                self.m_state = 0
                msg = Twist()
                self.cmd_pub.publish(msg)
        except:
            pass
    
    def automatic(self, target_tracking):  # running MPC
        curr_time = self.get_clock().now().nanoseconds / 1e9
        dt = curr_time - self.prev_time
        if target_tracking:
            interp_time = [1,4]
            # print("self:, target:", self.curr_pos, self.target_pos)
            points = interp1d(interp_time, np.vstack([self.curr_pos, self.target_pos]), axis=0)([1,2,3,4])
            
            points[:, 2] += 0.35
            self.traj = self.generate_traj(points)

        flat = self.sanitize_trajectory_dic(self.traj.update(curr_time-self.t0))

        try:
            # Getting position from tf
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.frame,
                rclpy.time.Time())
            
            tf_pos = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
            tf_quat = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
            
            vicon_pos = self.curr_pos
            vicon_quat = self.curr_quat
            pos = tf_pos
            quat = tf_quat

            v = (np.array(pos)-np.array(self.prev_pos))/dt  # velocity estimate
            v_est_sum = np.sum(v)
            if v_est_sum == 0.0:  # only update velocity if tf pos has changed
                v = self.prev_vel
            
            # clipping
            v = np.clip(v, -0.7, 0.7)
            
            curr_state = {
                        'x': np.array(pos),
                        'v': v,
                        'q': np.array(quat),
                        'w': self.angular_vel}
     
            # controller update
            u = self.controller.update(curr_time, curr_state, flat)
            roll = u['euler'][0]
            pitch = u['euler'][1]
            yaw = u['euler'][2]
            thrust = u['cmd_thrust']
            r_ddot_des = u['r_ddot_des']
            u1 = u['cmd_thrust']

            def map_u1(u1):  # mapping control thrust output to cmd_vel thrust
                # u1 ranges from -0.2 to 0.2
                trim_cmd = 53000 # was 43000
                min_cmd = 20000 # was 10000
                u1_trim = 0.327
                c = min_cmd
                m = (trim_cmd - min_cmd)/u1_trim
                mapped_u1 = u1*m + c
                if mapped_u1 > 60000:
                    mapped_u1 = 60000
                return mapped_u1


            # publish command
            msg = Twist()
            msg.linear.x = np.clip(np.degrees(pitch), -10., 10.)  # pitch
            msg.linear.y = np.clip(np.degrees(roll), -10., 10.)  # roll
            msg.linear.z = map_u1(thrust)
            msg.angular.z = np.degrees(0.) # hardcoding yawrate to be 0 for now
            self.cmd_pub.publish(msg) # publishing msg to the crazyflie

            # logging
            self.log_ros_info(roll, pitch, yaw, r_ddot_des, v, msg, flat, tf_pos, tf_quat, u1)
            
            if v_est_sum != 0:  # only update previous values if tf pos has changed
                self.prev_vel = v
                self.prev_time = curr_time
                self.prev_pos = pos
                
        except Exception as e:
            self.get_logger().error(f"Error in automatic function: {e}")

    def idle(self):
        '''
        publish zero commands for 3 seconds before switching to automatic
        '''
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.t0 <= 3:
            msg = Twist()
            self.cmd_pub.publish(msg)
        else:
            self.m_state = 1
            self.prev_time = self.get_clock().now().nanoseconds / 1e9
            self.t0 = self.get_clock().now().nanoseconds / 1e9

    def takeoff0(self):  # TODO
        msg = Twist()

        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.frame,
                rclpy.time.Time())
                
            z_ = transform.transform.translation.z
            while z_ <= 0.2:
                if self.m_thrust > 50000:
                    break
                self.m_thrust += 10000 * 0.002
                self.cmd_pub.publish(msg)
                transform = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    self.frame,
                    rclpy.time.Time())
                z_ = transform.transform.translation.z
            
            self.m_state = 1
            self.prev_time = self.get_clock().now().nanoseconds / 1e9
            self.t0 = self.get_clock().now().nanoseconds / 1e9
        except:
            pass

 
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


    def log_ros_info(self, roll, pitch, yaw, r_ddot_des, est_v, cmd_msg, flat, tf_pos, tf_quat, u1):
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
        u_msg.twist.linear.x = r_ddot_des[0]         
        u_msg.twist.linear.y = r_ddot_des[1]
        u_msg.twist.linear.z = r_ddot_des[2]
        
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
        cmd_stamped_msg.twist.angular.x = u1

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
    rclpy.shutdown()

if __name__ == '__main__':
    main()

