#!/usr/bin/env python
import numpy as np
import rospy
import tf2_ros as tf
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from tf import TransformListener
from std_msgs.msg import String

from scipy.spatial.transform import Rotation
import waypoint_traj as wt
from mpc_control import MPControl
from hybrid_control import HybridControl
from geometric_control import GeometriControl
from gp_control import GPControl
from scipy.interpolate import interp1d

class MPCDemo():
    def __init__(self):
        rospy.init_node('mpc_demo', anonymous=True)  # initializing node
        # self.m_serviceLand = rospy.Service('land', , self.landingService)
        # self.m_serviceTakeoff = rospy.Service('takeoff', , self.takeoffService)
        # frames and transforms
        self.worldFrame = rospy.get_param("~world_frame", "world")
        quad_name = rospy.get_param("~frame")
        self.frame = quad_name
        #self.frame = rospy.get_param("~frame")
        self.tf_listener = TransformListener()
        
        # subscribers and publishers
        self.rate = rospy.Rate(370)
        self.angular_vel = np.zeros([3,])  # angular velocity updated by imu subscriber
        self.curr_pos = np.zeros([3,])
        self.target_pos = np.zeros([3,])
        self.curr_quat = np.zeros([4,])
        self.est_vel_pub = rospy.Publisher('est_vel', TwistStamped, queue_size=1)  # publishing estimated velocity
        self.u_pub = rospy.Publisher('u_euler', TwistStamped, queue_size=1)  # publishing stamped 
        self.cmd_stamped_pub = rospy.Publisher('cmd_vel_stamped', TwistStamped, queue_size=1)  # publishing time stamped cmd_vel
        self.imu_sub = rospy.Subscriber('/crazyflie/imu', Imu, self.imu_callback)  # subscribing imu
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)  # publishing to cmd_vel to control crazyflie
        self.goal_pub = rospy.Publisher('goal', TwistStamped, queue_size=1)  # publishing waypoints along the trajectory        
        self.target_sub = rospy.Subscriber("/vicon/crazy_target/pose", PoseStamped, self.target_callback)
        self.vicon_sub = rospy.Subscriber("/vicon/" + quad_name + "/pose", PoseStamped, self.vicon_callback) 
        self.tf_pub = rospy.Publisher('tf_pos', PoseStamped, queue_size=1)
        # controller and waypoint, PoseStamped, self.target_callbacks
        self.m_state = 0 # Idle: 0, Automatic: 1, TakingOff: 2, Landing: 3
        self.m_thrust = 0
        self.m_startZ = 0
        
        t_final = 12
        radius = 0.5
        height = 0.55
        center_x = 0.2172 #0.1728 # 0.2172
        center_y = 4.5455#7.8015 # 4.5455
        t_plot = np.linspace(0, t_final, num=500)
        # circle center of the circle is 0.2172, 4.5455
        x_traj = radius * np.cos(t_plot) + center_x
        y_traj = radius * np.sin(t_plot) + center_y
  
        z_traj = np.zeros((len(t_plot),)) + height
        points = np.stack((x_traj, y_traj, z_traj), axis=1)
        points[-1, 2] = 0.2
        #points = np.array([  # points for generating trajectory
        #                   [-1.409, 2.826, 0.55],
        #                   [1.609, 2.826, 0.55],
        #                   [1.609, 5.826, 0.55],
        #                   [-1.409, 5.826, 0.55],
        #                   [-1.409, 2.826, 0.55],
        #                   [-1.409, 2.826, 0.3],
        #                   [-1.409, 2.826, 0.0]])
        rl_pt_count = 12
        rl_x = np.ones([rl_pt_count*2-1,1])*0.1376
        rl_z = np.ones([rl_pt_count*2-1,1])*0.7
        rl_y_to = np.linspace(1.80, 6.73, rl_pt_count)
        rl_y_fro = np.linspace(rl_y_to[-2], 1.80, rl_pt_count-1)
        rl_y = np.concatenate([rl_y_to, rl_y_fro], 0)
        rl_points = np.concatenate([rl_x, rl_y.reshape([-1, 1]), rl_z], 1)
        rl_points[-2,2] = 0.4
        rl_points[-1,2] = 0.2  # rl_points are for back and forth in y

        updown_count = 8
        max_h = 2.3
        min_h = 0.4
        up_z_s = np.linspace(min_h, max_h, updown_count).reshape([-1,1])
        down_z = np.linspace(up_z_s[-2], min_h, updown_count-1).reshape([-1,1])
        up_z = np.linspace(down_z[-2], max_h, updown_count-1).reshape([-1,1])
        
        updown_z = np.concatenate([up_z_s, down_z, 
                                   up_z, down_z, 
                                   up_z, down_z, 
                                   up_z, down_z],0)
        updown_len = len(updown_z)
        updown_x = np.ones([updown_len,1]) * 0.1376
        updown_y = np.ones([updown_len,1]) * 1.80
        # updown_points are for back and forth in z
        updown_points = np.concatenate([updown_x, updown_y, updown_z], 1)

        self.traj = self.generate_traj(points)  # trajectory
        
        ############# CONTROLLER ########### 
        self.controller = GeometriControl()  # controller
        
        self.initial_state = {'x': np.array([0, 0, 0]), # positions
                              'v': np.array([0, 0, 0]), # velocities
                              'q': np.array([0, 0, 0, 1]), # quaternion
                              'w': np.zeros(3,)} # angular vel
        self.t0 = rospy.get_time()
        self.prev_time = rospy.get_time()
        self.prev_pos = self.initial_state['x']
        self.prev_vel = np.zeros([3,])
        rospy.loginfo("=============== MPC Demo Initialized ===============")
    
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


    def takeoff(self, req):  # TODO
        transform = Transformstamped()
        self.tf_listener.waitForTransform(self.worldFrame, self.frame, rospy.Time(), rospy.Duration(20.0))
        if transform.translation_from_matrix().z > 0 + 0.1: # when the quad has lifted off
            self.state = 1 # switch to automatic
        else:
            pass

    def landingService(self, req):  # TODO
        pass

    def generate_traj(self, points):
        '''
        returns trajectory object generated from points
        '''
        return wt.WaypointTraj(points) 
    
    def takeoffService(self, req, res):  # TODO
        rospy.loginfo("Takeoff requested!")
        m_state = 2  # set state to taking off
        transform = TransformStamped()  # for getting transforms
        self.tf_listener.waitForTransform(self.worldFrame, self.frame, rospy.Time(), rospy.Duration(20.0))
        self.m_startZ = transform.translation_from_matrix().z  # set z coor for start position

    
    def land(self):  # TODO
        rospy.loginfo("landing")
        (pos, quat) = self.tf_listener.lookupTransform(self.worldFrame, self.frame, rospy.Time(0))
        if pos[3] <= self.initial_state['x'][2] + 0.05:
            self.m_state = 0
            msg = Twist()
            self.cmd_pub.publish(msg)
    
    def automatic(self, target_tracking):  # running MPC
        curr_time = rospy.get_time()
        dt = curr_time - self.prev_time
        if target_tracking:
            interp_time = [1,4]
            # print("self:, target:", self.curr_pos, self.target_pos)
            points = interp1d(interp_time, np.vstack([self.curr_pos, self.target_pos]), axis=0)([1,2,3,4])
            
            points[:, 2] += 0.35
            self.traj = self.generate_traj(points)

        flat = self.sanitize_trajectory_dic(self.traj.update(curr_time-self.t0))

        transform = TransformStamped()  # for getting transforms
        self.tf_listener.waitForTransform(self.worldFrame, self.frame, rospy.Time(), rospy.Duration(20.0))
        t = self.tf_listener.getLatestCommonTime(self.frame, self.worldFrame)
        (tf_pos, tf_quat) = self.tf_listener.lookupTransform(self.worldFrame, self.frame, t)  # position and quaternion in world frame
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

    def idle(self):
        '''
        publish zero commands for 3 seconds before switching to automatic
        '''
        while rospy.get_time() - self.t0 <= 3:
            msg = Twist()
            self.cmd_pub.publish(msg)
        self.m_state = 1
        self.prev_time = rospy.get_time()
        self.t0 = rospy.get_time()

    def takeoff0(self):  # TODO
        imsg = Twist()

        while z_ <= 0.2:
            if self.m_thrust > 50000:
                break
            transform = TransformStamped()  # for getting transforms
            self.tf_listener.waitForTransform(self.worldFrame, self.frame, rospy.Time(), rospy.Duration(20.0))
            t = self.tf_listener.getLatestCommonTime(self.frame, self.worldFrame)
            (pos, quat) = self.tf_listener.lookupTransform(self.worldFrame, self.frame, t)
            self.m_thrust += 10000 * 0.002
            self.cmd_pub.publish(msg)
            z_ = pos[2]
        
        self.m_state = 1
        self.prev_time = rospy.get_time()
        self.t0 = rospy.get_time()


    def run(self, target_tracking):
        '''
        State machine loop
        '''
        while not rospy.is_shutdown():
            if self.m_state == 0:
                self.idle()
            
            elif self.m_state == 3:
                self.land()
            
            elif self.m_state == 1:
                self.automatic(target_tracking)
            
            elif self.m_state == 2:
                self.takeoff()
            
            # self.rate.sleep()
            
 
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
        curr_log_time = rospy.Time.now()
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

if __name__ == '__main__':
    mpc_demo = MPCDemo()
    target_tracking = True
    mpc_demo.run(target_tracking)
    rospy.spin()
        
