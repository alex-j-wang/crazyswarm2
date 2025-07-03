#!/usr/bin/env python3

import os
import glob
import shutil

import numpy as np
import torch
from torch import Tensor
from torch.nn import functional as F
from scipy import signal
import threading
import queue

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# ROS2 message types
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Int32

# Import your existing modules (you'll need to adapt these for your new project)
from solvers import RK
from models import RigidHybridCascade
from NODE.NODE import NeuralODE

# TODO: move with other constants
BATCH_SKIP = 1

class OnlineLearningNode(Node):
    def __init__(self):
        super().__init__('mpc_train_online', automatically_declare_parameters_from_overrides=True)

        self.frame = self.get_parameter('frame').value
        self.model_period = self.get_parameter('model_period').value
        self.data_frequency = self.get_parameter('data_frequency').value
        self.noise = self.get_parameter('noise').value

        # Data collection variables
        self.curr_pos = np.zeros(3)
        self.curr_vel = np.zeros(3)
        self.curr_u = np.zeros(3)
        self.data_buffer = []
        self.model_cnt = 0
        self.active = False
        self.t0 = None

        # Subscribers and publishers
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pos_sub = self.create_subscription(PoseStamped, 'tf_pos', self.pos_callback, qos_profile)
        self.vel_sub = self.create_subscription(TwistStamped, 'est_vel', self.vel_callback, qos_profile)
        self.u_sub = self.create_subscription(TwistStamped, 'u_euler', self.u_callback, qos_profile)
        self.state_sub = self.create_subscription(Int32, '/command/cmd_state', self.cmd_state_callback, 1) # Phase request
        self.model_pub = self.create_publisher(Int32, 'model', 1) # Model update notification

        # Model setup
        ode_solve = RK
        step_size = 1 / 400
        torch.manual_seed(0)
        self.ode_train = NeuralODE(RigidHybridCascade(), ode_solve, step_size)
        
        # Clear previous models, load base model
        self.dirname = os.path.join('/', 'ros_ws', 'src', 'crazyflie_mpc', 'data', 'knode_models', 'online')
        for path in glob.glob(os.path.join(self.dirname, f'{self.frame}_*.pth')):
            os.remove(path)
        self.initdir = os.path.join('/', 'ros_ws', 'src', 'crazyflie_mpc', 'data', 'knode_models', 'init')
        shutil.copyfile(os.path.join(self.initdir, f'{self.frame}.pth'), os.path.join(self.dirname, f'{self.frame}_0.pth'))

        # Start training thread
        self.training_queue = queue.Queue()
        self.training_thread = threading.Thread(target=self.training_worker, daemon=True)
        self.training_thread.start()

        self.timer = self.create_timer(1.0 / self.data_frequency, self.timer_callback)

    def lpf(self, vel_data):
        """
        Low-pass Butterworth filter for velocity data
        """
        fs = self.data_frequency # sampling frequency
        fc = 10 # cut-off frequency
        w = fc / (fs / 2)
        b, a = signal.butter(5, w, 'low')
        bs, _ = vel_data.shape

        fil_x = signal.filtfilt(b, a, vel_data[:, 0].reshape(-1)).reshape(-1, 1)
        fil_y = signal.filtfilt(b, a, vel_data[:, 1].reshape(-1)).reshape(-1, 1)
        fil_z = signal.filtfilt(b, a, vel_data[:, 2].reshape(-1)).reshape(-1, 1)
        fil_vel = np.stack([fil_x, fil_y, fil_z], 1)

        return fil_vel.reshape([bs, -1])

    def pos_callback(self, data):
        self.curr_pos[0] = data.pose.position.x
        self.curr_pos[1] = data.pose.position.y
        self.curr_pos[2] = data.pose.position.z

    def vel_callback(self, data):
        self.curr_vel[0] = data.twist.linear.x
        self.curr_vel[1] = data.twist.linear.y
        self.curr_vel[2] = data.twist.linear.z

    def u_callback(self, data):
        self.curr_u[0] = data.twist.linear.x
        self.curr_u[1] = data.twist.linear.y
        self.curr_u[2] = data.twist.linear.z

    def cmd_state_callback(self, msg: Int32):
        """
        Callback function for state changes
        """
        match msg.data:
            case 2:
                self.get_logger().info('Trajectory requested!')
                self.t0 = self.get_clock().now().nanoseconds / 1e9
                self.model_pub.publish(Int32(data=self.model_cnt))
                self.model_cnt += 1
            case 3:
                self.get_logger().info('Online learning disabled!')
                self.active = False
                self.t0 = None
                self.training_queue.queue.clear()

    def training_worker(self):
        """
        Background thread for neural network training
        """
        while True:
            data = self.training_queue.get() 
            self.train_model(data)

    def train_model(self, data):
        train_set = torch.tensor(data, dtype=torch.float32)
        train_traj = train_set.detach().unsqueeze(1)
        train_traj = train_traj + torch.randn_like(train_traj) * self.noise
        
        EPOCHS = 200
        LR = 0.005
        LOOKAHEAD = 2
        L2_LAMBDA = 1e-7
        PLOT_FREQ = 50
        STEP_SKIP = 1

        save_path = os.path.join(self.dirname, f'{self.frame}_{self.model_cnt}.pth')
        # TODO: sliding window -> train_traj_list?
        self.sample_and_grow([train_traj], EPOCHS, LR, LOOKAHEAD, L2_LAMBDA,
                             plot_freq=PLOT_FREQ, save_path=save_path, step_skip=STEP_SKIP)
        self.model_pub.publish(Int32(data=self.model_cnt))
        self.model_cnt += 1

        # Cascade one layer
        self.ode_train.func.cascade()

    def sample_and_grow(self, traj_list, epochs, LR, lookahead, l2_lambda,
                        plot_freq=50, save_path=None, step_skip=1):
        state_dim = 6 # for rigid model the state is 6D
        optimizer = torch.optim.Adam(filter(lambda p: p.requires_grad, self.ode_train.parameters()), lr=LR)
        train_loss_arr = []
        for i in range(epochs):
            for idx, true_traj in enumerate(traj_list):
                n_segments, _, n_state = true_traj.size()
                true_segments_list = []

                for j in range(0, n_segments - lookahead + 2 - BATCH_SKIP, BATCH_SKIP):
                    j = j + i % BATCH_SKIP
                    true_sampled_time_segment = (Tensor(np.arange(lookahead)) * step_skip).detach()
                    true_sampled_segment = true_traj[j:j + lookahead]
                    true_segments_list.append(true_sampled_segment)
                # concatenating all batches together
                obs = torch.cat(true_segments_list, 1)

                pred_traj = []
                ode_input = obs[0, :, :].unsqueeze(1)  # initial condition has size [1499, 1, 17]
                pred_traj.append(ode_input)
                for k in range(len(true_sampled_segment) - 1):
                    z1 = self.ode_train(ode_input, Tensor(np.arange(2))).squeeze(1)
                    ode_input = torch.cat([z1[:, :state_dim].unsqueeze(1), obs[k + 1, :, state_dim:].unsqueeze(1)], 2)
                    pred_traj.append(ode_input)

                # prediction has size [plot_len, 1, 17]
                pred_traj = torch.swapaxes(torch.cat(pred_traj, 1), 0, 1)
                l2_norm = sum(p.pow(2.0).sum() for p in self.ode_train.parameters())
                # l2_lambda = 7e-8
                if idx == 0:
                    # loss from the first trajectory
                    loss = F.mse_loss(pred_traj[:, :, :state_dim], obs[:, :, :state_dim]) + l2_lambda * l2_norm
                else:
                    # adding loss from other trajectories
                    loss += F.mse_loss(pred_traj[:, :, :state_dim], obs[:, :, :state_dim]) + l2_lambda * l2_norm

            train_loss_arr.append(loss.item())
            optimizer.zero_grad()
            loss.backward(retain_graph=True)
            
            n_old_layers = self.ode_train.func.n_old_layers
            # freezing the weights of old layers
            for m in range(n_old_layers):
                if str(self.ode_train.func.nn_model[m])[:6] == 'Linear':
                    self.ode_train.func.nn_model[m].weight.grad *= 0
                    self.ode_train.func.nn_model[m].bias.grad *= 0
            
            optimizer.step()

            if i % plot_freq == 0:
                if i == 100: # reduce learning rate after 400 epochs
                    LR = 0.002
                elif i == 150:
                    LR = 0.001
                optimizer.param_groups[0]['lr'] = LR

                self.get_logger().info(
                    f'Iteration: {i} Step Size: {self.ode_train.STEP_SIZE} No. of Points: {n_segments} '
                    f'Lookahead: {lookahead} LR: {LR}\nTraining Loss: {loss.item():.3e}'
                )
                
        if save_path is not None:
            torch.save({'ode_train': self.ode_train, 'train_loss_arr': train_loss_arr}, save_path)

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if not self.active and self.t0 is not None and current_time - self.t0 > 3.0:
            self.get_logger().info('Online learning enabled!')
            self.active = True
            self.t0 = current_time
        
        if self.active:
            time_diff = current_time - self.t0
            if time_diff > self.model_period:
                # Update model
                curr_data = np.array(self.data_buffer)
                curr_data[:, 3:6] = self.lpf(curr_data[:, 3:6])
                self.training_queue.put(curr_data)

                # Save data to disk?
                # np.save(...)

                self.data_buffer = []
                self.t0 = current_time

            # Collect current data point
            agg_data = np.concatenate([self.curr_pos, self.curr_vel, self.curr_u])
            self.data_buffer.append(agg_data)
        
def main(args=None):
    rclpy.init(args=args)
    node = OnlineLearningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
