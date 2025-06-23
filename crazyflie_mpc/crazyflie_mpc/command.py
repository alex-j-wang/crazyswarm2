#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from rcl_interfaces.msg import ParameterValue, ParameterType

class CommandNode(Node):
    def __init__(self):
        super().__init__('command')
        self.declare_parameter('cfnames', ParameterValue(type=ParameterType.PARAMETER_STRING_ARRAY))
        self.cfnames = self.get_parameter('cfnames').value
        self.cfready = {name: False for name in self.cfnames}
        self.state_request = 0

        self.state_pub = self.create_publisher(Int32, 'cmd_state', 1)
        self.ready_sub = self.create_subscription(String, 'cf_ready', self.ready_callback, len(self.cfnames))
        self.timer = None

    def next_phase(self):
        self.timer.cancel()
        self.state_request += 1
        for name in self.cfready:
            self.cfready[name] = False
        msg = Int32()
        msg.data = self.state_request
        self.state_pub.publish(msg)
        self.get_logger().info(f'Phase {self.state_request} requested')

    def ready_callback(self, msg):
        if msg.data in self.cfready:
            self.cfready[msg.data] = True
            self.get_logger().info(f'Crazyflie {msg.data} ready [{sum(self.cfready.values())}/{len(self.cfnames)}]')
            if all(self.cfready.values()):
                self.get_logger().info(f'Phase {self.state_request} complete')

                if self.state_request > 3:
                    self.get_logger().info('Shutting down')
                    rclpy.shutdown()

                self.timer = self.create_timer(3, self.next_phase)
        else:
            self.get_logger().warn(f'Unknown Crazyflie: {msg.data}')

def main():
    rclpy.init()
    node = CommandNode()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
