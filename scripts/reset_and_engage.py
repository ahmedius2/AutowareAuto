#!/usr/bin/env python3

import rclpy
import time
import sys
from rclpy.node import Node
from std_msgs.msg import Header
from autoware_vehicle_msgs.msg import Engage
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.parameter import Parameter
from autoware_adapi_v1_msgs.msg import OperationModeState


class ResetAndEngageNode(Node):
    def __init__(self, operation):
        super().__init__('reset_and_engage')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.stopped = False
        self.autonomous = False
        self.timeout = False
        self.init_time = time.time()
        self.timeout_lim_sec = 300

        self.operation = operation

        # Create QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        if operation == 'engage':
            # Create publishers
            self.engage_pub = self.create_publisher(
                Engage,
                '/autoware/engage',
                qos_profile
            )
            
            self.traffic_reset_pub = self.create_publisher(
                Header,
                '/awsim/traffic_reset',
                qos_profile
            )

        self.state_sub = self.create_subscription(
            OperationModeState,
            '/system/operation_mode/state',
            self.state_callback,
            qos_profile
        )
        self.timeout_timer = self.create_timer(1.0, self.timeout_callback)


    def wait_for_sim_time(self):
        # Wait for the first few clock messages to ensure sim time is properly set
        num_clock_messages = 0
        while num_clock_messages < 5:
            if self.get_clock().now().nanoseconds > 0:
                num_clock_messages += 1
            rclpy.spin_once(self, timeout_sec=0.5)
            if num_clock_messages == 0:
                self.get_logger().info('RAE Waiting for simulation time...')

    def state_callback(self, msg):
        new_mode = int(msg.mode)
        if new_mode == 1:
            self.stopped = True
        elif new_mode == 2:
            self.autonomous = True

    def timeout_callback(self):
        if time.time() - self.init_time > self.timeout_lim_sec:
            self.timeout = True

    def run(self):
        self.wait_for_sim_time()
        if self.operation== 'engage':
            time_arg = int(sys.argv[2])
            # Get current time and add arg seconds
            current_time = self.get_clock().now()
            future_time = current_time + rclpy.duration.Duration(seconds=time_arg)
            
            # Create messages with future timestamp
            engage_msg = Engage()
            engage_msg.stamp = future_time.to_msg()
            engage_msg.engage = True
            
            header_msg = Header()
            header_msg.stamp = future_time.to_msg()
            header_msg.frame_id = '0_0' # seed and number of NPC cars

            start_time = time.time() - 5.0
            while not self.autonomous and not self.timeout:
                if time.time() - start_time >= 5.0:
                    self.get_logger().info('RAE Sending reset and engage messages.')
                    self.traffic_reset_pub.publish(header_msg)
                    self.engage_pub.publish(engage_msg)
                    start_time = time.time()
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)
            if not self.timeout:
                self.get_logger().info('RAE Car engaged successfully.')
        elif self.operation == 'waitstop':
            while not self.stopped and not self.timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)
            if not self.timeout:
                self.get_logger().info('RAE Car stopped successfully.')
        else:
            self.get_logger().error('RAE Unknown operation!.')

        if self.timeout:
            self.get_logger().warn('RAE timed out!')

def main():
    rclpy.init()
    
    # Create node
    node = ResetAndEngageNode(sys.argv[1])
    node.run()

    # Cleanup and shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
