#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Header
from autoware_vehicle_msgs.msg import Engage
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.parameter import Parameter

def main():
    rclpy.init()
    
    # Create node
    node = Node('engage_publisher')
    node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    
    # Wait for the first few clock messages to ensure sim time is properly set
    num_clock_messages = 0
    while num_clock_messages < 5:
        if node.get_clock().now().nanoseconds > 0:
            num_clock_messages += 1
        rclpy.spin_once(node, timeout_sec=1.0)
        if num_clock_messages == 0:
            node.get_logger().info('Waiting for simulation time...')

    # Create QoS profile for reliable communication
    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        depth=1
    )
    
    # Create publishers
    engage_pub = node.create_publisher(
        Engage,
        '/autoware/engage',
        qos_profile
    )
    
    traffic_reset_pub = node.create_publisher(
        Header,
        '/awsim/traffic_reset',
        qos_profile
    )


    # Get current time and add 5 seconds
    current_time = node.get_clock().now()
    future_time = current_time + rclpy.duration.Duration(seconds=5)
    
    # Create messages with future timestamp
    engage_msg = Engage()
    engage_msg.stamp = future_time.to_msg()
    engage_msg.engage = True
    
    header_msg = Header()
    header_msg.stamp = future_time.to_msg()
    header_msg.frame_id = '10_10' # seed and number of NPC cars

    # Publish messages
    traffic_reset_pub.publish(header_msg)
    engage_pub.publish(engage_msg)
    
    # Small sleep to ensure messages are published
    print('Sleeping for 5 seconds...')
    time.sleep(5.0)
    
    # Cleanup and shutdown
    node.destroy_node()
    rclpy.shutdown()
    print('Done!')

if __name__ == '__main__':
    main()
