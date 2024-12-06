from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = Node(
        package='rclcpp_components',
        executable='component_container_mt',  # Multithreaded container
        name='pc_acc_container',                # Name of the container
        output='screen'
    )

    # Define the composable node to load
    composable_node = ComposableNode(
        package='pc_accumulator_for_dnn',  # Name of the package
        plugin='pc_acc_for_dnn::PCloudAccForDnnComponent', # Name of the registered node class
        name='pc_acc_for_dnn_node',       # Node name
#        parameters=[{'param_name': 'param_value'}],  # Optional parameters
#        remappings=[('/input_topic', '/output_topic')]  # Optional topic remappings
    )

    # Launch description
    return LaunchDescription([
        container,
        LoadComposableNodes(
            target_container='pc_acc_container',  # Name of the container
            composable_node_descriptions=[composable_node],
        )
    ])