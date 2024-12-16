from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    input_pointcloud_arg = DeclareLaunchArgument(
        'input/pointcloud', 
        default_value='/sensing/lidar/pointcloud', 
        description='Topic name for input pointcloud'
    )
    output_objects_arg = DeclareLaunchArgument(
        'output/objects', 
        default_value='objects', 
        description='Topic name for output objects'
    )
    use_pointcloud_container_arg = DeclareLaunchArgument(
        'use_pointcloud_container', 
        default_value='false', 
        description='Whether to use a pointcloud container'
    )
    pointcloud_container_name_arg = DeclareLaunchArgument(
        'pointcloud_container_name', 
        default_value='pointcloud_container', 
        description='Name of the pointcloud container'
    )
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode'
    )

    input_pointcloud = LaunchConfiguration('input/pointcloud')
    output_objects = LaunchConfiguration('output/objects')
    use_pointcloud_container = LaunchConfiguration('use_pointcloud_container')
    pointcloud_container_name = LaunchConfiguration('pointcloud_container_name')
    debug = LaunchConfiguration('debug')

    container = ComposableNodeContainer(
        condition=UnlessCondition(use_pointcloud_container),
        name='pc_acc_container',              # Name of the container
        namespace='',
        package='rclcpp_components',
        executable='component_container',
    )

    # Define the composable node to load
    composable_node = ComposableNode(
        package='pc_accumulator_for_dnn',  # Name of the package
        plugin='pc_acc_for_dnn::PCloudAccForDnnComponent', # Name of the registered node class
        name='pc_acc_for_dnn_node',       # Node name
        parameters=[{'debug': debug}],  # Optional parameters
        remappings=[('input/pointcloud', input_pointcloud),],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    target_container = PythonExpression([
      "'", pointcloud_container_name, "' if '", use_pointcloud_container, "' == 'true' else 'pc_acc_container'"
    ])

    load_comp_node = LoadComposableNodes(
        target_container=target_container,
        composable_node_descriptions=[composable_node],
    )

    lidar_dnn_node = Node(
        package='autoware_lidar_valor',
        executable='lidar_objdet_valor',
        name='lidar_objdet_valor_dnn',
        remappings=[('valor_detected_objs', output_objects)]
    )

    # Launch description
    return LaunchDescription([
        input_pointcloud_arg,
        output_objects_arg,
        use_pointcloud_container_arg,
        pointcloud_container_name_arg,
        debug_arg,
        container,
#        composable_node,
        load_comp_node,
        lidar_dnn_node,
    ])
