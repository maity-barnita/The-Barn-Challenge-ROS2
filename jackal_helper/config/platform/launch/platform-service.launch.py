from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    launch_arg_imu_filter = DeclareLaunchArgument(
        'imu_filter',
        default_value='/jackal_ws/src/The-Barn-Challenge-Ros2/jackal_helper/config/platform/config/imu_filter.yaml',
        description='')

    imu_filter = LaunchConfiguration('imu_filter')

    # Include Packages
    pkg_clearpath_common = FindPackageShare('clearpath_common')

    # Declare launch files
    launch_file_platform = PathJoinSubstitution([
        pkg_clearpath_common, 'launch', 'platform.launch.py'])

    # Include launch files
    launch_platform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_platform]),
        launch_arguments=
            [
                (
                    'setup_path'
                    ,
                    '/jackal_ws/src/The-Barn-Challenge-Ros2/jackal_helper/config'
                )
                ,
                (
                    'use_sim_time'
                    ,
                    'true'
                )
                ,
                (
                    'namespace'
                    ,
                    '/'
                )
                ,
                (
                    'enable_ekf'
                    ,
                    'true'
                )
                ,
                (
                    'use_manipulation_controllers'
                    ,
                    'true'
                )
                ,
            ]
    )

    # Nodes
    node_cmd_vel_bridge = Node(
        name='cmd_vel_bridge',
        executable='parameter_bridge',
        package='ros_gz_bridge',
        namespace='/',
        output='screen',
        arguments=
            [
                '/cmd_vel@geometry_msgs/msg/TwistStamped[gz.msgs.Twist'
                ,
                '/model/robot/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist'
                ,
            ]
        ,
        remappings=
            [
                (
                    '/cmd_vel'
                    ,
                    'cmd_vel'
                )
                ,
                (
                    '/model/robot/cmd_vel'
                    ,
                    'platform/cmd_vel'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                }
                ,
            ]
        ,
    )

    node_odom_base_tf_bridge = Node(
        name='odom_base_tf_bridge',
        executable='parameter_bridge',
        package='ros_gz_bridge',
        namespace='/',
        output='screen',
        arguments=
            [
                '/model/robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
                ,
            ]
        ,
        remappings=
            [
                (
                    '/model/robot/tf'
                    ,
                    'tf'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                }
                ,
            ]
        ,
    )

    node_imu_0_gz_bridge = Node(
        name='imu_0_gz_bridge',
        executable='parameter_bridge',
        package='ros_gz_bridge',
        namespace='/',
        output='screen',
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                    'config_file': '/jackal_ws/src/The-Barn-Challenge-Ros2/jackal_helper/config/sensors/config/imu_0.yaml'
                    ,
                }
                ,
            ]
        ,
    )

    node_imu_filter_node = Node(
        name='imu_filter_node',
        executable='imu_filter_madgwick_node',
        package='imu_filter_madgwick',
        namespace='/',
        output='screen',
        remappings=
            [
                (
                    'imu/data_raw'
                    ,
                    'sensors/imu_0/data_raw'
                )
                ,
                (
                    'imu/mag'
                    ,
                    'sensors/imu_0/magnetic_field'
                )
                ,
                (
                    'imu/data'
                    ,
                    'sensors/imu_0/data'
                )
                ,
                (
                    '/tf'
                    ,
                    'tf'
                )
                ,
            ]
        ,
        parameters=
            [
                imu_filter
                ,
            ]
        ,
    )

    node_gps_0_gz_bridge = Node(
        name='gps_0_gz_bridge',
        executable='parameter_bridge',
        package='ros_gz_bridge',
        namespace='/',
        output='screen',
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                    'config_file': '/jackal_ws/src/The-Barn-Challenge-Ros2/jackal_helper/config/sensors/config/gps_0.yaml'
                    ,
                }
                ,
            ]
        ,
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_arg_imu_filter)
    ld.add_action(launch_platform)
    ld.add_action(node_cmd_vel_bridge)
    ld.add_action(node_odom_base_tf_bridge)
    ld.add_action(node_imu_0_gz_bridge)
    ld.add_action(node_imu_filter_node)
    ld.add_action(node_gps_0_gz_bridge)
    return ld
