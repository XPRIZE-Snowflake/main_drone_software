import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
import launch.conditions


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim', default_value='false', 
        description='Set to true to launch sim_flight_publisher.py, otherwise start seekcamera_publisher.cpp and odometry_publisher.py'
    )
    log_arg = DeclareLaunchArgument(
        'log', default_value='false', 
        description='Set to true to enable flight_logger.py'
    )
    high_arg = DeclareLaunchArgument(
        'high', default_value='true', 
        description='Set to false to run low_alt_filter.py, otherwise runs high_alt_filter.py and kmeans.py'
    )
    print_arg = DeclareLaunchArgument(
        'print', default_value='log', 
        description='set to true to print all messages to the terminal screen'
    )
    # debug_level_arg = DeclareLaunchArgument(
    #     'debug_level', default_value=30, 
    #     description='Set log level to (10=DEBUG, 20=INFO, 30=WARN, 40=ERROR, 50=FATAL)'
    # )

    # output_setting = LaunchConfiguration('print')

    sim_flight_publisher_node = launch_ros.actions.Node(
        package='img_capture', executable='sim_flight_publisher.py', 
        name='sim_flight_publisher'
    )
    seek_camera_publisher_node = launch_ros.actions.Node(
        package='img_capture', executable='seekcamera_publisher', 
        name='seekcamera_publisher'
    )
    odometry_publisher_python_node = launch_ros.actions.Node(
        package='img_capture', executable='odometry_publisher.py', 
        name='odometry_publisher'
        # ,parameters=[{'ros__logging_severity': LaunchConfiguration('log_level')}]
    )
    flight_logger_node = launch_ros.actions.Node(
        package='img_capture', executable='flight_logger.py', 
        name='flight_logger'
    )
    low_alt_filter_node = launch_ros.actions.Node(
        package='img_capture', executable='low_alt_filter.py', 
        name='low_alt_filter'
    )
    high_alt_filter_node = launch_ros.actions.Node(
        package='img_capture', executable='high_alt_filter.py', 
        name='high_alt_filter'
    )
    kmeans_node = launch_ros.actions.Node(
        package='img_capture', executable='kmeans.py', 
        name='kmeans'
    )

    sim_group = GroupAction(
        condition=launch.conditions.IfCondition(LaunchConfiguration('sim')),
        actions=[sim_flight_publisher_node],
    )
    default_group = GroupAction(
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('sim')),
        actions=[seek_camera_publisher_node, odometry_publisher_python_node],
    )
    logging_group = GroupAction(
        condition=launch.conditions.IfCondition(LaunchConfiguration('log')),
        actions=[flight_logger_node],
    )
    high_filter_group = GroupAction(
        condition=launch.conditions.IfCondition(LaunchConfiguration('high')),
        actions=[high_alt_filter_node, kmeans_node],
    )
    low_filter_group = GroupAction(
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('high')),
        actions=[low_alt_filter_node],
    )

    return launch.LaunchDescription([
        sim_arg,
        log_arg,
        high_arg,
        print_arg,
        # debug_level_arg,
        sim_group,
        default_group,
        logging_group,
        high_filter_group,
        low_filter_group,
    ])
