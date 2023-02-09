import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('piot_navigation2'),
            'map',
            'piot_world.yaml'))

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('piot_navigation2'),
            'param',
            'piot.yaml'))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    localization_launch_file_dir = os.path.join(get_package_share_directory('piot_localization'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')
    
    namespace = LaunchConfiguration('namespace')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    slam = LaunchConfiguration('slam')
    
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    configured_params = RewrittenYaml(
        source_file=param_dir,
        root_key=namespace,
        param_rewrites={},
        convert_types=True)
    

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'),
            
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'),
            
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='log level'),
            
        DeclareLaunchArgument(
            'use_respawn', 
            default_value='False',
            description='Whether to respawn if a node crashes. Applied when composition is disabled.'),
            
        DeclareLaunchArgument(
            'use_composition', 
            default_value='True',
            description='Whether to use composed bringup'),
        
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
            
        DeclareLaunchArgument(
            'slam',
            default_value='False',
            description='Whether run a SLAM'),


#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
#            launch_arguments={
#                'map': map_dir,
#                'use_sim_time': use_sim_time,
#                'params_file': param_dir}.items(),
#        ),

        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_file_dir, 'slam_launch.py')),
            condition=IfCondition(slam),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'use_respawn': use_respawn,
                              'params_file': param_dir}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(localization_launch_file_dir, 'localization_launch.py')),
            condition=IfCondition(PythonExpression(['not ', slam])),
            launch_arguments={'namespace': namespace,
                              'map': map_dir,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': param_dir,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_file_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': param_dir,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
