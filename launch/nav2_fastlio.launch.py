import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('nav2_waypoint_tools')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    planner_action_name = LaunchConfiguration('planner_action_name')
    path_frame = LaunchConfiguration('path_frame')
    samples_per_segment = LaunchConfiguration('samples_per_segment')
    tangent_scale = LaunchConfiguration('tangent_scale')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
        ),

        DeclareLaunchArgument(
            'planner_action_name',
            default_value='/compute_path_through_poses'
        ),
        DeclareLaunchArgument(
            'path_frame',
            default_value='map'
        ),
        DeclareLaunchArgument(
            'samples_per_segment',
            default_value='25'
        ),
        DeclareLaunchArgument(
            'tangent_scale',
            default_value='1.0'
        ),

        # Node(
        #     package='nav2_controller',
        #     executable='controller_server',
        #     name='controller_server',
        #     output='screen',
        #     parameters=[params_file],
        #     remappings=[('cmd_vel', 'cmd_vel_nav')]
        # ),

        # Node(
        #     package='nav2_planner',
        #     executable='planner_server',
        #     name='planner_server',
        #     output='screen',
        #     parameters=[params_file],
        #     remappings=[
        #         ('/plan', '/teaching_playback/path')
        #     ]
        # ),

        # Node(
        #     package='nav2_behaviors',
        #     executable='behavior_server',
        #     name='behavior_server',
        #     output='screen',
        #     parameters=[params_file]
        # ),

        # Node(
        #     package='nav2_bt_navigator',
        #     executable='bt_navigator',
        #     name='bt_navigator',
        #     output='screen',
        #     parameters=[params_file]
        # ),

        # Node(
        #     package='nav2_waypoint_follower',
        #     executable='waypoint_follower',
        #     name='waypoint_follower',
        #     output='screen',
        #     parameters=[params_file]
        # ),

        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_navigation',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': use_sim_time,
        #         'autostart': True,
        #         'node_names': [
        #             'controller_server',
        #             'planner_server',
        #             'behavior_server',
        #             'bt_navigator',
        #             'waypoint_follower'
        #         ]
        #     }]
        # ),

        Node(
            package='nav2_waypoint_tools',
            executable='pose_collector_node',
            name='pose_collector_node',
            output='screen',
            parameters=[{
                'action_name': planner_action_name
            }]
        ),

        Node(
            package='nav2_waypoint_tools',
            executable='smooth_path_planner_node',
            name='smooth_path_planner_node',
            output='screen',
            parameters=[{
                'action_name': planner_action_name,
                'path_frame': path_frame,
                'samples_per_segment': samples_per_segment,
                'tangent_scale': tangent_scale,
            }],
            remappings=[
                ('/plan', '/teaching_playback/path')
            ]
        ),
        Node(
            package='nav2_waypoint_tools',
            executable='path_to_pose_array_node',
            name='path_to_pose_array_node',
            output='screen',
            remappings=[
                ('/path', '/teaching_playback/path'),
                ('/path_pose_array', '/teaching_playback/path_pose_array'),
            ]
        ),
    ])