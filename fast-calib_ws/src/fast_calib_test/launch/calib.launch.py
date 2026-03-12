from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory

import os
import yaml

def generate_launch_description():
    # Launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    # Get package share directory
    pkg_share = FindPackageShare('fast_calib_test')
    
    # Load parameters
    params_file = PathJoinSubstitution([pkg_share, 'config', 'qr_params.yaml'])

    # Load parameters to read from
    yaml_file_path = os.path.join(
        get_package_share_directory('fast_calib_test')
        , 'config', 'qr_params.yaml')
    with open(yaml_file_path, 'r') as f:
        params_config = yaml.safe_load(f)
    node_params = params_config['fast_calib']['ros__parameters']

    # Load bag and image
    bag_path = PathJoinSubstitution([pkg_share, 'calib_data', 
        node_params.get('bag_path')])
    image_path = PathJoinSubstitution([pkg_share, 'calib_data', 
        node_params.get('image_path')])
    output_path = node_params.get('output_path')

    # Fast calibration node
    fast_calib_node = Node(
        package='fast_calib',
        executable='fast_calib',
        name='fast_calib',
        output='screen',
        parameters=[
            params_file,
            {
                'bag_path': bag_path,
                'image_path': image_path,
                'output_path': output_path
            }
        ]
    )
    
    # RViz node
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz_cfg', 'fast_livo2.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        rviz_arg,
        fast_calib_node,
        rviz_node
    ]) 	
