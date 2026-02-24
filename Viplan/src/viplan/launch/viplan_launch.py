import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    real_sense_pkg = get_package_share_directory('realsense2_camera')
    
    return LaunchDescription([
        # Include the RealSense camera launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(real_sense_pkg, 'launch', 'rs_launch.py')),
            launch_arguments={
                'depth_module.depth_profile': '1280x720x30',
                'rgb_camera.color_profile': '1280x720x30',
                'pointcloud.enable': 'True'
            }.items()
        ),
        
        # Static transform publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='my_static_transform_publisher',
            arguments=[
                '--x', '1.29457129',   
                '--y', '0.10',
                '--z', '0.55',
                '--qx', '-0.11639707',
                '--qy', '0.00970245',
                '--qz', '0.99261591',
                '--qw', '-0.03272986',
                '--frame-id', 'panda_link0',
                '--child-frame-id', 'camera_link'
            ]
        ),
        
        # VLM Node with the --generate_file argument
        Node(
            package='viplan',
            executable='vlm',
            name='vlm',
            output='screen',
            #arguments=[],  # Add the --generate_file argument here
            parameters=[
                {'generate_problem_file': True}  # Boolean parameter set to False
            ]
        )
    ])
