

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    real_sense_pkg = get_package_share_directory('realsense2_camera')
    launch_object_detection_cmd = Node(
            package='object_detection',
            executable='pose_estimator',
            name='pose_estimator_node',
            output='screen'
        )
    
    launch_camera_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(real_sense_pkg,'launch', 'rs_launch.py')),
            launch_arguments={'depth_module.depth_profile': '1280x720x30',
                              'rgb_camera.color_profile': '1280x720x30',
                              'pointcloud.enable': 'True'}.items())
    

    static_tf_to_marker_cmd = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='my_static_transform_publisher',
            arguments=[
                '--x', '1.61978536',   # Translation in meters
                '--y', '0.30729697', #0.147893
                '--z', '1.12545326', #'--z', '0.55674606',
                '--qx', '0.65927438',  # Rotation in radians     , ,  , 
                '--qy', '0.68452989',
                '--qz', '-0.23466228 ',
                '--qw', '-0.20422962',
                '--frame-id', 'panda_link0',
                '--child-frame-id', 'camera_link'
            ]
        )

    
    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(launch_camera_cmd)
    ld.add_action(static_tf_to_marker_cmd)
    ld.add_action(launch_object_detection_cmd)

    return ld