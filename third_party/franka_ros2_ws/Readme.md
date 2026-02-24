## To launch the franka Robot
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=robot_ip


## To test run the fixed positions for pick and place task
ros2 launch moveit2_scripts test_trajectory.launch.py robot_ip:=robot_ip

## The code for the fixed position cpp:
moveit2_scripts/src/test_trajectory.cpp


## To run the automatic for pick and place :
### First run found in ros2_ws:
ros2 launch object_detection launch_object_detection.py

### Second run found in franka_arm_ros2: 
ros2 launch moveit2_scripts bringup_automatic_launch.py robot_ip:=robot_ip

## codes needed for this are in the cpp file:
moveit2_scripts/src/move_to_pose_server
moveit2_scripts/src/open_gripper_server
moveit2_scripts/src/close_gripper_server

# NOTE: 
For more info check the launch files above, i think some changes might be needed to make it work but 

