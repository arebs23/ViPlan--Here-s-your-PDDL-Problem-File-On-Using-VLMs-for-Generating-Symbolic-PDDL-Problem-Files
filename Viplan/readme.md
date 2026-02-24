
# Before calibration you can build the workspace:
You can build the workspace ros2_ws directly but i'll recommend creating yours and building the necessary packages found below, but you can use the ros2_ws as a guide


# if you choose to build the ros2_ws
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select object_detection_msgs
colcon build --symlink-install --packages-select object_detection


colcon build --symlink-install \
  --packages-skip \
    object_detection_msgs object_detection \
    athena_core athena_protobuf athena_msgs athena_exe_msgs athena_launch athena_exe_launch \
    zed_wrapper zed_components zed_interfaces zed_topic_benchmark zed_tutorial_depth \
    zed_tutorial_pos_tracking zed_tutorial_video rviz_plugin_zed_od

# Building your own workspace required packages:
More detail for calibration and aruco pose estimation can be found in: 


* aruco poseestimator:
https://github.com/AIRLab-POLIMI/ros2-aruco-pose-estimation

* easy handeye:
https://github.com/marcoesposito1988/easy_handeye2

* Intel Realsense:
https://github.com/realsenseai/realsense-ros


* Realsense python needed for object pose estimation:
https://github.com/realsenseai/librealsense
pip install pyrealsense2





# Two steps for Calibrations
## Step1: For detecting the pose of the aruco marker launch the package ros2-aruco-pose-estimation:
ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py


## Step2: Launch the package for calibration easy_handeye2
ros2 launch easy_handeye2 calibrate.launch.py   name:=my_eob_calib_test2   calibration_type:=eye_on_base   tracking_base_frame:=camera_color_optical_frame  tracking_marker_frame:=aruco_marker_1   robot_base_frame:=panda_link0   robot_effector_frame:=panda_hand



* name: File name that saves the calibration paramters
* calibration_type: the type of calibration, keep the same
* tracking_base_frame: camera base frame set in :ros2_ws/src ros2-aruco-pose-estimation/aruco_pose_estimation/config/aruco_parameters.yaml

* tracking_marker_frame: name of the marker detected

* robot_base_frame: Robot base frame

* robot_effector_frame: Robot end effector frame

## After calibration, set the final extrinsic calibration parameters found in my_eob_calib_test2:
After calibration set the parameters there as a static transform in the object_detection launch file: 
ros2_ws/src/object_detection/launch/pose_estimator_launch.py

# example:
 arguments=[
                '--x', '1.61978536',   # Translation in meters
                '--y', '0.30729697', #0.147893
                '--z', '1.12545326', #'--z', '0.55674606',
                '--qx', '0.65927438',  # Rotation in radians     , ,  , 
                '--qy', '0.68452989',
                '--qz', '-0.23466228 ',
                '--qw', '-0.20422962',
                '--frame-id', 'panda_link0',
                '--child-frame-id', 'camera_color_optical_frame'
            ]

# Note:
More detail for calibration and aruco pose estimation can be found in: 

https://github.com/AIRLab-POLIMI/ros2-aruco-pose-estimation

https://github.com/marcoesposito1988/easy_handeye2


# To use the object pose estimation
* Grounded-sam:
https://github.com/IDEA-Research/Grounded-Segment-Anything
* supervision:
pip install supervision
https://pypi.org/project/supervision/



## Build object pose estimation package


colcon build --symlink-install --packages-select object_detection_msgs

colcon build --symlink-install --packages-select object_detection

## For object pose-estimation
ros2 launch object_detection pose_estimator_launch.py

## Necessary for object pose-estimation
ros2_ws/src/object_detection
ros2_ws/src/object_detection_msgs

## future:
Use language grasp module instead