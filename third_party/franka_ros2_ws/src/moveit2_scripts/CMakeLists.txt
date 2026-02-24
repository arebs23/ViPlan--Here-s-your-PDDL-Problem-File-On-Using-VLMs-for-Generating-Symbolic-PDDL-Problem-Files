cmake_minimum_required(VERSION 3.8)
project(moveit2_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(athena_exe_msgs REQUIRED)
find_package(object_detection_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)



add_executable(test_trajectory src/test_trajectory.cpp) #src/pick_place.cpp)
add_executable(test_noise src/test_noise.cpp)
add_executable(fk_main src/fk_main.cpp)

add_executable(test_collision src/test_collision.cpp)

target_include_directories(test_collision
PUBLIC include)

ament_target_dependencies(test_collision
ament_cmake
rclcpp
rclcpp_action
moveit_core
moveit_ros_planning
moveit_ros_planning_interface
control_msgs
moveit_visual_tools)


target_include_directories(test_trajectory
PUBLIC include)

target_include_directories(test_noise PUBLIC include)

ament_target_dependencies(test_trajectory
ament_cmake
rclcpp
rclcpp_action
moveit_core
moveit_ros_planning
moveit_ros_planning_interface
control_msgs)

ament_target_dependencies(test_noise
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  control_msgs
)

add_library(move_action_server SHARED
  src/move_to_pose_server.cpp)
target_include_directories(move_action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(move_action_server
  PRIVATE "ACTION_CPP_BUILDING_DLL")
ament_target_dependencies(move_action_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "athena_exe_msgs"
  "object_detection_msgs"
  "moveit_core"
  "moveit_ros_planning"
  "moveit_ros_planning_interface"
  "control_msgs"
  )
rclcpp_components_register_node(move_action_server PLUGIN "MoveToPoseActionServer" EXECUTABLE move_to_pose_server)


add_library(close_gripper_action SHARED
  src/close_gripper_server.cpp)
target_include_directories(close_gripper_action PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(close_gripper_action
  PRIVATE "ACTION_CPP_BUILDING_DLL")
ament_target_dependencies(close_gripper_action
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "athena_exe_msgs"
  "object_detection_msgs"
  "moveit_core"
  "moveit_ros_planning"
  "moveit_ros_planning_interface"
  "control_msgs")
rclcpp_components_register_node(close_gripper_action PLUGIN "CloseGripperActionServer" EXECUTABLE close_gripper_action_server)


add_library(open_gripper_action SHARED
  src/open_gripper_server.cpp)
target_include_directories(open_gripper_action PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(open_gripper_action
  PRIVATE "ACTION_CPP_BUILDING_DLL")
ament_target_dependencies(open_gripper_action
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "athena_exe_msgs"
  "object_detection_msgs"
  "moveit_core"
  "moveit_ros_planning"
  "moveit_ros_planning_interface"
  "control_msgs")
rclcpp_components_register_node(open_gripper_action PLUGIN "OpenGripperActionServer" EXECUTABLE open_gripper_action_server)

install(TARGETS
move_action_server
open_gripper_action
close_gripper_action
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

target_include_directories(fk_main
PUBLIC include)

ament_target_dependencies(fk_main
ament_cmake
rclcpp
rclcpp_action
moveit_core
moveit_ros_planning
moveit_ros_planning_interface
control_msgs)

install(TARGETS
test_trajectory
test_noise
fk_main
test_collision
DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
         launch
         DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
