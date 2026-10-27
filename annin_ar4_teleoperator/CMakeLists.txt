cmake_minimum_required(VERSION 3.8)
project(annin_ar4_teleoperator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(OsqpEigen REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  std_msgs
  trajectory_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2
  tf2_ros
  tf2_geometry_msgs
  yaml-cpp
  ament_index_cpp
  Threads
  control_msgs
  rclcpp_action
  controller_manager_msgs
  OsqpEigen
  )

add_library(workspace_sampling_lib src/workspace_sampling.cpp)

target_include_directories(workspace_sampling_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/headers>
  $<INSTALL_INTERFACE:include>
  )
ament_target_dependencies(workspace_sampling_lib ${dependencies})

add_executable(workspace_sampling src/workspace_sampling_node.cpp)
ament_target_dependencies(workspace_sampling ${dependencies})
target_include_directories(workspace_sampling PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(workspace_sampling workspace_sampling_lib)

add_executable(translation_scaling src/translation_scaling.cpp)
ament_target_dependencies(translation_scaling ${dependencies})
target_link_libraries(translation_scaling Eigen3::Eigen yaml-cpp Threads::Threads)
target_compile_features(translation_scaling PRIVATE cxx_std_17)

add_executable(dls_teleoperation_controller src/DLS_teleoperation_controller.cpp)
ament_target_dependencies(dls_teleoperation_controller ${dependencies})
target_link_libraries(dls_teleoperation_controller OsqpEigen::OsqpEigen)

add_executable(frame_translator src/frame_translator.cpp)
ament_target_dependencies(frame_translator ${dependencies})
target_link_libraries(frame_translator yaml-cpp)

add_executable(movement_calibration src/movement_calibration.cpp)
ament_target_dependencies(movement_calibration ${dependencies})
target_link_libraries(movement_calibration yaml-cpp)

install(
  TARGETS
  dls_teleoperation_controller
  frame_translator
  movement_calibration
  workspace_sampling_lib
  workspace_sampling
  translation_scaling
  EXPORT
  workspace_sampling_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY headers/
  DESTINATION include
)

install(
  DIRECTORY 
  launch 
  config
  DESTINATION share/${PROJECT_NAME}
)

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
