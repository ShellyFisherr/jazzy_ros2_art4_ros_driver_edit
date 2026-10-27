cmake_minimum_required(VERSION 3.8)
project(tracker_udp_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(OPENVR_SDK_ROOT "$~/openvr" CACHE PATH "Path to the OpenVR SDK")

find_path(OPENVR_INCLUDE_DIR
  NAMES openvr.h
  HINTS "${OPENVR_SDK_ROOT}/headers"
)

find_library(OPENVR_LIBRARY
  NAMES openvr_api
  HINTS "${OPENVR_SDK_ROOT}/lib/linux64"
)

if(NOT OPENVR_INCLUDE_DIR OR NOT OPENVR_LIBRARY)
  message(FATAL_ERROR
    "OpenVR SDK not found. set OPENVR_SDK_ROOT to the SDK directory.")
endif()

# find dependeincies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  )

add_executable(tracker_udp_bridge src/tracker_udp_bridge.cpp)
ament_target_dependencies(tracker_udp_bridge ${dependencies})

add_executable(timer_test src/timer_test.cpp)
ament_target_dependencies(timer_test ${dependencies})

add_executable(ROSTimer src/RosTimer_CTimer.cpp)
ament_target_dependencies(ROSTimer ${dependencies})

add_executable(openvr_tracker src/openvr_tracker.cpp)
ament_target_dependencies(openvr_tracker ${dependencies})
target_include_directories(openvr_tracker PRIVATE "${OPENVR_INCLUDE_DIR}")
target_link_libraries(openvr_tracker "${OPENVR_LIBRARY}")
target_compile_features(openvr_tracker PRIVATE cxx_std_17)
get_filename_component(OPENVR_LIBRARY_DIR "${OPENVR_LIBRARY}" DIRECTORY)
set_target_properties(openvr_tracker PROPERTIES
  INSTALL_RPATH "${OPENVR_LIBRARY_DIR}"
)

install(
  TARGETS
  tracker_udp_bridge
  timer_test
  ROSTimer
  openvr_tracker
  DESTINATION lib/${PROJECT_NAME}
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