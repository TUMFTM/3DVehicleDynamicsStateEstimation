find_package(param_manager_cpp REQUIRED)
find_package(ros_debug_helpers_cpp REQUIRED)
find_package(tum_types_cpp REQUIRED)
find_package(tum_helpers_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(state_estimation_cpp REQUIRED)
find_package(state_estimation_base REQUIRED)
find_package(state_estimation_constants REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tum_type_conversions_ros_cpp REQUIRED)
find_package(ros2_watchdog_cpp REQUIRED)

set(NODE_EXECUTABLE_NAMES 
  state_estimation_node_cpp_ekf_2d_kin
  state_estimation_node_cpp_ekf_3d_kin
  state_estimation_node_cpp_ekf_2d_nh
  state_estimation_node_cpp_ekf_3d_nh
  state_estimation_node_cpp_ekf_3d_stm
)
set(NODE_EXECUTABLE_PATHS 
  cpp/kinematic/state_estimation_node_cpp_ekf_2d_main.cpp
  cpp/kinematic/state_estimation_node_cpp_ekf_3d_main.cpp
  cpp/non_holonomic/state_estimation_node_cpp_ekf_2d_main.cpp
  cpp/non_holonomic/state_estimation_node_cpp_ekf_3d_main.cpp
  cpp/single_track_model/state_estimation_node_cpp_ekf_3d_main.cpp
)

# section: BUILD EXECUTABLE
# ====================================================================
foreach(NODE_EXECUTABLE_NAME NODE_EXECUTABLE_PATH IN ZIP_LISTS NODE_EXECUTABLE_NAMES NODE_EXECUTABLE_PATHS)

  add_executable(${NODE_EXECUTABLE_NAME} ${NODE_EXECUTABLE_PATH})

  target_include_directories(${NODE_EXECUTABLE_NAME} PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<INSTALL_INTERFACE:include>
  )
  target_compile_options(${NODE_EXECUTABLE_NAME} PUBLIC -Ofast)
  ament_target_dependencies(
    ${NODE_EXECUTABLE_NAME} 
    rclcpp
    param_manager_cpp
    tum_types_cpp
    tum_helpers_cpp
    ros_debug_helpers_cpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    nav_msgs 
    msgs
    std_msgs
    sensor_msgs
    diagnostic_msgs
    state_estimation_cpp
    state_estimation_base
    state_estimation_constants
    tum_type_conversions_ros_cpp
    ros2_watchdog_cpp
  )

  install(TARGETS ${NODE_EXECUTABLE_NAME} DESTINATION lib/${PROJECT_NAME})
  install(
    DIRECTORY ../include/
    DESTINATION include
  )
endforeach()
# section: GENERAL
# ====================================================================
