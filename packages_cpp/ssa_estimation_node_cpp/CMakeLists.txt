cmake_minimum_required(VERSION 3.12)

# CONFIGURE PKG HERE
# ==========================================
set(PROJECT_NAME ssa_estimation_node_cpp)
set(NODE_EXECUTABLE_NAME ssa_estimation_node)
set(NODE_EXECUTABLE_PATH src/ssa_estimation_node_main.cpp)
set(NODE_DEPENDENCIES
  rclcpp
  param_manager_cpp
  ros_param_helpers_cpp
  ros_debug_helpers_cpp
  tum_types_cpp
  tum_helpers_cpp
  msgs
  nav_msgs
  sensor_msgs
  ssa_estimation_constants
  ssa_estimation_cpp
  diagnostic_msgs
  tum_type_conversions_ros_cpp
  ros2_watchdog_cpp
)
# ============================================
# END OF CONFIGURATION

project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
foreach(pkg_name ${NODE_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()

# section: BUILD LIBRARY TARGET TO IMPORT NODE CLASS WITHIN OTHER PKGS
# ====================================================================
add_executable(${NODE_EXECUTABLE_NAME} ${NODE_EXECUTABLE_PATH})

target_include_directories(${NODE_EXECUTABLE_NAME}
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_options(${NODE_EXECUTABLE_NAME} PUBLIC -Ofast)
ament_target_dependencies(${NODE_EXECUTABLE_NAME} ${NODE_DEPENDENCIES})
ament_export_dependencies(${NODE_DEPENDENCIES}) # other dependencies so client does not need to call find_package on them

install(
  TARGETS ${NODE_EXECUTABLE_NAME}  
  DESTINATION lib/${PROJECT_NAME}
)
install(
    DIRECTORY include/
    DESTINATION include
  )

# section: GENERAL
# ====================================================================
ament_package()
