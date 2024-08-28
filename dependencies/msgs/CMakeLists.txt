cmake_minimum_required(VERSION 3.12)
# CONFIGURE PKG HERE
# ==========================================
set(PROJECT_NAME msgs)
set(pkg_DEPENDENCIES
  rclcpp
  std_msgs
  builtin_interfaces
  rosidl_default_generators
)
# ============================================
# END OF CONFIGURATION
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
foreach(pkg_name ${pkg_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()


# Get all msg files
file(GLOB msgFiles RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msgFiles}
  DEPENDENCIES ${pkg_DEPENDENCIES}
 )

ament_export_dependencies(${pkg_DEPENDENCIES} rosidl_default_generators rosidl_default_runtime)
ament_package()