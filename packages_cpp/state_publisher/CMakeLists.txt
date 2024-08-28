cmake_minimum_required(VERSION 3.12)

# CONFIGURE PKG HERE
# ==========================================
set(PROJECT_NAME state_publisher)
# ============================================
# END OF CONFIGURATION

project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(DIRECTORY
launch
urdf
DESTINATION share/${PROJECT_NAME}/
)

# section: GENERAL
# ====================================================================
ament_package()
