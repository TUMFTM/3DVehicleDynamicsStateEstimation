cmake_minimum_required(VERSION 3.12)

# CONFIGURE PKG HERE
# ==========================================
set(PROJECT_NAME tum_helpers_cpp)
set(DEPENDENCIES
  Eigen3
  tum_types_cpp
)
# ============================================
# END OF CONFIGURATION

project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
foreach(pkg_name ${DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()

# section: HEADER-ONLY LIBRARY TARGET
# ====================================================================
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_options(${PROJECT_NAME} INTERFACE -Ofast)
target_compile_features(${PROJECT_NAME} INTERFACE c_std_99 cxx_std_17)

ament_target_dependencies(${PROJECT_NAME} INTERFACE Eigen3 tum_types_cpp)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

ament_package()