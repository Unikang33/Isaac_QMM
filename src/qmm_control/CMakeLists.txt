cmake_minimum_required(VERSION 3.8)
project(qmm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(OpenMP REQUIRED)

# Find KDL directly from system
find_package(orocos_kdl QUIET)
if(NOT orocos_kdl_FOUND)
  find_library(orocos_kdl_LIBRARY orocos-kdl)
  find_path(orocos_kdl_INCLUDE_DIR kdl/chain.hpp)
endif()

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${orocos_kdl_INCLUDE_DIR}
)

# Build tf_test executable
add_executable(tf_test src/tf_test.cpp)
ament_target_dependencies(tf_test
  rclcpp
  tf2_msgs
  geometry_msgs
)
target_link_libraries(tf_test Eigen3::Eigen)

# Build GetOptPose_qmm executable
add_executable(GetOptPose_qmm 
  src/GetOptPose_qmm.cpp
  src/InverseReachabilityMap.cpp
)
ament_target_dependencies(GetOptPose_qmm
  ament_index_cpp
  kdl_parser
)
target_link_libraries(GetOptPose_qmm 
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
  ${orocos_kdl_LIBRARY}
)

# Build test_k1_ik executable
add_executable(test_k1_ik 
  src/test_k1_ik.cpp
)
ament_target_dependencies(test_k1_ik
  ament_index_cpp
  kdl_parser
)
target_link_libraries(test_k1_ik 
  Eigen3::Eigen
  ${orocos_kdl_LIBRARY}
)

# Install executables
install(TARGETS
  tf_test
  GetOptPose_qmm
  test_k1_ik
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/home_position_controller.py
  scripts/fk_ik_controller.py
  scripts/irm_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install map directory
install(DIRECTORY map/
  DESTINATION share/${PROJECT_NAME}/map
)

# Install urdf directory
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
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
