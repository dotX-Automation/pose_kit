cmake_minimum_required(VERSION 3.10)
project(pose_kit)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fvisibility=hidden)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)

# Pose library configuration
add_library(pose SHARED src/pose/pose.cpp)
add_library(pose_kit::pose ALIAS pose)
target_compile_features(pose PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_definitions(pose PRIVATE "POSE_BUILDING_LIBRARY")
target_include_directories(pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(pose
  Eigen3
  geometry_msgs
  rclcpp
  std_msgs
  tf2
  tf2_eigen)

# Kinematic Pose library configuration
add_library(kinematicpose SHARED src/kinematic_pose/kinematic_pose.cpp)
add_library(pose_kit::kinematicpose ALIAS kinematicpose)
target_compile_features(kinematicpose PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_definitions(kinematicpose PRIVATE "KINEMATIC_POSE_BUILDING_LIBRARY")
target_include_directories(kinematicpose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(kinematicpose
  pose)
ament_target_dependencies(kinematicpose
  Eigen3
  geometry_msgs
  tf2
  std_msgs)

# Dynamic Pose library configuration
add_library(dynamicpose SHARED src/dynamic_pose/dynamic_pose.cpp)
add_library(pose_kit::dynamicpose ALIAS dynamicpose)
target_compile_features(dynamicpose PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_definitions(dynamicpose PRIVATE "DYNAMIC_POSE_BUILDING_LIBRARY")
target_include_directories(dynamicpose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(dynamicpose
  kinematicpose)
ament_target_dependencies(dynamicpose
  Eigen3
  sensor_msgs
  tf2)

# Libraries installation
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS pose kinematicpose dynamicpose
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

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

# Export all dependencies and library targets for this package
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(
  Eigen3
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen)

ament_package()
