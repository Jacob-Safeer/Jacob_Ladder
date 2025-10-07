cmake_minimum_required(VERSION 3.5)
project(precision_land)

set(CMAKE_CXX_STANDARD 20)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(px4_ros2_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(${Eigen3_INCLUDE_DIRS})

# Original static platform version
add_executable(precision_land
    PrecisionLand.cpp
)
ament_target_dependencies(precision_land rclcpp Eigen3 px4_ros2_cpp geometry_msgs tf2_ros tf2)
target_compile_features(precision_land PUBLIC c_std_99 cxx_std_17)

# New track & follow version
add_executable(track_follow
    TrackFollow.cpp
)
ament_target_dependencies(track_follow rclcpp Eigen3 px4_ros2_cpp geometry_msgs tf2_ros tf2)
target_compile_features(track_follow PUBLIC c_std_99 cxx_std_17)

# Front camera approach mission
add_executable(front_approach
    FrontApproach.cpp
)
ament_target_dependencies(front_approach rclcpp Eigen3 px4_ros2_cpp geometry_msgs tf2_ros tf2)
target_compile_features(front_approach PUBLIC c_std_99 cxx_std_17)

# Combined front approach + precision land mission
add_executable(front_to_precision_land
    FrontApproachPrecisionLandCombined.cpp
)
ament_target_dependencies(front_to_precision_land rclcpp Eigen3 px4_ros2_cpp geometry_msgs tf2_ros tf2 px4_msgs)
target_compile_features(front_to_precision_land PUBLIC c_std_99 cxx_std_17)

# Install both executables
install(TARGETS
  precision_land
  track_follow
  front_approach
  front_to_precision_land
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# Install launch and config folders
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
