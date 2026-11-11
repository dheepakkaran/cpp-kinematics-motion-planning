cmake_minimum_required(VERSION 3.16)
project(kinmp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mujoco REQUIRED)

# Header-only kinematics (Eigen only, no ROS, no MuJoCo).
add_library(kinmp_kinematics INTERFACE)
target_include_directories(kinmp_kinematics INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(kinmp_kinematics INTERFACE Eigen3::Eigen)

# OMPL planner + trajectory optimization.
add_library(kinmp_planner src/planner.cpp src/chomp.cpp)
target_link_libraries(kinmp_planner PUBLIC kinmp_kinematics ompl::ompl)

# ROS 2 node.
add_library(kinmp_node src/planning_node.cpp)
target_link_libraries(kinmp_node PUBLIC kinmp_planner rclcpp::rclcpp ${std_msgs_TARGETS} ${sensor_msgs_TARGETS})
add_executable(kinmp_planning_node src/main.cpp)
target_link_libraries(kinmp_planning_node kinmp_node)

# Demo data dump (IK per target, one planned path three ways) for the README / project page.
add_executable(kinmp_demo_dump tools/demo_dump.cpp)
target_link_libraries(kinmp_demo_dump kinmp_planner)

install(TARGETS kinmp_planning_node kinmp_demo_dump kinmp_node kinmp_planner DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(RESULTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/results")
  ament_add_gtest(test_kinematics test/test_kinematics.cpp TIMEOUT 300)
  target_link_libraries(test_kinematics kinmp_kinematics mujoco::mujoco)
  target_compile_definitions(test_kinematics PRIVATE RESULTS_DIR="${RESULTS_DIR}"
    MODEL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/assets/franka_emika_panda/panda_nohand.xml")
  ament_add_gtest(test_planner test/test_planner.cpp TIMEOUT 300)
  target_link_libraries(test_planner kinmp_planner)
  target_compile_definitions(test_planner PRIVATE RESULTS_DIR="${RESULTS_DIR}")
  ament_add_gtest(test_chomp test/test_chomp.cpp TIMEOUT 600)
  target_link_libraries(test_chomp kinmp_planner)
  target_compile_definitions(test_chomp PRIVATE RESULTS_DIR="${RESULTS_DIR}")
  ament_add_gtest(test_node test/test_node.cpp TIMEOUT 300)
  target_link_libraries(test_node kinmp_node)
  target_compile_definitions(test_node PRIVATE RESULTS_DIR="${RESULTS_DIR}")
endif()

ament_package()
