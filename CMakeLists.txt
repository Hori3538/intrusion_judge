cmake_minimum_required(VERSION 3.0.2)
project(intrusion_judge)
add_compile_options(-std=c++17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    sensor_msgs
    nav_msgs
    )

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES intrusion_judge
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(intrusion_judge_node src/intrusion_judge_node.cpp src/intrusion_judge.cpp)
add_dependencies(intrusion_judge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
    intrusion_judge_node
    ${catkin_LIBRARIES}
    )

add_executable(dummy_person_generator_node src/dummy_person_generator_node.cpp src/dummy_person_generator.cpp)
add_dependencies(dummy_person_generator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
    dummy_person_generator_node
    ${catkin_LIBRARIES}
    )
