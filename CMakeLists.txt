cmake_minimum_required(VERSION 2.8.3)
project(bezier_planer)

# Ensure C++14 or higher is used
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # Optional: to ensure no GNU extensions

add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/sdf_tools)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs
    nav_msgs
    visualization_msgs 
    quadrotor_msgs
    tf
    geometry_msgs   # Added for geometry_msgs/Twist (used in the new C++ file)
    sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
)

link_directories( ${PROJECT_SOURCE_DIR}/third_party/mosek/lib/mosek8_1 )

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/third_party/mosek/include)
include_directories(${PROJECT_SOURCE_DIR}/third_party/sdf_tools/include)

include_directories(
    include 
    SYSTEM 
    third_party
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

# Remove -std=c++0x and rely on CMAKE_CXX_STANDARD instead
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror

# Add the executable for b_traj_node
add_executable(
    b_traj_node 
    src/b_traj_node.cpp
    src/bezier_base.cpp
    src/trajectory_generator.cpp
    src/a_star.cpp
    third_party/fast_methods/console/console.cpp
    third_party/fast_methods/fm/fmdata/fmcell.cpp
    third_party/fast_methods/ndgridmap/cell.cpp
)
target_link_libraries(b_traj_node ${catkin_LIBRARIES} sdf_tools ${PCL_LIBRARIES} mosek64)

# Add the executable for b_traj_server
add_executable ( b_traj_server src/traj_server.cpp src/bezier_base.cpp)
target_link_libraries( b_traj_server
                        ${catkin_LIBRARIES}
)

# Add the executable for random_forest_sensing
add_executable ( random_forest_sensing src/random_forest_sensing.cpp )
target_link_libraries( random_forest_sensing
                        ${catkin_LIBRARIES}
                        ${PCL_LIBRARIES}
)

# Add the executable for odom_generator
add_executable ( odom_generator src/odom_generator.cpp )
target_link_libraries( odom_generator
                        ${catkin_LIBRARIES}
)

# Add the executable for the new velocity_mapper node (mapping /position_cmd to /cmd_vel)
add_executable(velocity_map src/velocity_map.cpp)  # Assuming you named the new C++ file velocity_mapper.cpp
target_link_libraries(velocity_map
                      ${catkin_LIBRARIES}
)
