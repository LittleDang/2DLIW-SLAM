cmake_minimum_required(VERSION 3.0.2)
project(lvio_2d)

add_compile_options(-std=c++17)
SET(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3  -g ")
add_definitions(-DNDEBUG)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  tf2_ros
)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(

)

include_directories(
  /usr/include/eigen3
  src
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)
add_library(utilies src/utilies/common.cpp 
src/utilies/params.cpp
src/utilies/visualization.cpp
)


add_library(trajectory  src/trajectory/trajectory.cpp
src/trajectory/camera_manager.cpp
src/trajectory/laser_manager.cpp
src/trajectory/keyframe_manager.cpp
src/factor/solver.cpp
)


add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp 
)

target_link_libraries(${PROJECT_NAME}_node 
  utilies
  trajectory
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CERES_LIBRARIES}
)
