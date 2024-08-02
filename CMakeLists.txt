cmake_minimum_required(VERSION 2.8.3)
project(intensity_td)

SET(CMAKE_BUILD_TYPE "Release")

ADD_COMPILE_OPTIONS(-std=c++17)
ADD_COMPILE_OPTIONS(-std=c++17)

set(CMAKE_CXX_FLAGS "-std=c++17 -O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -std=c++0x -std=c++17 -fexceptions")

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 9)
    add_definitions(-DMP_EN)
  endif()
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  std_msgs
  pcl_ros
  tf
  message_generation
  eigen_conversions
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(GTSAM REQUIRED QUIET)

message(Eigen: ${EIGEN3_INCLUDE_DIR})
message("GTSAM_INCLUDE_DIR: " ${GTSAM_INCLUDE_DIR})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include)


catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs message_runtime
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS
)

link_directories(
 src
)

file(GLOB intensity_td_thirdparty_files
  "include/ikd-Tree/ikd_Tree.cpp"
  "include/AutoExposure/image_processing.cpp"
)

add_executable(demo_online_opt demo/demo_online_opt.cpp ${intensity_td_thirdparty_files})
target_link_libraries(demo_online_opt ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARY_DIRS} ${OpenCV_LIBS} gtsam libintensity_td.so)

add_executable(demo_sthereo_opt demo/demo_sthereo_opt.cpp ${intensity_td_thirdparty_files})
target_link_libraries(demo_sthereo_opt ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARY_DIRS} ${OpenCV_LIBS} gtsam libintensity_td.so)

add_executable(demo_fusionportable_opt demo/demo_fusionportable_opt.cpp ${intensity_td_thirdparty_files})
target_link_libraries(demo_fusionportable_opt ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARY_DIRS} ${OpenCV_LIBS} gtsam libintensity_td.so)