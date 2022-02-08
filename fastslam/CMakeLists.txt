cmake_minimum_required(VERSION 3.0.2)
project(fastslam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

add_definitions(-std=c++14)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -mavx2")
message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -Wall -march=native -mavx2")
message("CMAKE_CXX_FLAGS_RELWITHDEBINFO is ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_filters
  custom_msgs
  nav_msgs
  visualization_msgs
  tf
)

find_package(PCL REQUIRED)

# use self "compiled" Eigen version if found in default location
SET(EIGEN3_INCLUDE_DIR "/usr/local/include/eigen3/")
# if not found, use system version to build at least somehow
IF( NOT EXISTS EIGEN3_INCLUDE_DIR )
	message("Custom Eigen not found, defaulting to system version")
	find_package(Eigen3 REQUIRED)
ENDIF()
# Stan: 16.04 ships with 3.2beta of Eigen that causes dozens of 
# warnings if compiling with GCC >6.1
# ROS requires system Eigen version 3.2beta so I use this custom Eigen 3.3

find_package(OpenMP REQUIRED)

message("Eigen found in ${EIGEN3_INCLUDE_DIR}")

if(OPENMP_FOUND)
	message(STATUS "OPENMP FOUND")
	set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
        set(OpenMP_LIBS gomp)
endif()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS roscpp std_msgs tf custom_msgs nav_msgs
  LIBRARIES fastslam
  DEPENDS OpenMP
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  src/
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare fastslam C++ library
add_library(fastslam
   src/fastslam.cpp src/tools.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_compile_options(fastslam PRIVATE ${OpenMP_FLAGS})

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} custom_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
fastslam
${catkin_LIBRARIES}
${OpenMP_LIBS}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/test_fastslam.cpp)
if(TARGET ${PROJECT_NAME}-test)
   target_link_libraries(${PROJECT_NAME}-test fastslam ${catkin_LIBRARIES}
${OpenMP_LIBS} 
)
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
