cmake_minimum_required(VERSION 3.10)
project(kf_applied)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    roslint
)
## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)



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
    include
    ## This is only necessary because Eigen3 sets a non-standard EIGEN3_INCLUDE_DIR variable
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}_core
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
  DEPENDS
  ## find_package(Eigen3) provides a non standard EIGEN3_INCLUDE_DIR instead of Eigen3_INCLUDE_DIRS.
  ## Therefore, the DEPEND does not work as expected and we need to add the directory to the INCLUDE_DIRS
  # Eigen3

  ## Boost is not part of the DEPENDS since it is only used in source files,
  ## Dependees do not depend on Boost when they depend on this package.
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # Set manually because Eigen sets a non standard INCLUDE DIR
  ${EIGEN3_INCLUDE_DIR}
  # Set because Boost is an internal dependency, not transitive.
  ${Boost_INCLUDE_DIRS}
)


## Declare a C++ library
add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}_node.cpp
   src/simulate.cpp
)

## Declare a C++ executable
add_executable(
    ${PROJECT_NAME}_node 
    src/kf_applied_node.cpp
)

## Link C++ libraries
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_beginner_tutorials.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
