cmake_minimum_required(VERSION 2.8.3)
project(neo_sick_s300)

set(CMAKE_CXX_STANDARD 11)

## By default build with code optimization and no debug info
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release"
  	  CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
# add include search paths
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/SickS300Receiver.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(neo_sick_s300_node src/neo_sick_s300_node.cpp)
add_executable(neo_scan_filter_node src/neo_scan_filter_node.cpp)
target_link_libraries(neo_sick_s300_node ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(neo_scan_filter_node ${catkin_LIBRARIES})
add_dependencies(neo_sick_s300_node ${catkin_EXPORTED_TARGETS})
add_dependencies(neo_scan_filter_node ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} neo_sick_s300_node neo_scan_filter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


