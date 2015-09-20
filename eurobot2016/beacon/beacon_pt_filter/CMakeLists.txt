cmake_minimum_required(VERSION 2.8.3)
project(beacon_pt_filter)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
    dynamic_reconfigure 
	std_msgs
	geometry_msgs
	sensor_msgs
	tf
	pcl_ros
	pcl_conversions
	laser_geometry
)

# protected region additional user defined REQUIREMENTS on begin #
# protected region additional user defined REQUIREMENTS end #

generate_dynamic_reconfigure_options(cfg/beacon_pt_filter.cfg )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    common/src
  CATKIN_DEPENDS 
  	roscpp
    dynamic_reconfigure
	std_msgs
	geometry_msgs
	sensor_msgs
	tf
	pcl_ros
	pcl_conversions
	laser_geometry
   
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(common/include)
include_directories(common/src)
include_directories(${catkin_INCLUDE_DIRS})


## Declare a cpp executable
add_executable(beacon_pt_filter ros/src/beacon_pt_filter_ros.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(beacon_pt_filter beacon_pt_filter_gencfg)
add_dependencies(beacon_pt_filter ${beacon_pt_filter_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(beacon_pt_filter
  ${catkin_LIBRARIES}
)

# protected region additional user defined BUILD STATEMENTS on begin #
# protected region additional user defined BUILD STATEMENTS end #


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS beacon_pt_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# protected region user Cmake macros on begin #
# protected region user Cmake macros end #
