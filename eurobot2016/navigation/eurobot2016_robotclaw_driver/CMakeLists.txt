cmake_minimum_required(VERSION 2.8.3)
project(eurobot2016_robotclaw_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
    dynamic_reconfigure 
	geometry_msgs
)

# protected region additional user defined REQUIREMENTS on begin #
# protected region additional user defined REQUIREMENTS end #

generate_dynamic_reconfigure_options(cfg/eurobot2016_robotclaw_driver.cfg )

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
	geometry_msgs
   
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
add_executable(eurobot2016_robotclaw_driver ros/src/eurobot2016_robotclaw_driver_ros.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(eurobot2016_robotclaw_driver eurobot2016_robotclaw_driver_gencfg)
add_dependencies(eurobot2016_robotclaw_driver ${eurobot2016_robotclaw_driver_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(eurobot2016_robotclaw_driver
  ${catkin_LIBRARIES}
)

# protected region additional user defined BUILD STATEMENTS on begin #
# protected region additional user defined BUILD STATEMENTS end #


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS eurobot2016_robotclaw_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# protected region user Cmake macros on begin #
# protected region user Cmake macros end #
