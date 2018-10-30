cmake_minimum_required(VERSION 2.4.6)
project(robot_calibration)

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    roscpp
    message_generation
    tf
)



#####
# Detection of CSM library using CMake routines
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSL REQUIRED gsl)
pkg_check_modules(CSM REQUIRED csm)

IF(${CSM_FOUND})
	MESSAGE("CSM_LIBRARY_DIRS: ${CSM_LIBRARY_DIRS}")
	MESSAGE("CSM_LIBRARIES: ${CSM_LIBRARIES}")
	MESSAGE("CSM_INCLUDE_DIRS: ${CSM_INCLUDE_DIRS}")

	INCLUDE_DIRECTORIES(${CSM_INCLUDE_DIRS})
	LINK_DIRECTORIES(${CSM_LIBRARY_DIRS})

ELSE(${CSM_FOUND})	
	MESSAGE(SEND_ERROR "CSM not found. Make sure PKG_CONFIG_PATH contains the directory where the file 'csm.pc' can be found.")
ENDIF(${CSM_FOUND})	
#####

INCLUDE_DIRECTORIES(include 
	           # ${EIGEN_INCLUDES}
		   )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
        roscpp
        rosbag
        message_runtime
)

add_library(robot_calibration
            src/robot_calibration.cpp
            src/calib_tuple.cpp
            src/gsl_jacobian.cpp
            src/solver_utils.cpp
            src/solver2_meat.cpp
                     )

target_link_libraries(robot_calibration
                      ${catkin_LIBRARIES}
	             # ${EIGEN_LIBRARIES}
                      )

target_link_libraries(robot_calibration 
                      ${GSL_LIBRARIES})
target_link_libraries(robot_calibration 
                      ${CSM_LIBRARIES})

add_executable(robot_calibration_node
                src/robot_calibration_node.cpp
               )

target_link_libraries(robot_calibration_node robot_calibration)
set_target_properties(robot_calibration_node PROPERTIES OUTPUT_NAME robot_calibration)

install(DIRECTORY include/robot_calibration/
        DESTINATION /opt/ros/indigo/include)

install(TARGETS 
        robot_calibration
        robot_calibration_node 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

