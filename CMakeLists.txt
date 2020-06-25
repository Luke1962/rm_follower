
cmake_minimum_required(VERSION 2.8.3)
project(rm_follower)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_geometry_msgs
  tf2_ros
  tf2
  visualization_msgs
 # image_transport
 # cv_bridge
  std_msgs
  dnn_detect #genera l'array delle detection usata da questo nodo
  actionlib_msgs
  actionlib
  vision_msgs
)

##############
## OpenCV   ##
##############
# find_package(OpenCV 3.4.6 REQUIRED)
set(OpenCV_INCLUDE_DIRS
  /usr/local/include/opencv4/opencv2
)

# set(OpenCV_LIB_DIR
#   /usr/local/lib
# )

# set(OpenCV_LIBS
#   opencv_core
#   opencv_highgui
#   opencv_imgcodecs
# )

include_directories(${OpenCV_INCLUDE_DIRS} include())
link_directories(${OpenCV_LIB_DIR})

message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")

########################################à

add_message_files(
   FILES
   DetectedObject.msg
   DetectedObjectArray.msg
)

# actionlib --------
# use the add_action_files macro to declare the actions you want to be generated:
add_action_files(
  DIRECTORY action
  FILES Move.action
)

add_service_files(
  FILES
  Detect.srv
)


#Questa sezione va usata quando devo usare messaggi di altri pkg e/o del proprio package
# in package.xml vanno poi aggiunti
# <build_depend>message_generation</build_depend>
# <exec_depend>message_runtime</exec_depend>
generate_messages(
	DEPENDENCIES
	std_msgs
	actionlib_msgs
	vision_msgs
	visualization_msgs
	
)



#catkin_package(INCLUDE_DIRS DEPENDS OpenCV actionlib_msgs)
catkin_package(INCLUDE_DIRS DEPENDS  actionlib_msgs visualization_msgs)

###########
## Build ##
###########

add_definitions(-std=c++11)

include_directories(${catkin_INCLUDE_DIRS})
#include_directories(${OpenCV_INCLUDE_DIRS})

#add_executable(rm_detect src/rm_filter_person.cpp)
add_executable(rm_follower src/rm_follower2.cpp  )

#add_dependencies(rm_detect ${${PROJECT_NAME}_EXPORTED_TARGETS}
#                 ${catkin_EXPORTED_TARGETS})
add_dependencies(rm_follower ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
                 
# Link your application with OpenCV libraries                 
#target_link_libraries(rm_detect ${catkin_LIBRARIES} ${OpenCV_LIBS} opencv_dnn opencv_imgproc)
target_link_libraries(rm_follower ${catkin_LIBRARIES} )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS rm_follower
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
) 

