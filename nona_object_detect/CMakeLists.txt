cmake_minimum_required(VERSION 2.8.3)
project(nona_object_detect)
# find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp rospy std_msgs)

find_package(catkin REQUIRED sensor_msgs image_geometry cv_bridge roscpp std_msgs image_transport pcl_conversions pcl_ros)
find_package(PCL REQUIRED QUIET COMPONENTS common)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui QtOpenGL)

catkin_package(
   INCLUDE_DIRS include 
   ${PCL_COMMON_INCLUDE_DIRS}
   ${PCL_INCLUDE_DIRS}

#  LIBRARIES nona_object_detect  
#  CATKIN_DEPENDS cv_bridge roscpp
#  DEPENDS system_lib
)



###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)
include_directories(SYSTEM
	${PCL_COMMON_INCLUDE_DIRS}
)
find_package(OpenCV REQUIRED)
find_package(cvBlob REQUIRED)

add_executable(nona_red_track_kinect src/red_track_kinect.cpp)
target_link_libraries(nona_red_track_kinect ${OpenCV_LIBS})
target_link_libraries(nona_red_track_kinect ${cvBlob_LIBS})
target_link_libraries(nona_red_track_kinect ${catkin_LIBRARIES} )


add_executable(nona_localize_object src/nona_localize_object.cpp)
target_link_libraries(nona_localize_object ${OpenCV_LIBS})
target_link_libraries(nona_localize_object ${cvBlob_LIBS})
target_link_libraries(nona_localize_object ${catkin_LIBRARIES} )

add_executable(nona_test_kinect test/test_kinect.cpp)
target_link_libraries(nona_test_kinect ${catkin_LIBRARIES} )
target_link_libraries(nona_test_kinect ${PCL_COMMON_INCLUDE_DIRS} )
target_link_libraries(nona_test_kinect ${PCL_LIBRARY_DIRS})

add_executable(nona_face_track src/face_track.cpp)
target_link_libraries(nona_face_track ${OpenCV_LIBS})
target_link_libraries(nona_face_track ${catkin_LIBRARIES} )


