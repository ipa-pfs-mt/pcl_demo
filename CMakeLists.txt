cmake_minimum_required(VERSION 2.8.3)
project(ipa_pcl_demo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions pcl_ros roscpp sensor_msgs message_generation OpenCV image_transport cv_bridge
)
find_package(HDF5 REQUIRED)

catkin_package(
  #  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
#INCLUDE_DIRS include
#LIBRARIES ${PROJECT_NAME}
#CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs image_transport rospy std_msgs
)

include_directories(
include ${catkin_INCLUDE_DIRS}
${OpenCV_LIBRARIES}
)
include_directories(${HDF5_INCLUDE_DIRS})
add_library(${PROJECT_NAME}

src/demo3.cpp
)

add_executable(example3 src/demo3main.cpp)
target_link_libraries(example3 ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES}
    ${Boost_LIBRARIES} ${FLANN_LIBRARIES})

add_executable(akit_perception akit_perception.cpp)
target_link_libraries(akit_perception ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES}
    ${Boost_LIBRARIES} ${FLANN_LIBRARIES})

add_executable(pass_through pass_through.cpp)
target_link_libraries(pass_through ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES}
    ${Boost_LIBRARIES} ${FLANN_LIBRARIES})

add_executable(ipa_akit_detection ipa_akit_detection.cpp)
target_link_libraries(ipa_akit_detection ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES}
    ${Boost_LIBRARIES} ${FLANN_LIBRARIES})

add_executable(robust_pose robust_pose.cpp)
target_link_libraries(robust_pose ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES}
    ${Boost_LIBRARIES} ${FLANN_LIBRARIES})


add_executable(example demo.cpp)
target_link_libraries(example ${catkin_LIBRARIES})

add_executable(example2 demo2.cpp)
target_link_libraries(example2 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(example4 demo3backup.cpp)
target_link_libraries(example4 ${catkin_LIBRARIES} ${PCL_LIBRARIES}
    ${Boost_LIBRARIES} ${HDF5_LIBRARIES})

#add_executable(detection_with_opencv detection_with_opencv.cpp)
#target_link_libraries(detection_with_opencv ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable (correspondence_grouping correspondence_grouping.cpp)
target_link_libraries (correspondence_grouping ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable (pcd_write pcd_write.cpp)
target_link_libraries (pcd_write ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(model_training model_training.cpp)
target_link_libraries(model_training ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${HDF5_LIBRARIES})

#add_executable(local_pipeline local_pipeline.cpp)
#target_link_libraries(local_pipeline ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${HDF5_LIBRARIES})
