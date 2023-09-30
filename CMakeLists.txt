cmake_minimum_required(VERSION 2.8.3)
project(lio_sam)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_DEBUG "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
  # interactive_markers
  serial
)

find_package( GTSAMCMakeTools )
find_package(Boost REQUIRED COMPONENTS timer)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
  # interactive_markers
)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
  /opt/ros/melodic/include/
  /usr/include/
  /usr/local/include/
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

###########
## Build ##
###########

# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Feature Association
add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
add_dependencies(${PROJECT_NAME}_featureExtraction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmization.cpp)
add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptmization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam)

# IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

# LiAuto
add_executable(${PROJECT_NAME}_liauto src/liauto.cpp)
target_include_directories(${PROJECT_NAME}_liauto PUBLIC ${lanelet2_core_INCLUDE_DIRS} ${lanelet2_io_INCLUDE_DIRS} ${lanelet2_projection_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_liauto ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${lanelet2_core_LIBRARIES} ${lanelet2_io_LIBRARIES} ${lanelet2_projection_LIBRARIES})

#drive
# add_executable(${PROJECT_NAME}_drive src/drive.cpp)
# add_dependencies(${PROJECT_NAME}_drive ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_drive ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#stdmessage
add_library(std_message src/std_message.cpp)
# #usbcan
# add_executable(${PROJECT_NAME}_usbcan src/usbcan.cpp)
# add_dependencies(${PROJECT_NAME}_usbcan ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_usbcan ${catkin_LIBRARIES} ${PCL_LIBRARIES} std_message)
#tcp
add_executable(${PROJECT_NAME}_tcp src/tcp.cpp)
add_dependencies(${PROJECT_NAME}_tcp ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_tcp ${catkin_LIBRARIES} ${PCL_LIBRARIES} std_message)

#ttlControl
add_executable(${PROJECT_NAME}_ttlControl src/ttlControl.cpp)
add_dependencies(${PROJECT_NAME}_ttlControl ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_ttlControl ${catkin_LIBRARIES} ${PCL_LIBRARIES} std_message)

#paperData
# add_executable(${PROJECT_NAME}_paperData src/paperData.cpp)
# target_include_directories(${PROJECT_NAME}_paperData PUBLIC ${lanelet2_core_INCLUDE_DIRS} ${lanelet2_io_INCLUDE_DIRS} ${lanelet2_projection_INCLUDE_DIRS})
# target_link_libraries(${PROJECT_NAME}_paperData ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${lanelet2_core_LIBRARIES} ${lanelet2_io_LIBRARIES} ${lanelet2_projection_LIBRARIES})

#speedAndLight
add_executable(${PROJECT_NAME}_test src/test.cpp)
add_dependencies(${PROJECT_NAME}_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} std_message)

# scenePedestrain
add_executable(${PROJECT_NAME}_scenePedestrain src/scenePedestrain.cpp)
target_include_directories(${PROJECT_NAME}_scenePedestrain PUBLIC ${lanelet2_core_INCLUDE_DIRS} ${lanelet2_io_INCLUDE_DIRS} ${lanelet2_projection_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_scenePedestrain ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${lanelet2_core_LIBRARIES} ${lanelet2_io_LIBRARIES} ${lanelet2_projection_LIBRARIES})