cmake_minimum_required(VERSION 2.8.3)
project(saver_service)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge 
  pcl_ros
  message_generation
  roscpp
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS signals thread)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  pcl_ros 
  rosconsole
  DEPENDS 
  opencv 
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

add_service_files(
  FILES
  BollesDataSaver.srv
)

include_directories(${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(GTK2)
add_definitions(-DHAVE_GTK)
include_directories(${GTK2_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/saver_service.cpp)

add_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${GTK_LIBRARIES}
  ${GTK2_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  yaml-cpp
)
