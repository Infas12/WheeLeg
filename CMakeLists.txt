cmake_minimum_required(VERSION 3.0.2)
project(WheeLeg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)


catkin_package()

catkin_install_python(PROGRAMS scripts/test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_compile_options(-lpthread)

include_directories(${catkin_INCLUDE_DIRS} src/socketCan2/inc)

set(SRCS 
    src/socketCan2/src/CanManager.cpp
    src/socketCan2/src/Math.cpp
    src/socketCan2/src/M2006.cpp
    src/socketCan2/src/Pid.cpp
    src/socketCan2/src/M3508.cpp
    )

add_executable(chassis ${SRCS} src/ChassisController.cpp)
target_link_libraries(chassis ${catkin_LIBRARIES} pthread)