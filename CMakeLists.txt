cmake_minimum_required(VERSION 2.8.3)
project(RoverArm)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    roscpp
    serial
)

include_directories(${catkin_INCLUDE_DIRS})
#generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package()



# these are all of the nodes that I can call
add_executable(joy_teleop src/joy_teleop.cpp)
target_link_libraries(joy_teleop ${catkin_LIBRARIES})

add_executable(direct_motor_control src/direct_motor_control.cpp)
target_link_libraries(direct_motor_control ${catkin_LIBRARIES})

#FIND_LIBRARY(WIRINGPI_LIBRARY wiringPi /home/wiringPi)
#add_executable(blink src/blink.cpp)
#target_link_libraries(blink ${catkin_LIBRARIES} ${WIRINGPI_LIBRARY})
