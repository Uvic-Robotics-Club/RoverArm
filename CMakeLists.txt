cmake_minimum_required(VERSION 2.8.3)
project(RoverArm)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    message_generation
    roscpp
    rospy
    serial
)

add_message_files(FILES
  joint_velocities.msg
  joint_enable.msg
  arm_velocity.msg
  joint_angles.msg
)

add_service_files(FILES
  point_to_angle.srv
  angles_to_points.srv
)

include_directories(${catkin_INCLUDE_DIRS})
generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package()



# these are all of the nodes that I can call
add_executable(joy_teleop src/joy_teleop.cpp)
target_link_libraries(joy_teleop ${catkin_LIBRARIES})

add_executable(point_list src/preprogramed_points.cpp)
target_link_libraries(point_list ${catkin_LIBRARIES})

add_executable(program_teleop src/program_teleop.cpp)
target_link_libraries(program_teleop ${catkin_LIBRARIES})

add_executable(direct_motor_control src/direct_motor_control.cpp)
target_link_libraries(direct_motor_control ${catkin_LIBRARIES})
