cmake_minimum_required(VERSION 2.8.3)
project(pr2eus_tutorials)

find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs sensor_msgs message_generation)
add_message_files(DIRECTORY msg FILES
  DecoImages.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs sensor_msgs
  )

catkin_package()

install(DIRECTORY euslisp launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # indigo: pr2_simulator pr2.launch arg API changed from kinetic
  # melodic: pr2_simulator is not released yet
  find_package(pr2_gazebo QUIET)
  if("$ENV{ROS_DISTRO}" STRGREATER "indigo" AND pr2_gazebo_FOUND)
      # disable test: too long time
      # add_rostest(test/test_pr2_tabletop_sim.test)
  endif()
endif()
