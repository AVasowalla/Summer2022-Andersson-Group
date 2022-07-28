cmake_minimum_required(VERSION 3.0.2)
project(wireless_com)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  transfer_rate.msg
  sent_data.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/bluetooth_drone.py scripts/bluetooth_sensor.py scripts/plot_rates.py scripts/integrate_rates.py scripts/plot_data.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
