cmake_minimum_required(VERSION 3.0.2)
project(pybullet_utils)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  geometry_msgs
)

 add_service_files(
	 FILES
   SpawnModel.srv
   DeleteModel.srv
   ChangeControlMode.srv
   SaveState.srv
   RestoreState.srv
   DeleteState.srv
   SensorReset.srv
 )

 generate_messages(
	 DEPENDENCIES
	 std_msgs
   geometry_msgs
 )

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
