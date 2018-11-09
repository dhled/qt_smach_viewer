cmake_minimum_required(VERSION 2.8.3)

project(qt_smach_viewer)

find_package(catkin REQUIRED COMPONENTS
  message_generation smach std_msgs )

add_message_files(FILES
  SmachContainerStructure.msg
  SmachStateMachineStatus.msg
  SmachStateMachineStructure.msg
  )

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime smach std_msgs)

install(DIRECTORY resources
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/smach_viewer
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
