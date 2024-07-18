ADD_LIBRARY( PluginROS SHARED PluginROS.cpp
)


TARGET_LINK_LIBRARIES( PluginROS
	${roscpp_LIBRARIES}
	${sensor_msgs_LIBRARIES}
	${${PROJECT_NAME}_generate_messages}
	FileIO
	XMLExecutionInterpreter
	XMLIKIDInterpreter
	${Boost_LIBRARIES}
	XercesC::XercesC
	FileLogger
)

#copy the library to the bin because catkin put it in devel
add_custom_command(TARGET PluginROS POST_BUILD 
COMMAND "${CMAKE_COMMAND}" -E copy 
   "$<TARGET_FILE:PluginROS>"
   "${_output_dir}/$<TARGET_FILE_NAME:PluginROS>" 
COMMENT "Copying to output directory")

ADD_LIBRARY( PluginEMGROS SHARED PluginEMGROS.cpp
)

TARGET_LINK_LIBRARIES( PluginEMGROS
	${roscpp_LIBRARIES}
	${sensor_msgs_LIBRARIES}
	${${PROJECT_NAME}_generate_messages}
	FileIO
	XMLExecutionInterpreter
	XMLEMGInterpreter
	${Boost_LIBRARIES}
	XercesC::XercesC
	FileLogger
)

#copy the library to the bin because catkin put it in devel
add_custom_command(TARGET PluginEMGROS POST_BUILD 
COMMAND "${CMAKE_COMMAND}" -E copy 
   "$<TARGET_FILE:PluginEMGROS>"
   "${_output_dir}/$<TARGET_FILE_NAME:PluginEMGROS>" 
COMMENT "Copying to output directory")



