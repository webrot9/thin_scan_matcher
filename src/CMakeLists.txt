# Include the subdirectories
ADD_SUBDIRECTORY(tsm_core)

add_executable(thin_scan_matcher_node 
  message_handler.cpp 
  thin_scan_matcher_node.cpp
)

target_link_libraries(thin_scan_matcher_node
  tsm_library
  ${catkin_LIBRARIES}
)
