# Include the subdirectories
ADD_SUBDIRECTORY(tsm_core)

add_executable(thin_scan_matcher_node 
  thin_scan_matcher_node.cpp
)

target_link_libraries(thin_scan_matcher_node
  tsm_library
  ${QGLVIEWER_LIBRARY} 
  ${QT_QTXML_LIBRARY} 
  ${QT_QTOPENGL_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${catkin_LIBRARIES}
)
