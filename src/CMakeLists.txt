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


add_executable(my_test 
  my_test.cpp 
)

target_link_libraries(my_test
  tsm_library
  ${catkin_LIBRARIES}
  ${CSPARSE_LIBRARY}
  ${G2O_TYPES_DATA}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_TYPES_SLAM2D}
)

add_executable(my_test2 
  my_test2.cpp 
)

target_link_libraries(my_test2
  tsm_library
  ${catkin_LIBRARIES}
  ${CSPARSE_LIBRARY}
  ${G2O_TYPES_DATA}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_TYPES_SLAM2D}
)

add_executable(my_test3
  my_test3.cpp 
  graph_slam/vertices_finder.cpp
)

target_link_libraries(my_test3
  tsm_library
  ${catkin_LIBRARIES}
  ${CSPARSE_LIBRARY}
  ${G2O_TYPES_DATA}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_TYPES_SLAM2D}
)
