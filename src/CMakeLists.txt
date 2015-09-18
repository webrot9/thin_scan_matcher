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

add_executable(test1
  test1.cpp 
  graph_slam/simple_graph.cpp 
  graph_slam/graph_manipulator.cpp
)

target_link_libraries(test1
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

add_executable(test2 
  test2.cpp
  graph_slam/simple_graph.cpp 
  graph_slam/graph_manipulator.cpp
)

target_link_libraries(test2
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

add_executable(test3
  test3.cpp 
  graph_slam/vertices_finder.cpp
  graph_slam/closure_buffer.cpp
  graph_slam/closure_checker.cpp
  graph_slam/graph_manipulator.cpp
  graph_slam/simple_graph.cpp 
  graph_slam/closures_finder.cpp
)

target_link_libraries(test3
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

add_executable(test4
  test4.cpp 
  graph_slam/vertices_finder.cpp
  graph_slam/closure_buffer.cpp
  graph_slam/closure_checker.cpp
  graph_slam/graph_manipulator.cpp
  graph_slam/simple_graph.cpp 
  graph_slam/closures_finder.cpp
)

target_link_libraries(test4
  tsm_library
  ${catkin_LIBRARIES}
  ${CSPARSE_LIBRARY}
  ${G2O_TYPES_DATA}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_TYPES_SLAM2D}
  ${G2O_VIEWER}
)
