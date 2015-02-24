ADD_LIBRARY(tsm_library SHARED
 cloud2d.cpp cloud2d.h
 projector2d.cpp projector2d.h
 solver2d.cpp solver2d.h
 cloud_processor.cpp cloud_processor.h
 tracker.cpp tracker.h
 correspondence_finder2d.cpp correspondence_finder2d.h
 )

TARGET_LINK_LIBRARIES(tsm_library  ${OpenCV_LIBS} ${OPENGL_gl_LIBRARY})

#ADD_EXECUTABLE (depth_utils_test depth_utils_test.cpp)

#TARGET_LINK_LIBRARIES(depth_utils_test  ps_core_library ${OpenCV_LIBS} ${OPENGL_gl_LIBRARY})
