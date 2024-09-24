## Copyright (C) 2020  I. Bogoslavskyi, C. Stachniss
##
## GNU-GPL licence that follows one of libQGLViewer.


IF (MY_QT_LIBRARIES AND QGLVIEWER_LIBRARY)
  message(STATUS "*** QT FOUND! visualizer.cpp WILL be built!")
  add_library(visualization SHARED visualizer.cpp)
  target_link_libraries(visualization
                        identifiable
                        ${QGLVIEWER_LIBRARY}
                        ${MY_QT_LIBRARIES}
                        ${OPENGL_gl_LIBRARY}
                        ${OPENGL_glu_LIBRARY})
ELSE()
  message(STATUS "*** QT LIBS NOT FOUND! visualizer.cpp WILL NOT be built!")
ENDIF()

if(${PCL_FOUND})
  add_library(cloud_saver SHARED cloud_saver.cpp)
  target_link_libraries(cloud_saver ${PCL_LIBRARIES} ${OpenCV_LIBS})
endif()
