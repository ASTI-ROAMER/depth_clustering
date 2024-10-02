## Copyright (C) 2020  I. Bogoslavskyi, C. Stachniss
##
## Permission is hereby granted, free of charge, to any person obtaining a
## copy of this software and associated documentation files (the "Software"),
## to deal in the Software without restriction, including without limitation
## the rights to use, copy, modify, merge, publish, distribute, sublicense,
## and/or sell copies of the Software, and to permit persons to whom the
## Software is furnished to do so, subject to the following conditions:
##
## The above copyright notice and this permission notice shall be included in
## all copies or substantial portions of the Software.
##
## THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
## IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
## FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
## AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
## LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
## FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
## DEALINGS IN THE SOFTWARE.

cmake_minimum_required(VERSION 3.1)
project(depth_clustering)

include(DistVersion.cmake)
system_info(DISTRO)
message(STATUS "DISTRO: ${DISTRO}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC -pthread")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

cmake_policy(SET CMP0045 OLD)

SET(CMAKE_MODULE_PATH
      ${CMAKE_MODULE_PATH}
      "${PROJECT_SOURCE_DIR}/cmake_modules"
      "/usr/share/cmake-2.8/Modules"  ## Hack for travis
    )

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)


find_package(GTest)
IF(GTest_FOUND)
  MESSAGE("GTest found!")
ELSE()
  MESSAGE("GTest NOT found!")
  IF(IS_DIRECTORY "/usr/src/gtest/")
    MESSAGE("Found google test sources in /usr/src/gtest/")
    MESSAGE("Mimick the behaviour of find_package(GTest)")
    ADD_SUBDIRECTORY(/usr/src/gtest/ gtest)

    # mimick the behaviour of find_package(GTest)
    SET(GTEST_FOUND TRUE)
    SET(GTEST_BOTH_LIBRARIES gtest gtest_main)
    SET(GTEST_LIBRARIES gtest)
    SET(GTEST_MAIN_LIBRARIES gtest_main)
  ENDIF()
ENDIF()


set(Boost_USE_STATIC_LIBS OFF)

option(BUILD_WITH_COVERAGE "Enable compilation with coverage information." OFF)
if(BUILD_WITH_COVERAGE)
  message(STATUS "Enabled generation of code coverage information with gcc.")
  set(CMAKE_CXX_FLAGS "-fprofile-arcs -ftest-coverage ${CMAKE_CXX_FLAGS}")
endif()

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem regex program_options)
find_package(PCL QUIET COMPONENTS common io)
find_package(QGLViewer)


set(MY_QT_LIBRARIES FALSE)
# find correct qt version
if(${DISTRO} MATCHES "14.04")
  find_package(Qt4 REQUIRED COMPONENTS QtCore QtXml QtOpenGL QtGui QtMultimedia)
  include(${QT_USE_FILE})
  include_directories(${QT_INCLUDES})
  set(MY_QT_LIBRARIES
      Qt4::QtCore
      Qt4::QtXml
      Qt4::QtOpenGL
      Qt4::QtGui)
  if(Qt4_FOUND)
    message(STATUS "*** Qt4 for DISTRO[${DISTRO}] found!")
  else()
    message(STATUS "*** Qt4 for DISTRO[${DISTRO}] NOT found!")
  endif()
elseif(${DISTRO} MATCHES "16.04" OR ${DISTRO} MATCHES "18.04" OR ${DISTRO} MATCHES "20.04")
  find_package(Qt5 REQUIRED COMPONENTS Core Xml OpenGL Gui Widgets)
  include_directories(${Qt5Core_INCLUDE_DIRS}
                      ${Qt5Xml_INCLUDE_DIRS}
                      ${Qt5Gui_INCLUDE_DIRS}
                      ${Qt5Widgets_INCLUDE_DIRS}
                      ${Qt5OpenGL_INCLUDE_DIRS})
  set(MY_QT_LIBRARIES
      ${Qt5Widgets_LIBRARIES}
      ${Qt5Core_LIBRARIES}
      ${Qt5Gui_LIBRARIES}
      ${Qt5Xml_LIBRARIES}
      ${Qt5OpenGL_LIBRARIES})
  if(Qt5_FOUND)
    message(STATUS "*** Qt5 for DISTRO[${DISTRO}] found!")
  else()
    message(STATUS "*** Qt5 for DISTRO[${DISTRO}] NOT found!")
  endif()
else()
  message(STATUS "DISTRO ${DISTRO} is not compatible!")
endif()

## Uncomment this if you don't want visualizer related things
# set(MY_QT_LIBRARIES FALSE)

# some status messages
message(STATUS "Linking against Qt libs: ${MY_QT_LIBRARIES}")
message(STATUS "Linking against QGlViewer lib: ${QGLVIEWER_LIBRARY}")
# message(STATUS "found QGLVIEWER libs: ${QGLVIEWER_FOUND}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  message_filters
  tf
  eigen_conversions
)

# add some useful constants to configuration
configure_file(config/cmake_config.h.in
               ${PROJECT_SOURCE_DIR}/config/cmake_config.h)
include_directories(config)


if (${PCL_FOUND})
  message(STATUS "PCL is found. PCL-related code will be built.")
  if (${CMAKE_BUILD_TYPE} MATCHES "Debug" AND ${DISTRO} MATCHES "14.04")
    message(FATAL_ERROR
            "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
            "You are building PCL in Debug mode with C++11 enabled.\n"
            "This causes segmentation fault due to a bug in PCL.\n"
            "Build in Release mode to avoid this bug.\n"
            "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
  endif()
  add_definitions(-DPCL_FOUND)
else()
  message(WARNING "PCL NOT FOUND! PCL-related code WILL NOT be built")
endif()

if(${roscpp_FOUND})
  message(STATUS "ROS found, building ROS related parts")
  set(ROS_FOUND YES)
else()
  message(WARNING "ROS NOT FOUND. NOT building ROS related parts!")
  set(ROS_FOUND NO)
endif()


IF (MY_QT_LIBRARIES AND QGLVIEWER_LIBRARY)
  catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
        cloud
        identifiable
        ground_remove
        difference
        image_labeler
        projections
        ros_bridge
        velodyne_utils
        folder_reader
        visualization
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs nav_msgs message_filters tf
  )
ELSE()
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
          cloud
          identifiable
          ground_remove
          difference
          image_labeler
          projections
          ros_bridge
          velodyne_utils
          folder_reader
    CATKIN_DEPENDS roscpp sensor_msgs std_msgs nav_msgs message_filters tf
    )
ENDIF()

include_directories(
  # for qt/widgets headers, for QT moc
  src
  include
  # for QT moc files - for opengl_folder_player.cpp (in src) to find ui_opengl_folder_player.h  (in build)
  ${PROJECT_BINARY_DIR}/src        
  # since QT MOC file ui_opengl_folder_player.h needs qt/viewer/viewer.h (in src)
  include/${PROJECT_NAME}
  # dependencies we have no control over
  SYSTEM ${EIGEN3_INCLUDE_DIR}
  SYSTEM ${Boost_INCLUDE_DIRS}
  SYSTEM ${PCL_INCLUDE_DIRS}
  SYSTEM ${OpenCV_INCLUDE_DIRS}
  SYSTEM ${OpenGL_INCLUDE_DIRS}
  SYSTEM ${catkin_INCLUDE_DIRS}
)

IF (MY_QT_LIBRARIES AND QGLVIEWER_LIBRARY)
  message(STATUS "**** Including QGLVIEWER_INCLUDE_DIR!!!")
  include_directories(
    SYSTEM ${QGLVIEWER_INCLUDE_DIR}
  )
ELSE()
  message(STATUS "**** NOT including QGLVIEWER_INCLUDE_DIR!!!")
ENDIF()

add_subdirectory(src)
add_subdirectory(examples)
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

message(STATUS "BUILD_TESTING: ${BUILD_TESTING}")
if (${BUILD_TESTING})
  add_subdirectory(test)
endif()
