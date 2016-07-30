cmake_minimum_required(VERSION 2.8)
PROJECT(ICPR14)

#FIND OPENCV
FIND_PACKAGE(OpenCV REQUIRED)

# find qt4 and qglviewer and set variables
SET (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(Qt4 REQUIRED)
FIND_PACKAGE(QGLViewer)
SET(QT_USE_QTOPENGL TRUE) 
SET(QT_USE_QTXML TRUE) 
INCLUDE(${QT_USE_FILE})
link_directories(${QGLViewer_LIBRARY_DIR})
INCLUDE_DIRECTORIES(${QGLViewer_INCLUDE_DIR}) 

# find eigen library
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -lGLU -fopenmp -lrt -O3")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
add_executable(${PROJECT_NAME} src/main.cpp src/groundtruth.cpp src/RenderSurface.cpp src/rxso3.hpp src/se2.hpp src/se3.hpp src/sim3.hpp src/sophus.hpp src/Track.cpp src/Viewer.cpp src/Voxelgrid.cpp)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS} ${QT_LIBRARIES} ${QGLViewer_LIBRARIES} ${OPENGL_gl_LIBRARY} OpenGL32 glut32 freeglut)