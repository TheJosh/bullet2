cmake_minimum_required(VERSION 2.4)

PROJECT(BULLET_PHYSICS)
SET(BULLET_VERSION 2.74)

IF (NOT CMAKE_BUILD_TYPE)
# SET(CMAKE_BUILD_TYPE "Debug")
 SET(CMAKE_BUILD_TYPE "Release")
ENDIF (NOT CMAKE_BUILD_TYPE) 

 
  
IF(COMMAND cmake_policy)
   cmake_policy(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)
  
IF (APPLE)
  FIND_LIBRARY(OPENCL_LIBRARY OpenCL DOC "OpenCL lib for OSX")
  FIND_PATH(OPENCL_INCLUDE_DIR OpenCL/cl.h DOC "Include for OpenCL on OSX")
ENDIF (APPLE)
 

# This is the shortcut to finding GLU, GLUT and OpenGL if they are properly installed on your system
# This should be the case.

FIND_PACKAGE(OpenGL)
IF (OPENGL_FOUND)
MESSAGE("OPENGL FOUND")
MESSAGE(${OPENGL_LIBRARIES})
ELSE (OPENGL_FOUND)
MESSAGE("OPENGL NOT FOUND")
SET(OPENGL_gl_LIBRARY opengl32)
SET(OPENGL_glu_LIBRARY glu32)
ENDIF (OPENGL_FOUND)

# ADD_DEFINITIONS(-DBT_USE_FREEGLUT)

FIND_PACKAGE(GLU)

FIND_PACKAGE(GLUT)
IF (GLUT_FOUND)
MESSAGE("GLUT FOUND")
MESSAGE(${GLUT_glut_LIBRARY})
ELSE (GLUT_FOUND)

IF (MINGW)
MESSAGE ("GLUT NOT FOUND not found, trying to use MINGW glut32")
SET(GLUT_glut_LIBRARY glut32)
ENDIF (MINGW)

IF (MSVC)
MESSAGE ("GLUT NOT FOUND, trying to use Bullet/Glut/glut32.lib for MSVC")
SET(GLUT_glut_LIBRARY ${BULLET_PHYSICS_SOURCE_DIR}/Glut/glut32.lib)
ENDIF (MSVC)
ENDIF (GLUT_FOUND)

IF (WIN32)
SET(GLEW_glew_LIBRARY ${BULLET_PHYSICS_SOURCE_DIR}/Glut/glew32.lib)
ELSE (WIN32)
#FIND_PACKAGE(GLEW)
MESSAGE("GLEW NOT FOUND")
ENDIF (WIN32)

IF (WIN32)
  INCLUDE_DIRECTORIES(${BULLET_PHYSICS_SOURCE_DIR}/Glut)
ELSE (WIN32)
  # This is the lines for linux.  This should always work if everything is installed and working fine.
  INCLUDE_DIRECTORIES(/usr/include /usr/local/include ${GLUT_INCLUDE_DIR}) 
ENDIF (WIN32)


OPTION(BUILD_DEMOS "Set when you want to build the demos" ON)
IF(BUILD_DEMOS)
  SUBDIRS(Demos)
ENDIF(BUILD_DEMOS)


SUBDIRS(src)
