

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src 
${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL
${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL
)


IF (APPLE)
  FIND_LIBRARY(OPENCL_LIBRARY OpenCL DOC "OpenCL lib for OSX")
  FIND_PATH(OPENCL_INCLUDE_DIR OpenCL/cl.h DOC "Include for OpenCL on OSX")
ENDIF (APPLE)


IF (USE_GLUT)
	LINK_LIBRARIES(
		OpenGLSupport 
		BulletSoftBodySolvers_OpenCL_Apple
		BulletSoftBodySolvers_CPU
		BulletMultiThreaded
		BulletSoftBody
		BulletDynamics  
		BulletCollision  
		LinearMath
		${OPENCL_LIBRARY}
		${GLUT_glut_LIBRARY} 
		${OPENGL_gl_LIBRARY} 
		${OPENGL_glu_LIBRARY}
		${CMAK_GLEW_LIBRARY}
	)


	ADD_EXECUTABLE(AppClothDemo_Apple
		../cl_cloth_demo.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOclUtils.h
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOclCommon.h
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOclUtils.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOclCommon.cpp
		../gl_win.cpp
		../clstuff.cpp
		../bmpLoader.cpp
		../bmpLoader.hpp
		../clstuff.hpp
		../gl_win.hpp			

	)
ELSE (USE_GLUT)
ENDIF (USE_GLUT)


ADD_CUSTOM_COMMAND(	TARGET AppClothDemo_Apple	POST_BUILD
			COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenCLClothDemo/amdFlag.bmp ${CMAKE_CURRENT_BINARY_DIR}
			COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenCLClothDemo/atiFlag.bmp ${CMAKE_CURRENT_BINARY_DIR}
			)

IF (UNIX)
  TARGET_LINK_LIBRARIES(AppClothDemo_Apple pthread)
ENDIF(UNIX)

