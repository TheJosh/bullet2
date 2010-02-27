# AppMiniCLVectorAdd is a very basic test for MiniCL.


INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src 
${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL
${BULLET_PHYSICS_SOURCE_DIR}/Demos/BasicDemoOpenCL/Shared
)


ADD_DEFINITIONS(-DUSE_MINICL)


IF (USE_GLUT)
	LINK_LIBRARIES(
		OpenGLSupport 
		BulletDynamics  
		BulletCollision 
		BulletMultiThreaded 
		LinearMath 
		3dGridBroadphaseOCL_Mini 
		BatchConstraintSolverOCL_Mini  
		${GLUT_glut_LIBRARY} 
		${OPENGL_gl_LIBRARY} 
		${OPENGL_glu_LIBRARY}
	)


	ADD_EXECUTABLE(AppBasicDemoOpenCL_Mini
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/BasicDemoOpenCL/Shared/BasicDemoOpenCL.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/BasicDemoOpenCL/Shared/main.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/BasicDemoOpenCL/Shared/BasicDemoOpenCL.h
	)
ELSE (USE_GLUT)
ENDIF (USE_GLUT)

IF (UNIX)
  TARGET_LINK_LIBRARIES(AppBasicDemoOpenCL_Mini pthread)
ENDIF(UNIX)

