

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src 
${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL
${BULLET_PHYSICS_SOURCE_DIR}/Demos/BasicDemoOpenCL/Shared
$ENV{NVSDKCOMPUTE_ROOT}/OpenCL/common/inc	
)

IF (CMAKE_CL_64)
	SET(CMAK_NVSDKCOMPUTE_ROOT
		)
ELSE(CMAKE_CL_64)
	SET(CMAK_NVSDKCOMPUTE_LIBPATH
		$ENV{NVSDKCOMPUTE_ROOT}/OpenCL/common/lib/Win32
		)
ENDIF(CMAKE_CL_64)



IF (USE_GLUT)
	LINK_LIBRARIES(
		OpenGLSupport 
		BulletDynamics  
		BulletCollision 
		BulletMultiThreaded 
		LinearMath 
		3dGridBroadphaseOCL_Nv 
		BatchConstraintSolverOCL_Nv  
		${GLUT_glut_LIBRARY} 
		${OPENGL_gl_LIBRARY} 
		${OPENGL_glu_LIBRARY}
		${CMAK_NVSDKCOMPUTE_LIBPATH}/OpenCL.lib
	)


	ADD_EXECUTABLE(AppBasicDemoOpenCL_Nv
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/BasicDemoOpenCL/Shared/BasicDemoOpenCL.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/BasicDemoOpenCL/Shared/main.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/BasicDemoOpenCL/Shared/BasicDemoOpenCL.h
	)
ELSE (USE_GLUT)
ENDIF (USE_GLUT)

IF (UNIX)
  TARGET_LINK_LIBRARIES(AppBasicDemoOpenCL_Nv pthread)
ENDIF(UNIX)

