

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src 
${BULLET_PHYSICS_SOURCE_DIR}/Extras/OpenCL/Shared
${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL
)

ADD_DEFINITIONS(-DUSE_MINICL)

IF(WIN32)
IF (CMAKE_CL_64)
	SET(CMAK_GLEW_LIBRARY
		${BULLET_PHYSICS_SOURCE_DIR}/Glut/glew64.lib
		)
ELSE(CMAKE_CL_64)
	SET(CMAK_GLEW_LIBRARY
		${BULLET_PHYSICS_SOURCE_DIR}/Glut/glew32.lib
		)
ENDIF(CMAKE_CL_64)
ENDIF()

IF (USE_GLUT)
	LINK_LIBRARIES(
		OpenGLSupport 
		BulletDynamics  
		BulletCollision 
		BulletMultiThreaded 
		LinearMath 
		${GLUT_glut_LIBRARY} 
		${OPENGL_gl_LIBRARY} 
		${OPENGL_glu_LIBRARY}
		${CMAK_GLEW_LIBRARY}
	)


	ADD_EXECUTABLE(AppSpheresOCL_Mini
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/cutil_gl_error.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/btOclUtils.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/cutil_math.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/SpheresGridDemo.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/btSpheresGridDemoDynamicsWorld.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/btSpheresGridDemoSharedDefs.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/btSpheresGridDemoSharedTypes.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/btIntegrationDemoDynamicsWorld.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/btIntegrationDemoDynamicsWorld.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/oecakeLoader.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/shaders.h
					${BULLET_PHYSICS_SOURCE_DIR}/Extras/OpenCL/Shared/btOclCommon.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/btOclUtils.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/btSpheresGridDemoDynamicsWorld.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/main.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/oecakeLoader.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/shaders.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/SpheresGridDemo.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/MiniCL/MiniCLTaskWrap.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Extras/OpenCL/Shared/btOclCommon.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/SpheresGrid.cl
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/Integration.cl
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/Integration.cu
	)
ELSE (USE_GLUT)
ENDIF (USE_GLUT)

IF (UNIX)
  TARGET_LINK_LIBRARIES(AppSpheresOCL_Mini pthread)
ENDIF(UNIX)

