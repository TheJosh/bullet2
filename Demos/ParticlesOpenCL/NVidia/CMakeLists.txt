

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src 
${BULLET_PHYSICS_SOURCE_DIR}/Extras/OpenCL/Shared
${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL
$ENV{NVSDKCOMPUTE_ROOT}/OpenCL/common/inc	
)

IF (CMAKE_CL_64)
	SET(CMAK_NVSDKCOMPUTE_LIBPATH
		)
	SET(CMAK_GLEW_LIBRARY
		${BULLET_PHYSICS_SOURCE_DIR}/Glut/glew64.lib
		)
ELSE(CMAKE_CL_64)
	SET(CMAK_NVSDKCOMPUTE_LIBPATH
		$ENV{NVSDKCOMPUTE_ROOT}/OpenCL/common/lib/Win32
		)
	SET(CMAK_GLEW_LIBRARY
		${BULLET_PHYSICS_SOURCE_DIR}/Glut/glew32.lib
		)
ENDIF(CMAKE_CL_64)


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
		${CMAK_NVSDKCOMPUTE_LIBPATH}/OpenCL.lib
	)


	ADD_EXECUTABLE(AppPaticlesOCL_Nv
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/ParticlesOpenCL/btParticlesDynamicsWorld.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/ParticlesOpenCL/btParticlesSharedDefs.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/ParticlesOpenCL/btParticlesSharedTypes.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/ParticlesOpenCL/ParticlesDemo.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/ParticlesOpenCL/btOclUtils.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/ParticlesOpenCL/shaders.h	
					${BULLET_PHYSICS_SOURCE_DIR}/Extras/OpenCL/Shared/btOclCommon.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/ParticlesOpenCL/btOclUtils.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/ParticlesOpenCL/btParticlesDemoDynamicsWorld.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/ParticlesOpenCL/main.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/ParticlesOpenCL/ParticlesDemo.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/ParticlesOpenCL/shaders.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Extras/OpenCL/Shared/btOclCommon.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/ParticlesOpenCL/ParticlesOCL.cl
	)
ELSE (USE_GLUT)
ENDIF (USE_GLUT)

IF (UNIX)
  TARGET_LINK_LIBRARIES(AppPaticlesOCL_Nv pthread)
ENDIF(UNIX)

