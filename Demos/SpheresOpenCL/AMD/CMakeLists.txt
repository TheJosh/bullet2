

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src 
${BULLET_PHYSICS_SOURCE_DIR}/Extras/OpenCL/Shared
${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL
$ENV{AMDAPPSDKROOT}/include
)

IF (CMAKE_CL_64)
	SET(CMAK_AMDAPPSDKROOT_LIBPATH
		)
	SET(CMAK_GLEW_LIBRARY
		${BULLET_PHYSICS_SOURCE_DIR}/Glut/glew64.lib
		)
ELSE(CMAKE_CL_64)
	SET(CMAK_AMDAPPSDKROOT_LIBPATH
		$ENV{AMDAPPSDKROOT}/lib/x86
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
		${CMAK_AMDAPPSDKROOT_LIBPATH}/OpenCL.lib
	)


	ADD_EXECUTABLE(AppSpheresOCL_AMD
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
					${BULLET_PHYSICS_SOURCE_DIR}/Extras/OpenCL/Shared/btOclCommon.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/SpheresGrid.cl
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/Integration.cl
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/Integration.cu
	)
	IF (NOT INTERNAL_CREATE_MSVC_RELATIVE_PATH_PROJECTFILES)
			IF (CMAKE_CL_64)
				ADD_CUSTOM_COMMAND(
					TARGET AppSpheresOCL_AMD
					POST_BUILD
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/glut64.dll ${CMAKE_CURRENT_BINARY_DIR}
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/glew64.dll ${CMAKE_CURRENT_BINARY_DIR}
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/SpheresGrid.cl ${CMAKE_CURRENT_BINARY_DIR}
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/Integration.cl ${CMAKE_CURRENT_BINARY_DIR}
				)
			ELSE(CMAKE_CL_64)
				ADD_CUSTOM_COMMAND(
					TARGET AppSpheresOCL_AMD
					POST_BUILD
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/GLUT32.DLL ${CMAKE_CURRENT_BINARY_DIR}
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/glew32.DLL ${CMAKE_CURRENT_BINARY_DIR}
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/SpheresGrid.cl ${CMAKE_CURRENT_BINARY_DIR}
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/Demos/SpheresOpenCL/Shared/Integration.cl ${CMAKE_CURRENT_BINARY_DIR}
				)
			ENDIF(CMAKE_CL_64)
		ENDIF (NOT INTERNAL_CREATE_MSVC_RELATIVE_PATH_PROJECTFILES)
ELSE (USE_GLUT)
ENDIF (USE_GLUT)

IF (UNIX)
  TARGET_LINK_LIBRARIES(AppSpheresOCL_AMD pthread)
ENDIF(UNIX)

