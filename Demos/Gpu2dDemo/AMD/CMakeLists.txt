

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src 
${BULLET_PHYSICS_SOURCE_DIR}/Extras/OpenCL/Shared
${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL
$ENV{AMDAPPSDKROOT}/include
)

IF (CMAKE_CL_64)
	SET(CMAK_AMDAPPSDKROOT_LIBPATH
		)
ELSE(CMAKE_CL_64)
	SET(CMAK_AMDAPPSDKROOT_LIBPATH
		$ENV{AMDAPPSDKROOT}/lib/x86
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
		3dGridBroadphaseOCL_AMD
		${CMAK_AMDAPPSDKROOT_LIBPATH}/OpenCL.lib
	)


	ADD_EXECUTABLE(AppGpu2dDemo_AMD
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu2dDemo/BasicDemo.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu2dDemo/btGpu2dDemoOCLWrap.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu2dDemo/btGpuDemo2dCpuFunc.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu2dDemo/btGpuDemoDynamicsWorld.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu2dDemo/btGpuDemoPairCache.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu2dDemo/btOclUtils.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu2dDemo/Gpu2dDemoOCL.cl
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu2dDemo/main.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu2dDemo/oecakeLoader.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu2dDemo/BasicDemo.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu2dDemo/btGpuDefines.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu2dDemo/btGpuDemo2dOCLWrap.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu2dDemo/btGpuDemo2dSharedCode.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu2dDemo/btGpuDemo2dSharedDefs.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu2dDemo/btGpuDemo2dSharedTypes.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu2dDemo/btGpuDemoDynamicsWorld.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu2dDemo/btGpuDemoPairCache.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu2dDemo/btOclUtils.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu2dDemo/oecakeLoader.h					
	)
	IF (NOT INTERNAL_CREATE_MSVC_RELATIVE_PATH_PROJECTFILES)
			IF (CMAKE_CL_64)
				ADD_CUSTOM_COMMAND(
					TARGET AppGpu2dDemo_AMD
					POST_BUILD
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/glut64.dll ${CMAKE_CURRENT_BINARY_DIR}
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/test1.oec ${CMAKE_CURRENT_BINARY_DIR}
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu2dDemo/Gpu2dDemoOCL.cl ${CMAKE_CURRENT_BINARY_DIR}
				)
			ELSE(CMAKE_CL_64)
				ADD_CUSTOM_COMMAND(
					TARGET AppGpu2dDemo_AMD
					POST_BUILD
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/GLUT32.DLL ${CMAKE_CURRENT_BINARY_DIR}
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/test1.oec ${CMAKE_CURRENT_BINARY_DIR}
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu2dDemo/Gpu2dDemoOCL.cl ${CMAKE_CURRENT_BINARY_DIR}

				)
			ENDIF(CMAKE_CL_64)
		ENDIF (NOT INTERNAL_CREATE_MSVC_RELATIVE_PATH_PROJECTFILES)
ELSE (USE_GLUT)
ENDIF (USE_GLUT)

IF (UNIX)
  TARGET_LINK_LIBRARIES(AppGpu2dDemo_AMD pthread)
ENDIF(UNIX)

