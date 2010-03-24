

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src 
${BULLET_PHYSICS_SOURCE_DIR}/Extras/OpenCL/Shared
${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL
)


IF(INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
	INCLUDE_DIRECTORIES( $ENV{==NVSDKCOMPUTE_ROOT=}/OpenCL/common/inc	)
	IF (CMAKE_CL_64)
		SET(CMAK_NVSDKCOMPUTE_LIBPATH		)
	ELSE(CMAKE_CL_64)
		SET(CMAK_NVSDKCOMPUTE_LIBPATH		$ENV{==NVSDKCOMPUTE_ROOT=}/OpenCL/common/lib/x64	)
	ENDIF(CMAKE_CL_64)
ELSE()
	INCLUDE_DIRECTORIES( $ENV{NVSDKCOMPUTE_ROOT}/OpenCL/common/inc	)
	IF (CMAKE_CL_64)
		SET(CMAK_NVSDKCOMPUTE_LIBPATH		)
	ELSE(CMAKE_CL_64)
		SET(CMAK_NVSDKCOMPUTE_LIBPATH		$ENV{NVSDKCOMPUTE_ROOT}/OpenCL/common/lib/Win32	)
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
		3dGridBroadphaseOCL_Nv
		${CMAK_NVSDKCOMPUTE_LIBPATH}/OpenCL.lib
	)


	ADD_EXECUTABLE(AppGpu3dDemo_Nv
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu3dDemo/BasicDemo3d.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu3dDemo/btGpuDefines.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu3dDemo/btGpuDemo3dOCLWrap.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu3dDemo/btGpuDemo3dSharedCode.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu3dDemo/btGpuDemo3dSharedDefs.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu3dDemo/btGpuDemo3dSharedTypes.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu3dDemo/btGpuDemoDynamicsWorld3D.h
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu3dDemo/btOclUtils.h	
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu3dDemo/BasicDemo3d.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu3dDemo/btGpu3dDemoOCLWrap.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu3dDemo/btGpuDemo3dCpuFunc.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu3dDemo/btGpuDemoDynamicsWorld3D.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu3dDemo/btOclUtils.cpp
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu3dDemo/main.cpp	
					${BULLET_PHYSICS_SOURCE_DIR}/Demos/Gpu3dDemo/Gpu3dDemoOCL.cl	
	)
ELSE (USE_GLUT)
ENDIF (USE_GLUT)

IF (UNIX)
  TARGET_LINK_LIBRARIES(AppGpu3dDemo_Nv pthread)
ENDIF(UNIX)

