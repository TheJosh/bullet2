

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src 
${BULLET_PHYSICS_SOURCE_DIR}/Extras/OpenCL/Shared
${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL
${NVIDIA_OPENCL_BASE_DIR}/include
)

IF (CMAKE_CL_64)
	SET(CMAK_NVSDKCOMPUTE_LIBPATH
	${NVIDIA_OPENCL_BASE_DIR}/lib/x64
		)
ELSE(CMAKE_CL_64)
	SET(CMAK_NVSDKCOMPUTE_LIBPATH
		${NVIDIA_OPENCL_BASE_DIR}/lib/Win32
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
		3dGridBroadphaseOCL_Nv
		${CMAK_NVSDKCOMPUTE_LIBPATH}/OpenCL.lib
	)


	ADD_EXECUTABLE(AppGpu2dDemo_Nv
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
ELSE (USE_GLUT)
ENDIF (USE_GLUT)

IF (UNIX)
  TARGET_LINK_LIBRARIES(AppGpu2dDemo_Nv pthread)
ENDIF(UNIX)

