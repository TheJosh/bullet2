
INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src
)

# Configure use of OpenCL and DX11
# Generates the settings file and defines libraries and include paths
OPTION(USE_OPENCL "Use OpenCL"	OFF)
OPTION(USE_DX11 "Use DirectX 11"	OFF)

#SET(DX11_INCLUDE_PATH "C:/Program Files (x86)/Microsoft DirectX SDK (February 2010)/Include" CACHE DOCSTRING "Microsoft directX SDK include path")
#SET(OPENCL_INCLUDE_PATH "S:/Perforce/advtech/contrib/opencl/include" CACHE DOCSTRING "OpenCL SDK include path")


# OpenCL and DX11 setting options driven from OPTIONS above
if( USE_OPENCL )
	SET(OPENCL_INCLUDE_PATH "Unknown" CACHE DOCSTRING "OpenCL SDK include path")
endif( USE_OPENCL )
if( USE_DX11 )
	SET(DX11_INCLUDE_PATH "Unknown" CACHE DOCSTRING "Microsoft directX SDK include path")
endif( USE_DX11 )
SET(BULLET_USE_OPENCL false)
SET(BULLET_USE_DX11 false)
if( USE_OPENCL )
	SET(BULLET_USE_OPENCL true)
endif( USE_OPENCL )
if( USE_DX11 )
	SET(BULLET_USE_DX11 true)
endif( USE_DX11 )


# Create the btAcceleratedSoftBody_Settings.h file that defines the
# values enabling use of DX11 and OpenCL for users of the
# library.
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/btAcceleratedSoftBody_Settings.h.in ${CMAKE_CURRENT_BINARY_DIR}/btAcceleratedSoftBody_Settings.h)


SET(BulletSoftBody_SRCS
	btSoftBody.cpp
	btSoftBodyConcaveCollisionAlgorithm.cpp
	btSoftBodyHelpers.cpp
	btSoftBodyRigidBodyCollisionConfiguration.cpp
	btSoftRigidCollisionAlgorithm.cpp
	btSoftRigidDynamicsWorld.cpp
	btSoftSoftCollisionAlgorithm.cpp

	btAcceleratedSoftBody_CPUSolver.cpp
	btAcceleratedSoftBody_DX11Solver.cpp
	btAcceleratedSoftBody_OpenCLSolver.cpp
	btAcceleratedSoftBody_DX11Buffer.cpp

)

SET(BulletSoftBody_HDRS
	btSoftBody.h
	btSoftBodyConcaveCollisionAlgorithm.h
	btSoftBodyHelpers.h
	btSoftBodyRigidBodyCollisionConfiguration.h
	btSoftRigidCollisionAlgorithm.h
	btSoftRigidDynamicsWorld.h
	btSoftSoftCollisionAlgorithm.h
	btSparseSDF.h

	btAcceleratedSoftBody_Settings.h
	btAcceleratedSoftBodyData.h
	btAcceleratedSoftBody_DXVertexBuffers.h
	btAcceleratedSoftBody_VertexBuffers.h
	btAcceleratedSoftBodyData.h
	btAcceleratedSoftBody_OpenCLSolver.h
	btAcceleratedSoftBody_DX11Solver.h
	btAcceleratedSoftBody_CPUSolver.h
	btAcceleratedSoftBody_Solvers.h
	btAcceleratedSoftBody_VertexDataOpenCL.h
	btAcceleratedSoftBody_VertexDataDX11.h
	btAcceleratedSoftBody_TriangleDataOpenCL.h
	btAcceleratedSoftBody_TriangleDataDX11.h
	btAcceleratedSoftBody_LinkDataDX11.h
	btAcceleratedSoftBody_LinkDataOpenCL.h
	btAcceleratedSoftBody_OpenCLBuffer.h
	btAcceleratedSoftBody_DX11Buffer.h
	btOpenCLSupport.h
	btDirectComputeSupport.h

)

# OpenCL and HLSL Shaders.
# Build rules generated to stringify these into headers
# which are needed by some of the sources
SET(BulletSoftBody_Shaders
	OutputToVertexArray
	UpdateNormals
	Integrate
	UpdateConstants
	UpdatePositions
	UpdateNodes
	SolvePositions
	UpdatePositionsFromVelocities
	VSolveLinks
	ApplyForces
	PrepareLinks
	CollideCylinder
	AddVelocity
	CollideSphere
)

foreach(f ${BulletSoftBody_Shaders})
    LIST(APPEND BulletSoftBody_OpenCLC "${f}CL.h")
endforeach(f) 

foreach(f ${BulletSoftBody_Shaders})
    LIST(APPEND BulletSoftBody_HLSL "${f}HLSL.h")
endforeach(f) 

foreach( f ${BulletSoftBody_Shaders})
	ADD_CUSTOM_COMMAND(
		OUTPUT ${f}HLSL.h
		COMMAND StringifyFile ${f}HLSL.h HLSL/${f}.hlsl ${f}HLSLString
		DEPENDS StringifyFile HLSL/${f}.hlsl
	)
endforeach(f)

foreach( f ${BulletSoftBody_Shaders})
	ADD_CUSTOM_COMMAND(
		OUTPUT ${f}CL.h
		COMMAND StringifyFile ${f}CL.h OpenCLC/${f}.cl ${f}CLString
		DEPENDS StringifyFile OpenCLC/${f}.cl
	)
endforeach(f)

if( BULLET_USE_OPENCL )
	INCLUDE_DIRECTORIES(${OPENCL_INCLUDE_PATH})
endif( BULLET_USE_OPENCL )

if( BULLET_USE_DX11 )
	INCLUDE_DIRECTORIES(${DX11_INCLUDE_PATH})
endif( BULLET_USE_DX11 )




ADD_LIBRARY(BulletSoftBody  ${BulletSoftBody_SRCS} ${BulletSoftBody_HDRS} ${BulletSoftBody_OpenCLC} ${BulletSoftBody_HLSL})
SET_TARGET_PROPERTIES(BulletSoftBody PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(BulletSoftBody PROPERTIES SOVERSION ${BULLET_VERSION})
IF (BUILD_SHARED_LIBS)
	TARGET_LINK_LIBRARIES(BulletSoftBody BulletDynamics)
ENDIF (BUILD_SHARED_LIBS)

IF (INSTALL_LIBS)
	IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
		IF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.5)
			IF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
				INSTALL(TARGETS BulletSoftBody DESTINATION .)
			ELSE (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
			INSTALL(TARGETS BulletSoftBody DESTINATION lib${LIB_SUFFIX})
				INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include FILES_MATCHING PATTERN "*.h")
			ENDIF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
		ENDIF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.5)

		IF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
			SET_TARGET_PROPERTIES(BulletSoftBody PROPERTIES FRAMEWORK true)
			SET_TARGET_PROPERTIES(BulletSoftBody PROPERTIES PUBLIC_HEADER "${BulletSoftBody_HDRS}")
		ENDIF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
	ENDIF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
ENDIF (INSTALL_LIBS)
