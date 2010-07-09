cmake_minimum_required(VERSION 2.4)


#this line has to appear before 'PROJECT' in order to be able to disable incremental linking
SET(MSVC_INCREMENTAL_DEFAULT ON)

PROJECT(BulletClothExample)
SET(BULLET_VERSION 2.76)

IF (NOT CMAKE_BUILD_TYPE)
# SET(CMAKE_BUILD_TYPE "Debug")
 SET(CMAKE_BUILD_TYPE "Release")
ENDIF (NOT CMAKE_BUILD_TYPE) 


OPTION(USE_MSVC_RUNTIME_LIBRARY_DLL "Use MSVC Runtime Library DLL (/MD or /MDd)" ON)
OPTION(USE_MSVC_INCREMENTAL_LINKING "Use MSVC Incremental Linking" OFF)


IF(MSVC)
	IF (NOT USE_MSVC_INCREMENTAL_LINKING)
		#MESSAGE("MSVC_INCREMENTAL_DEFAULT"+${MSVC_INCREMENTAL_DEFAULT})
		SET( MSVC_INCREMENTAL_YES_FLAG "/INCREMENTAL:NO")
		 
		STRING(REPLACE "INCREMENTAL:YES" "INCREMENTAL:NO" replacementFlags ${CMAKE_EXE_LINKER_FLAGS_DEBUG}) 
		SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "/INCREMENTAL:NO ${replacementFlags}" )
		MESSAGE("CMAKE_EXE_LINKER_FLAGS_DEBUG=${CMAKE_EXE_LINKER_FLAGS_DEBUG}")
		
		STRING(REPLACE "INCREMENTAL:YES" "INCREMENTAL:NO" replacementFlags3 ${CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO})
		SET(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO ${replacementFlags3}) 
		SET(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO "/INCREMENTAL:NO ${replacementFlags3}" )
	ENDIF (NOT USE_MSVC_INCREMENTAL_LINKING)

	IF (NOT USE_MSVC_RUNTIME_LIBRARY_DLL)
		#We statically link to reduce dependancies
		FOREACH(flag_var CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
			IF(${flag_var} MATCHES "/MD")
				STRING(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
			ENDIF(${flag_var} MATCHES "/MD")
			IF(${flag_var} MATCHES "/MDd")
				STRING(REGEX REPLACE "/MDd" "/MTd" ${flag_var} "${${flag_var}}")
			ENDIF(${flag_var} MATCHES "/MDd")
		ENDFOREACH(flag_var)
	ENDIF (NOT USE_MSVC_RUNTIME_LIBRARY_DLL)

	OPTION(USE_MSVC_SSE "Use MSVC /arch:sse option"	ON)
	IF (USE_MSVC_SSE)
		ADD_DEFINITIONS(/arch:SSE)
	ENDIF()
	OPTION(USE_MSVC_FAST_FLOATINGPOINT "Use MSVC /fp:fast option"	ON)
	IF (USE_MSVC_FAST_FLOATINGPOINT)
		ADD_DEFINITIONS(/fp:fast)
  ENDIF()
ENDIF(MSVC)

   
IF(COMMAND cmake_policy)
   cmake_policy(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)
   



SET(DXSDK_DIR $ENV{DXSDK_DIR})
SET(DX11_INCLUDE_PATH  "${DXSDK_DIR}Include" CACHE DOCSTRING "Microsoft directX SDK include path")
SET(DX11_LIB_PATH "${DXSDK_DIR}Lib/x86" CACHE DOCSTRING "Microsoft DirectX SDK library path")

SET(BulletClothExample_SRCS
	DXUT/Core/DXUT.cpp
	DXUT/Optional/DXUTcamera.cpp
	DXUT/Core/DXUTDevice11.cpp
	DXUT/Core/DXUTDevice9.cpp
	DXUT/Optional/DXUTgui.cpp
	DXUT/Core/DXUTmisc.cpp
	DXUT/Optional/DXUTres.cpp
	DXUT/Optional/DXUTsettingsdlg.cpp
	DXUT/Optional/SDKmesh.cpp
	DXUT/Optional/SDKmisc.cpp	

	cloth_renderer.cpp
)


SET(BulletClothExample_HDRS
	DXUT/Core/DXUT.h
	DXUT/Optional/DXUTcamera.h
	DXUT/Core/DXUTDevice11.h
	DXUT/Core/DXUTDevice9.h
	DXUT/Optional/DXUTgui.h
	DXUT/Core/DXUTmisc.h
	DXUT/Optional/DXUTres.h
	DXUT/Optional/DXUTsettingsdlg.h
	DXUT/Optional/SDKmesh.h
	DXUT/Optional/SDKmisc.h

	btDirectComputeSupport.h
	cap.h
	cloth.h
	cylinder.h
	FreeImage.h
	resource.h
)

SET(BULLET_PATH "../Bullet/BulletTrunk" CACHE DOCSTRING "Path to Bullet")
SET(BULLET_INCLUDE_PATH "${BULLET_PATH}/src" CACHE DOCSTRING "Bullet includes")
SET(BULLET_LIB_PATH "${BULLET_PATH}/lib/" CACHE DOCSTRING "Bullet libraries")

INCLUDE_DIRECTORIES(${DX11_INCLUDE_PATH} ${BULLET_INCLUDE_PATH} "DXUT/Core" "DXUT/Optional")
LINK_DIRECTORIES(${BULLET_LIB_PATH} ${DX11_LIB_PATH})

ADD_DEFINITIONS(-DUNICODE)
ADD_DEFINITIONS(-D_UNICODE)

ADD_EXECUTABLE(BulletClothExample WIN32
	${BulletClothExample_SRCS} 
	${BulletClothExample_HDRS} 
)
TARGET_LINK_LIBRARIES(BulletClothExample
	d3dcompiler
	dxerr
	dxguid
	d3dx9
	d3d9
	winmm
	comctl32
	d3dx11
	BulletDynamics
	BulletCollision
	LinearMath
	BulletSoftBody
	BulletSoftBodySolvers_CPU
	BulletSoftBodySolvers_DX11
)