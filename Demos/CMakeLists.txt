
IF (USE_GLUT)


if (CMAKE_SIZEOF_VOID_P MATCHES "8")
	SUBDIRS()
else (CMAKE_SIZEOF_VOID_P MATCHES "8")
	SUBDIRS( 		
			MiniCL_VectorAdd
			BasicDemoOpenCL
			Gpu2dDemo
			Gpu3dDemo
			ParticlesOpenCL
			SpheresOpenCL
			OpenGL
			Benchmarks
			)
endif (CMAKE_SIZEOF_VOID_P MATCHES "8")

ELSE (USE_GLUT)

SUBDIRS( 
		MiniCL_VectorAdd
		BasicDemoOpenCL
		)

ENDIF (USE_GLUT)