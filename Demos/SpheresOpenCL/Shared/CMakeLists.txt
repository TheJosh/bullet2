# AppMiniCLSpheres is a testbed to run OpenCL kernels on CPU platforms lacking OpenCL implementation.
# MiniCL is a rudimentary subset of the OpenCL implementation


INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src ${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL ${OPENCL_INCLUDE}
)

LINK_LIBRARIES(
OpenGLSupport  BulletMultiThreaded  BulletDynamics  BulletCollision LinearMath ${OPENCL_LIBRARY} ${GLEW_glew_LIBRARY} ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)

ADD_EXECUTABLE(AppSpheresOpenCL
../Shared/SpheresGridDemo.cpp
../Shared/SpheresGridDemo.h
../Shared/oecakeLoader.cpp
../Shared/oecakeLoader.h
../Shared/main.cpp
../Shared/btSpheresGridDemoDynamicsWorld.cpp
../Shared/btSpheresGridDemoDynamicsWorld.h
../Shared/btSpheresGridDemoSharedDefs.h
../Shared/btIntegrationDemoDynamicsWorld.cpp
../Shared/btIntegrationDemoDynamicsWorld.h
../Shared/btOclUtils.cpp
../Shared/btOclUtils.h
../Shared/btSpheresGridDemoSharedTypes.h
../Shared/cutil_gl_error.h
../Shared/cutil_math.h
../Shared/shaders.cpp
../Shared/shaders.h
)

