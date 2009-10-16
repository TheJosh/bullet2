# AppMiniCLVectorAdd is a very basic test for MiniCL.


INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src }
)

LINK_LIBRARIES(
 BulletMultiThreaded LinearMath MiniCL
)

ADD_EXECUTABLE(AppMiniCLVectorAdd
MiniCL_VectorAdd.cpp
)

