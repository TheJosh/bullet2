/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006, 2007 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include <cstdlib>
#include <cstdio>
#include <string.h>

#include <GL/glut.h>
#include <cuda_gl_interop.h>

#include "cutil_math.h"
#include "math_constants.h"

#include <vector_types.h>

//! Check for CUDA error
#define BT_GPU_CHECK_ERROR(errorMessage)									\
	do																		\
	{																		\
		cudaError_t err = cudaGetLastError();								\
		if(err != cudaSuccess)												\
		{																	\
			fprintf(stderr,"Cuda error: %s in file '%s' in line %i : %s.\n",\
				errorMessage, __FILE__, __LINE__, cudaGetErrorString( err));\
			btCuda_exit(EXIT_FAILURE);                                      \
		}                                                                   \
		err = cudaThreadSynchronize();                                      \
		if(err != cudaSuccess)												\
		{																	\
			fprintf(stderr,"Cuda error: %s in file '%s' in line %i : %s.\n",\
				errorMessage, __FILE__, __LINE__, cudaGetErrorString( err));\
			btCuda_exit(EXIT_FAILURE);										\
		}																	\
	}																		\
	while(0)


#define BT_GPU_SAFE_CALL_NO_SYNC(call)										\
	do																		\
	{																		\
		cudaError err = call;												\
		if(err != cudaSuccess)												\
		{																	\
			fprintf(stderr, "Cuda error in file '%s' in line %i : %s.\n",	\
				__FILE__, __LINE__, cudaGetErrorString( err) );             \
			btCuda_exit(EXIT_FAILURE);										\
		}																	\
	}																		\
	while(0)


#define BT_GPU_SAFE_CALL(call)												\
	do																		\
	{																		\
		BT_GPU_SAFE_CALL_NO_SYNC(call);										\
		cudaError err = cudaThreadSynchronize();							\
		if(err != cudaSuccess)												\
		{																	\
			fprintf(stderr,"Cuda errorSync in file '%s' in line %i : %s.\n",\
				__FILE__, __LINE__, cudaGetErrorString( err) );				\
			btCuda_exit(EXIT_FAILURE);										\
		}																	\
	} while (0)


//Round a / b to nearest higher integer value
uint iDivUp(uint a, uint b){
    return (a % b != 0) ? (a / b + 1) : (a / b);
}

// compute grid and thread block size for a given number of elements
void computeGridSize(uint n, uint blockSize, uint &numBlocks, uint &numThreads)
{
    numThreads = min(blockSize, n);
    numBlocks = iDivUp(n, numThreads);
}


__global__ void kIntegrateMotion(	float4* pPos,
									float4* pLinVel,
									int numObjects,
									float4* pParams, 
									float timeStep)
{
    uint index = __umul24(blockIdx.x,blockDim.x) + threadIdx.x;
    if (index >= numObjects) return;
	float4 pos = pPos[index];
	float4 linVel = pLinVel[index];
	float4 gravity = pParams[0];
	linVel += gravity * timeStep;
	pos += linVel * timeStep;
	pPos[index] = pos;
	pLinVel[index] = linVel;
}




extern "C"
{

void btCuda_exit(int val)
{
    fprintf(stderr, "Press ENTER key to terminate the program\n");
    getchar();
	exit(val);
}

void btCuda_allocateArray(void** devPtr, unsigned int size)
{
    BT_GPU_SAFE_CALL(cudaMalloc(devPtr, size));
}

void btCuda_freeArray(void* devPtr)
{
    BT_GPU_SAFE_CALL(cudaFree(devPtr));
}

void btCuda_copyArrayFromDevice(void* host, const void* device, unsigned int size)
{   
    BT_GPU_SAFE_CALL(cudaMemcpy(host, device, size, cudaMemcpyDeviceToHost));
}

void btCuda_copyArrayToDevice(void* device, const void* host, unsigned int size)
{
    BT_GPU_SAFE_CALL(cudaMemcpy((char*)device, host, size, cudaMemcpyHostToDevice));
}


void btCuda_registerGLBufferObject(unsigned int vbo)
{
    BT_GPU_SAFE_CALL(cudaGLRegisterBufferObject(vbo));
}

void* btCuda_mapGLBufferObject(unsigned int vbo)
{
    void *ptr;
    BT_GPU_SAFE_CALL(cudaGLMapBufferObject(&ptr, vbo));
    return ptr;
}

void btCuda_unmapGLBufferObject(unsigned int vbo)
{
    BT_GPU_SAFE_CALL(cudaGLUnmapBufferObject(vbo));
}


void btCuda_integrateMotion(void* pPos, 
							void* pLinVel, 
							int numObjects,
							void* pParams, 
							float timeStep)
{
    uint numThreads, numBlocks;
//    computeGridSize(numObjects, 256, numBlocks, numThreads);
    computeGridSize(numObjects, 128, numBlocks, numThreads);
    // execute the kernel
    kIntegrateMotion<<< numBlocks, numThreads >>>(	(float4*)pPos,
											(float4*)pLinVel,
											numObjects,
											(float4*)pParams,
											timeStep);
     BT_GPU_CHECK_ERROR("Kernel execution failed: kIntegrateMotion");
}


}