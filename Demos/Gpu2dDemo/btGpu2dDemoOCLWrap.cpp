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

#include <stdio.h>
#include <stdlib.h>

#ifdef __APPLE__
//CL_PLATFORM_MINI_CL could be defined in build system
#else
#include <GL/glew.h>
#include <CL/cl_platform.h> //for CL_PLATFORM_MINI_CL definition
#endif //__APPLE__

#include "LinearMath/btScalar.h"
#include "LinearMath/btMinMax.h"
#include "btOclUtils.h"

#include "btGpuDemo2dOCLWrap.h"


cl_context			btGpuDemo2dOCLWrap::m_cxMainContext;
cl_device_id		btGpuDemo2dOCLWrap::m_cdDevice;
cl_command_queue	btGpuDemo2dOCLWrap::m_cqCommandQue;
cl_program			btGpuDemo2dOCLWrap::m_cpProgram;
btKernelInfo		btGpuDemo2dOCLWrap::m_kernels[GPUDEMO2D_KERNEL_TOTAL];


cl_mem	btGpuDemo2dOCLWrap::m_dPos;
cl_mem	btGpuDemo2dOCLWrap::m_dRot;
cl_mem	btGpuDemo2dOCLWrap::m_dVel;
cl_mem	btGpuDemo2dOCLWrap::m_dAngVel;
cl_mem	btGpuDemo2dOCLWrap::m_dpPos;
cl_mem	btGpuDemo2dOCLWrap::m_dpRot;
cl_mem	btGpuDemo2dOCLWrap::m_dpVel;
cl_mem	btGpuDemo2dOCLWrap::m_dpAngVel;
cl_mem	btGpuDemo2dOCLWrap::m_dIds;
cl_mem	btGpuDemo2dOCLWrap::m_dBatchIds;
cl_mem	btGpuDemo2dOCLWrap::m_dLambdaDtBox;
cl_mem	btGpuDemo2dOCLWrap::m_dContact; // 8 floats : pos.x, pos.y, pos.z, penetration, norm.x, norm.y, norm.z, reserved
cl_mem	btGpuDemo2dOCLWrap::m_dInvMass;
cl_mem	btGpuDemo2dOCLWrap::m_dcPos;
cl_mem	btGpuDemo2dOCLWrap::m_dcRot;
cl_mem	btGpuDemo2dOCLWrap::m_dcVel;
cl_mem	btGpuDemo2dOCLWrap::m_dcAngVel;
cl_mem	btGpuDemo2dOCLWrap::m_dShapeBuffer;
cl_mem	btGpuDemo2dOCLWrap::m_dShapeIds;
cl_mem	btGpuDemo2dOCLWrap::m_dParams;

int	btGpuDemo2dOCLWrap::m_maxObjs;
int	btGpuDemo2dOCLWrap::m_maxNeighbors;
int	btGpuDemo2dOCLWrap::m_maxConstr;
int	btGpuDemo2dOCLWrap::m_maxVtxPerObj;
int	btGpuDemo2dOCLWrap::m_maxBatches;
int	btGpuDemo2dOCLWrap::m_maxShapeBufferSize;


cl_mem	btGpuDemo2dOCLWrap::m_dBodiesHash;
cl_mem	btGpuDemo2dOCLWrap::m_dCellStart;
cl_mem	btGpuDemo2dOCLWrap::m_dPairBuff; 
cl_mem	btGpuDemo2dOCLWrap::m_dPairBuffStartCurr = NULL; // needed for resetPool()
cl_mem	btGpuDemo2dOCLWrap::m_dAABB;
cl_mem	btGpuDemo2dOCLWrap::m_dPairScan;
cl_mem	btGpuDemo2dOCLWrap::m_dPairOut;
cl_mem	btGpuDemo2dOCLWrap::m_dBpParams;

int btGpuDemo2dOCLWrap::m_maxHandles;
int btGpuDemo2dOCLWrap::m_maxLargeHandles;
int btGpuDemo2dOCLWrap::m_maxPairsPerBody;
int btGpuDemo2dOCLWrap::m_numCells;
int btGpuDemo2dOCLWrap::m_hashSize;
bool btGpuDemo2dOCLWrap::m_broadphaseInited = false;

void btGpuDemo2dOCLWrap::initCL(int argc, char** argv)
{
    cl_int ciErrNum;

#ifndef CL_PLATFORM_MINI_CL
    m_cxMainContext = clCreateContextFromType(0, CL_DEVICE_TYPE_ALL, NULL, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
 
	m_cdDevice = btOclGetMaxFlopsDev(m_cxMainContext);

	// create a command-queue
	m_cqCommandQue = clCreateCommandQueue(m_cxMainContext, m_cdDevice, 0, &ciErrNum);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	// Program Setup
	size_t program_length;
	char* fileName = "Gpu2dDemoOCL.cl";
	FILE * fp = fopen(fileName, "rb");
	char newFileName[512];
	
	
	if (fp == NULL)
	{
		sprintf(newFileName,"..//%s",fileName);
		fp = fopen(newFileName, "rb");
		if (fp)
			fileName = newFileName;
	}
	
	if (fp == NULL)
	{
		sprintf(newFileName,"Demos//Gpu2dDemo//%s",fileName);
		fp = fopen(newFileName, "rb");
		if (fp)
			fileName = newFileName;
	}

	if (fp == NULL)
	{
		sprintf(newFileName,"..//..//Demos//Gpu2dDemo//%s",fileName);
		fp = fopen(newFileName, "rb");
		if (fp)
			fileName = newFileName;
		else
		{
			printf("cannot find %s\n",newFileName);
			exit(0);
		}
	}

	char *source = btOclLoadProgSource(fileName, "", &program_length);
	if(source == NULL)
	{
		printf("ERROR : OpenCL can't load file %s\n", fileName);
	}
	btAssert(source != NULL);

	// create the program
	printf("OpenCL compiles %s ...", fileName);
	m_cpProgram = clCreateProgramWithSource(m_cxMainContext, 1, (const char**)&source, &program_length, &ciErrNum);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	free(source);

	// build the program
	ciErrNum = clBuildProgram(m_cpProgram, 0, NULL, "-I .", NULL, NULL);
	if(ciErrNum != CL_SUCCESS)
	{
		// write out standard error
		char cBuildLog[10240];
		clGetProgramBuildInfo(m_cpProgram, btOclGetFirstDev(m_cxMainContext), CL_PROGRAM_BUILD_LOG, 
							  sizeof(cBuildLog), cBuildLog, NULL );
		printf("\n\n%s\n\n\n", cBuildLog);
		printf("Press ENTER key to terminate the program\n");
		getchar();
		exit(-1); 
	}
	printf("OK\n");
#elif defined(CL_PLATFORM_MINI_CL)
	#if 0
	///create kernels from binary
		int numDevices = 1;
		::size_t* lengths = (::size_t*) malloc(numDevices * sizeof(::size_t));
		const unsigned char** images = (const unsigned char**) malloc(numDevices * sizeof(const void*));
		for(int i = 0; i < numDevices; ++i) {
			images[i] = 0;
			lengths[i] = 0;
		}
		cl_device_id cdDevices[2];
		cdDevices[0] = m_cdDevice;
		m_cpProgram = clCreateProgramWithBinary(m_cxMainContext, 1, cdDevices,lengths, images, 0, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	#endif
#endif

}



void btGpuDemo2dOCLWrap::initKernel(int kernelId, char* pName)
{
	
	cl_int ciErrNum;
	cl_kernel kernel = clCreateKernel(m_cpProgram, pName, &ciErrNum);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	size_t wgSize;
	ciErrNum = clGetKernelWorkGroupInfo(kernel, m_cdDevice, CL_KERNEL_WORK_GROUP_SIZE, sizeof(size_t), &wgSize, NULL);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	m_kernels[kernelId].m_Id = kernelId;
	m_kernels[kernelId].m_kernel = kernel;
	m_kernels[kernelId].m_name = pName;
	m_kernels[kernelId].m_workgroupSize = (int)wgSize;
	return;
}

void btGpuDemo2dOCLWrap::runKernelWithWorkgroupSize(int kernelId, int globalSize)
{
	if(globalSize <= 0)
	{
		return;
	}
	cl_kernel kernelFunc = m_kernels[kernelId].m_kernel;
	cl_int ciErrNum = clSetKernelArg(kernelFunc, 0, sizeof(int), (void*)&globalSize);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	int workgroupSize = m_kernels[kernelId].m_workgroupSize;
	if(workgroupSize <= 0)
	{ // let OpenCL library calculate workgroup size
		size_t globalWorkSize[2];
		globalWorkSize[0] = globalSize;
		globalWorkSize[1] = 1;
		ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, kernelFunc, 1, NULL, globalWorkSize, NULL, 0,0,0 );
	}
	else
	{
//		#if defined(CL_PLATFORM_MINI_CL)
//			workgroupSize = 4;
//		#endif
		size_t localWorkSize[2], globalWorkSize[2];
		workgroupSize = btMin(workgroupSize, globalSize);
		int num_t = globalSize / workgroupSize;
		int num_g = num_t * workgroupSize;
		if(num_g < globalSize)
		{
			num_t++;
		}
		localWorkSize[0]  = workgroupSize;
		globalWorkSize[0] = num_t * workgroupSize;
		localWorkSize[1] = 1;
		globalWorkSize[1] = 1;
		ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, kernelFunc, 1, NULL, globalWorkSize, localWorkSize, 0,0,0 );
	}
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
}


void btGpuDemo2dOCLWrap::allocateArray(cl_mem* ppBuf, int memSize)
{
    cl_int ciErrNum;
    *ppBuf = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
}



void btGpuDemo2dOCLWrap::allocateBuffers(int maxObjs, int maxNeighbors, int maxVtxPerObj, int maxBatches, int maxShapeBufferSize)
{
    cl_int ciErrNum;

	m_maxObjs = maxObjs;
	m_maxNeighbors = maxNeighbors;
	m_maxVtxPerObj = maxVtxPerObj;
	m_maxBatches = maxBatches;
	m_maxConstr = m_maxObjs * m_maxNeighbors;
	m_maxShapeBufferSize = maxShapeBufferSize;

	int sz = m_maxObjs * m_maxNeighbors;

	allocateArray(&m_dPos, sizeof(float) * 4 * m_maxObjs);
	allocateArray(&m_dRot, sizeof(float) * m_maxObjs);
	allocateArray(&m_dVel, sizeof(float) * 4 * m_maxObjs);
	allocateArray(&m_dAngVel, sizeof(float) * m_maxObjs);
	allocateArray(&m_dpPos, sizeof(float) * 4 * m_maxObjs);
	allocateArray(&m_dpRot, sizeof(float) * m_maxObjs);
	allocateArray(&m_dpVel, sizeof(float) * 4 * m_maxObjs);
	allocateArray(&m_dpAngVel, sizeof(float) * m_maxObjs);
	allocateArray(&m_dInvMass, sizeof(float) * m_maxObjs);
	allocateArray(&m_dIds, sizeof(int) * 2 * m_maxConstr);
	allocateArray(&m_dBatchIds, sizeof(int) * m_maxConstr);
	allocateArray(&m_dLambdaDtBox, sizeof(float) * m_maxConstr * m_maxVtxPerObj);
	allocateArray(&m_dContact, sizeof(float) * m_maxConstr * m_maxVtxPerObj * 8);
	allocateArray(&m_dShapeBuffer, m_maxShapeBufferSize);
	allocateArray(&m_dShapeIds, sizeof(int) * 2 * m_maxObjs);
	allocateArray(&m_dParams, sizeof(float) * 4 * 2);

	// broadphase
	if(m_broadphaseInited)
	{
		int memSize;
		memSize = m_hashSize * 2 * sizeof(unsigned int);
		m_dBodiesHash = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);

		memSize = m_numCells * sizeof(unsigned int);
		m_dCellStart = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);

		memSize = m_maxHandles * m_maxPairsPerBody * sizeof(unsigned int);
		m_dPairBuff = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);

		memSize = (m_maxHandles * 2 + 1) * sizeof(unsigned int);
		m_dPairBuffStartCurr = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	//!!!!!!	btCuda_copyArrayToDevice(m_dPairBuffStartCurr, m_hPairBuffStartCurr, (m_maxHandles * 2 + 1) * sizeof(unsigned int)); 

		unsigned int numAABB = m_maxHandles + m_maxLargeHandles;
		memSize = numAABB * sizeof(float) * 4 * 2;
		m_dAABB = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);

		memSize = (m_maxHandles + 1) * sizeof(unsigned int);
		m_dPairScan = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);

		memSize = m_maxHandles * m_maxPairsPerBody * sizeof(unsigned int);
		m_dPairOut = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);

		memSize = 3 * 4 * sizeof(float);
		m_dBpParams = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	}
}

void btGpuDemo2dOCLWrap::initKernels()
{
	initKernel(GPUDEMO2D_KERNEL_CLEAR_ACCUM_IMPULSE, "kClearAccumImpulse");
	setKernelArg(GPUDEMO2D_KERNEL_CLEAR_ACCUM_IMPULSE, 1, sizeof(cl_mem),	(void*)&m_dLambdaDtBox);
	setKernelArg(GPUDEMO2D_KERNEL_CLEAR_ACCUM_IMPULSE, 2, sizeof(int),		(void*)&m_maxVtxPerObj);

	initKernel(GPUDEMO2D_KERNEL_COMPUTE_CONSTRAINTS, "kComputeConstraints");
	setKernelArg(GPUDEMO2D_KERNEL_COMPUTE_CONSTRAINTS, 1, sizeof(cl_mem),	(void*)&m_dIds);
	setKernelArg(GPUDEMO2D_KERNEL_COMPUTE_CONSTRAINTS, 2, sizeof(cl_mem),	(void*)&m_dPos);
	setKernelArg(GPUDEMO2D_KERNEL_COMPUTE_CONSTRAINTS, 3, sizeof(cl_mem),	(void*)&m_dRot);
	setKernelArg(GPUDEMO2D_KERNEL_COMPUTE_CONSTRAINTS, 4, sizeof(cl_mem),	(void*)&m_dShapeBuffer);
	setKernelArg(GPUDEMO2D_KERNEL_COMPUTE_CONSTRAINTS, 5, sizeof(cl_mem),	(void*)&m_dShapeIds);
	setKernelArg(GPUDEMO2D_KERNEL_COMPUTE_CONSTRAINTS, 6, sizeof(cl_mem),	(void*)&m_dContact);


	initKernel(GPUDEMO2D_KERNEL_COLLISION_WITH_WALL, "kCollisionWithWallBox");
	setKernelArg(GPUDEMO2D_KERNEL_COLLISION_WITH_WALL, 1, sizeof(cl_mem),	(void*)&m_dPos);
	setKernelArg(GPUDEMO2D_KERNEL_COLLISION_WITH_WALL, 2, sizeof(cl_mem),	(void*)&m_dVel);
	setKernelArg(GPUDEMO2D_KERNEL_COLLISION_WITH_WALL, 3, sizeof(cl_mem),	(void*)&m_dRot);
	setKernelArg(GPUDEMO2D_KERNEL_COLLISION_WITH_WALL, 4, sizeof(cl_mem),	(void*)&m_dAngVel);
	setKernelArg(GPUDEMO2D_KERNEL_COLLISION_WITH_WALL, 5, sizeof(cl_mem),	(void*)&m_dShapeBuffer);
	setKernelArg(GPUDEMO2D_KERNEL_COLLISION_WITH_WALL, 6, sizeof(cl_mem),	(void*)&m_dShapeIds);
	setKernelArg(GPUDEMO2D_KERNEL_COLLISION_WITH_WALL, 7, sizeof(cl_mem),	(void*)&m_dInvMass);
	setKernelArg(GPUDEMO2D_KERNEL_COLLISION_WITH_WALL, 8, sizeof(cl_mem),	(void*)&m_dParams);


	initKernel(GPUDEMO2D_KERNEL_SOLVE_CONSTRAINTS, "kSolveConstraints");
	setKernelArg(GPUDEMO2D_KERNEL_SOLVE_CONSTRAINTS, 1, sizeof(cl_mem),	(void*)&m_dIds);
	setKernelArg(GPUDEMO2D_KERNEL_SOLVE_CONSTRAINTS, 2, sizeof(cl_mem),	(void*)&m_dBatchIds);
	setKernelArg(GPUDEMO2D_KERNEL_SOLVE_CONSTRAINTS, 3, sizeof(cl_mem),	(void*)&m_dPos);
	setKernelArg(GPUDEMO2D_KERNEL_SOLVE_CONSTRAINTS, 4, sizeof(cl_mem),	(void*)&m_dVel);
	setKernelArg(GPUDEMO2D_KERNEL_SOLVE_CONSTRAINTS, 5, sizeof(cl_mem),	(void*)&m_dRot);
	setKernelArg(GPUDEMO2D_KERNEL_SOLVE_CONSTRAINTS, 6, sizeof(cl_mem),	(void*)&m_dAngVel);
	setKernelArg(GPUDEMO2D_KERNEL_SOLVE_CONSTRAINTS, 7, sizeof(cl_mem),	(void*)&m_dLambdaDtBox);
	setKernelArg(GPUDEMO2D_KERNEL_SOLVE_CONSTRAINTS, 8, sizeof(cl_mem),	(void*)&m_dContact);
	setKernelArg(GPUDEMO2D_KERNEL_SOLVE_CONSTRAINTS, 9, sizeof(cl_mem),	(void*)&m_dInvMass);

	if(m_broadphaseInited)
	{
		initKernel(GPUDEMO2D_KERNEL_CALC_HASH_AABB,	"kCalcHashAABB");
		setKernelArg(GPUDEMO2D_KERNEL_CALC_HASH_AABB, 1, sizeof(cl_mem),(void*)&m_dAABB);
		setKernelArg(GPUDEMO2D_KERNEL_CALC_HASH_AABB, 2, sizeof(cl_mem),(void*)&m_dBodiesHash);
		setKernelArg(GPUDEMO2D_KERNEL_CALC_HASH_AABB, 3, sizeof(cl_mem),(void*)&m_dBpParams);

		initKernel(GPUDEMO2D_KERNEL_CLEAR_CELL_START, "kClearCellStart");
		setKernelArg(GPUDEMO2D_KERNEL_CLEAR_CELL_START, 1, sizeof(cl_mem),(void*)&m_dCellStart);

		initKernel(GPUDEMO2D_KERNEL_FIND_CELL_START, "kFindCellStart");
		setKernelArg(GPUDEMO2D_KERNEL_FIND_CELL_START, 1, sizeof(cl_mem),(void*)&m_dBodiesHash);
		setKernelArg(GPUDEMO2D_KERNEL_FIND_CELL_START, 2, sizeof(cl_mem),(void*)&m_dCellStart);

		initKernel(GPUDEMO2D_KERNEL_FIND_OVERLAPPING_PAIRS, "kFindOverlappingPairs");
		setKernelArg(GPUDEMO2D_KERNEL_FIND_OVERLAPPING_PAIRS, 1, sizeof(cl_mem),(void*)&m_dAABB);
		setKernelArg(GPUDEMO2D_KERNEL_FIND_OVERLAPPING_PAIRS, 2, sizeof(cl_mem),(void*)&m_dBodiesHash);
		setKernelArg(GPUDEMO2D_KERNEL_FIND_OVERLAPPING_PAIRS, 3, sizeof(cl_mem),(void*)&m_dCellStart);
		setKernelArg(GPUDEMO2D_KERNEL_FIND_OVERLAPPING_PAIRS, 4, sizeof(cl_mem),(void*)&m_dPairBuff);
		setKernelArg(GPUDEMO2D_KERNEL_FIND_OVERLAPPING_PAIRS, 5, sizeof(cl_mem),(void*)&m_dPairBuffStartCurr);
		setKernelArg(GPUDEMO2D_KERNEL_FIND_OVERLAPPING_PAIRS, 6, sizeof(cl_mem),(void*)&m_dBpParams);

		initKernel(GPUDEMO2D_KERNEL_FIND_PAIRS_LARGE, "kFindPairsLarge");
		setKernelArg(GPUDEMO2D_KERNEL_FIND_PAIRS_LARGE, 1, sizeof(cl_mem),(void*)&m_dAABB);
		setKernelArg(GPUDEMO2D_KERNEL_FIND_PAIRS_LARGE, 2, sizeof(cl_mem),(void*)&m_dBodiesHash);
		setKernelArg(GPUDEMO2D_KERNEL_FIND_PAIRS_LARGE, 3, sizeof(cl_mem),(void*)&m_dCellStart);
		setKernelArg(GPUDEMO2D_KERNEL_FIND_PAIRS_LARGE, 4, sizeof(cl_mem),(void*)&m_dPairBuff);
		setKernelArg(GPUDEMO2D_KERNEL_FIND_PAIRS_LARGE, 5, sizeof(cl_mem),(void*)&m_dPairBuffStartCurr);

		initKernel(GPUDEMO2D_KERNEL_COMPUTE_CACHE_CHANGES, "kComputePairCacheChanges");
		setKernelArg(GPUDEMO2D_KERNEL_COMPUTE_CACHE_CHANGES, 1, sizeof(cl_mem),(void*)&m_dPairBuff);
		setKernelArg(GPUDEMO2D_KERNEL_COMPUTE_CACHE_CHANGES, 2, sizeof(cl_mem),(void*)&m_dPairBuffStartCurr);
		setKernelArg(GPUDEMO2D_KERNEL_COMPUTE_CACHE_CHANGES, 3, sizeof(cl_mem),(void*)&m_dPairScan);
		setKernelArg(GPUDEMO2D_KERNEL_COMPUTE_CACHE_CHANGES, 4, sizeof(cl_mem),(void*)&m_dAABB);

		initKernel(GPUDEMO2D_KERNEL_SQUEEZE_PAIR_BUFF, "kSqueezeOverlappingPairBuff");
		setKernelArg(GPUDEMO2D_KERNEL_SQUEEZE_PAIR_BUFF, 1, sizeof(cl_mem),(void*)&m_dPairBuff);
		setKernelArg(GPUDEMO2D_KERNEL_SQUEEZE_PAIR_BUFF, 2, sizeof(cl_mem),(void*)&m_dPairBuffStartCurr);
		setKernelArg(GPUDEMO2D_KERNEL_SQUEEZE_PAIR_BUFF, 3, sizeof(cl_mem),(void*)&m_dPairScan);
		setKernelArg(GPUDEMO2D_KERNEL_SQUEEZE_PAIR_BUFF, 4, sizeof(cl_mem),(void*)&m_dPairOut);
		setKernelArg(GPUDEMO2D_KERNEL_SQUEEZE_PAIR_BUFF, 5, sizeof(cl_mem),(void*)&m_dAABB);


		initKernel(GPUDEMO2D_KERNEL_BITONIC_SORT_CELL_ID_LOCAL, "kBitonicSortCellIdLocal");
		initKernel(GPUDEMO2D_KERNEL_BITONIC_SORT_CELL_ID_LOCAL_1, "kBitonicSortCellIdLocal1");
		initKernel(GPUDEMO2D_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL, "kBitonicSortCellIdMergeGlobal");
		initKernel(GPUDEMO2D_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL, "kBitonicSortCellIdMergeLocal");
	}
}

void btGpuDemo2dOCLWrap::setKernelArg(int kernelId, int argNum, int argSize, void* argPtr)
{
    cl_int ciErrNum;
	ciErrNum  = clSetKernelArg(m_kernels[kernelId].m_kernel, argNum, argSize, argPtr);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
}



void btGpuDemo2dOCLWrap::copyArrayToDevice(cl_mem device, const void* host, unsigned int size, int devOffs, int hostOffs)
{
    cl_int ciErrNum;
	char* pHost = (char*)host + hostOffs;
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, device, CL_TRUE, devOffs, size, pHost, 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
}

void btGpuDemo2dOCLWrap::copyArrayFromDevice(void* host, const cl_mem device, unsigned int size, int hostOffs, int devOffs)
{
    cl_int ciErrNum;
	char* pHost = (char*)host + hostOffs;
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, device, CL_TRUE, devOffs, size, pHost, 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
}


static unsigned int getMaxPowOf2(unsigned int num)
{
	unsigned int maxPowOf2 = 1;
	for(int bit = 1; bit < 32; bit++)
	{
		if(maxPowOf2 >= num)
		{
			break;
		}
		maxPowOf2 <<= 1;
	}
	return maxPowOf2;
}


void btGpuDemo2dOCLWrap::setBroadphaseBuffers(int maxHandles, int maxLargeHandles, int maxPairsPerBody, int numCells)
{
	m_maxHandles = maxHandles;
	m_maxLargeHandles = maxLargeHandles;
	m_maxPairsPerBody = maxPairsPerBody;
	m_numCells = numCells;
	m_hashSize = getMaxPowOf2(m_maxHandles);
	m_broadphaseInited = true;
}


//Note: logically shared with BitonicSort OpenCL code!

void btGpuDemo2dOCLWrap::bitonicSortNv(cl_mem pKey, unsigned int batch, unsigned int arrayLength, unsigned int dir)
{
	unsigned int localSizeLimit = m_kernels[GPUDEMO2D_KERNEL_BITONIC_SORT_CELL_ID_LOCAL].m_workgroupSize * 2;
    if(arrayLength < 2)
        return;
    //Only power-of-two array lengths are supported so far
    dir = (dir != 0);
    cl_int ciErrNum;
    size_t localWorkSize, globalWorkSize;
    if(arrayLength <= localSizeLimit)
    {
        btAssert( (batch * arrayLength) % localSizeLimit == 0);
        //Launch bitonicSortLocal
		ciErrNum  = clSetKernelArg(m_kernels[GPUDEMO2D_KERNEL_BITONIC_SORT_CELL_ID_LOCAL].m_kernel, 0,   sizeof(cl_mem), (void *)&pKey);
        ciErrNum |= clSetKernelArg(m_kernels[GPUDEMO2D_KERNEL_BITONIC_SORT_CELL_ID_LOCAL].m_kernel, 1,  sizeof(cl_uint), (void *)&arrayLength);
        ciErrNum |= clSetKernelArg(m_kernels[GPUDEMO2D_KERNEL_BITONIC_SORT_CELL_ID_LOCAL].m_kernel, 2,  sizeof(cl_uint), (void *)&dir);
        oclCHECKERROR(ciErrNum, CL_SUCCESS);

        localWorkSize  = localSizeLimit / 2;
        globalWorkSize = batch * arrayLength / 2;
        ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, m_kernels[GPUDEMO2D_KERNEL_BITONIC_SORT_CELL_ID_LOCAL].m_kernel, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
        oclCHECKERROR(ciErrNum, CL_SUCCESS);
    }
    else
    {
        //Launch bitonicSortLocal1
        ciErrNum  = clSetKernelArg(m_kernels[GPUDEMO2D_KERNEL_BITONIC_SORT_CELL_ID_LOCAL_1].m_kernel, 0,  sizeof(cl_mem), (void *)&pKey);
        oclCHECKERROR(ciErrNum, CL_SUCCESS);

        localWorkSize  = localSizeLimit / 2;
        globalWorkSize = batch * arrayLength / 2;
        ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, m_kernels[GPUDEMO2D_KERNEL_BITONIC_SORT_CELL_ID_LOCAL_1].m_kernel, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
        oclCHECKERROR(ciErrNum, CL_SUCCESS);

        for(unsigned int size = 2 * localSizeLimit; size <= arrayLength; size <<= 1)
        {
            for(unsigned stride = size / 2; stride > 0; stride >>= 1)
            {
                if(stride >= localSizeLimit)
                {
                    //Launch bitonicMergeGlobal
                    ciErrNum  = clSetKernelArg(m_kernels[GPUDEMO2D_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL].m_kernel, 0,  sizeof(cl_mem), (void *)&pKey);
                    ciErrNum |= clSetKernelArg(m_kernels[GPUDEMO2D_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL].m_kernel, 1, sizeof(cl_uint), (void *)&arrayLength);
                    ciErrNum |= clSetKernelArg(m_kernels[GPUDEMO2D_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL].m_kernel, 2, sizeof(cl_uint), (void *)&size);
                    ciErrNum |= clSetKernelArg(m_kernels[GPUDEMO2D_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL].m_kernel, 3, sizeof(cl_uint), (void *)&stride);
                    ciErrNum |= clSetKernelArg(m_kernels[GPUDEMO2D_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL].m_kernel, 4, sizeof(cl_uint), (void *)&dir);
					oclCHECKERROR(ciErrNum, CL_SUCCESS);

                    localWorkSize  = localSizeLimit / 4;
                    globalWorkSize = batch * arrayLength / 2;

                    ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, m_kernels[GPUDEMO2D_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL].m_kernel, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
					oclCHECKERROR(ciErrNum, CL_SUCCESS);
                }
                else
                {
                    //Launch bitonicMergeLocal
					ciErrNum  = clSetKernelArg(m_kernels[GPUDEMO2D_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL].m_kernel, 0,  sizeof(cl_mem), (void *)&pKey);
                    ciErrNum |= clSetKernelArg(m_kernels[GPUDEMO2D_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL].m_kernel, 1, sizeof(cl_uint), (void *)&arrayLength);
                    ciErrNum |= clSetKernelArg(m_kernels[GPUDEMO2D_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL].m_kernel, 2, sizeof(cl_uint), (void *)&stride);
                    ciErrNum |= clSetKernelArg(m_kernels[GPUDEMO2D_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL].m_kernel, 3, sizeof(cl_uint), (void *)&size);
                    ciErrNum |= clSetKernelArg(m_kernels[GPUDEMO2D_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL].m_kernel, 4, sizeof(cl_uint), (void *)&dir);
					oclCHECKERROR(ciErrNum, CL_SUCCESS);

                    localWorkSize  = localSizeLimit / 2;
                    globalWorkSize = batch * arrayLength / 2;

                    ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, m_kernels[GPUDEMO2D_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL].m_kernel, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
					oclCHECKERROR(ciErrNum, CL_SUCCESS);
                    break;
                }
            }
        }
    }
}

