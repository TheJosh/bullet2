/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006 - 2010 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "LinearMath/btAlignedAllocator.h"
#include "LinearMath/btQuickprof.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"

#include "btOclCommon.h"
#include "btHier3dGridBroadphaseOCL.h"

#include <stdio.h>
#include <string.h>


#define BT_HIER3DGRID_PAIR_FOUND_FLG (0x40000000)
#define BT_HIER3DGRID_PAIR_NEW_FLG   (0x20000000)
#define BT_HIER3DGRID_PAIR_ANY_FLG   (BT_HIER3DGRID_PAIR_FOUND_FLG | BT_HIER3DGRID_PAIR_NEW_FLG)



#define MSTRINGIFY(A) #A

static const char* spProgramSource = 
#include "btHier3dGridBroadphaseOCL.cl"


btHier3dGridBroadphaseOCL::btHier3dGridBroadphaseOCL(	btOverlappingPairCache* overlappingPairCache,
														const btVector3& cellSize, 
														int gridSizeX, int gridSizeY, int gridSizeZ,
														int maxProxies, int maxPairsPerProxy,
														cl_context context, cl_device_id device, cl_command_queue queue) : 
	btSimpleBroadphase(maxProxies, overlappingPairCache)
{
	m_hParams.m_invCellSize[0] = btScalar(1.f) / cellSize[0];
	m_hParams.m_invCellSize[1] = btScalar(1.f) / cellSize[1];
	m_hParams.m_invCellSize[2] = btScalar(1.f) / cellSize[2];
	btScalar minSize = btMin(cellSize[0], btMin(cellSize[1], cellSize[2]));
	m_hParams.m_invCellSize[3] = btScalar(1.f) / minSize;
	m_hParams.m_gridSizeX = gridSizeX;
	m_hParams.m_gridSizeY = gridSizeY;
	m_hParams.m_gridSizeZ = gridSizeZ;
	m_hParams.m_gridDepth = btMax(gridSizeX, btMax(gridSizeY, gridSizeZ));
	m_maxPairsPerProxy = maxPairsPerProxy;
	initCL(context, device, queue);
	allocateBuffers();
	prefillBuffers();
	initKernels();
}



btHier3dGridBroadphaseOCL::~btHier3dGridBroadphaseOCL()
{
	//btSimpleBroadphase will free memory of btSortedOverlappingPairCache, because m_ownsPairCache
	assert(m_bInitialized);
}

#ifdef CL_PLATFORM_MINI_CL
// there is a problem with MSVC9 : static constructors are not called if variables defined in library and are not used
// looks like it is because of optimization
// probably this will happen with other compilers as well
// so to make it robust, register kernels again (it is safe)
#define MINICL_DECLARE(a) extern "C" void a();
MINICL_DECLARE(kCalcHashAABB)
MINICL_DECLARE(kClearCellStart)
MINICL_DECLARE(kFindCellStart)
MINICL_DECLARE(kFindOverlappingPairs)
MINICL_DECLARE(kComputePairCacheChanges)
MINICL_DECLARE(kSqueezeOverlappingPairBuff)
MINICL_DECLARE(kBitonicSortCellIdLocal)
MINICL_DECLARE(kBitonicSortCellIdLocal1)
MINICL_DECLARE(kBitonicSortCellIdMergeGlobal)
MINICL_DECLARE(kBitonicSortCellIdMergeLocal)
#undef MINICL_DECLARE
#endif

void btHier3dGridBroadphaseOCL::initCL(cl_context context, cl_device_id device, cl_command_queue queue)
{

	#ifdef CL_PLATFORM_MINI_CL
		// call constructors here
		MINICL_REGISTER(kCalcHashAABB)
		MINICL_REGISTER(kClearCellStart)
		MINICL_REGISTER(kFindCellStart)
		MINICL_REGISTER(kFindOverlappingPairs)
		MINICL_REGISTER(kComputePairCacheChanges)
		MINICL_REGISTER(kSqueezeOverlappingPairBuff)
		MINICL_REGISTER(kBitonicSortCellIdLocal)
		MINICL_REGISTER(kBitonicSortCellIdLocal1)
		MINICL_REGISTER(kBitonicSortCellIdMergeGlobal)
		MINICL_REGISTER(kBitonicSortCellIdMergeLocal)
	#endif

	cl_int ciErrNum;

	if(context != NULL)
	{
		m_cxMainContext = context;
	}
	else
	{
		// create a context 
		m_cxMainContext = btOclCommon::createContextFromType(CL_DEVICE_TYPE_ALL, &ciErrNum);
		HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
	}
	if(device != NULL)
	{
		m_cdDevice = device;
	}
	else
	{
		// get device
		size_t paramSize;
		cl_device_id* devices;
		ciErrNum = clGetContextInfo(m_cxMainContext, CL_CONTEXT_DEVICES, 0, NULL, &paramSize);
		HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
		devices = (cl_device_id*)malloc(paramSize);
		ciErrNum = clGetContextInfo(m_cxMainContext, CL_CONTEXT_DEVICES, paramSize, devices, NULL);
		HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
		m_cdDevice = devices[0]; // grab the first for now
		free(devices);
	}
	if(queue != NULL)
	{
		m_cqCommandQue = queue;
	}
	else
	{
		// create a command-queue
		m_cqCommandQue = clCreateCommandQueue(m_cxMainContext, m_cdDevice, 0, &ciErrNum);
		HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
	}
	// create the program
	size_t programLength = strlen(spProgramSource);
	printf("OpenCL compiles btHier3dGridBroadphaseOCL.cl ...");
	m_cpProgram = clCreateProgramWithSource(m_cxMainContext, 1, &spProgramSource, &programLength, &ciErrNum);
	// build the program
	ciErrNum = clBuildProgram(m_cpProgram, 0, NULL, "-DGUID_ARG=""""", NULL, NULL);
	if(ciErrNum != CL_SUCCESS)
	{
		// write out standard error
		char cBuildLog[102400];
		clGetProgramBuildInfo(m_cpProgram, m_cdDevice, CL_PROGRAM_BUILD_LOG, 
							  sizeof(cBuildLog), cBuildLog, NULL );
		printf("\n\n%s\n\n\n", cBuildLog);
		printf("Press ENTER key to terminate the program\n");
		getchar();
		exit(-1); 
	}
	printf("OK\n");
}


void btHier3dGridBroadphaseOCL::initKernels()
{
	initKernel(HIER3DGRIDOCL_KERNEL_CALC_HASH_AABB,	"kCalcHashAABB");
	setKernelArg(HIER3DGRIDOCL_KERNEL_CALC_HASH_AABB, 1, sizeof(cl_mem),(void*)&m_dAABB);
	setKernelArg(HIER3DGRIDOCL_KERNEL_CALC_HASH_AABB, 2, sizeof(cl_mem),(void*)&m_dBodiesHash);
	setKernelArg(HIER3DGRIDOCL_KERNEL_CALC_HASH_AABB, 3, sizeof(cl_mem),(void*)&m_dParams);

	initKernel(HIER3DGRIDOCL_KERNEL_CLEAR_CELL_START, "kClearCellStart");
	setKernelArg(HIER3DGRIDOCL_KERNEL_CLEAR_CELL_START, 1, sizeof(cl_mem),(void*)&m_dCellStart);

	initKernel(HIER3DGRIDOCL_KERNEL_FIND_CELL_START, "kFindCellStart");
	setKernelArg(HIER3DGRIDOCL_KERNEL_FIND_CELL_START, 1, sizeof(cl_mem),(void*)&m_dBodiesHash);
	setKernelArg(HIER3DGRIDOCL_KERNEL_FIND_CELL_START, 2, sizeof(cl_mem),(void*)&m_dCellStart);

	initKernel(HIER3DGRIDOCL_KERNEL_FIND_OVERLAPPING_PAIRS, "kFindOverlappingPairs");
	setKernelArg(HIER3DGRIDOCL_KERNEL_FIND_OVERLAPPING_PAIRS, 1, sizeof(cl_mem),(void*)&m_dAABB);
	setKernelArg(HIER3DGRIDOCL_KERNEL_FIND_OVERLAPPING_PAIRS, 2, sizeof(cl_mem),(void*)&m_dBodiesHash);
	setKernelArg(HIER3DGRIDOCL_KERNEL_FIND_OVERLAPPING_PAIRS, 3, sizeof(cl_mem),(void*)&m_dCellStart);
	setKernelArg(HIER3DGRIDOCL_KERNEL_FIND_OVERLAPPING_PAIRS, 4, sizeof(cl_mem),(void*)&m_dPairBuff);
	setKernelArg(HIER3DGRIDOCL_KERNEL_FIND_OVERLAPPING_PAIRS, 5, sizeof(cl_mem),(void*)&m_dPairBuffStartCurr);
	setKernelArg(HIER3DGRIDOCL_KERNEL_FIND_OVERLAPPING_PAIRS, 6, sizeof(cl_mem),(void*)&m_dParams);

	initKernel(HIER3DGRIDOCL_KERNEL_COMPUTE_CACHE_CHANGES, "kComputePairCacheChanges");
	setKernelArg(HIER3DGRIDOCL_KERNEL_COMPUTE_CACHE_CHANGES, 1, sizeof(cl_mem),(void*)&m_dPairBuff);
	setKernelArg(HIER3DGRIDOCL_KERNEL_COMPUTE_CACHE_CHANGES, 2, sizeof(cl_mem),(void*)&m_dPairBuffStartCurr);
	setKernelArg(HIER3DGRIDOCL_KERNEL_COMPUTE_CACHE_CHANGES, 3, sizeof(cl_mem),(void*)&m_dPairScan);
	setKernelArg(HIER3DGRIDOCL_KERNEL_COMPUTE_CACHE_CHANGES, 4, sizeof(cl_mem),(void*)&m_dAABB);

	initKernel(HIER3DGRIDOCL_KERNEL_SQUEEZE_PAIR_BUFF, "kSqueezeOverlappingPairBuff");
	setKernelArg(HIER3DGRIDOCL_KERNEL_SQUEEZE_PAIR_BUFF, 1, sizeof(cl_mem),(void*)&m_dPairBuff);
	setKernelArg(HIER3DGRIDOCL_KERNEL_SQUEEZE_PAIR_BUFF, 2, sizeof(cl_mem),(void*)&m_dPairBuffStartCurr);
	setKernelArg(HIER3DGRIDOCL_KERNEL_SQUEEZE_PAIR_BUFF, 3, sizeof(cl_mem),(void*)&m_dPairScan);
	setKernelArg(HIER3DGRIDOCL_KERNEL_SQUEEZE_PAIR_BUFF, 4, sizeof(cl_mem),(void*)&m_dPairOut);
	setKernelArg(HIER3DGRIDOCL_KERNEL_SQUEEZE_PAIR_BUFF, 5, sizeof(cl_mem),(void*)&m_dAABB);

	initKernel(HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_LOCAL, "kBitonicSortCellIdLocal");
	initKernel(HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_LOCAL_1, "kBitonicSortCellIdLocal1");
	initKernel(HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL, "kBitonicSortCellIdMergeGlobal");
	initKernel(HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL, "kBitonicSortCellIdMergeLocal");
}


void btHier3dGridBroadphaseOCL::allocateBuffers()
{
    cl_int ciErrNum;
    unsigned int memSize;
	memSize = sizeof(btHier3dGridOCLParams);
	m_dParams = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
	HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
	// current version of bitonic sort works for power of 2 arrays only, so ...
	m_hashSize = 1;
	for(int bit = 1; bit < 32; bit++)
	{
		if(m_hashSize >= m_maxHandles)
		{
			break;
		}
		m_hashSize <<= 1;
	}
	m_hBodiesHash.resize(2 * m_hashSize);
	memSize = m_hashSize * 2 * sizeof(unsigned int);
	m_dBodiesHash = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
	HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

	m_hAABB.resize(m_maxHandles * 2);
	memSize = m_maxHandles * sizeof(btVector3) * 2;
	m_dAABB = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
	HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
	
	m_numOfCells = m_hParams.m_gridSizeX * m_hParams.m_gridSizeY * m_hParams.m_gridSizeZ;
	m_hCellStart.resize(m_numOfCells);
	memSize = m_numOfCells * sizeof(int);
	m_dCellStart = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
	HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

	m_hPairBuff.resize(m_maxHandles * m_maxPairsPerProxy);
	memSize = m_maxHandles * m_maxPairsPerProxy * sizeof(unsigned int);
	m_dPairBuff = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
	HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

	m_hPairBuffStartCurr.resize(m_maxHandles * 2 + 1);
	memSize = (m_maxHandles * 2 + 1) * sizeof(unsigned int);
	m_dPairBuffStartCurr = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
	HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

	m_hPairScan.resize(m_maxHandles + 1);
	memSize = (m_maxHandles + 1) * sizeof(unsigned int);
	m_dPairScan = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
	HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

	m_hPairOut.resize(m_maxHandles * m_maxPairsPerProxy);
	memSize = m_maxHandles * m_maxPairsPerProxy * sizeof(unsigned int);
	m_dPairOut = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
	HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

}

void btHier3dGridBroadphaseOCL::prefillBuffers()
{
	int* pHash = &(m_hBodiesHash[0]);
	memset(pHash, 0x7F, m_maxHandles * 2 * sizeof(int));
	copyArrayToDevice(m_dBodiesHash, pHash, m_maxHandles * 2 * sizeof(int));
	// now fill the rest (bitonic sorting works with size == pow of 2)
	int remainder = m_hashSize - m_maxHandles;
	if(remainder)
	{
		copyArrayToDevice(m_dBodiesHash, pHash, remainder * 2 * sizeof(unsigned int), m_maxHandles * 2 * sizeof(unsigned int), 0);
	}
	memset(&(m_hPairBuff[0]), 0x00, m_maxHandles * m_maxPairsPerProxy * sizeof(unsigned int));
	copyArrayToDevice(m_dPairBuff, &(m_hPairBuff[0]), m_maxHandles * m_maxPairsPerProxy * sizeof(unsigned int));
	m_hPairBuffStartCurr[0] = 0;
	m_hPairBuffStartCurr[1] = 0;
	for(int i = 1; i <= m_maxHandles; i++) 
	{
		m_hPairBuffStartCurr[i * 2] = m_hPairBuffStartCurr[(i-1) * 2] + m_maxPairsPerProxy;
		m_hPairBuffStartCurr[i * 2 + 1] = 0;
	}
	copyArrayToDevice(m_dPairBuffStartCurr, &(m_hPairBuffStartCurr[0]), (m_maxHandles * 2 + 1) * sizeof(unsigned int)); 
}


void btHier3dGridBroadphaseOCL::initKernel(int kernelId, char* pName)
{
	
	cl_int ciErrNum;
	cl_kernel kernel = clCreateKernel(m_cpProgram, pName, &ciErrNum);
	HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
	size_t wgSize;
	ciErrNum = clGetKernelWorkGroupInfo(kernel, m_cdDevice, CL_KERNEL_WORK_GROUP_SIZE, sizeof(size_t), &wgSize, NULL);
	HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
	m_kernels[kernelId].m_Id = kernelId;
	m_kernels[kernelId].m_kernel = kernel;
	m_kernels[kernelId].m_name = pName;
	m_kernels[kernelId].m_workgroupSize = (int)wgSize;
	return;
}

void btHier3dGridBroadphaseOCL::runKernelWithWorkgroupSize(int kernelId, int globalSize)
{
	if(globalSize <= 0)
	{
		return;
	}
	cl_kernel kernelFunc = m_kernels[kernelId].m_kernel;
	cl_int ciErrNum = clSetKernelArg(kernelFunc, 0, sizeof(int), (void*)&globalSize);
	HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
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
	HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
	ciErrNum = clFinish(m_cqCommandQue);
	HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
}


void btHier3dGridBroadphaseOCL::setKernelArg(int kernelId, int argNum, int argSize, void* argPtr)
{
    cl_int ciErrNum;
	ciErrNum  = clSetKernelArg(m_kernels[kernelId].m_kernel, argNum, argSize, argPtr);
	HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
}


void btHier3dGridBroadphaseOCL::copyArrayToDevice(cl_mem device, const void* host, unsigned int size, int devOffs, int hostOffs)
{
    cl_int ciErrNum;
	char* pHost = (char*)host + hostOffs;
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, device, CL_TRUE, devOffs, size, pHost, 0, NULL, NULL);
	HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
}

void btHier3dGridBroadphaseOCL::copyArrayFromDevice(void* host, const cl_mem device, unsigned int size, int hostOffs, int devOffs)
{
    cl_int ciErrNum;
	char* pHost = (char*)host + hostOffs;
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, device, CL_TRUE, devOffs, size, pHost, 0, NULL, NULL);
	HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
}


void btHier3dGridBroadphaseOCL::calculateOverlappingPairs(btDispatcher* dispatcher)
{
	{
		BT_PROFILE("copyParams");
		copyArrayToDevice(m_dParams, &m_hParams, sizeof(btHier3dGridOCLParams)); 
	}
	// prepare AABB array
	prepareAABB();
	// calculate hash
	calcHashAABB();
	// sort hash
	sortHash();
	// find start of each hash basket
	findCellStart();
	// findOverlappingPairs
	findOverlappingPairs();
	// add pairs to CPU cache
	computePairCacheChanges();
	scanOverlappingPairBuff();
	squeezeOverlappingPairBuff();
	addPairsToCache(dispatcher);
	return;
}


void btHier3dGridBroadphaseOCL::prepareAABB()
{
	BT_PROFILE("prepareAABB");
	int i;
	btVector3* pBB = &(m_hAABB[0]);
	for(i = 0; i <= m_numHandles; i++) 
	{
		btSimpleBroadphaseProxy* proxy0 = &m_pHandles[i];
		if(!proxy0->m_clientObject)
		{
			continue;
		}
		*pBB = proxy0->m_aabbMin;
		*((int*)pBB + 3) = (int)proxy0->m_multiSapParentProxy;
		pBB++;
		*pBB = proxy0->m_aabbMax;
		*((int*)pBB + 3) = i;
		pBB++;
	}
	copyArrayToDevice(m_dAABB, &(m_hAABB[0]), sizeof(btVector3) * 2 * m_numHandles); 
	return;
}

void btHier3dGridBroadphaseOCL::calcHashAABB()
{
	BT_PROFILE("calcHashAABB");
	runKernelWithWorkgroupSize(HIER3DGRIDOCL_KERNEL_CALC_HASH_AABB, m_numHandles);
	return;
}


void btHier3dGridBroadphaseOCL::sortHash()
{
	BT_PROFILE("sortHash");
#ifdef CL_PLATFORM_MINI_CL
	class bt3DGridHashKey
	{
	public:
	   unsigned int hash;
	   unsigned int index;
	   void quickSort(bt3DGridHashKey* pData, int lo, int hi)
	   {
			int i=lo, j=hi;
			bt3DGridHashKey x = pData[(lo+hi)/2];
			do
			{    
				while(pData[i].hash < x.hash) i++; 
				while(x.hash < pData[j].hash) j--;
				if(i <= j)
				{
					bt3DGridHashKey t = pData[i];
					pData[i] = pData[j];
					pData[j] = t;
					i++; j--;
				}
			} while(i <= j);
			if(lo < j) pData->quickSort(pData, lo, j);
			if(i < hi) pData->quickSort(pData, i, hi);
	   }
	};
	copyArrayFromDevice(&(m_hBodiesHash[0]), m_dBodiesHash, m_numHandles * 2 * sizeof(unsigned int));
	bt3DGridHashKey* pHash = (bt3DGridHashKey*)&(m_hBodiesHash[0]);
	pHash->quickSort(pHash, 0, m_numHandles - 1);
	copyArrayToDevice(m_dBodiesHash, &(m_hBodiesHash[0]), m_numHandles * 2 * sizeof(unsigned int));
#else
	int dir = 1;
	bitonicSort(m_dBodiesHash, m_hashSize, dir);
#endif
	return;
}


void btHier3dGridBroadphaseOCL::findCellStart()
{
	BT_PROFILE("findCellStart");
#ifdef CL_PLATFORM_MINI_CL
		copyArrayFromDevice(&(m_hBodiesHash[0]), m_dBodiesHash, m_numHandles * 2 * sizeof(unsigned int));
		for(int i = 0; i < m_numOfCells; i++)
		{
			m_hCellStart[i] = -1;
		}
		// find start of each cell in sorted hash
		int hashPrev = m_hBodiesHash[0];
		m_hCellStart[hashPrev] = 0;
		for(int i = 1; i < m_numHandles; i++)
		{
			int hashCurr = m_hBodiesHash[i*2];
			if(hashPrev != hashCurr)
			{
				m_hCellStart[hashCurr] = i;
			}
			hashPrev = hashCurr;
		}
		copyArrayToDevice(m_dCellStart, &(m_hCellStart[0]), m_numOfCells * sizeof(int));
#else
	runKernelWithWorkgroupSize(HIER3DGRIDOCL_KERNEL_CLEAR_CELL_START, m_numOfCells);
	runKernelWithWorkgroupSize(HIER3DGRIDOCL_KERNEL_FIND_CELL_START, m_numHandles);
#endif
	return;
}

void btHier3dGridBroadphaseOCL::findOverlappingPairs()
{
	BT_PROFILE("findOverlappingPairs");
	runKernelWithWorkgroupSize(HIER3DGRIDOCL_KERNEL_FIND_OVERLAPPING_PAIRS, m_numHandles);
	return;
}

void btHier3dGridBroadphaseOCL::computePairCacheChanges()
{
	BT_PROFILE("computePairCacheChanges");
	runKernelWithWorkgroupSize(HIER3DGRIDOCL_KERNEL_COMPUTE_CACHE_CHANGES, m_numHandles);
	return;
}


void btHier3dGridBroadphaseOCL::scanOverlappingPairBuff()
{
	copyArrayFromDevice(&(m_hPairScan[0]), m_dPairScan, sizeof(unsigned int)*(m_numHandles + 1)); 
	m_hPairScan[0] = 0;
	for(int i = 1; i <= m_numHandles; i++) 
	{
		unsigned int delta = m_hPairScan[i];
		m_hPairScan[i] = m_hPairScan[i-1] + delta;
	}
	copyArrayToDevice(m_dPairScan, &(m_hPairScan[0]), sizeof(unsigned int)*(m_numHandles + 1)); 
	return;
}


void btHier3dGridBroadphaseOCL::squeezeOverlappingPairBuff()
{
	BT_PROFILE("btCuda_squeezeOverlappingPairBuff");
	runKernelWithWorkgroupSize(HIER3DGRIDOCL_KERNEL_SQUEEZE_PAIR_BUFF, m_numHandles);
	copyArrayFromDevice(&(m_hPairOut[0]), m_dPairOut, sizeof(unsigned int) * m_hPairScan[m_numHandles]); 
	return;
}


void btHier3dGridBroadphaseOCL::addPairsToCache(btDispatcher* dispatcher)
{
	int numPairsAdded = 0;
	int numPairsRemoved = 0;
	for(int i = 0; i < m_numHandles; i++) 
	{
		unsigned int num = m_hPairScan[i+1] - m_hPairScan[i];
		if(!num)
		{
			continue;
		}
		int* pInp = &(m_hPairOut[m_hPairScan[i]]);
		unsigned int index0 = *((int*)&(m_hAABB[i * 2 + 1][3]));
		btSimpleBroadphaseProxy* proxy0 = &m_pHandles[index0];
		for(unsigned int j = 0; j < num; j++)
		{
			unsigned int indx1_s = pInp[j];
			unsigned int index1 = indx1_s & (~BT_HIER3DGRID_PAIR_ANY_FLG);
			btSimpleBroadphaseProxy* proxy1;
			proxy1 = &m_pHandles[index1];
			if(indx1_s & BT_HIER3DGRID_PAIR_NEW_FLG)
			{
				m_pairCache->addOverlappingPair(proxy0,proxy1);
				numPairsAdded++;
			}
			else
			{
				m_pairCache->removeOverlappingPair(proxy0,proxy1,dispatcher);
				numPairsRemoved++;
			}
		}
	}
}


void btHier3dGridBroadphaseOCL::resetPool(btDispatcher* dispatcher)
{
	btSimpleBroadphase::resetPool(dispatcher);
	prefillBuffers();
}

btBroadphaseProxy*	btHier3dGridBroadphaseOCL::createProxy(  const btVector3& aabbMin,  const btVector3& aabbMax,int shapeType,void* userPtr ,short int collisionFilterGroup,short int collisionFilterMask, btDispatcher* dispatcher,void* multiSapProxy)
{
	btBroadphaseProxy* pProxy = btSimpleBroadphase::createProxy(aabbMin, aabbMax, shapeType, userPtr, collisionFilterGroup, collisionFilterMask, dispatcher, multiSapProxy);
	btVector3 bbDiag = pProxy->m_aabbMax - pProxy->m_aabbMin;
	btScalar maxSz = bbDiag.length();
	btScalar fTimes = maxSz * m_hParams.m_invCellSize[3];
	int iTimes = (int)fTimes;
	int mask = 1;
	for(int i = 0; i < 31; i++)
	{
		if(iTimes < mask)
		{
			break;
		}
		mask <<= 1;
	}
	unsigned int proxyMask = (-mask) & (m_hParams.m_gridDepth - 1);
	pProxy->m_multiSapParentProxy = (void*)proxyMask;
	return pProxy;
}


//Note: logically shared with BitonicSort OpenCL code!

void btHier3dGridBroadphaseOCL::bitonicSort(cl_mem pKey, unsigned int arrayLength, unsigned int dir)
{
	unsigned int localSizeLimit = m_kernels[HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_LOCAL].m_workgroupSize * 2;
    if(arrayLength < 2)
        return;
    //Only power-of-two array lengths are supported so far
    dir = (dir != 0);
    cl_int ciErrNum;
    size_t localWorkSize, globalWorkSize;
    if(arrayLength <= localSizeLimit)
    {
        btAssert( (arrayLength % localSizeLimit) == 0);
        //Launch bitonicSortLocal
		ciErrNum  = clSetKernelArg(m_kernels[HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_LOCAL].m_kernel, 0,   sizeof(cl_mem), (void *)&pKey);
        ciErrNum |= clSetKernelArg(m_kernels[HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_LOCAL].m_kernel, 1,  sizeof(cl_uint), (void *)&arrayLength);
        ciErrNum |= clSetKernelArg(m_kernels[HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_LOCAL].m_kernel, 2,  sizeof(cl_uint), (void *)&dir);
		HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

        localWorkSize  = localSizeLimit / 2;
        globalWorkSize = arrayLength / 2;
        ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, m_kernels[HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_LOCAL].m_kernel, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
		HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
    }
    else
    {
        //Launch bitonicSortLocal1
        ciErrNum  = clSetKernelArg(m_kernels[HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_LOCAL_1].m_kernel, 0,  sizeof(cl_mem), (void *)&pKey);
		HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

        localWorkSize  = localSizeLimit / 2;
        globalWorkSize = arrayLength / 2;
        ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, m_kernels[HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_LOCAL_1].m_kernel, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
		HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

        for(unsigned int size = 2 * localSizeLimit; size <= arrayLength; size <<= 1)
        {
            for(unsigned stride = size / 2; stride > 0; stride >>= 1)
            {
                if(stride >= localSizeLimit)
                {
                    //Launch bitonicMergeGlobal
                    ciErrNum  = clSetKernelArg(m_kernels[HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL].m_kernel, 0,  sizeof(cl_mem), (void *)&pKey);
                    ciErrNum |= clSetKernelArg(m_kernels[HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL].m_kernel, 1, sizeof(cl_uint), (void *)&arrayLength);
                    ciErrNum |= clSetKernelArg(m_kernels[HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL].m_kernel, 2, sizeof(cl_uint), (void *)&size);
                    ciErrNum |= clSetKernelArg(m_kernels[HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL].m_kernel, 3, sizeof(cl_uint), (void *)&stride);
                    ciErrNum |= clSetKernelArg(m_kernels[HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL].m_kernel, 4, sizeof(cl_uint), (void *)&dir);
					HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

                    localWorkSize  = localSizeLimit / 4;
                    globalWorkSize = arrayLength / 2;

                    ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, m_kernels[HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL].m_kernel, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
					HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
                }
                else
                {
                    //Launch bitonicMergeLocal
					ciErrNum  = clSetKernelArg(m_kernels[HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL].m_kernel, 0,  sizeof(cl_mem), (void *)&pKey);
                    ciErrNum |= clSetKernelArg(m_kernels[HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL].m_kernel, 1, sizeof(cl_uint), (void *)&arrayLength);
                    ciErrNum |= clSetKernelArg(m_kernels[HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL].m_kernel, 2, sizeof(cl_uint), (void *)&stride);
                    ciErrNum |= clSetKernelArg(m_kernels[HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL].m_kernel, 3, sizeof(cl_uint), (void *)&size);
                    ciErrNum |= clSetKernelArg(m_kernels[HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL].m_kernel, 4, sizeof(cl_uint), (void *)&dir);
					HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

                    localWorkSize  = localSizeLimit / 2;
                    globalWorkSize = arrayLength / 2;

                    ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, m_kernels[HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL].m_kernel, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
					HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
                    break;
                }
            }
        }
    }
	ciErrNum = clFinish(m_cqCommandQue);
	HIER3DGRIDOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
}

