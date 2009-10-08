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



#include "LinearMath/btAlignedAllocator.h"
#include "LinearMath/btQuickprof.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"

#include "btGpu2dGridBroadphaseOCL.h"
//#include "radixsort.cuh"

#include "btOclUtils.h"
#include "btGpuDemo2dOCLWrap.h"



//#define BT_GPU_PREF(func) btCuda_##func
//#include "../../src/BulletMultiThreaded/btGpuUtilsSharedDefs.h"
//#include "../../src/BulletMultiThreaded/btGpu3DGridBroadphaseSharedDefs.h"
//#undef BT_GPU_PREF

//extern "C" void btCuda_setParameters(bt3DGridBroadphaseParams* hostParams);



#include <stdio.h>




btGpu2dGridBroadphaseOCL::btGpu2dGridBroadphaseOCL(	btOverlappingPairCache* overlappingPairCache,
									const btVector3& worldAabbMin,const btVector3& worldAabbMax, 
									int gridSizeX, int gridSizeY, int gridSizeZ, 
									int maxSmallProxies, int maxLargeProxies, int maxPairsPerSmallProxy,
									int maxSmallProxiesPerCell,
									btScalar cellFactorAABB) :
	btGpu3DGridBroadphase(overlappingPairCache, worldAabbMin, worldAabbMax, gridSizeX, gridSizeY, gridSizeZ, maxSmallProxies, maxLargeProxies, maxPairsPerSmallProxy, maxSmallProxiesPerCell, cellFactorAABB)
{
	_initialize();
}



btGpu2dGridBroadphaseOCL::~btGpu2dGridBroadphaseOCL()
{
	//btSimpleBroadphase will free memory of btSortedOverlappingPairCache, because m_ownsPairCache
	assert(m_bInitialized);
	_finalize();
}



void btGpu2dGridBroadphaseOCL::_initialize()
{
    // allocate GPU data
/*
    btCuda_allocateArray((void**)&m_dBodiesHash[0], m_maxHandles * 2 * sizeof(unsigned int));
    btCuda_allocateArray((void**)&m_dBodiesHash[1], m_maxHandles * 2 * sizeof(unsigned int));

	btCuda_allocateArray((void**)&m_dCellStart, m_params.m_numCells * sizeof(unsigned int));

    btCuda_allocateArray((void**)&m_dPairBuff, m_maxHandles * m_maxPairsPerBody * sizeof(unsigned int));
	btCuda_copyArrayToDevice(m_dPairBuff, m_hPairBuff, m_maxHandles * m_maxPairsPerBody * sizeof(unsigned int));  // needed?

    btCuda_allocateArray((void**)&m_dPairBuffStartCurr, (m_maxHandles * 2 + 1) * sizeof(unsigned int));
	btCuda_copyArrayToDevice(m_dPairBuffStartCurr, m_hPairBuffStartCurr, (m_maxHandles * 2 + 1) * sizeof(unsigned int)); 

	unsigned int numAABB = m_maxHandles + m_maxLargeHandles;
	btCuda_allocateArray((void**)&m_dAABB, numAABB * sizeof(bt3DGrid3F1U) * 2);

    btCuda_allocateArray((void**)&m_dPairScan, (m_maxHandles + 1) * sizeof(unsigned int));

	btCuda_allocateArray((void**)&m_dPairOut, m_maxHandles * m_maxPairsPerBody * sizeof(unsigned int));
*/
	btGpuDemo2dOCLWrap::setBroadphaseBuffers(m_maxHandles, m_maxLargeHandles, m_maxPairsPerBody, m_params.m_numCells);
}



void btGpu2dGridBroadphaseOCL::_finalize()
{
    assert(m_bInitialized);
/*
    btCuda_freeArray(m_dBodiesHash[0]);
    btCuda_freeArray(m_dBodiesHash[1]);
    btCuda_freeArray(m_dCellStart);
    btCuda_freeArray(m_dPairBuffStartCurr);
    btCuda_freeArray(m_dAABB);
    btCuda_freeArray(m_dPairBuff);
	btCuda_freeArray(m_dPairScan);
	btCuda_freeArray(m_dPairOut);
*/
}



//
// overrides for OpenCL version
//



void btGpu2dGridBroadphaseOCL::prepareAABB()
{
	btGpu3DGridBroadphase::prepareAABB();
	btGpuDemo2dOCLWrap::copyArrayToDevice(btGpuDemo2dOCLWrap::m_dAABB, m_hAABB, sizeof(float) * 4 * 2 * (m_numHandles + m_numLargeHandles)); 
	return;
}



void btGpu2dGridBroadphaseOCL::setParameters(bt3DGridBroadphaseParams* hostParams)
{
	btGpu3DGridBroadphase::setParameters(hostParams);
	bt2dGridBroadphaseParamsOCL hParams;
	hParams.m_cellSize[0] = this->m_params.m_cellSizeX;
	hParams.m_cellSize[1] = this->m_params.m_cellSizeY;
	hParams.m_cellSize[2] = this->m_params.m_cellSizeZ;
	hParams.m_cellSize[3] = 0.f;
	hParams.m_gridSize[0] = this->m_params.m_gridSizeX;
	hParams.m_gridSize[1] = this->m_params.m_gridSizeY;
	hParams.m_gridSize[2] = this->m_params.m_gridSizeZ;
	hParams.m_gridSize[3] = 0;
	hParams.m_worldMin[0] = this->m_params.m_worldOriginX;
	hParams.m_worldMin[1] = this->m_params.m_worldOriginY;
	hParams.m_worldMin[2] = this->m_params.m_worldOriginZ;
	hParams.m_worldMin[3] = 0.f;
	btGpuDemo2dOCLWrap::copyArrayToDevice(btGpuDemo2dOCLWrap::m_dBpParams, &hParams, sizeof(bt2dGridBroadphaseParamsOCL));
	return;
}



void btGpu2dGridBroadphaseOCL::calcHashAABB()
{
	BT_PROFILE("calcHashAABB");
#if 1
	static bool firstCall = true;
	if(firstCall)
	{	// need to pre-fill sime device buffers once
		memset(m_hBodiesHash, 0x7F, m_maxHandles*2*sizeof(unsigned int));
		btGpuDemo2dOCLWrap::copyArrayToDevice(btGpuDemo2dOCLWrap::m_dBodiesHash, m_hBodiesHash, m_maxHandles * 2 * sizeof(unsigned int));
		// now fill the rest (bitonic sorting works with size == pow of 2)
		int remainder = btGpuDemo2dOCLWrap::m_hashSize - m_maxHandles;
		if(remainder)
		{
			btGpuDemo2dOCLWrap::copyArrayToDevice(	btGpuDemo2dOCLWrap::m_dBodiesHash, 
													m_hBodiesHash, 
													remainder * 2 * sizeof(unsigned int),
													m_maxHandles * 2 * sizeof(unsigned int),
													0);
		}
		btGpuDemo2dOCLWrap::copyArrayToDevice(btGpuDemo2dOCLWrap::m_dPairBuffStartCurr, m_hPairBuffStartCurr, (m_maxHandles * 2 + 1) * sizeof(unsigned int)); 
		btGpuDemo2dOCLWrap::copyArrayToDevice(btGpuDemo2dOCLWrap::m_dPairBuff, m_hPairBuff, m_maxHandles * m_maxPairsPerBody * sizeof(unsigned int));
		firstCall = false;
	}
	btGpuDemo2dOCLWrap::runKernelWithWorkgroupSize(GPUDEMO2D_KERNEL_CALC_HASH_AABB, m_numHandles);
//	btGpuDemo3dOCLWrap::copyArrayFromDevice(m_hBodiesHash, btGpuDemo3dOCLWrap::m_dBodiesHash, sizeof(unsigned int) * 2 * m_numHandles);
//	btCuda_calcHashAABB(m_dAABB, m_dBodiesHash[0], m_numHandles);
#else
	btGpu3DGridBroadphase::calcHashAABB();
#endif
	return;
}



void btGpu2dGridBroadphaseOCL::sortHash()
{
	BT_PROFILE("RadixSort-- CUDA");
#if 1
	int dir = 1;
	btGpuDemo2dOCLWrap::bitonicSortNv(btGpuDemo2dOCLWrap::m_dBodiesHash, 1, btGpuDemo2dOCLWrap::m_hashSize, dir);
//	btGpuDemo3dOCLWrap::copyArrayFromDevice(m_hBodiesHash, btGpuDemo3dOCLWrap::m_dBodiesHash, sizeof(unsigned int) * 2 * m_numHandles);
//	RadixSort((KeyValuePair*)m_dBodiesHash[0], (KeyValuePair*)m_dBodiesHash[1], m_numHandles, 32);
#else
	btGpu3DGridBroadphase::sortHash();
#endif
	return;
}



void btGpu2dGridBroadphaseOCL::findCellStart()
{
#if 1
	BT_PROFILE("btCuda_findCellStart");

//	btGpuDemo3dOCLWrap::copyArrayToDevice(btGpuDemo3dOCLWrap::m_dBodiesHash, m_hBodiesHash, sizeof(unsigned int) * 2 * m_numHandles);


	btGpuDemo2dOCLWrap::runKernelWithWorkgroupSize(GPUDEMO2D_KERNEL_CLEAR_CELL_START, m_numCells);
	btGpuDemo2dOCLWrap::runKernelWithWorkgroupSize(GPUDEMO2D_KERNEL_FIND_CELL_START, m_numHandles);
	//	btCuda_findCellStart(m_dBodiesHash[0],	m_dCellStart, m_numHandles, m_params.m_numCells);

//	btGpuDemo3dOCLWrap::copyArrayFromDevice(m_hCellStart, btGpuDemo3dOCLWrap::m_dCellStart, sizeof(unsigned int) * m_params.m_numCells);

//	unsigned int numFilled = 0;
//	for(int i = 0; i < m_numCells; i++)
//	{
//		if(m_hCellStart[i] != 0xFFFFFFFF) numFilled++;
//	}
//	printf("Fil : %d\n", numFilled);
#else
	btGpu3DGridBroadphase::findCellStart();
#endif
	return;
}



void btGpu2dGridBroadphaseOCL::findOverlappingPairs()
{
	BT_PROFILE("btCuda_findOverlappingPairs");
#if 1
	btGpuDemo2dOCLWrap::runKernelWithWorkgroupSize(GPUDEMO2D_KERNEL_FIND_OVERLAPPING_PAIRS, m_numHandles);
//	btCuda_findOverlappingPairs(m_dAABB, m_dBodiesHash[0], m_dCellStart, m_dPairBuff, m_dPairBuffStartCurr,	m_numHandles);
//	btGpuDemo3dOCLWrap::copyArrayFromDevice(m_hPairBuff, btGpuDemo3dOCLWrap::m_dPairBuff, m_numHandles * m_maxPairsPerBody * sizeof(unsigned int));
//	btGpuDemo3dOCLWrap::copyArrayFromDevice(m_hPairBuffStartCurr, btGpuDemo3dOCLWrap::m_dPairBuffStartCurr, m_numHandles * 2 * sizeof(unsigned int)); 

#else
	btGpu3DGridBroadphase::findOverlappingPairs();
#endif
	return;
}



void btGpu2dGridBroadphaseOCL::findPairsLarge()
{
	BT_PROFILE("btCuda_findPairsLarge");
#if 1
	if(m_numLargeHandles)
	{
		btGpuDemo2dOCLWrap::setKernelArg(GPUDEMO2D_KERNEL_FIND_PAIRS_LARGE, 6, sizeof(int),(void*)&m_numLargeHandles);
		btGpuDemo2dOCLWrap::runKernelWithWorkgroupSize(GPUDEMO2D_KERNEL_FIND_PAIRS_LARGE, m_numHandles);
	}
//	btCuda_findPairsLarge(m_dAABB, m_dBodiesHash[0], m_dCellStart, m_dPairBuff, m_dPairBuffStartCurr,	m_numHandles, m_numLargeHandles);
#else
	btGpu3DGridBroadphase::findPairsLarge();
#endif
	return;
}



void btGpu2dGridBroadphaseOCL::computePairCacheChanges()
{
	BT_PROFILE("btCuda_computePairCacheChanges");
#if 1
	btGpuDemo2dOCLWrap::runKernelWithWorkgroupSize(GPUDEMO2D_KERNEL_COMPUTE_CACHE_CHANGES, m_numHandles);
//	btGpuDemo3dOCLWrap::copyArrayFromDevice(m_hPairScan, btGpuDemo3dOCLWrap::m_dPairScan, sizeof(unsigned int)*(m_numHandles + 1)); 
//	btCuda_computePairCacheChanges(m_dPairBuff, m_dPairBuffStartCurr, m_dPairScan, m_dAABB, m_numHandles);
#else
	btGpu3DGridBroadphase::computePairCacheChanges();
#endif
	return;
}



void btGpu2dGridBroadphaseOCL::scanOverlappingPairBuff()
{
#if 1
	btGpuDemo2dOCLWrap::copyArrayFromDevice(m_hPairScan, btGpuDemo2dOCLWrap::m_dPairScan, sizeof(unsigned int)*(m_numHandles + 1)); 
	btGpu3DGridBroadphase::scanOverlappingPairBuff();
	btGpuDemo2dOCLWrap::copyArrayToDevice(btGpuDemo2dOCLWrap::m_dPairScan, m_hPairScan, sizeof(unsigned int)*(m_numHandles + 1)); 
#else
	btGpu3DGridBroadphase::scanOverlappingPairBuff();
#endif
	return;
}



void btGpu2dGridBroadphaseOCL::squeezeOverlappingPairBuff()
{
	BT_PROFILE("btCuda_squeezeOverlappingPairBuff");
#if 1
	btGpuDemo2dOCLWrap::runKernelWithWorkgroupSize(GPUDEMO2D_KERNEL_SQUEEZE_PAIR_BUFF, m_numHandles);
//	btCuda_squeezeOverlappingPairBuff(m_dPairBuff, m_dPairBuffStartCurr, m_dPairScan, m_dPairOut, m_dAABB, m_numHandles);
	btGpuDemo2dOCLWrap::copyArrayFromDevice(m_hPairOut, btGpuDemo2dOCLWrap::m_dPairOut, sizeof(unsigned int) * m_hPairScan[m_numHandles]); 
#else
	btGpu3DGridBroadphase::squeezeOverlappingPairBuff();
#endif
	return;
}



void btGpu2dGridBroadphaseOCL::resetPool(btDispatcher* dispatcher)
{
	btGpu3DGridBroadphase::resetPool(dispatcher);
	if(btGpuDemo2dOCLWrap::m_dPairBuffStartCurr != NULL)
	{
		btGpuDemo2dOCLWrap::copyArrayToDevice(btGpuDemo2dOCLWrap::m_dPairBuffStartCurr, m_hPairBuffStartCurr, (m_maxHandles * 2 + 1) * sizeof(unsigned int)); 
	}
}


