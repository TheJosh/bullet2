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



#ifndef BTGPU3DGRIDBROADPHASEOCL_H
#define BTGPU3DGRIDBROADPHASEOCL_H

#ifdef __APPLE__
//CL_PLATFORM_MINI_CL could be defined in build system
#else
#include <GL/glew.h>
#include <CL/cl_platform.h> //for CL_PLATFORM_MINI_CL definition
#endif //__APPLE__


#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"

#include "../../src/BulletMultiThreaded/btGpu3DGridBroadphaseSharedTypes.h"
#include "../../src/BulletMultiThreaded/btGpu3DGridBroadphase.h"

struct bt3DGridBroadphaseParamsOCL
{
	float m_worldMin[4];
	float m_cellSize[4];
	int   m_gridSize[4];
};


///The btGpu3DGridBroadphaseOCL uses OpenCL-capable GPU to compute overlapping pairs

class btGpu3DGridBroadphaseOCL : public btGpu3DGridBroadphase
{
protected:
    // GPU data
	// moved to btGpuDemo3dOCLWrap for now
//  cl_mem	m_dBodiesHash;
//  cl_mem	m_dCellStart;
//	cl_mem	m_dPairBuff; 
//	cl_mem	m_dPairBuffStartCurr;
//	cl_mem	m_dAABB;
//	cl_mem	m_dPairScan;
//	cl_mem	m_dPairOut;
public:
	btGpu3DGridBroadphaseOCL(	btOverlappingPairCache* overlappingPairCache,
						const btVector3& worldAabbMin,const btVector3& worldAabbMax, 
						int gridSizeX, int gridSizeY, int gridSizeZ, 
						int maxSmallProxies, int maxLargeProxies, int maxPairsPerSmallProxies,
						int maxSmallProxiesPerCell = 8,
						btScalar cellFactorAABB = btScalar(1.0f));
	virtual ~btGpu3DGridBroadphaseOCL();
protected:
	void _initialize();
	void _finalize();
	void allocateArray(void** devPtr, unsigned int size);
	void freeArray(void* devPtr);
// overrides for OpenCL version
	virtual void setParameters(bt3DGridBroadphaseParams* hostParams);
	virtual void prepareAABB();
	virtual void calcHashAABB();
	virtual void sortHash();	
	virtual void findCellStart();
	virtual void findOverlappingPairs();
	virtual void findPairsLarge();
	virtual void computePairCacheChanges();
	virtual void scanOverlappingPairBuff();
	virtual void squeezeOverlappingPairBuff();
	virtual void resetPool(btDispatcher* dispatcher);
};

#endif //BTGPU3DGRIDBROADPHASEOCL_H