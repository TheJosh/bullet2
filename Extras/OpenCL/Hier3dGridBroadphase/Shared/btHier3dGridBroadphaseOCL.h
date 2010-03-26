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



#ifndef BTHIER3DGRIDBROADPHASEOCL_H
#define BTHIER3DGRIDBROADPHASEOCL_H

#ifdef __APPLE__
#ifdef USE_MINICL
	#include <MiniCL/cl.h>
#else
	#include <MiniCL/cl.h>
#endif
//CL_PLATFORM_MINI_CL could be defined in build system
#else
//#include <GL/glew.h>
// standard utility and system includes
#ifdef USE_MINICL
	#include <MiniCL/cl.h>
#else
	#include <CL/cl.h>
#endif
// Extra CL/GL include
//#include <CL/cl_gl.h>
#endif //__APPLE__


#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"

#define HIER3DGRIDOCL_CHECKERROR(a, b) if((a)!=(b)) { printf("HIER 3D GRID OCL Error : %d\n", (a)); btAssert((a) == (b)); }

enum
{
	HIER3DGRIDOCL_KERNEL_CALC_HASH_AABB = 0,
	HIER3DGRIDOCL_KERNEL_CLEAR_CELL_START,
	HIER3DGRIDOCL_KERNEL_FIND_CELL_START,
	HIER3DGRIDOCL_KERNEL_FIND_OVERLAPPING_PAIRS,
	HIER3DGRIDOCL_KERNEL_COMPUTE_CACHE_CHANGES,
	HIER3DGRIDOCL_KERNEL_SQUEEZE_PAIR_BUFF,
	HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_LOCAL,
	HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_LOCAL_1,
	HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL,
	HIER3DGRIDOCL_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL,
	HIER3DGRIDOCL_KERNEL_TOTAL
};

struct btHier3dGridOCLKernelInfo
{
	int			m_Id;
	cl_kernel	m_kernel;
	char*		m_name;
	int			m_workgroupSize;
};

struct btHier3dGridOCLParams
{
	btVector3	m_invCellSize;
	int			m_gridSizeX;
	int			m_gridSizeY;
	int			m_gridSizeZ;
	int			m_gridDepth;
};

///The btHier3dGridBroadphaseOCL uses OpenCL-capable GPU to compute overlapping pairs

class btHier3dGridBroadphaseOCL : public btSimpleBroadphase
{
protected:
	bool					m_bInitialized;
	int						m_hashSize;
	int						m_numOfCells;
	int						m_maxPairsPerProxy;
	// Open CL stuff
	cl_context				m_cxMainContext;
	cl_device_id			m_cdDevice;
	cl_command_queue		m_cqCommandQue;
	cl_program				m_cpProgram;
	btHier3dGridOCLKernelInfo	m_kernels[HIER3DGRIDOCL_KERNEL_TOTAL];
	// data buffers (CPU side)
	btHier3dGridOCLParams	m_hParams;
	btAlignedObjectArray<btVector3>	m_hAABB;
	btAlignedObjectArray<int>		m_hBodiesHash;
	btAlignedObjectArray<int>		m_hCellStart;
	btAlignedObjectArray<int>		m_hPairBuffStartCurr;
	btAlignedObjectArray<int>		m_hPairBuff;
	btAlignedObjectArray<int>		m_hPairScan;
	btAlignedObjectArray<int>		m_hPairOut;
	// data buffers (GPU side)
	cl_mem					m_dParams;
	cl_mem					m_dAABB;
	cl_mem					m_dBodiesHash;
	cl_mem					m_dCellStart;
	cl_mem					m_dPairBuffStartCurr;
	cl_mem					m_dPairBuff;
	cl_mem					m_dPairScan;
	cl_mem					m_dPairOut;

public:

	btHier3dGridBroadphaseOCL(	btOverlappingPairCache* overlappingPairCache,
								const btVector3& cellSize, 
								int gridSizeX, int gridSizeY, int gridSizeZ,
								int maxProxies, int maxPairsPerProxy,
								cl_context context = NULL,
								cl_device_id device = NULL,
								cl_command_queue queue = NULL);
	virtual ~btHier3dGridBroadphaseOCL();


	virtual void	calculateOverlappingPairs(btDispatcher* dispatcher);
	virtual void	resetPool(btDispatcher* dispatcher);
	virtual btBroadphaseProxy*	createProxy(  const btVector3& aabbMin,  const btVector3& aabbMax,int shapeType,void* userPtr ,short int collisionFilterGroup,short int collisionFilterMask, btDispatcher* dispatcher,void* multiSapProxy);

protected:
	void initCL(cl_context context, cl_device_id device, cl_command_queue queue);
	void initKernels();
	void allocateBuffers();
	void prefillBuffers();
	void initKernel(int kernelId, char* pName);
	void allocateArray(void** devPtr, unsigned int size);
	void freeArray(void* devPtr);
	void runKernelWithWorkgroupSize(int kernelId, int globalSize);
	void setKernelArg(int kernelId, int argNum, int argSize, void* argPtr);
	void copyArrayToDevice(cl_mem device, const void* host, unsigned int size, int devOffs = 0, int hostOffs = 0);
	void copyArrayFromDevice(void* host, const cl_mem device, unsigned int size, int hostOffs = 0, int devOffs = 0);
	void bitonicSort(cl_mem pKey, unsigned int arrayLength, unsigned int dir);

	void prepareAABB();
	void calcHashAABB();
	void sortHash();
	void findCellStart();
	void findOverlappingPairs();
	void computePairCacheChanges();
	void scanOverlappingPairBuff();
	void squeezeOverlappingPairBuff();
	void addPairsToCache(btDispatcher* dispatcher);
};

#endif //BTHIER3DGRIDBROADPHASEOCL_H