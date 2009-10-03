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
#ifndef BTGPUDEMO2DOCLWRAP_H
#define BTGPUDEMO2DOCLWRAP_H

#ifdef __APPLE__
#include <OpenCL/cl.h>
#else
// standard utility and system includes
#include <CL/cl.h>
// Extra CL/GL include
#include <CL/cl_gl.h>
#endif

enum
{
	GPUDEMO2D_KERNEL_CLEAR_ACCUM_IMPULSE = 0,
	GPUDEMO2D_KERNEL_COMPUTE_CONSTRAINTS,
	GPUDEMO2D_KERNEL_COLLISION_WITH_WALL,
	GPUDEMO2D_KERNEL_SOLVE_CONSTRAINTS,

	GPUDEMO3D_KERNEL_INTEGRATE_VELOCITIES,
	GPUDEMO3D_KERNEL_INTEGRATE_TRANSFORMS,
	GPUDEMO3D_KERNEL_CALC_HASH_AABB,
	GPUDEMO3D_KERNEL_CLEAR_CELL_START,
	GPUDEMO3D_KERNEL_FIND_CELL_START,
	GPUDEMO3D_KERNEL_FIND_OVERLAPPING_PAIRS,
	GPUDEMO3D_KERNEL_FIND_PAIRS_LARGE,
	GPUDEMO3D_KERNEL_COMPUTE_CACHE_CHANGES,
	GPUDEMO3D_KERNEL_SQUEEZE_PAIR_BUFF,
	GPUDEMO3D_KERNEL_BITONIC_SORT_CELL_ID_LOCAL,
	GPUDEMO3D_KERNEL_BITONIC_SORT_CELL_ID_LOCAL_1,
	GPUDEMO3D_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL,
	GPUDEMO3D_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL,
	GPUDEMO2D_KERNEL_TOTAL
};


struct btKernelInfo
{
	int			m_Id;
	cl_kernel	m_kernel;
	char*		m_name;
	int			m_workgroupSize;
};


class btGpuDemo2dOCLWrap
{
protected:
	static cl_context		m_cxMainContext;
	static cl_device_id		m_cdDevice;
	static cl_command_queue	m_cqCommandQue;
	static cl_program		m_cpProgram;
	static btKernelInfo		m_kernels[GPUDEMO2D_KERNEL_TOTAL];
	static bool m_broadphaseInited;


public:

	static int	m_maxObjs;
	static int	m_maxNeighbors;
	static int	m_maxConstr;
	static int	m_maxVtxPerObj;
	static int	m_maxBatches;
	static int	m_maxShapeBufferSize;

	static cl_mem	m_dPos;
	static cl_mem	m_dRot;
	static cl_mem	m_dVel;
	static cl_mem	m_dAngVel;
	static cl_mem	m_dpPos;
	static cl_mem	m_dpRot;
	static cl_mem	m_dpVel;
	static cl_mem	m_dpAngVel;
	static cl_mem	m_dIds;
	static cl_mem	m_dBatchIds;
	static cl_mem	m_dLambdaDtBox;
	static cl_mem	m_dContact; // 8 floats : pos.x, pos.y, pos.z, penetration, norm.x, norm.y, norm.z, reserved
	static cl_mem	m_dInvMass;
	static cl_mem	m_dShapeBuffer;
	static cl_mem	m_dShapeIds;
	static cl_mem	m_dParams;

	static cl_mem	m_dcPos;
	static cl_mem	m_dcRot;
	static cl_mem	m_dcVel;
	static cl_mem	m_dcAngVel;

	// broadphase data
	static cl_mem	m_dBodiesHash;
	static cl_mem	m_dCellStart;
	static cl_mem	m_dPairBuff; 
	static cl_mem	m_dPairBuffStartCurr;
	static cl_mem	m_dAABB;
	static cl_mem	m_dPairScan;
	static cl_mem	m_dPairOut;
	static cl_mem	m_dBpParams;


	static int m_maxHandles;
	static int m_maxLargeHandles;
	static int m_maxPairsPerBody;
	static int m_numCells;
	static int m_hashSize;



	static void initCL(int argc, char** argv);
	static void allocateArray(cl_mem* ppBuf, int memSize);
	static void initKernels();
	static void allocateBuffers(int maxObjs, int maxNeighbors, int maxVtxPerObj, int maxBatches, int maxShapeBufferSize);
	static void initKernel(int kernelId, char* pName);
	static void runKernelWithWorkgroupSize(int kernelId, int globalSize);
	static void setKernelArg(int kernelId, int argNum, int argSize, void* argPtr);

	static void copyArrayToDevice(cl_mem device, const void* host, unsigned int size, int devOffs = 0, int hostOffs = 0);
	static void copyArrayFromDevice(void* host, const cl_mem device, unsigned int size, int hostOffs = 0, int devOffs = 0);

	static void setBroadphaseBuffers(int maxHandles, int maxLargeHandles, int maxPairsPerBody, int numCells);
	static void bitonicSortNv(cl_mem pKey, unsigned int batch, unsigned int arrayLength, unsigned int dir);

};

#endif // BTGPUDEMO2DOCLWRAP