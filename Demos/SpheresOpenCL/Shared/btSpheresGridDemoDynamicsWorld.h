/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006 - 2009 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef BT_SPHERES_GRID_DEMO_DYNAMICS_WORLD_H
#define BT_SPHERES_GRID_DEMO_DYNAMICS_WORLD_H

#ifdef __APPLE__
#include <OpenCL/cl.h>
#else
// standard utility and system includes
#include <CL/cl.h>
// Extra CL/GL include
#include <CL/cl_gl.h>
#endif




#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"

#include "btSpheresGridDemoSharedDefs.h"
#include "btSpheresGridDemoSharedTypes.h"




#define SPHERES_GRID_MAX_OBJS (65536)
#define SPHERES_GRID_MAX_NEIGHBORS (32)
//#define SPHERES_GRID_MAX_BATCHES (20)
#define SPHERES_GRID_MAX_BATCHES (32)

// REAL number of threads executing in parallel
#if defined(CL_PLATFORM_MINI_CL)
#define SPHERES_GRID_MAX_WORKGROUP_SIZE  (4) // TODO : get from device
#else
// CUDA 1.0
#define SPHERES_GRID_MAX_WORKGROUP_SIZE  (512) // TODO : get from device
#endif

#define SPHERES_GRID_MANIFOLD_CACHE_SIZE	(4)
#define SPHERES_GRID_NUM_OBJ_MANIFOLDS		(12)

struct btPairId
{
	int m_objA;
	int m_objB;
	int m_sphA;
	int m_sphB;
	int m_batch;
	int m_pair;
	int m_pad[2];
	void quickSort(btPairId* pData, int lo, int hi)
	{
		if(lo >= hi) return;
		int i=lo, j=hi;
		btPairId x = pData[(lo+hi)/2];
		do
		{    
			while(pData[i].m_batch < x.m_batch) i++; 
			while(x.m_batch < pData[j].m_batch) j--;
			if(i <= j)
			{
				btPairId t = pData[i];
				pData[i] = pData[j];
				pData[j] = t;
				i++; j--;
			}
		} while(i <= j);
		if(lo < j) pData->quickSort(pData, lo, j);
		if(i < hi) pData->quickSort(pData, i, hi);
	}
};


struct btSpheresContPair
{
	btVector3 m_contact; // + penetration in w
	btVector3 m_normal;  // + impulse accumulator in w
};

enum
{
	GPUDEMO_KERNEL_APPLY_GRAVITY = 0,
	GPUDEMO_KERNEL_COMPUTE_CELL_ID,
	GPUDEMO_KERNEL_CLEAR_CELL_START,
	GPUDEMO_KERNEL_FIND_CELL_START,
	GPUDEMO_KERNEL_BITONIC_SORT_CELL_ID_ALL_GLOB,
	GPUDEMO_KERNEL_BITONIC_SORT_CELL_ID_LOCAL,
	GPUDEMO_KERNEL_BITONIC_SORT_CELL_ID_LOCAL_1,
	GPUDEMO_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL,
	GPUDEMO_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL,
	GPUDEMO_KERNEL_FIND_PAIRS,
	GPUDEMO_KERNEL_COMPUTE_CONTACTS_1,
	GPUDEMO_KERNEL_SCAN_PAIRS_EXCLUSIVE_LOCAL_1,
	GPUDEMO_KERNEL_SCAN_PAIRS_EXCLUSIVE_LOCAL_2,
	GPUDEMO_KERNEL_SCAN_PAIRS_UNIFORM_UPDATE,
	GPUDEMO_KERNEL_SCAN_PAIRS,
	GPUDEMO_KERNEL_COMPACT_PAIRS,
	GPUDEMO_KERNEL_INIT_BATCHES,
	GPUDEMO_KERNEL_COMPUTE_BATCHES,
	GPUDEMO_KERNEL_CHECK_BATCHES,
	GPUDEMO_KERNEL_COMPUTE_CONTACTS,
	GPUDEMO_KERNEL_SOLVE_CONSTRAINTS,
	GPUDEMO_KERNEL_INTEGRATE_TRANSFORMS,
	GPUDEMO_KERNEL_TOTAL
};


enum 
{
	SIMSTAGE_NONE = 0,
	SIMSTAGE_APPLY_GRAVITY,
	SIMSTAGE_COMPUTE_CELL_ID,
	SIMSTAGE_SORT_CELL_ID,
	SIMSTAGE_FIND_CELL_START,
	SIMSTAGE_FIND_PAIRS,
	SIMSTAGE_SCAN_PAIRS,
	SIMSTAGE_COMPACT_PAIRS,
	SIMSTAGE_COMPUTE_BATCHES,
	SIMSTAGE_COMPUTE_CONTACTS,
	SIMSTAGE_SOLVE_CONSTRAINTS,
	SIMSTAGE_INTEGRATE_TRANSFORMS,
	SIMSTAGE_TOTAL
};

struct btKernelInfo
{
	int			m_Id;
	cl_kernel	m_kernel;
	char*		m_name;
	int			m_workgroupSize;
};

class btSpheresGridDemoDynamicsWorld : public btDiscreteDynamicsWorld
{
public:
	int			m_numSpheres;
	int			m_usedDevice;
	btScalar	m_sphereRad;
	struct GL_ToggleControl* m_useCpuControls[SIMSTAGE_TOTAL];
	
protected:
	int			m_numObjects;
	int			m_hashSize; // power of 2 >= m_numSpheres;
	int			m_scanSize; // power of 2 >= m_numSpheres;
	int			m_numGridCells; 
	int			m_maxNeighbors;
	int			m_numPairs; 
	int			m_maxPairs;
	int			m_numBatches;
	int			m_maxBatches;
	int			m_workGroupSize;
	int			m_numSolverIterations;
	// CPU side data
	btAlignedObjectArray<btVector3>	m_hShapeBuf;
	btAlignedObjectArray<btInt2>	m_hShapeIds; // per each body : (start index, num_spheres)
	btAlignedObjectArray<int>		m_hBodyIds;  // per each sphere : parent body index
public:
	btAlignedObjectArray<btVector3>	m_hPos;
	btAlignedObjectArray<btVector3>	m_hLinVel;
protected:
	btAlignedObjectArray<btVector3>	m_hTrans;
	btAlignedObjectArray<btVector3>	m_hAngVel;
	btAlignedObjectArray<btVector3>	m_hInvInertiaMass;
	btAlignedObjectArray<btInt2>	m_hPosHash;
	btAlignedObjectArray<int>		m_hCellStart;
	btAlignedObjectArray<int>		m_hPairBuff;
	btAlignedObjectArray<int>		m_hPairBuffStart;
	btAlignedObjectArray<int>		m_hPairBuffCurr;
	btAlignedObjectArray<int>		m_hPairScan;
	btAlignedObjectArray<btPairId>	m_hPairIds;
	btAlignedObjectArray<int>		m_hObjUsed;
	//btAlignedObjectArray<int>		m_hNumPairsInBatch;
	btAlignedObjectArray<btSpheresContPair> m_hContacts;
	// persistent manifolds (cache for object-object contact points)
	btAlignedObjectArray<btVector3>	m_hManifBuff;
	btAlignedObjectArray<int>		m_hManifObjId;
	// GPU side data
	cl_mem		m_dShapeBuf;
	cl_mem		m_dShapeIds;
	cl_mem		m_dBodyIds;
	cl_mem		m_dPos;
	cl_mem		m_dTrans;
	cl_mem		m_dLinVel;
	cl_mem		m_dAngVel;
	cl_mem		m_dInvInertiaMass;
	cl_mem		m_dPosHash;
	cl_mem		m_dCellStart;
	cl_mem		m_dPairBuff;
	cl_mem		m_dPairBuffStart;
	cl_mem		m_dPairBuffCurr;
	cl_mem		m_dPairScan;
	cl_mem		m_dPairIds;
	cl_mem		m_dObjUsed;
	cl_mem		m_dContacts;
	cl_mem		m_dSimParams; // copy of m_simParams : global simulation paramerers such as gravity, etc. 
	cl_mem		m_dScanTmpBuffer;
	// persistent manifolds (cache for object-object contact points)
	cl_mem		m_dManifBuff; // (<contact+penetration><normal>) * SPHERES_GRID_MANIFOLD_CACHE_SIZE * SPHERES_GRID_NUM_OBJ_MANIFOLDS
	cl_mem		m_dManifObjId; // <ObjId> * SPHERES_GRID_NUM_OBJ_MANIFOLDS


	// OpenCL 
public:
	cl_context			m_cxMainContext;
	cl_device_id		m_cdDevice;
	cl_command_queue	m_cqCommandQue;
	cl_program			m_cpProgram;
protected:
	btKernelInfo		m_kernels[GPUDEMO_KERNEL_TOTAL];

	btScalar			m_minSphereRad;
	btScalar			m_maxSphereRad;
	btVector3			m_cellSize;

public:
	btVector3			m_worldMin;
	btVector3			m_worldMax;
	// vbo variables
	GLuint			m_vbo;
	unsigned int	m_posVbo;
	unsigned int	m_colVbo;
	btSimParams		m_simParams;
	float			m_timeStep;
	int				m_pairOffset;

	int getNumSpheres() { return m_hShapeBuf.size(); }
	float* getPosBuffer() { return (float*)&(m_hPos[0]); }


	btSpheresGridDemoDynamicsWorld(btDispatcher* dispatcher,btBroadphaseInterface* pairCache,btConstraintSolver* constraintSolver,btCollisionConfiguration* collisionConfiguration,
			int maxObjs = SPHERES_GRID_MAX_OBJS, int maxNeighbors = SPHERES_GRID_MAX_NEIGHBORS)
		: btDiscreteDynamicsWorld(dispatcher, pairCache, constraintSolver, collisionConfiguration)
	{ 
		m_usedDevice = 1;
//		m_useCPU = false;
		m_sphereRad = btScalar(0.5f);
		m_simParams.m_gravity[0] = 0.f;
		m_simParams.m_gravity[1] = -10.f;
		m_simParams.m_gravity[2] = 0.f;
		m_simParams.m_gravity[3] = 0.f;
		m_workGroupSize = SPHERES_GRID_MAX_WORKGROUP_SIZE;
		m_numSolverIterations = 4;
	}
	virtual ~btSpheresGridDemoDynamicsWorld();
	virtual int	stepSimulation( btScalar timeStep,int maxSubSteps=1, btScalar fixedTimeStep=btScalar(1.)/btScalar(60.));

	void initDeviceData();
	void initCLKernels(int argc, char** argv);
	void createVBO();
	void postInitDeviceData();
	void getShapeData();
	void allocateBuffers();
	void grabSimulationData();
	void adjustGrid();
	void runComputeCellIdKernel();
	void runApplyGravityKernel();
	void runIntegrateTransformsKernel();
	void runSortHashKernel();
	void runFindCellStartKernel();
	void runFindPairsKernel();
	void runScanPairsKernel();
	void runCompactPairsKernel();
	void runComputeBatchesKernel();
	void runComputeContactsKernel();
	void runComputeContacts1Kernel();
	void runSolveConstraintsKernel();
	void solvePairCPU(btSpheresContPair* pPair, int pairIdx, int batchNum);
	void initKernel(int kernelId, char* pName);
	void runKernelWithWorkgroupSize(int kernelId, int globalSize);
	void bitonicSortNv(cl_mem pKey, unsigned int batch, unsigned int arrayLength, unsigned int dir);
	void scanExclusiveLocal1(cl_mem d_Dst, cl_mem d_Src, unsigned int n, unsigned int size);
	void scanExclusiveLocal2(cl_mem d_Buffer, cl_mem d_Dst, cl_mem d_Src, unsigned int n, unsigned int size);
	void uniformUpdate(cl_mem d_Dst, cl_mem d_Buffer, unsigned int n);
	void scanExclusive(cl_mem d_Dst, cl_mem d_Src, unsigned int arrayLength);
};


#endif //BT_SPHERES_GRID_DEMO_DYNAMICS_WORLD_H
