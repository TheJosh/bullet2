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

#ifndef BATCH_CONSTRAINT_SOLVER_OCL_H
#define BATCH_CONSTRAINT_SOLVER_OCL_H

#ifdef __APPLE__
#include <OpenCL/cl.h>
#else
// standard utility and system includes
#include <CL/cl.h>
// Extra CL/GL include
//#include <CL/cl_gl.h>
#endif


#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"

#define BCSOCL_DEF_MAX_BODIES (1024)
#define BCSOCL_DEF_MAX_CONSTR (BCSOCL_DEF_MAX_BODIES * 16)
#define BCSOCL_DEF_MAX_BATCHES 16

#define BCSOCL_CHECKERROR(a, b) if((a)!=(b)) { printf("Batch Constraint Solver OCL Error : %d\n", (a)); btAssert((a) == (b)); }

struct btBCSOCLKernelInfo
{
	int			m_Id;
	cl_kernel	m_kernel;
	char*		m_name;
	int			m_workgroupSize;
};

enum
{
	BCSOCL_KERNEL_SOLVE_CONSTRAINT_ROW = 0,
	BCSOCL_KERNEL_SETUP_FRICTION_CONSTRAINT,
	BCSOCL_KERNEL_SOLVE_CONSTR_ROW_LOW_LIM,
	BCSOCL_KERNEL_TOTAL
};

enum
{
	BCSOCL_SOLVER_GPU = 0,
	BCSOCL_SOLVER_BULLET_SIMD,
	BCSOCL_SOLVER_NUM_OF_MODES
};


class btBatchConstraintSolverOCL : public btSequentialImpulseConstraintSolver
{
protected:
	int m_maxBodies;
	int m_maxConstraints;
	int m_maxBatches;
	btAlignedObjectArray<int> m_hConstraintIdx;
	btAlignedObjectArray<int> m_batchData;

	int m_numBodies;
	int m_numJointConstraints;
	int m_numContactConstraints;
	int m_numFrictionConstraints;
	int m_numConstraints;

	int m_solverMode;

	btAlignedObjectArray<int> m_bodyMarkers;
	btAlignedObjectArray<int> m_constrMarkers;

	int m_maxBatchSize;
	int m_numJointBatches;
	int m_numContactBatches;
	int m_numFrictionBatches;
	int m_numMergedBatches;

// OpenCL stuff
	cl_context			m_cxMainContext;
	cl_device_id		m_cdDevice;
	cl_command_queue	m_cqCommandQue;
	cl_program			m_cpProgram;
	btBCSOCLKernelInfo	m_kernels[BCSOCL_KERNEL_TOTAL];
// GPU data
	cl_mem				m_dBodies;
	cl_mem				m_dConstraints;
	cl_mem				m_dConstraintIdx;


// internal methods
	int prepareBatches(btConstraintArray& constraints, int batchOffset, int constraintOffset, btConstraintArray* auxConstraints = NULL, int auxConstrOffs = 0);
	void solveGroupIterationsCPU(const btContactSolverInfo& infoGlobal);
	void solveGroupIterationsGPU(const btContactSolverInfo& infoGlobal);
	void solveBatchesCPU(btConstraintArray& constraints, int numBatches, int batchOffs, int indexOffs);
	void solveBatchesLowLimCPU(btConstraintArray& constraints, int numBatches, int batchOffs, int indexOffs);
	void solveBatchesGPU(int numBatches, int batchOffs);
	void solveBatchesLowLimGPU(int numBatches, int batchOffs);
	void initCL(cl_context context, cl_device_id device, cl_command_queue queue);
	void initKernels();
	void allocateBuffers();
	void initKernel(int kernelId, char* pName);
	void setKernelArg(int kernelId, int argNum, int argSize, void* argPtr);
	void runKernelWithWorkgroupSize(int kernelId, int globalSize);
	void copyArrayToDevice(cl_mem device, const void* host, unsigned int size, int devOffs = 0, int hostOffs = 0);
	void copyArrayFromDevice(void* host, const cl_mem device, unsigned int size, int hostOffs = 0, int devOffs = 0);

public:
	btBatchConstraintSolverOCL(	int maxBodies = BCSOCL_DEF_MAX_BODIES, 
								int maxConstraints = BCSOCL_DEF_MAX_CONSTR, 
								int maxBatches = BCSOCL_DEF_MAX_BATCHES,
								cl_context context = NULL,
								cl_device_id device = NULL,
								cl_command_queue queue = NULL);

	virtual ~btBatchConstraintSolverOCL();
	virtual btScalar solveGroup(btCollisionObject** bodies,int numBodies,btPersistentManifold** manifold,int numManifolds,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& info, btIDebugDraw* debugDrawer, btStackAlloc* stackAlloc,btDispatcher* dispatcher);
	// mode access
	int getSolverMode() { return m_solverMode; }
	void setSolverMode(int mode) { m_solverMode = mode % BCSOCL_SOLVER_NUM_OF_MODES; }
};

#endif // BATCH_CONSTRAINT_SOLVER_OCL_H

