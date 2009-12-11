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

#include <new>
#include <string.h> //for memset
#include <stdio.h> //for memset

#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
//#include "btContactConstraint.h"
//#include "btSolve2LinearConstraint.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include "LinearMath/btIDebugDraw.h"
//#include "btJacobianEntry.h"
#include "LinearMath/btMinMax.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "LinearMath/btStackAlloc.h"
#include "LinearMath/btQuickprof.h"
//#include "btSolverBody.h"
//#include "btSolverConstraint.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btBatchConstraintSolverOCL.h"


#define MERGE_JOINTS_AND_CONTACTS 1

#define MSTRINGIFY(A) #A

static const char* spProgramSource = 
#include "btBatchConstraintSolverOCL.cl"



btBatchConstraintSolverOCL::btBatchConstraintSolverOCL(int maxBodies, int maxConstraints, int maxBatches,
														cl_context context, cl_device_id device, cl_command_queue queue)
{

	maxBatches = 128;

	m_solverMode = BCSOCL_SOLVER_GPU;
	m_hConstraintIdx.resize(maxConstraints, -1);
	m_batchData.resize(maxBatches * 2 * 2, -1);
	m_bodyMarkers.resize(maxBodies, -1);
	m_constrMarkers.resize(maxConstraints, -1);
	m_maxBodies = maxBodies;
	m_maxConstraints = maxConstraints;
	m_maxBatches = maxBatches;
	initCL(context, device, queue);
	allocateBuffers();
	initKernels();
}


btBatchConstraintSolverOCL::~btBatchConstraintSolverOCL()
{
}


#ifdef CL_PLATFORM_MINI_CL
// there is a problem with MSVC9 : static constructors are not called if variables defined in library and are not used
// looks like it is because of optimization
// probably this will happen with other compilers as well
// so to make it robust, register kernels again (it is safe)
#define MINICL_DECLARE(a) extern "C" void a();
MINICL_DECLARE(kSolveConstraintRow)
MINICL_DECLARE(kSetupFrictionConstraint)
MINICL_DECLARE(kSolveConstrRowLowLim)
#undef MINICL_DECLARE
#endif


void btBatchConstraintSolverOCL::initCL(cl_context context, cl_device_id device, cl_command_queue queue)
{

	#ifdef CL_PLATFORM_MINI_CL
		// call constructors here
		MINICL_REGISTER(kSolveConstraintRow)
		MINICL_REGISTER(kSetupFrictionConstraint)
		MINICL_REGISTER(kSolveConstrRowLowLim)
	#endif

	cl_int ciErrNum;

	if(context != NULL)
	{
		m_cxMainContext = context;
	}
	else
	{
		// create a context 
		m_cxMainContext = clCreateContextFromType(0, CL_DEVICE_TYPE_ALL, NULL, NULL, &ciErrNum);
		BCSOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
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
		BCSOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
		devices = (cl_device_id*)malloc(paramSize);
		ciErrNum = clGetContextInfo(m_cxMainContext, CL_CONTEXT_DEVICES, paramSize, devices, NULL);
		BCSOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
		m_cdDevice = devices[0]; // grab the first for now
		free(devices);
	}

	cl_ulong locMemSize;
	ciErrNum = clGetDeviceInfo(m_cdDevice, CL_DEVICE_LOCAL_MEM_SIZE, sizeof(cl_ulong), &locMemSize, NULL);
	BCSOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

	if(queue != NULL)
	{
		m_cqCommandQue = queue;
	}
	else
	{
		// create a command-queue
		m_cqCommandQue = clCreateCommandQueue(m_cxMainContext, m_cdDevice, 0, &ciErrNum);
		BCSOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
	}
	// create the program
	size_t programLength = strlen(spProgramSource);
	printf("OpenCL compiles btBatchConstraintSolverOCL.cl ...");
	m_cpProgram = clCreateProgramWithSource(m_cxMainContext, 1, &spProgramSource, &programLength, &ciErrNum);
	// build the program
	ciErrNum = clBuildProgram(m_cpProgram, 0, NULL, "-DGUID_ARG=""""", NULL, NULL);
	if(ciErrNum != CL_SUCCESS)
	{
		// write out standard error
		char cBuildLog[65536];
		clGetProgramBuildInfo(m_cpProgram, m_cdDevice, CL_PROGRAM_BUILD_LOG, 
							  sizeof(cBuildLog), cBuildLog, NULL );
		printf("\n\n%s\n\n\n", cBuildLog);
		printf("Press ENTER key to terminate the program\n");
		getchar();
		exit(-1); 
	}
	printf("OK\n");
}


void btBatchConstraintSolverOCL::initKernels()
{
	initKernel(BCSOCL_KERNEL_SOLVE_CONSTRAINT_ROW,	"kSolveConstraintRow");
	setKernelArg(BCSOCL_KERNEL_SOLVE_CONSTRAINT_ROW, 1, sizeof(cl_mem),(void*)&m_dBodies);
	setKernelArg(BCSOCL_KERNEL_SOLVE_CONSTRAINT_ROW, 2, sizeof(cl_mem),(void*)&m_dConstraints);
	setKernelArg(BCSOCL_KERNEL_SOLVE_CONSTRAINT_ROW, 3, sizeof(cl_mem),(void*)&m_dConstraintIdx);

	initKernel(BCSOCL_KERNEL_SETUP_FRICTION_CONSTRAINT,	"kSetupFrictionConstraint");
	setKernelArg(BCSOCL_KERNEL_SETUP_FRICTION_CONSTRAINT, 1, sizeof(cl_mem),(void*)&m_dConstraints);

	initKernel(BCSOCL_KERNEL_SOLVE_CONSTR_ROW_LOW_LIM,	"kSolveConstrRowLowLim");
	setKernelArg(BCSOCL_KERNEL_SOLVE_CONSTR_ROW_LOW_LIM, 1, sizeof(cl_mem),(void*)&m_dBodies);
	setKernelArg(BCSOCL_KERNEL_SOLVE_CONSTR_ROW_LOW_LIM, 2, sizeof(cl_mem),(void*)&m_dConstraints);
	setKernelArg(BCSOCL_KERNEL_SOLVE_CONSTR_ROW_LOW_LIM, 3, sizeof(cl_mem),(void*)&m_dConstraintIdx);
}


void btBatchConstraintSolverOCL::allocateBuffers()
{
    cl_int ciErrNum;
    unsigned int memSize;

	memSize = m_maxBodies * sizeof(btSolverBody);
	m_dBodies = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
	BCSOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

	memSize = m_maxConstraints * sizeof(btSolverConstraint);
	m_dConstraints = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
	BCSOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

	memSize = m_maxConstraints * sizeof(int);
	m_dConstraintIdx = clCreateBuffer(m_cxMainContext, CL_MEM_READ_ONLY, memSize, NULL, &ciErrNum);
	BCSOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
}

void btBatchConstraintSolverOCL::initKernel(int kernelId, char* pName)
{
	
	cl_int ciErrNum;
	cl_kernel kernel = clCreateKernel(m_cpProgram, pName, &ciErrNum);
	BCSOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
	size_t wgSize;
	ciErrNum = clGetKernelWorkGroupInfo(kernel, m_cdDevice, CL_KERNEL_WORK_GROUP_SIZE, sizeof(size_t), &wgSize, NULL);
	BCSOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
	m_kernels[kernelId].m_Id = kernelId;
	m_kernels[kernelId].m_kernel = kernel;
	m_kernels[kernelId].m_name = pName;
	if(wgSize > 128) wgSize = 128;
	m_kernels[kernelId].m_workgroupSize = (int)wgSize;
	return;
}

void btBatchConstraintSolverOCL::setKernelArg(int kernelId, int argNum, int argSize, void* argPtr)
{
    cl_int ciErrNum;
	ciErrNum  = clSetKernelArg(m_kernels[kernelId].m_kernel, argNum, argSize, argPtr);
	BCSOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
}


void btBatchConstraintSolverOCL::runKernelWithWorkgroupSize(int kernelId, int globalSize)
{
	if(globalSize <= 0)
	{
		return;
	}
	cl_kernel kernelFunc = m_kernels[kernelId].m_kernel;
	cl_int ciErrNum = clSetKernelArg(kernelFunc, 0, sizeof(int), (void*)&globalSize);
	BCSOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
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
	BCSOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
	ciErrNum = clFlush(m_cqCommandQue);
	BCSOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
}


void btBatchConstraintSolverOCL::copyArrayToDevice(cl_mem device, const void* host, unsigned int size, int devOffs, int hostOffs)
{
	if(!size) return;
    cl_int ciErrNum;
	char* pHost = (char*)host + hostOffs;
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, device, CL_TRUE, devOffs, size, pHost, 0, NULL, NULL);
	BCSOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
}

void btBatchConstraintSolverOCL::copyArrayFromDevice(void* host, const cl_mem device, unsigned int size, int hostOffs, int devOffs)
{
	if(!size) return;
    cl_int ciErrNum;
	char* pHost = (char*)host + hostOffs;
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, device, CL_TRUE, devOffs, size, pHost, 0, NULL, NULL);
	BCSOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
}



btScalar btBatchConstraintSolverOCL::solveGroup(btCollisionObject** bodies,int numBodies,btPersistentManifold** manifoldPtr, int numManifolds,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer,btStackAlloc* stackAlloc,btDispatcher* dispatcher)
{
	BT_PROFILE("btBatchConstraintSolverOCL::solveGroup");
	btAssert(bodies);
	btAssert(numBodies);
	if(m_solverMode == BCSOCL_SOLVER_BULLET_SIMD)
	{
		return btSequentialImpulseConstraintSolver::solveGroup(	bodies, numBodies, 
																manifoldPtr, numManifolds, 
																constraints, numConstraints,
																infoGlobal, debugDrawer, stackAlloc, dispatcher);
	}
	int i;

	{
		BT_PROFILE("btBatchConstraintSolverOCL::Setup");
		solveGroupCacheFriendlySetup( bodies, numBodies, manifoldPtr,  numManifolds,constraints, numConstraints,infoGlobal,debugDrawer, stackAlloc);
	}
	int batchOffset = 0;
	int constraintOffset = 0;

	m_numBodies = m_tmpSolverBodyPool.size();
	m_numJointConstraints = m_tmpSolverNonContactConstraintPool.size();
	m_numContactConstraints = m_tmpSolverContactConstraintPool.size();
	m_numFrictionConstraints = m_tmpSolverContactFrictionConstraintPool.size();
	m_numConstraints = m_numJointConstraints + m_numContactConstraints + m_numFrictionConstraints;

#if MERGE_JOINTS_AND_CONTACTS
	int contOffs = m_numJointConstraints;
	m_numMergedBatches = prepareBatches(m_tmpSolverNonContactConstraintPool, batchOffset, constraintOffset, &m_tmpSolverContactConstraintPool, contOffs);
	batchOffset += m_numMergedBatches;
	constraintOffset += m_numJointConstraints + m_numContactConstraints;
#else
	m_numJointBatches    = prepareBatches(m_tmpSolverNonContactConstraintPool, batchOffset, constraintOffset);
	batchOffset += m_numJointBatches;
	constraintOffset += m_numJointConstraints;

	m_numContactBatches  = prepareBatches(m_tmpSolverContactConstraintPool, batchOffset, constraintOffset);
	batchOffset += m_numContactBatches;
	constraintOffset += m_numContactConstraints;
#endif

	m_numFrictionBatches = prepareBatches(m_tmpSolverContactFrictionConstraintPool, batchOffset, constraintOffset);

#if MERGE_JOINTS_AND_CONTACTS
	printf("(%4d,%4d) ", m_numJointConstraints + m_numContactConstraints, m_numMergedBatches);
#else
//	printf("(%4d,%4d) ", m_tmpSolverNonContactConstraintPool.size(), m_numJointBatches);
//	printf("(%4d,%4d) ", m_tmpSolverContactConstraintPool.size(), m_numContactBatches);
#endif
	printf("(%4d,%4d) ", m_tmpSolverContactFrictionConstraintPool.size(), m_numFrictionBatches);
	printf("\n");

#if 0
	solveGroupIterationsCPU(infoGlobal);
#else
	{
		BT_PROFILE("btBatchConstraintSolverOCL::Iterations");
		solveGroupIterationsGPU(infoGlobal);
	}
#endif

	int numPoolConstraints = m_tmpSolverContactConstraintPool.size();
	int j;

	for (j=0;j<numPoolConstraints;j++)
	{

		const btSolverConstraint& solveManifold = m_tmpSolverContactConstraintPool[j];
		btManifoldPoint* pt = (btManifoldPoint*) solveManifold.m_originalContactPoint;
		btAssert(pt);
		pt->m_appliedImpulse = solveManifold.m_appliedImpulse;
		if (infoGlobal.m_solverMode & SOLVER_USE_FRICTION_WARMSTARTING)
		{
			pt->m_appliedImpulseLateral1 = m_tmpSolverContactFrictionConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse;
			pt->m_appliedImpulseLateral2 = m_tmpSolverContactFrictionConstraintPool[solveManifold.m_frictionIndex+1].m_appliedImpulse;
		}

		//do a callback here?
	}

	if (infoGlobal.m_splitImpulse)
	{		
		for ( i=0;i<m_tmpSolverBodyPool.size();i++)
		{
			m_tmpSolverBodyPool[i].writebackVelocity(infoGlobal.m_timeStep);
		}
	} else
	{
		for ( i=0;i<m_tmpSolverBodyPool.size();i++)
		{
			m_tmpSolverBodyPool[i].writebackVelocity();
		}
	}


	m_tmpSolverBodyPool.resize(0);
	m_tmpSolverContactConstraintPool.resize(0);
	m_tmpSolverNonContactConstraintPool.resize(0);
	m_tmpSolverContactFrictionConstraintPool.resize(0);
	
//	m_hJointIdx.resize(0);
//	m_hContactIdx.resize(0);
//	m_hFrictionIdx.resize(0);

	return 0.f;
} 


int btBatchConstraintSolverOCL::prepareBatches(btConstraintArray& constraints, int batchOffset, int constraintOffset, btConstraintArray* pAuxConstr, int auxConstrOffs)
{
	BT_PROFILE("btBatchConstraintSolverOCL::prepareBatches");
	int numConstraints = constraints.size();
	int numConstrA = numConstraints;
	if(pAuxConstr != NULL)
	{
		numConstraints +=  pAuxConstr->size();
	}
	for(int n = 0; n < numConstraints; n++)
	{
		m_constrMarkers[n] = -1;
	}
	int nBatch, totalBatches = 0;
	int currConstrOffset = constraintOffset;
	for(nBatch = 0; nBatch < m_maxBatches; nBatch++)
	{
		bool lastBatch = (nBatch == (m_maxBatches - 1));
		int numInBatch = 0;
		for(int nBody = 0; nBody < m_numBodies; nBody++)
		{
			if(m_tmpSolverBodyPool[nBody].m_invMass[0] > 0.f)
			{
				m_bodyMarkers[nBody] = -1;
			}
			else
			{
				m_bodyMarkers[nBody] = -2;
			}
		}
		for(int nConstr = 0; nConstr < numConstraints; nConstr++)
		{
			if(m_constrMarkers[nConstr] >= 0)
			{
				continue;
			}
			btSolverConstraint& ct = (nConstr < numConstrA) ? constraints[nConstr] : (*pAuxConstr)[nConstr - numConstrA];
			
			if(!ct.m_numConsecutiveRowsPerKernel)
			{
				continue;
			}

			int bodyIdA = ct.m_solverBodyIdA;
			if((m_bodyMarkers[bodyIdA] >= 0) && (!lastBatch))
			{
				continue;
			}
			int bodyIdB = ct.m_solverBodyIdB;
			if((m_bodyMarkers[bodyIdB] >= 0) && (!lastBatch))
			{
				continue;
			}
			m_constrMarkers[nConstr] = nBatch;
			m_hConstraintIdx[currConstrOffset + numInBatch] = (nConstr < numConstrA) ? constraintOffset + nConstr : auxConstrOffs + nConstr - numConstrA;
			numInBatch++;
			if(m_bodyMarkers[bodyIdA] == -1)
			{
				m_bodyMarkers[bodyIdA] = 0;
			}
			if(m_bodyMarkers[bodyIdB] == -1)
			{
				m_bodyMarkers[bodyIdB] = 0;
			}
		}
		if(!numInBatch)
		{
			break;
		}
		m_batchData[(batchOffset + nBatch) * 2 + 0] = currConstrOffset;
		m_batchData[(batchOffset + nBatch) * 2 + 1] = numInBatch;
		currConstrOffset += numInBatch;
		totalBatches++;
	} // for nBatch
	return totalBatches;
}



void btBatchConstraintSolverOCL::solveGroupIterationsCPU(const btContactSolverInfo& infoGlobal)
{
	BT_PROFILE("btBatchConstraintSolverOCL::solveGroupIterations");
	for(int iteration = 0; iteration < infoGlobal.m_numIterations; iteration++)
	{			
		// solve all joint constraints
		int batchOffs = 0;
		int indexOffs = 0;
		solveBatchesCPU(m_tmpSolverNonContactConstraintPool, m_numJointBatches, batchOffs, indexOffs);
		batchOffs += m_numJointBatches;
		indexOffs += m_numJointConstraints;
		// solve contact constraints
		solveBatchesCPU(m_tmpSolverContactConstraintPool, m_numContactBatches, batchOffs, indexOffs);
		batchOffs += m_numContactBatches;
		indexOffs += m_numContactConstraints;
		// set limits for friction constraints
		int numFrictionPoolConstraints = m_tmpSolverContactFrictionConstraintPool.size();
		for(int j = 0; j < numFrictionPoolConstraints; j++)
		{
			btSolverConstraint& solveManifold = m_tmpSolverContactFrictionConstraintPool[m_orderFrictionConstraintPool[j]];
			btScalar totalImpulse = m_tmpSolverContactConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse;
			if (totalImpulse>btScalar(0))
			{
				solveManifold.m_lowerLimit = -(solveManifold.m_friction*totalImpulse);
				solveManifold.m_upperLimit = solveManifold.m_friction*totalImpulse;
			}
		}
		// solve all friction constraints
		solveBatchesLowLimCPU(m_tmpSolverContactFrictionConstraintPool, m_numFrictionBatches, batchOffs, indexOffs);
	}
	return;
}

void btBatchConstraintSolverOCL::solveBatchesCPU(btConstraintArray& constraints, int numBatches, int batchOffs, int indexOffs)
{
	for(int nBatch = 0; nBatch < numBatches; nBatch++)
	{
		int constrOffs = m_batchData[(batchOffs + nBatch) * 2 + 0];
		int numConstr  = m_batchData[(batchOffs + nBatch) * 2 + 1];
		for(int nConstr = 0; nConstr < numConstr; nConstr++)
		{
			int constrIdx = m_hConstraintIdx[constrOffs + nConstr] - indexOffs;
			int numRows = constraints[constrIdx].m_numConsecutiveRowsPerKernel;
			for(int nRow = 0; nRow < numRows; nRow++)
			{
				btSolverConstraint& ct = constraints[constrIdx + nRow];
				resolveSingleConstraintRowGeneric(m_tmpSolverBodyPool[ct.m_solverBodyIdA], m_tmpSolverBodyPool[ct.m_solverBodyIdB], ct);
			}
		}
	}
}

void btBatchConstraintSolverOCL::solveBatchesLowLimCPU(btConstraintArray& constraints, int numBatches, int batchOffs, int indexOffs)
{
	for(int nBatch = 0; nBatch < numBatches; nBatch++)
	{
		int constrOffs = m_batchData[(batchOffs + nBatch) * 2 + 0];
		int numConstr  = m_batchData[(batchOffs + nBatch) * 2 + 1];
		for(int nConstr = 0; nConstr < numConstr; nConstr++)
		{
			int constrIdx = m_hConstraintIdx[constrOffs + nConstr] - indexOffs;
			int numRows = constraints[constrIdx].m_numConsecutiveRowsPerKernel;
			for(int nRow = 0; nRow < numRows; nRow++)
			{
				btSolverConstraint& ct = constraints[constrIdx + nRow];
				resolveSingleConstraintRowLowerLimit(m_tmpSolverBodyPool[ct.m_solverBodyIdA], m_tmpSolverBodyPool[ct.m_solverBodyIdB], ct);
			}
		}
	}
}


void btBatchConstraintSolverOCL::solveBatchesGPU(int numBatches, int batchOffs)
{
	for(int nBatch = 0; nBatch < numBatches; nBatch++)
	{
		int constrOffs = m_batchData[(batchOffs + nBatch) * 2 + 0];
		int numConstr  = m_batchData[(batchOffs + nBatch) * 2 + 1];
		setKernelArg(BCSOCL_KERNEL_SOLVE_CONSTRAINT_ROW, 4, sizeof(int),(void*)&constrOffs);
		runKernelWithWorkgroupSize(BCSOCL_KERNEL_SOLVE_CONSTRAINT_ROW, numConstr);
	}
}

void btBatchConstraintSolverOCL::solveBatchesLowLimGPU(int numBatches, int batchOffs)
{
	for(int nBatch = 0; nBatch < numBatches; nBatch++)
	{
		int constrOffs = m_batchData[(batchOffs + nBatch) * 2 + 0];
		int numConstr  = m_batchData[(batchOffs + nBatch) * 2 + 1];
		setKernelArg(BCSOCL_KERNEL_SOLVE_CONSTR_ROW_LOW_LIM, 4, sizeof(int),(void*)&constrOffs);
		runKernelWithWorkgroupSize(BCSOCL_KERNEL_SOLVE_CONSTR_ROW_LOW_LIM, numConstr);
	}
}


void btBatchConstraintSolverOCL::solveGroupIterationsGPU(const btContactSolverInfo& infoGlobal)
{
	int memSize = m_numBodies * sizeof(btSolverBody);
	int devOffset;
	{
		BT_PROFILE("btBatchConstraintSolverOCL::Copying to GPU");
		copyArrayToDevice(m_dBodies, &(m_tmpSolverBodyPool[0]), memSize);
		memSize = m_numJointConstraints * sizeof(btSolverConstraint);
		copyArrayToDevice(m_dConstraints, &(m_tmpSolverNonContactConstraintPool[0]), memSize);
		devOffset = memSize;
		memSize = m_numContactConstraints * sizeof(btSolverConstraint);
		copyArrayToDevice(m_dConstraints, &(m_tmpSolverContactConstraintPool[0]), memSize, devOffset, 0);
		devOffset += memSize;
		memSize = m_numFrictionConstraints * sizeof(btSolverConstraint);
		copyArrayToDevice(m_dConstraints, &(m_tmpSolverContactFrictionConstraintPool[0]), memSize, devOffset, 0);
		int totalNumConstraints = m_numJointConstraints + m_numContactConstraints + m_numFrictionConstraints;
		memSize = totalNumConstraints * sizeof(int);
		copyArrayToDevice(m_dConstraintIdx, &(m_hConstraintIdx[0]), memSize);
	}
	for(int iteration = 0; iteration < infoGlobal.m_numIterations; iteration++)
	{
		// solve all non_friction constraints
		int batchOffs = 0;
#if MERGE_JOINTS_AND_CONTACTS
		{
			BT_PROFILE("btBatchConstraintSolverOCL::Solve non-friction");
			solveBatchesGPU(m_numMergedBatches, batchOffs);
		}
		batchOffs += m_numMergedBatches;
#else
		solveBatchesGPU(m_numJointBatches, batchOffs);
		batchOffs += m_numJointBatches;
		solveBatchesGPU(m_numContactBatches, batchOffs);
		batchOffs += m_numContactBatches;
#endif
		// set limits for friction constraints
		{
			BT_PROFILE("btBatchConstraintSolverOCL::setup friction");
			int frictOffs = m_numJointConstraints + m_numContactConstraints;
			setKernelArg(BCSOCL_KERNEL_SETUP_FRICTION_CONSTRAINT, 2, sizeof(int),(void*)&frictOffs);
			int contOffs = m_numJointConstraints;
			setKernelArg(BCSOCL_KERNEL_SETUP_FRICTION_CONSTRAINT, 3, sizeof(int),(void*)&contOffs);
			runKernelWithWorkgroupSize(BCSOCL_KERNEL_SETUP_FRICTION_CONSTRAINT, m_numFrictionConstraints);
		}
/*
		int numFrictionPoolConstraints = m_tmpSolverContactFrictionConstraintPool.size();
		for(int j = 0; j < numFrictionPoolConstraints; j++)
		{
			btSolverConstraint& solveManifold = m_tmpSolverContactFrictionConstraintPool[m_orderFrictionConstraintPool[j]];
			btScalar totalImpulse = m_tmpSolverContactConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse;
			if (totalImpulse>btScalar(0))
			{
				solveManifold.m_lowerLimit = -(solveManifold.m_friction*totalImpulse);
				solveManifold.m_upperLimit = solveManifold.m_friction*totalImpulse;
			}
		}
*/
		// solve all friction constraints
		{
			BT_PROFILE("btBatchConstraintSolverOCL::Solve friction");
			solveBatchesGPU(m_numFrictionBatches, batchOffs);
//			solveBatchesLowLimGPU(m_numFrictionBatches, batchOffs);
		}
	}
	{
		BT_PROFILE("btBatchConstraintSolverOCL::Copying from GPU");
		memSize = m_numBodies * sizeof(btSolverBody);
		copyArrayFromDevice(&(m_tmpSolverBodyPool[0]), m_dBodies, memSize);
		// read solver constraints back (needed for warmstarting)
		memSize = m_numJointConstraints * sizeof(btSolverConstraint);
		if(memSize)
		{
			copyArrayFromDevice(&(m_tmpSolverNonContactConstraintPool[0]), m_dConstraints, memSize);
		}
		devOffset = memSize;
		memSize = m_numContactConstraints * sizeof(btSolverConstraint);
		copyArrayFromDevice(&(m_tmpSolverContactConstraintPool[0]), m_dConstraints, memSize, 0, devOffset);
	}
}

#if 0
void btBatchConstraintSolverOCL::solveGroupIterationsCPU(const btContactSolverInfo& infoGlobal)
{
	BT_PROFILE("btBatchConstraintSolverOCL::solveGroupIterations");
//	int j;
	int numConstr;
	for(int iteration = 0; iteration < infoGlobal.m_numIterations; iteration++)
	{			
		// solve all joint constraints
		numConstr = m_tmpSolverNonContactConstraintPool.size();
		for(int i = 0; i < numConstr; i++)
		{
			btSolverConstraint& ct = m_tmpSolverNonContactConstraintPool[i];
			resolveSingleConstraintRowGeneric(m_tmpSolverBodyPool[ct.m_solverBodyIdA], m_tmpSolverBodyPool[ct.m_solverBodyIdB], ct);
		}
		numConstr = m_tmpSolverContactConstraintPool.size();
		for(int i = 0; i < numConstr; i++)
		{
			btSolverConstraint& ct = m_tmpSolverContactConstraintPool[i];
			resolveSingleConstraintRowGeneric(m_tmpSolverBodyPool[ct.m_solverBodyIdA], m_tmpSolverBodyPool[ct.m_solverBodyIdB], ct);
		}
		// set limits for friction constraints
		int numFrictionPoolConstraints = m_tmpSolverContactFrictionConstraintPool.size();
		for(int j = 0; j < numFrictionPoolConstraints; j++)
		{
			btSolverConstraint& solveManifold = m_tmpSolverContactFrictionConstraintPool[m_orderFrictionConstraintPool[j]];
			btScalar totalImpulse = m_tmpSolverContactConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse;
			if (totalImpulse>btScalar(0))
			{
				solveManifold.m_lowerLimit = -(solveManifold.m_friction*totalImpulse);
				solveManifold.m_upperLimit = solveManifold.m_friction*totalImpulse;
			}
		}
		// solve all friction constraints
		numConstr = m_tmpSolverContactFrictionConstraintPool.size();
		for(int i = 0; i < numConstr; i++)
		{
			btSolverConstraint& ct = m_tmpSolverContactFrictionConstraintPool[i];
			resolveSingleConstraintRowLowerLimit(m_tmpSolverBodyPool[ct.m_solverBodyIdA], m_tmpSolverBodyPool[ct.m_solverBodyIdB], ct);
		}
	}
	return;
}
#endif
