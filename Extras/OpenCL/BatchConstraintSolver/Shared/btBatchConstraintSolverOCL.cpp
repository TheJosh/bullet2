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
#include "btOclCommon.h"
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
	m_hConstraintsRO.resize(maxConstraints);
	m_hConstraintsRW.resize(maxConstraints);
	m_hConstraintsIndx.resize(maxConstraints);
	m_batchData.resize(maxBatches * 2 * 2, -1);
	m_bodyMarkers.resize(maxBodies, -1);
	m_constrMarkers.resize(maxConstraints, -1);
	m_contactReverseIndx.resize(maxConstraints);
	m_hBodies.resize(maxBodies);
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
#undef MINICL_DECLARE
#endif


void btBatchConstraintSolverOCL::initCL(cl_context context, cl_device_id device, cl_command_queue queue)
{

	#ifdef CL_PLATFORM_MINI_CL
		// call constructors here
		MINICL_REGISTER(kSolveConstraintRow)
		MINICL_REGISTER(kSetupFrictionConstraint)
	#endif

	cl_int ciErrNum;

	if(context != NULL)
	{
		m_cxMainContext = context;
	}
	else
	{
		// create a context 
		//m_cxMainContext = clCreateContextFromType(0, CL_DEVICE_TYPE_ALL, NULL, NULL, &ciErrNum);
		m_cxMainContext = btOclCommon::createContextFromType(CL_DEVICE_TYPE_ALL, &ciErrNum);
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
		char cBuildLog[655360];
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
	setKernelArg(BCSOCL_KERNEL_SOLVE_CONSTRAINT_ROW, 2, sizeof(cl_mem),(void*)&m_dConstraintsRO);
	setKernelArg(BCSOCL_KERNEL_SOLVE_CONSTRAINT_ROW, 3, sizeof(cl_mem),(void*)&m_dConstraintsRW);
	setKernelArg(BCSOCL_KERNEL_SOLVE_CONSTRAINT_ROW, 4, sizeof(cl_mem),(void*)&m_dConstraintsIndx);

	initKernel(BCSOCL_KERNEL_SETUP_FRICTION_CONSTRAINT,	"kSetupFrictionConstraint");
	setKernelArg(BCSOCL_KERNEL_SETUP_FRICTION_CONSTRAINT, 1, sizeof(cl_mem),(void*)&m_dConstraintsRO);
	setKernelArg(BCSOCL_KERNEL_SETUP_FRICTION_CONSTRAINT, 2, sizeof(cl_mem),(void*)&m_dConstraintsRW);
}


void btBatchConstraintSolverOCL::allocateBuffers()
{
    cl_int ciErrNum;
    unsigned int memSize;

	memSize = m_maxBodies * sizeof(btBCSOCLBody);
	m_dBodies = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
	BCSOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

	memSize = m_maxConstraints * sizeof(btBCSOCLConstrRO);
	m_dConstraintsRO = clCreateBuffer(m_cxMainContext, CL_MEM_READ_ONLY, memSize, NULL, &ciErrNum);
	BCSOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

	memSize = m_maxConstraints * sizeof(btBCSOCLConstrRW);
	m_dConstraintsRW = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
	BCSOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

	memSize = m_maxConstraints * sizeof(int);
	m_dConstraintsIndx = clCreateBuffer(m_cxMainContext, CL_MEM_READ_ONLY, memSize, NULL, &ciErrNum);
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

int btBatchConstraintSolverOCL::getOrInitSolverBody(btRigidBody* pBody)
{
	int id = pBody->getCompanionId();
	if(id >= 0)
	{
		return id;
	}
	btScalar invMass = pBody->getInvMass();
	if(invMass > btScalar(0))
	{
		id = m_numBodies;
		m_numBodies++;
		m_hBodies[id].m_deltaLinVel_invMass = pBody->internalGetDeltaLinearVelocity();
		m_hBodies[id].m_deltaLinVel_invMass[3] = invMass;
		m_hBodies[id].m_deltaAngVel_frict = pBody->internalGetDeltaAngularVelocity();
		m_hBodies[id].m_deltaAngVel_frict[3] = pBody->getFriction();
	}
	else
	{
		id = 0;
	}
	pBody->setCompanionId(id);
	m_solverBodies[id] = pBody;
	return id;
}

void btBatchConstraintSolverOCL::prepareSolverBodies(btPersistentManifold** manifoldPtr,int numManifolds,btTypedConstraint** constraints,int numConstraints)
{
	m_hBodies[0].m_deltaLinVel_invMass = btVector3(0, 0, 0);
	m_hBodies[0].m_deltaLinVel_invMass[3] = btScalar(0);
	m_hBodies[0].m_deltaAngVel_frict = btVector3(0, 0, 0);
	m_hBodies[0].m_deltaAngVel_frict[3] = btScalar(0);
	m_solverBodies[0] = &getFixedBody();
	m_numBodies = 1; // the first one is fixed body
	btPersistentManifold* manifold = 0;
	for(int i=0; i < numManifolds; i++)
	{
		manifold = manifoldPtr[i];
		btCollisionObject* colObj0=0,*colObj1=0;
		colObj0 = (btCollisionObject*)manifold->getBody0();
		colObj1 = (btCollisionObject*)manifold->getBody1();
		btRigidBody* bodyA = btRigidBody::upcast(colObj0);
		btRigidBody* bodyB = btRigidBody::upcast(colObj1);
		getOrInitSolverBody(bodyA);
		getOrInitSolverBody(bodyB);
	}
	btTypedConstraint* constraint = 0;
	for(int i=0; i < numConstraints; i++)
	{
		constraint = constraints[i];
		btRigidBody& bodyA = constraint->getRigidBodyA();
		btRigidBody& bodyB = constraint->getRigidBodyB();
		getOrInitSolverBody(&bodyA);
		getOrInitSolverBody(&bodyB);
	}
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
	
	{
		BT_PROFILE("btBatchConstraintSolverOCL::Setup");
		solveGroupCacheFriendlySetup( bodies, numBodies, manifoldPtr,  numManifolds,constraints, numConstraints,infoGlobal,debugDrawer, stackAlloc);
	}
	int batchOffset = 0;
	int constraintOffset = 0;

//	m_numBodies = m_tmpSolverBodyPool.size();
	m_numJointConstraints = m_tmpSolverNonContactConstraintPool.size();
	m_numContactConstraints = m_tmpSolverContactConstraintPool.size();
	m_numFrictionConstraints = m_tmpSolverContactFrictionConstraintPool.size();
	m_numConstraints = m_numJointConstraints + m_numContactConstraints + m_numFrictionConstraints;

/*
	for(int nBody = 0; nBody < m_numBodies; nBody++)
	{
		m_hBodies[nBody].m_deltaLinVel_invMass = m_tmpSolverBodyPool[nBody].m_deltaLinearVelocity;
		m_hBodies[nBody].m_deltaLinVel_invMass[3] = m_tmpSolverBodyPool[nBody].m_invMass[0];
		m_hBodies[nBody].m_deltaAngVel_frict = m_tmpSolverBodyPool[nBody].m_deltaAngularVelocity;
		m_hBodies[nBody].m_deltaAngVel_frict[3] = m_tmpSolverBodyPool[nBody].m_friction;
	}
*/
	{
		BT_PROFILE("btBatchConstraintSolverOCL::prepareSolverBodies");
		m_solverBodies.resize(numBodies);
		prepareSolverBodies(manifoldPtr, numManifolds, constraints, numConstraints);
	}

#if MERGE_JOINTS_AND_CONTACTS
	int contOffs = m_numJointConstraints;
	m_currIndxOffs = 0;
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
	m_frictionIndxStart = m_currIndxOffs;
	m_frictionConstrStart = constraintOffset;
	m_numFrictionBatches = prepareBatches(m_tmpSolverContactFrictionConstraintPool, batchOffset, constraintOffset);

#if MERGE_JOINTS_AND_CONTACTS
//	printf("(%4d,%4d) ", m_numJointConstraints + m_numContactConstraints, m_numMergedBatches);
#else
//	printf("(%4d,%4d) ", m_tmpSolverNonContactConstraintPool.size(), m_numJointBatches);
//	printf("(%4d,%4d) ", m_tmpSolverContactConstraintPool.size(), m_numContactBatches);
#endif
//	printf("(%4d,%4d) ", m_tmpSolverContactFrictionConstraintPool.size(), m_numFrictionBatches);
//	printf("\n");

	{
		BT_PROFILE("btBatchConstraintSolverOCL::Iterations");
		solveGroupIterationsGPU(infoGlobal);
	}

	for(int nBody = 0; nBody < m_numBodies; nBody++)
	{
		m_solverBodies[nBody]->internalGetDeltaLinearVelocity() = m_hBodies[nBody].m_deltaLinVel_invMass;
		m_solverBodies[nBody]->internalGetDeltaAngularVelocity() = m_hBodies[nBody].m_deltaAngVel_frict;
	}
	m_solverBodies.resize(0);

	solveGroupCacheFriendlyFinish(bodies, numBodies, manifoldPtr,  numManifolds,constraints, numConstraints,infoGlobal,debugDrawer, stackAlloc);

#if 0
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


	for(int nBody = 0; nBody < m_numBodies; nBody++)
	{
		m_tmpSolverBodyPool[nBody].m_deltaLinearVelocity = m_hBodies[nBody].m_deltaLinVel_invMass;
		m_tmpSolverBodyPool[nBody].m_deltaAngularVelocity = m_hBodies[nBody].m_deltaAngVel_frict;
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
#endif

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
		int numRowsInBatch = 0;
		for(int nBody = 0; nBody < m_numBodies; nBody++)
		{
//			if(m_tmpSolverBodyPool[nBody].m_invMass[0] > 0.f)
			if(m_solverBodies[nBody]->getInvMass() > 0.f)
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

//			int bodyIdA = ct.m_solverBodyIdA;
			int bodyIdA = ct.m_solverBodyA->getCompanionId();
			if((m_bodyMarkers[bodyIdA] >= 0) && (!lastBatch))
			{
				continue;
			}
//			int bodyIdB = ct.m_solverBodyIdB;
			int bodyIdB = ct.m_solverBodyB->getCompanionId();
			if((m_bodyMarkers[bodyIdB] >= 0) && (!lastBatch))
			{
				continue;
			}
			m_hConstraintsIndx[m_currIndxOffs + numInBatch] = currConstrOffset + numRowsInBatch;

			for(int nRow = 0; nRow < ct.m_numConsecutiveRowsPerKernel; nRow++)
			{
				btSolverConstraint& ct1 = (nConstr < numConstrA) ? constraints[nConstr + nRow] : (*pAuxConstr)[nConstr + nRow - numConstrA];
				m_constrMarkers[nConstr] = nBatch;
				if(nConstr >= numConstrA)
				{
					m_contactReverseIndx[nConstr - numConstrA] = currConstrOffset + numRowsInBatch;
				}
				btBCSOCLConstrRO& ctro =  m_hConstraintsRO[currConstrOffset + numRowsInBatch + nRow];
				ctro.m_relpos1CrossNormal_jacDiagABInv = ct1.m_relpos1CrossNormal;
				ctro.m_relpos1CrossNormal_jacDiagABInv[3] = ct1.m_jacDiagABInv;
				ctro.m_contactNormal_friction = ct1.m_contactNormal;
				ctro.m_contactNormal_friction[3] = ct1.m_friction;
				ctro.m_relpos2CrossNormal_rhs = ct1.m_relpos2CrossNormal;
				ctro.m_relpos2CrossNormal_rhs[3] = ct1.m_rhs;
				ctro.m_angularComponentA_cfm = ct1.m_angularComponentA;
				ctro.m_angularComponentA_cfm[3] = ct1.m_cfm;
				ctro.m_angularComponentB = ct1.m_angularComponentB;
				ctro.m_numConsecutiveRowsPerKernel = ct1.m_numConsecutiveRowsPerKernel;
				ctro.m_frictionIndex = ct1.m_frictionIndex;
//				ctro.m_solverBodyIdA = ct1.m_solverBodyIdA;
				ctro.m_solverBodyIdA = ct1.m_solverBodyA->getCompanionId();
//				ctro.m_solverBodyIdB = ct1.m_solverBodyIdB;
				ctro.m_solverBodyIdB = ct1.m_solverBodyB->getCompanionId();

				btBCSOCLConstrRW& ctrw =  m_hConstraintsRW[currConstrOffset + numRowsInBatch + nRow];
				ctrw.m_appliedImpulse = ct1.m_appliedImpulse;
				ctrw.m_lowerLimit = ct1.m_lowerLimit;
				ctrw.m_upperLimit = ct1.m_upperLimit;
				ctrw.m_pOrigData = &ct1;
			}
			numRowsInBatch += ct.m_numConsecutiveRowsPerKernel;
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
		m_batchData[(batchOffset + nBatch) * 2 + 0] = m_currIndxOffs;
		m_batchData[(batchOffset + nBatch) * 2 + 1] = numInBatch;
		currConstrOffset += numRowsInBatch;
		m_currIndxOffs += numInBatch;
		totalBatches++;
	} // for nBatch
	return totalBatches;
}

void btBatchConstraintSolverOCL::solveBatchesGPU(int numBatches, int batchOffs)
{
	for(int nBatch = 0; nBatch < numBatches; nBatch++)
	{
		int constrOffs = m_batchData[(batchOffs + nBatch) * 2 + 0];
		int numConstr  = m_batchData[(batchOffs + nBatch) * 2 + 1];
		setKernelArg(BCSOCL_KERNEL_SOLVE_CONSTRAINT_ROW, 5, sizeof(int),(void*)&constrOffs);
		runKernelWithWorkgroupSize(BCSOCL_KERNEL_SOLVE_CONSTRAINT_ROW, numConstr);
	}
}

void btBatchConstraintSolverOCL::solveGroupIterationsGPU(const btContactSolverInfo& infoGlobal)
{
	// fix the friction indices
	for(int nFrict = 0; nFrict < m_numFrictionConstraints; nFrict++)
	{
		int oldIndex = m_hConstraintsRO[m_frictionConstrStart + nFrict].m_frictionIndex;
		int newIndex = m_contactReverseIndx[oldIndex];
		m_hConstraintsRO[m_frictionConstrStart + nFrict].m_frictionIndex = newIndex;
	}
	int memSize = m_numBodies * sizeof(btBCSOCLBody);
	{
		BT_PROFILE("btBatchConstraintSolverOCL::Copying to GPU");
		copyArrayToDevice(m_dBodies, &(m_hBodies[0]), memSize);
		memSize = m_numConstraints * sizeof(btBCSOCLConstrRO);
		copyArrayToDevice(m_dConstraintsRO, &(m_hConstraintsRO[0]), memSize);
		memSize = m_numConstraints * sizeof(btBCSOCLConstrRW);
		copyArrayToDevice(m_dConstraintsRW, &(m_hConstraintsRW[0]), memSize);
		memSize = m_currIndxOffs * sizeof(int);
		copyArrayToDevice(m_dConstraintsIndx, &(m_hConstraintsIndx[0]), memSize);
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
			setKernelArg(BCSOCL_KERNEL_SETUP_FRICTION_CONSTRAINT, 3, sizeof(int),(void*)&frictOffs);
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
		}
	}
	{
		BT_PROFILE("btBatchConstraintSolverOCL::Copying from GPU");
		memSize = m_numBodies * sizeof(btBCSOCLBody);
		copyArrayFromDevice(&(m_hBodies[0]), m_dBodies, memSize);
		// read solver constraints back (needed for warmstarting)
		memSize = m_numConstraints * sizeof(btBCSOCLConstrRW);
		copyArrayFromDevice(&(m_hConstraintsRW[0]), m_dConstraintsRW, memSize);
		for(int nConstr = 0; nConstr < m_numConstraints; nConstr++)
		{
			btSolverConstraint* pCt = (btSolverConstraint*)m_hConstraintsRW[nConstr].m_pOrigData;
			pCt->m_appliedImpulse = m_hConstraintsRW[nConstr].m_appliedImpulse;
		}
	}
}

