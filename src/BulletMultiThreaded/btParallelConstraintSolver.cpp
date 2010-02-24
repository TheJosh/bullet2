/*
   Copyright (C) 2010 Sony Computer Entertainment Inc.
   All rights reserved.

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

#define USE_SEQUENTIAL_SOLVER 0
#define RUN_KERNELS_DIRECTLY 0

#include "btParallelConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include "LinearMath/btQuickprof.h"
#include "MiniCL/cl.h"

#define oclCHECKERROR(a, b) if((a)!=(b)) { printf("OCL Error : %d\n", (a)); btAssert((a) == (b)); }

static cl_context			s_cxMainContext;
static cl_device_id			s_cdDevice;
static cl_command_queue		s_cqCommandQue;
static cl_program			s_cpProgram;
static cl_kernel			s_setupContactKernel;
static cl_kernel			s_solveContactKernel;

btParallelConstraintSolver::btParallelConstraintSolver()
{
	//initialize MiniCL here
    cl_int ciErrNum;

	s_cxMainContext = clCreateContextFromType(0, CL_DEVICE_TYPE_ALL, NULL, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);

    size_t dataSzBytes;
    cl_device_id* clDevices;
    clGetContextInfo(s_cxMainContext, CL_CONTEXT_DEVICES, 0, NULL, &dataSzBytes);
    clDevices = (cl_device_id*) malloc(dataSzBytes);

    clGetContextInfo(s_cxMainContext, CL_CONTEXT_DEVICES, dataSzBytes, clDevices, NULL);

    s_cdDevice = clDevices[0];

	free(clDevices);

	// create a command-queue
	s_cqCommandQue = clCreateCommandQueue(s_cxMainContext, s_cdDevice, 0, &ciErrNum);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	size_t program_length = 0;
	s_cpProgram = clCreateProgramWithSource(s_cxMainContext, 1, NULL, &program_length, &ciErrNum);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

    // Create kernels
    s_setupContactKernel = clCreateKernel(s_cpProgram, "kSetupContact", &ciErrNum);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	size_t wgSize;
	ciErrNum = clGetKernelWorkGroupInfo(s_setupContactKernel, s_cdDevice, CL_KERNEL_WORK_GROUP_SIZE, sizeof(size_t), &wgSize, NULL);
	m_localGroupSize = wgSize; 

    s_solveContactKernel = clCreateKernel(s_cpProgram, "kSolveContact", &ciErrNum);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

}
	
btParallelConstraintSolver::~btParallelConstraintSolver()
{
	//exit MiniCL

}

struct btPairMarker
{
	int						m_bodyIdA;
	int						m_bodyIdB;
	int						m_batchId;
	btPersistentManifold*	m_pManifold;
	int						m_numPoints;
	int						m_threadId;
};

int btParallelConstraintSolver::prepareBatches(btPersistentManifold** manifoldPtr,int numManifolds, const btContactSolverInfo& infoGlobal)
{
	if(!numManifolds)
	{
		return 0;
	}
	btPersistentManifold* manifold = 0;
	int numSolverBodies = 1; // the first one is fixed body
	int numPairs = 0;
	int numContactConstraints = 0;
	int numFrictionConstraints = 0;
	m_numFrictonPerContact = (infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS) ? 2 : 1;
	btAlignedObjectArray<int> bodyMarkers;
	bodyMarkers.resize(1);
	bodyMarkers[0] = -1;
	int numDynDyn = 0;
	btAlignedObjectArray<btPairMarker> pairMarkers;
	for(int i=0; i < numManifolds; i++)
	{
		manifold = manifoldPtr[i];
		btCollisionObject* colObj0=0,*colObj1=0;
		colObj0 = (btCollisionObject*)manifold->getBody0();
		colObj1 = (btCollisionObject*)manifold->getBody1();
		btRigidBody* solverBodyA = btRigidBody::upcast(colObj0);
		btRigidBody* solverBodyB = btRigidBody::upcast(colObj1);
		int idA = solverBodyA->getCompanionId();
		int idB = solverBodyB->getCompanionId();
		if(idA < 0)
		{
			idA = (solverBodyA->getInvMass() > 0) ? numSolverBodies++ : 0;
			solverBodyA->setCompanionId(idA);
		}
		if(idB < 0)
		{
			idB = (solverBodyB->getInvMass() > 0) ? numSolverBodies++ : 0;
			solverBodyB->setCompanionId(idB);
		}
		if((!idA) && (!idB)) 
		{
			continue;
		}
		if(idA && idB) 
		{
			numDynDyn++;
		}
		int numPointsPerPair = 0;
		for (int j=0;j<manifold->getNumContacts();j++)
		{
			btManifoldPoint& cp = manifold->getContactPoint(j);
			if (cp.getDistance() <= manifold->getContactProcessingThreshold())
			{
				numPointsPerPair++;
				numContactConstraints++;
			}
		}
		if(numPointsPerPair)
		{
			btPairMarker& pairMark = pairMarkers.expand();
			pairMark.m_batchId = -1;
			pairMark.m_bodyIdA = idA;
			pairMark.m_bodyIdB = idB;
			pairMark.m_pManifold = manifold;
			pairMark.m_numPoints = numPointsPerPair;
		}
	}
	if(!numContactConstraints)
	{
		return 0;
	}
	bodyMarkers.resize(numSolverBodies);
	m_tmpSolverContactConstraintPool.resize(numContactConstraints);
	numFrictionConstraints = numContactConstraints * m_numFrictonPerContact;
	m_tmpSolverContactFrictionConstraintPool.resize(numFrictionConstraints);
	numPairs = pairMarkers.size();
	btAlignedObjectArray<int> pairsOrder;
	pairsOrder.resize(numPairs);
	int currIndex = 0;
	int pairsToDispatch = numPairs;
	int nBatch;
	for(nBatch = 0; pairsToDispatch; nBatch++)
	{
		for(int i = 1; i < numSolverBodies; i++)
		{
			bodyMarkers[i] = 0;
		}
		for(int nPair = 0; nPair < numPairs; nPair++)
		{
			btPairMarker& pairMark = pairMarkers[nPair];
			if(pairMark.m_batchId >= 0)
			{ // pair already dispatched
				continue;
			}
			if(pairMark.m_bodyIdA && bodyMarkers[pairMark.m_bodyIdA])
			{ // bodyA busy
				continue;
			}
			if(pairMark.m_bodyIdB && bodyMarkers[pairMark.m_bodyIdB])
			{ // bodyA busy
				continue;
			}
			// OK, dispatch this pair
			pairsOrder[currIndex] = nPair;
			currIndex++;
			pairMark.m_batchId = nBatch;
			if(pairMark.m_bodyIdA)
			{
				bodyMarkers[pairMark.m_bodyIdA]++;
			}
			if(pairMark.m_bodyIdB)
			{
				bodyMarkers[pairMark.m_bodyIdB]++;
			}
			pairsToDispatch--;
		}
	}
//	printf("Batches : %d\n", nBatch);
	// now dispatch pairs to threads
	for(int i = 0; i < m_localGroupSize; i++)
	{
		m_taskParams[i].m_numContactConstraints = 0;
	}
	for(int nOrd = 0; nOrd < numPairs; nOrd++)
	{
		int nPair = pairsOrder[nOrd];
		int nThread = nOrd % m_localGroupSize;
		btPairMarker& pairMark = pairMarkers[nPair];
		m_taskParams[nThread].m_numContactConstraints += pairMark.m_numPoints;
		pairMark.m_threadId = nThread;
	}
	m_taskParams[0].m_startIndex = 0;
	m_taskParams[0].m_currIndex = 0;
	for(int i = 1; i < m_localGroupSize; i++)
	{
		m_taskParams[i].m_startIndex = m_taskParams[i-1].m_startIndex + m_taskParams[i-1].m_numContactConstraints;
		m_taskParams[i].m_currIndex = m_taskParams[i].m_startIndex;
	}

	// now pre-fill solver constraints
	for(int i = 0; i < numContactConstraints; i++)
	{
		int frictIndex = i * m_numFrictonPerContact;
		m_tmpSolverContactConstraintPool[i].m_frictionIndex = frictIndex;
	}
	for(int i = 0; i < numFrictionConstraints; i++)
	{
		int frictIndex = i / m_numFrictonPerContact;
		m_tmpSolverContactFrictionConstraintPool[i].m_frictionIndex = frictIndex;
	}
	for(int nOrd = 0; nOrd < numPairs; nOrd++)
	{
		int nPair = pairsOrder[nOrd];
		btPairMarker& pairMark = pairMarkers[nPair];
		int nThread = pairMark.m_threadId;
		int contConstrInd = m_taskParams[nThread].m_currIndex;
		btPersistentManifold* pManifold = pairMark.m_pManifold;
		btRigidBody* rbA = (btRigidBody*)pManifold->getBody0();
		btRigidBody* rbB = (btRigidBody*)pManifold->getBody1();
		int numPoints = pManifold->getNumContacts();
		for(int nPoint = 0; nPoint < numPoints; nPoint++)
		{
			btManifoldPoint& cp = pManifold->getContactPoint(nPoint);
			if (cp.getDistance() <= pManifold->getContactProcessingThreshold())
			{
				m_tmpSolverContactConstraintPool[contConstrInd].m_solverBodyA = rbA;
				m_tmpSolverContactConstraintPool[contConstrInd].m_solverBodyB = rbB;
				m_tmpSolverContactConstraintPool[contConstrInd].m_originalContactPoint = &cp;
				contConstrInd++;
			}
		}
		m_taskParams[nThread].m_currIndex = contConstrInd;
	}
	// check
	for(int i = 0; i < m_localGroupSize; i++)
	{
		int numConstr = m_taskParams[i].m_currIndex - m_taskParams[i].m_startIndex;
		if(numConstr != m_taskParams[i].m_numContactConstraints)
		{
			printf("\nERROR : num constraints mismatch at thread %d : (%d : %d)\n", i, numConstr, m_taskParams[i].m_numContactConstraints);
		}
	} 
	return numContactConstraints;
}


//static int nCall = 0;
	
btScalar btParallelConstraintSolver::solveGroupCacheFriendlySetup(btCollisionObject** bodies,int numBodies,btPersistentManifold** manifoldPtr, int numManifolds,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer,btStackAlloc* stackAlloc)
{
	BT_PROFILE("solveGroupCacheFriendlySetup");
#if USE_SEQUENTIAL_SOLVER
	return btSequentialImpulseConstraintSolver::solveGroupCacheFriendlySetup(bodies, numBodies, manifoldPtr, numManifolds, constraints, numConstraints, infoGlobal, debugDrawer, stackAlloc);
#else
	(void)stackAlloc;
	(void)debugDrawer;
// nCall++;
// printf("Call : %d\n", nCall);
// if(nCall == 18)
// {
//	printf("=========  HIT : %d\n", nCall);
// }

	if (!(numConstraints + numManifolds))
	{
		//		printf("empty\n");
		return 0.f;
	}

	int numConstr;
	{
		BT_PROFILE("prepareBatches");
		numConstr = prepareBatches(manifoldPtr, numManifolds, infoGlobal);
	}

	if(!numConstr)
	{
		return 0.f;
	}

	{
		BT_PROFILE("runSetupKernel");

	#if RUN_KERNELS_DIRECTLY
		for(int i = 0; i < m_localGroupSize; i++)
		{
			kSetupContact(this, m_taskParams, (btContactSolverInfo*)&infoGlobal, i);
		}
	#else
		// Set the Argument values
		cl_int ciErrNum;
		btParallelConstraintSolver* pSolver = this;
		btParallelConstraintSolverSetupTaskParams* pTaskParams = m_taskParams;
		btContactSolverInfo* pInfoGlobal = (btContactSolverInfo*)&infoGlobal;
		ciErrNum  = clSetKernelArg(s_setupContactKernel, 0, sizeof(cl_mem), (void*)&pSolver);
		ciErrNum |= clSetKernelArg(s_setupContactKernel, 1, sizeof(cl_mem), (void*)&pTaskParams);
		ciErrNum |= clSetKernelArg(s_setupContactKernel, 2, sizeof(cl_mem), (void*)&pInfoGlobal);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		size_t szWorkSize[1];
		szWorkSize[0] = m_localGroupSize;
		ciErrNum = clEnqueueNDRangeKernel(s_cqCommandQue, s_setupContactKernel, 1, NULL, szWorkSize, szWorkSize, 0, NULL, NULL);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		clFlush(s_cqCommandQue);
	#endif
	}
#if 0
	{
		FILE* ff = fopen("C:\\ttt\\theadstat.txt", "wt");
		int totalCont = m_tmpSolverContactConstraintPool.size();
		int nRows = (totalCont  + m_localGroupSize - 1)/ m_localGroupSize;
		for(int n = 0; n < nRows; n++)
		{
			for(int i = 0; i < m_localGroupSize; i++)
			{
				int indx = m_taskParams[i].m_startIndex + n;
				if(indx < totalCont)
				{
					int curr = m_tmpSolverContactConstraintPool[indx].m_numConsecutiveRowsPerKernel;
					int delta;
					if(n)
					{
						delta = curr - m_tmpSolverContactConstraintPool[indx-1].m_numConsecutiveRowsPerKernel;
					}
					else delta = 0;
					fprintf(ff, "%10d(%6d)\t", curr, delta);
				}
			}
			fprintf(ff, "\n");
		}
		fclose(ff);
	}
#endif
	{
		BT_PROFILE("setOrder");

		///@todo: use stack allocator for such temporarily memory, same for solver bodies/constraints
		int numConstraintPool = m_tmpSolverContactConstraintPool.size();
		int numFrictionPool = m_tmpSolverContactFrictionConstraintPool.size();
		m_orderTmpConstraintPool.resize(numConstraintPool);
		m_orderFrictionConstraintPool.resize(numFrictionPool);
		{
			int i;
			for (i=0;i<numConstraintPool;i++)
			{
				m_orderTmpConstraintPool[i] = i;
			}
			for (i=0;i<numFrictionPool;i++)
			{
				m_orderFrictionConstraintPool[i] = i;
			}
		}
	}
	return 0.f;
#endif
}


btScalar btParallelConstraintSolver::solveGroupCacheFriendlyIterations(btCollisionObject** bodies,int numBodies,btPersistentManifold** manifoldPtr, int numManifolds,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer,btStackAlloc* stackAlloc)
{
	BT_PROFILE("runIterations");
#if USE_SEQUENTIAL_SOLVER
	return btSequentialImpulseConstraintSolver::solveGroupCacheFriendlyIterations(bodies, numBodies, manifoldPtr, numManifolds, constraints, numConstraints, infoGlobal, debugDrawer, stackAlloc);
#else
	int iteration;
	#if RUN_KERNELS_DIRECTLY
		for ( iteration = 0;iteration<infoGlobal.m_numIterations;iteration++)
		{
			for(int i = 0; i < m_localGroupSize; i++)
			{
				kSolveContact(this, m_taskParams, (btContactSolverInfo*)&infoGlobal, i);
			}
		}
	#else
		cl_int ciErrNum;
		btParallelConstraintSolver* pSolver = this;
		btParallelConstraintSolverSetupTaskParams* pTaskParams = m_taskParams;
		btContactSolverInfo* pInfoGlobal = (btContactSolverInfo*)&infoGlobal;
		ciErrNum  = clSetKernelArg(s_solveContactKernel, 0, sizeof(cl_mem), (void*)&pSolver);
		ciErrNum |= clSetKernelArg(s_solveContactKernel, 1, sizeof(cl_mem), (void*)&pTaskParams);
		ciErrNum |= clSetKernelArg(s_solveContactKernel, 2, sizeof(cl_mem), (void*)&pInfoGlobal);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		size_t szWorkSize[1];
		szWorkSize[0] = m_localGroupSize;
		for ( iteration = 0;iteration<infoGlobal.m_numIterations;iteration++)
		{
			ciErrNum = clEnqueueNDRangeKernel(s_cqCommandQue, s_solveContactKernel, 1, NULL, szWorkSize, szWorkSize, 0, NULL, NULL);
			oclCHECKERROR(ciErrNum, CL_SUCCESS);
			clFlush(s_cqCommandQue);
		}
	#endif
	return 0.f;
#endif
}

static btClock sClock;


#include <MiniCL/cl.h>
#define GUID_ARG ,int __guid_arg
#define GUID_ARG_VAL ,__guid_arg

void kSetupContact(btParallelConstraintSolver* pSolver, 
				  btParallelConstraintSolverSetupTaskParams* pParams, 
				  btContactSolverInfo* pInfoGlobal, int threadId)
{
	int numConstraints = pParams[threadId].m_numContactConstraints;
	unsigned long int timeStamp;
	int startIndex = pParams[threadId].m_startIndex;
	btContactSolverInfo& infoGlobal = *pInfoGlobal;
	for(int i = 0; i < numConstraints; i++)
	{
		timeStamp = sClock.getTimeMicroseconds();
		btSolverConstraint& solverConstraint = pSolver->m_tmpSolverContactConstraintPool[startIndex + i];
		solverConstraint.m_numConsecutiveRowsPerKernel = timeStamp;
		btCollisionObject* colObj0 = (btCollisionObject*)solverConstraint.m_solverBodyA;
		btCollisionObject* colObj1 = (btCollisionObject*)solverConstraint.m_solverBodyB;
		btRigidBody* solverBodyA = btRigidBody::upcast(colObj0);
		btRigidBody* solverBodyB = btRigidBody::upcast(colObj1);
		btManifoldPoint& cp = *((btManifoldPoint*)(solverConstraint.m_originalContactPoint));
		btVector3 rel_pos1;
		btVector3 rel_pos2;
		btScalar relaxation;
		btScalar rel_vel;
		btVector3 vel;
		pSolver->setupContactConstraint(solverConstraint, colObj0, colObj1, cp, infoGlobal, vel, rel_vel, relaxation, rel_pos1, rel_pos2);
		int currFrictIndex = solverConstraint.m_frictionIndex;
		if (!(infoGlobal.m_solverMode & SOLVER_ENABLE_FRICTION_DIRECTION_CACHING) || !cp.m_lateralFrictionInitialized)
		{
			cp.m_lateralFrictionDir1 = vel - cp.m_normalWorldOnB * rel_vel;
			btScalar lat_rel_vel = cp.m_lateralFrictionDir1.length2();
			if(!(infoGlobal.m_solverMode & SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION) && lat_rel_vel > SIMD_EPSILON)
			{
				cp.m_lateralFrictionDir1 /= btSqrt(lat_rel_vel);
				if((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS))
				{
					cp.m_lateralFrictionDir2 = cp.m_lateralFrictionDir1.cross(cp.m_normalWorldOnB);
					cp.m_lateralFrictionDir2.normalize();//??
					applyAnisotropicFriction(colObj0,cp.m_lateralFrictionDir2);
					applyAnisotropicFriction(colObj1,cp.m_lateralFrictionDir2);
					btSolverConstraint& frictionConstraint = pSolver->m_tmpSolverContactFrictionConstraintPool[currFrictIndex];
					currFrictIndex++;
					pSolver->setupFrictionConstraint(frictionConstraint, cp.m_lateralFrictionDir2,solverBodyA,solverBodyB,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
				}
				applyAnisotropicFriction(colObj0,cp.m_lateralFrictionDir1);
				applyAnisotropicFriction(colObj1,cp.m_lateralFrictionDir1);
				btSolverConstraint& frictionConstraint = pSolver->m_tmpSolverContactFrictionConstraintPool[currFrictIndex];
				currFrictIndex++;
				pSolver->setupFrictionConstraint(frictionConstraint, cp.m_lateralFrictionDir1,solverBodyA,solverBodyB,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
				cp.m_lateralFrictionInitialized = true;
			} 
			else
			{
				//re-calculate friction direction every frame, todo: check if this is really needed
				btPlaneSpace1(cp.m_normalWorldOnB,cp.m_lateralFrictionDir1,cp.m_lateralFrictionDir2);
				if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS))
				{
					applyAnisotropicFriction(colObj0,cp.m_lateralFrictionDir2);
					applyAnisotropicFriction(colObj1,cp.m_lateralFrictionDir2);
					btSolverConstraint& frictionConstraint = pSolver->m_tmpSolverContactFrictionConstraintPool[currFrictIndex];
					currFrictIndex++;
					pSolver->setupFrictionConstraint(frictionConstraint, cp.m_lateralFrictionDir2,solverBodyA,solverBodyB,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
				}
				applyAnisotropicFriction(colObj0,cp.m_lateralFrictionDir1);
				applyAnisotropicFriction(colObj1,cp.m_lateralFrictionDir1);
				btSolverConstraint& frictionConstraint = pSolver->m_tmpSolverContactFrictionConstraintPool[currFrictIndex];
				currFrictIndex++;
				pSolver->setupFrictionConstraint(frictionConstraint, cp.m_lateralFrictionDir1,solverBodyA,solverBodyB,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
				cp.m_lateralFrictionInitialized = true;
			}
		} 
		else
		{
			btSolverConstraint& frictionConstraint = pSolver->m_tmpSolverContactFrictionConstraintPool[currFrictIndex];
			currFrictIndex++;
			pSolver->setupFrictionConstraint(frictionConstraint, cp.m_lateralFrictionDir1,solverBodyA,solverBodyB,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation,cp.m_contactMotion1, cp.m_contactCFM1);
			if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS))
			{
				btSolverConstraint& frictionConstraint = pSolver->m_tmpSolverContactFrictionConstraintPool[currFrictIndex];
				currFrictIndex++;
				pSolver->setupFrictionConstraint(frictionConstraint, cp.m_lateralFrictionDir2,solverBodyA,solverBodyB,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation, cp.m_contactMotion2, cp.m_contactCFM2);
			}
		}
		pSolver->setFrictionConstraintImpulse( solverConstraint, solverBodyA, solverBodyB, cp, infoGlobal);
	}
}



void kSolveContact(	btParallelConstraintSolver* pSolver, 
					btParallelConstraintSolverSetupTaskParams* pParams, 
					btContactSolverInfo* pInfoGlobal, int threadId)
{
	int numConstraints = pParams[threadId].m_numContactConstraints;
	int startIndex = pParams[threadId].m_startIndex;
	btContactSolverInfo& infoGlobal = *pInfoGlobal;
	for(int i = 0; i < numConstraints; i++)
	{
		const btSolverConstraint& solveManifold = pSolver->m_tmpSolverContactConstraintPool[startIndex + i];
		pSolver->resolveSingleConstraintRowLowerLimitSIMD(*solveManifold.m_solverBodyA, *solveManifold.m_solverBodyB, solveManifold);
	}
	int numFrictionConstraints = numConstraints * pSolver->m_numFrictonPerContact;
	startIndex *= pSolver->m_numFrictonPerContact;
	for(int i = 0; i < numFrictionConstraints; i++)
	{
		btSolverConstraint& solveManifold = pSolver->m_tmpSolverContactFrictionConstraintPool[startIndex + i];
		btScalar totalImpulse = pSolver->m_tmpSolverContactConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse;
		if (totalImpulse>btScalar(0))
		{
			solveManifold.m_lowerLimit = -(solveManifold.m_friction*totalImpulse);
			solveManifold.m_upperLimit = solveManifold.m_friction*totalImpulse;
			pSolver->resolveSingleConstraintRowGenericSIMD(*solveManifold.m_solverBodyA, *solveManifold.m_solverBodyB,solveManifold);
		}
	}
}


MINICL_REGISTER(kSetupContact)
MINICL_REGISTER(kSolveContact)


