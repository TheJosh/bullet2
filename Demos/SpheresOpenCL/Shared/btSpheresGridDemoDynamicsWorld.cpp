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

#include <stdio.h>
#include <GL/glew.h>

#include "btOclUtils.h"

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include "LinearMath/btQuickprof.h"
#include "GlutStuff.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"

#include "btSpheresGridDemoDynamicsWorld.h"

btSpheresGridDemoDynamicsWorld::~btSpheresGridDemoDynamicsWorld()
{
}

int gStepNum = 0;

int	btSpheresGridDemoDynamicsWorld::stepSimulation( btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep)
{
	startProfiling(timeStep);
	m_timeStep = timeStep;
	BT_PROFILE("stepSimulation");
//	printf("Step : %d\n", gStepNum);
	{
		BT_PROFILE("Predict");
		runPredictUnconstrainedMotionKernel();
	}
	{
		BT_PROFILE("SetSpheres");
		runSetSpheresKernel();
	}
	{
		BT_PROFILE("SortHash");
		runSortHashKernel();
	}
	{
		BT_PROFILE("FindCellStart");
		runFindCellStartKernel();
	}
	{
		BT_PROFILE("BroadphaseCD");
		runBroadphaseCDKernel();
	}
	{
		BT_PROFILE("ScanPairs");
		runScanPairsKernel();
	}
	{
		BT_PROFILE("CompactPairs");
		runCompactPairsKernel();
	}
	{
		BT_PROFILE("SetupBatches");
		runSetupBatchesKernel();
	}
	{
		BT_PROFILE("SetupContacts");
		runSetupContactsKernel();
	}
	{
		BT_PROFILE("SolveConstraints");
		runSolveConstraintsKernel();
	}
	{
		BT_PROFILE("Integrate");
		runIntegrateTransformsKernel();
	}

	gStepNum++;

#ifndef BT_NO_PROFILE
	CProfileManager::Increment_Frame_Counter();
#endif //BT_NO_PROFILE
	return 1;
}


void btSpheresGridDemoDynamicsWorld::initDeviceData()
{
	getShapeData();
}



void btSpheresGridDemoDynamicsWorld::postInitDeviceData()
{
	m_numSpheres = m_hShapeBuf.size();
	m_numObjects = getNumCollisionObjects();
	m_hashSize = 1;
	for(int bit = 0; bit < 32; bit++)
	{
		if(m_hashSize >= m_numSpheres)
		{
			break;
		}
		m_hashSize <<= 1;
	}
	createVBO();
	allocateBuffers();
	adjustGrid();
	grabSimulationData();
}


void btSpheresGridDemoDynamicsWorld::getShapeData()
{
	int numObjects = getNumCollisionObjects();
	m_hShapeIds.resize(numObjects);
	btCollisionObjectArray& collisionObjects = getCollisionObjectArray();
	for(int i = 0; i < numObjects; i++)
	{
		btCollisionObject* colObj = collisionObjects[i];
		btCollisionShape* pShape = colObj->getCollisionShape();
		int shapeType = pShape->getShapeType();
		if(shapeType == MULTI_SPHERE_SHAPE_PROXYTYPE)
		{
			btMultiSphereShape* pMs = (btMultiSphereShape*)pShape;
			int numSpheres = pMs->getSphereCount();
			int start_idx = m_hShapeBuf.size();
			m_hShapeIds[i].x = start_idx;
			m_hShapeIds[i].y = numSpheres;
			for(int j = 0; j < numSpheres; j++)
			{
				btVector3 sphPos = pMs->getSpherePosition(j);
				float sphRad = pMs->getSphereRadius(j);
				sphPos.setW(sphRad);
				m_hShapeBuf.push_back(sphPos);
			}
		}
		else
		{
			btAssert(0);
		}
	}
	int totalSpheres = m_hShapeBuf.size();
	printf("total number of spheres : %d\n", totalSpheres);
}

void btSpheresGridDemoDynamicsWorld::allocateBuffers()
{
    cl_int ciErrNum;
	// positions of spheres
	m_hPos.resize(m_numSpheres);
    unsigned int memSize = sizeof(btVector3) *  m_numSpheres;
    m_dPos = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	// per object : transform, linear and angular velocity
	m_hTrans.resize(m_numObjects * 4);
	m_hLinVel.resize(m_numObjects);
	m_hAngVel.resize(m_numObjects);
	m_hInvInertiaMass.resize(m_numObjects * 3);
	m_hPosHash.resize(m_hashSize); 
	for(int i = 0; i < m_hashSize; i++) { m_hPosHash[i].x = 0x7FFFFFFF; m_hPosHash[i].y = 0; }
	memSize = m_numObjects * 4 * sizeof(btVector3);
    m_dTrans = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	memSize = m_numObjects * sizeof(btVector3);
    m_dLinVel = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
    m_dAngVel = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	// shape buffer and shape ids
	m_hBodyIds.resize(m_numSpheres);
	for(int i = 0; i < m_numObjects; i++)
	{
		int start = m_hShapeIds[i].x;
		int count = m_hShapeIds[i].y;
		for(int j = 0; j < count; j++)
		{
			m_hBodyIds[start + j] = i;
		}
	}
	memSize = m_numSpheres * sizeof(int);
	m_dBodyIds = clCreateBuffer(m_cxMainContext, CL_MEM_READ_ONLY, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dBodyIds, CL_TRUE, 0, memSize, &(m_hBodyIds[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	memSize = m_numSpheres * sizeof(btVector3);
	m_dShapeBuf = clCreateBuffer(m_cxMainContext, CL_MEM_READ_ONLY, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	memSize = m_numObjects * sizeof(btInt2);
	m_dShapeIds = clCreateBuffer(m_cxMainContext, CL_MEM_READ_ONLY, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	memSize = m_numObjects * 3 * sizeof(btVector3);
    m_dInvInertiaMass = clCreateBuffer(m_cxMainContext, CL_MEM_READ_ONLY, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	memSize = m_hashSize * sizeof(btInt2);
	m_dPosHash = clCreateBuffer(m_cxMainContext, CL_MEM_READ_ONLY, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	// pair buffer
	m_maxNeighbors = 12; // enough for 2D case
	m_hPairBuff.resize(m_numSpheres * m_maxNeighbors);
	m_hPairBuffStartCurr.resize(m_numSpheres + 1);
	for(int i = 0; i < m_numSpheres + 1; i++)
	{
		m_hPairBuffStartCurr[i].x = i * m_maxNeighbors;
		m_hPairBuffStartCurr[i].y = 0;
	}
	memSize = m_numSpheres * m_maxNeighbors * sizeof(int);
	m_dPairBuff = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	memSize = (m_numSpheres + 1) * sizeof(btInt2);
	m_dPairBuffStartCurr = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dPairBuffStartCurr, CL_TRUE, 0, memSize, &(m_hPairBuffStartCurr[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	m_hPairScan.resize(m_numSpheres);
	memSize = m_numSpheres * sizeof(int);
	m_dPairScan = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	// collision pairs
	m_maxPairs = m_numSpheres * 4;
	m_hPairIds.resize(m_maxPairs);
	memSize = m_maxPairs * sizeof(btPairId);
	m_dPairIds = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	m_maxBatches = SPHERES_GRID_MAX_BATCHES;
	m_hObjUsed.resize(m_numObjects);
	memSize = m_numObjects * sizeof(int);
	m_dObjUsed = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	// contact constraints
	m_hContacts.resize(m_maxPairs);
	memSize = m_maxPairs * sizeof(btSpheresContPair);
	m_dContacts = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	// global simulation parameters
	memSize = sizeof(btSimParams);
	m_dSimParams = clCreateBuffer(m_cxMainContext, CL_MEM_READ_ONLY, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
}

void btSpheresGridDemoDynamicsWorld::adjustGrid()
{
	btVector3 wmin( BT_LARGE_FLOAT,  BT_LARGE_FLOAT,  BT_LARGE_FLOAT);
	btVector3 wmax(-BT_LARGE_FLOAT, -BT_LARGE_FLOAT, -BT_LARGE_FLOAT);
	btCollisionObjectArray& collisionObjects = getCollisionObjectArray();
	for(int i = 0; i < m_numObjects; i++)
	{
		btCollisionObject* colObj = collisionObjects[i];
		btRigidBody* rb = btRigidBody::upcast(colObj);
		btVector3 boxMin, boxMax;
		rb->getAabb(boxMin, boxMax);
		wmin.setMin(boxMin);
		wmax.setMax(boxMax);
	}
	m_worldMin = wmin;
	m_worldMax = wmax;
	btScalar maxRad = -BT_LARGE_FLOAT;
	btScalar minRad =  BT_LARGE_FLOAT;
	for(int i = 0; i < m_numSpheres; i++)
	{
		btScalar rad = m_hShapeBuf[i][3];
		btSetMin(minRad, rad);
		btSetMax(maxRad, rad);
	}
	m_minSphereRad = minRad;
	m_maxSphereRad = maxRad;
	m_cellSize[0] = m_cellSize[1] = m_cellSize[2] = maxRad * btScalar(2.f);
	*((btVector3*)m_simParams.m_worldMin) = m_worldMin;
	*((btVector3*)m_simParams.m_cellSize) = m_cellSize;
	btVector3 wsize = m_worldMax - m_worldMin;
	m_simParams.m_gridSize[0] = (int)(wsize[0] / m_cellSize[0]);
	m_simParams.m_gridSize[1] = (int)(wsize[1] / m_cellSize[1]);
	m_simParams.m_gridSize[2] = (int)(wsize[2] / m_cellSize[2]);
	m_numGridCells = m_simParams.m_gridSize[0] * m_simParams.m_gridSize[1] * m_simParams.m_gridSize[2];
	m_hCellStart.resize(m_numGridCells);
    unsigned int memSize = sizeof(int) *  m_numGridCells;
    cl_int ciErrNum;
	m_dCellStart = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
}


void btSpheresGridDemoDynamicsWorld::grabSimulationData()
{
	btVector3 gravity = getGravity();
	*((btVector3*)m_simParams.m_gravity) = gravity;
	btCollisionObjectArray& collisionObjects = getCollisionObjectArray();
	for(int i = 0; i < m_numObjects; i++)
	{
		btCollisionObject* colObj = collisionObjects[i];
		btRigidBody* rb = btRigidBody::upcast(colObj);
		btVector3 v;
		const btTransform& tr = rb->getCenterOfMassTransform();
		m_hTrans[i * 4 + 0] = tr.getBasis().getColumn(0);
		m_hTrans[i * 4 + 1] = tr.getBasis().getColumn(1);
		m_hTrans[i * 4 + 2] = tr.getBasis().getColumn(2);
		m_hTrans[i * 4 + 3] = rb->getCenterOfMassPosition();
		m_hLinVel[i] = rb->getLinearVelocity();
		m_hAngVel[i] = rb->getAngularVelocity();
		m_hInvInertiaMass[i * 3 + 0] = rb->getInvInertiaTensorWorld().getRow(0);
		m_hInvInertiaMass[i * 3 + 1] = rb->getInvInertiaTensorWorld().getRow(1);
		m_hInvInertiaMass[i * 3 + 2] = rb->getInvInertiaTensorWorld().getRow(2);
		m_hInvInertiaMass[i * 3 + 0][3] = rb->getInvMass();
	}
	// copy to GPU
    cl_int ciErrNum;
	unsigned int memSize = sizeof(btVector3) * 4 * m_numObjects;
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dTrans, CL_TRUE, 0, memSize, &(m_hTrans[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	memSize = sizeof(btVector3) * m_numObjects;
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dLinVel, CL_TRUE, 0, memSize, &(m_hLinVel[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dAngVel, CL_TRUE, 0, memSize, &(m_hAngVel[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	memSize = sizeof(btVector3) * m_numSpheres;
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dShapeBuf, CL_TRUE, 0, memSize, &(m_hShapeBuf[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	memSize = sizeof(btInt2) * m_numObjects;
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dShapeIds, CL_TRUE, 0, memSize, &(m_hShapeIds[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	memSize = sizeof(btVector3) * 3 * m_numObjects;
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dInvInertiaMass, CL_TRUE, 0, memSize, &(m_hInvInertiaMass[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	memSize = sizeof(btSimParams);
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dSimParams, CL_TRUE, 0, memSize, &m_simParams, 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	memSize = m_hashSize * sizeof(btInt2);
	ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dPosHash, CL_TRUE, 0, memSize, &(m_hPosHash[0]), 0, NULL, NULL);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
}


void btSpheresGridDemoDynamicsWorld::createVBO()
{
    // create buffer object
    glGenBuffers(1, &m_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	// positions of spheres
    unsigned int memSize = sizeof(btVector3) *  m_numSpheres;
    glBufferData(GL_ARRAY_BUFFER, memSize, 0, GL_DYNAMIC_DRAW);
	// colors
	GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, memSize, 0, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    m_colVbo = vbo;
    // fill color buffer
    glBindBufferARB(GL_ARRAY_BUFFER, m_colVbo);
    float *data = (float*)glMapBufferARB(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    float *ptr = data;
    for(int i = 0; i < m_numSpheres; i++) {
        float t = i / (float)m_numSpheres;
		ptr[0] = 0.f;
		ptr[1] = 1.f;
		ptr[2] = 0.f;
        ptr+=3;
        *ptr++ = 1.0f;
    }
    glUnmapBufferARB(GL_ARRAY_BUFFER);
}



void btSpheresGridDemoDynamicsWorld::initCLKernels(int argc, char** argv)
{
    cl_int ciErrNum;

	// create the OpenCL context 
    m_cxMainContext = clCreateContextFromType(0, CL_DEVICE_TYPE_GPU, NULL, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
  
    // Get and log the device info
//    if(cutCheckCmdLineFlag(argc, (const char**)argv, "device"))
//	{
//		int device_nr = 0;
//		cutGetCmdLineArgumenti(argc, (const char**)argv, "device", &device_nr);
//		m_cdDevice = oclGetDev(m_cxMainContext, device_nr);
//		m_cdDevice = btOclGetDev(m_cxMainContext, device_nr);
//	}
//	else
	{
//		m_cdDevice = oclGetMaxFlopsDev(m_cxMainContext);
		m_cdDevice = btOclGetMaxFlopsDev(m_cxMainContext);
	}
//	oclPrintDevInfo(LOGBOTH, m_cdDevice);

	// create a command-queue
	m_cqCommandQue = clCreateCommandQueue(m_cxMainContext, m_cdDevice, 0, &ciErrNum);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	// Program Setup
#if defined CL_PLATFORM_NVIDIA
	size_t program_length;
//	char *source = oclLoadProgSource(".//Demos//SpheresGrid//SpheresGrid.cl", "", &program_length);
	char *source = btOclLoadProgSource(".//Demos//SpheresOpenCL//Shared//SpheresGrid.cl", "", &program_length);
//	oclCHECKERROR (source == NULL, oclFALSE);   
	btAssert(source != NULL);

	// create the program
	m_cpProgram = clCreateProgramWithSource(m_cxMainContext, 1, (const char**)&source, &program_length, &ciErrNum);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	free(source);

	// build the program
	ciErrNum = clBuildProgram(m_cpProgram, 0, NULL, "-I .", NULL, NULL);
	if(ciErrNum != CL_SUCCESS)
	{
		// write out standard error
//		oclLog(LOGBOTH | ERRORMSG, (double)ciErrNum, STDERROR);
		// write out the build log and ptx, then exit
		char cBuildLog[10240];
//		char* cPtx;
//		size_t szPtxLength;
		clGetProgramBuildInfo(m_cpProgram, btOclGetFirstDev(m_cxMainContext), CL_PROGRAM_BUILD_LOG, 
							  sizeof(cBuildLog), cBuildLog, NULL );
//		oclGetProgBinary(m_cpProgram, oclGetFirstDev(m_cxMainContext), &cPtx, &szPtxLength);
//		oclLog(LOGBOTH | CLOSELOG, 0.0, "\n\nLog:\n%s\n\n\n\n\nPtx:\n%s\n\n\n", cBuildLog, cPtx);
		printf("\n\n%s\n\n\n", cBuildLog);
		printf("Press ENTER key to terminate the program\n");
		getchar();
		exit(-1); 
	}
#elif defined(CL_PLATFORM_MINI_CL)
///create kernels from binary
	int numDevices = 1;
	::size_t* lengths = (::size_t*) malloc(numDevices * sizeof(::size_t));
	const unsigned char** images = (const unsigned char**) malloc(numDevices * sizeof(const void*));
	for(int i = 0; i < numDevices; ++i) {
		images[i] = 0;
		lengths[i] = 0;
	}
	cl_device_id cdDevices[2];
	cdDevices[0] = m_cdDevice;
	m_cpProgram = clCreateProgramWithBinary(m_cxMainContext, 1, cdDevices,lengths, images, 0, &ciErrNum);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
#endif

	// create the kernels
	m_ckSetSpheresKernel = clCreateKernel(m_cpProgram, "kSetSpheres", &ciErrNum);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	postInitDeviceData();

	// set the args values 
	ciErrNum |= clSetKernelArg(m_ckSetSpheresKernel, 0, sizeof(cl_mem), (void *) &m_dPos);
	ciErrNum |= clSetKernelArg(m_ckSetSpheresKernel, 1, sizeof(cl_mem), (void*) &m_dTrans);
	ciErrNum |= clSetKernelArg(m_ckSetSpheresKernel, 2, sizeof(cl_mem), (void*) &m_dShapeBuf);
	ciErrNum |= clSetKernelArg(m_ckSetSpheresKernel, 3, sizeof(cl_mem), (void*) &m_dBodyIds);
	ciErrNum |= clSetKernelArg(m_ckSetSpheresKernel, 4, sizeof(cl_mem), (void*) &m_dPosHash);
	ciErrNum |= clSetKernelArg(m_ckSetSpheresKernel, 5, sizeof(cl_mem), (void*) &m_dSimParams);
	ciErrNum |= clSetKernelArg(m_ckSetSpheresKernel, 6, sizeof(int), &m_numObjects);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	m_ckPredictUnconstrainedMotionKernel = clCreateKernel(m_cpProgram, "kPredictUnconstrainedMotion", &ciErrNum);
	ciErrNum |= clSetKernelArg(m_ckPredictUnconstrainedMotionKernel, 0, sizeof(cl_mem), (void *) &m_dLinVel);
	ciErrNum |= clSetKernelArg(m_ckPredictUnconstrainedMotionKernel, 1, sizeof(cl_mem), (void *) &m_dAngVel);
	ciErrNum |= clSetKernelArg(m_ckPredictUnconstrainedMotionKernel, 2, sizeof(cl_mem), (void *) &m_dSimParams);
	ciErrNum |= clSetKernelArg(m_ckPredictUnconstrainedMotionKernel, 3, sizeof(cl_mem), (void *) &m_dInvInertiaMass);
	ciErrNum |= clSetKernelArg(m_ckPredictUnconstrainedMotionKernel, 4, sizeof(int), &m_numObjects);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);


	m_ckIntegrateTransformsKernel = clCreateKernel(m_cpProgram, "kIntegrateTransforms", &ciErrNum);
	ciErrNum |= clSetKernelArg(m_ckIntegrateTransformsKernel, 0, sizeof(cl_mem), (void *) &m_dLinVel);
	ciErrNum |= clSetKernelArg(m_ckIntegrateTransformsKernel, 1, sizeof(cl_mem), (void *) &m_dAngVel);
	ciErrNum |= clSetKernelArg(m_ckIntegrateTransformsKernel, 2, sizeof(cl_mem), (void *) &m_dSimParams);
	ciErrNum |= clSetKernelArg(m_ckIntegrateTransformsKernel, 3, sizeof(cl_mem), (void*) &m_dTrans);
	ciErrNum |= clSetKernelArg(m_ckIntegrateTransformsKernel, 4, sizeof(cl_mem), (void *) &m_dInvInertiaMass);
	ciErrNum |= clSetKernelArg(m_ckIntegrateTransformsKernel, 5, sizeof(int), &m_numObjects);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);


	m_ckBroadphaseCDKernel = clCreateKernel(m_cpProgram, "kBroadphaseCD", &ciErrNum);
	ciErrNum |= clSetKernelArg(m_ckBroadphaseCDKernel, 0, sizeof(cl_mem), (void *) &m_dPos);
	ciErrNum |= clSetKernelArg(m_ckBroadphaseCDKernel, 1, sizeof(cl_mem), (void *) &m_dShapeBuf);
	ciErrNum |= clSetKernelArg(m_ckBroadphaseCDKernel, 2, sizeof(cl_mem), (void *) &m_dBodyIds);
	ciErrNum |= clSetKernelArg(m_ckBroadphaseCDKernel, 3, sizeof(cl_mem), (void *) &m_dPosHash);
	ciErrNum |= clSetKernelArg(m_ckBroadphaseCDKernel, 4, sizeof(cl_mem), (void *) &m_dCellStart);
	ciErrNum |= clSetKernelArg(m_ckBroadphaseCDKernel, 5, sizeof(cl_mem), (void *) &m_dPairBuff);
	ciErrNum |= clSetKernelArg(m_ckBroadphaseCDKernel, 6, sizeof(cl_mem), (void *) &m_dPairBuffStartCurr);
	ciErrNum |= clSetKernelArg(m_ckBroadphaseCDKernel, 7, sizeof(cl_mem), (void *) &m_dSimParams);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);


	m_ckInitObjUsageTabKernel = clCreateKernel(m_cpProgram, "kInitObjUsageTab", &ciErrNum);
	ciErrNum |= clSetKernelArg(m_ckInitObjUsageTabKernel, 0, sizeof(cl_mem), (void *) &m_dObjUsed);
	ciErrNum |= clSetKernelArg(m_ckInitObjUsageTabKernel, 1, sizeof(cl_mem), (void *) &m_dInvInertiaMass);
	ciErrNum |= clSetKernelArg(m_ckInitObjUsageTabKernel, 2, sizeof(cl_mem), (void *) &m_dSimParams);
	ciErrNum |= clSetKernelArg(m_ckInitObjUsageTabKernel, 3, sizeof(int), &m_numObjects);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	m_ckSetupBatchesKernel = clCreateKernel(m_cpProgram, "kSetupBatches", &ciErrNum);
	ciErrNum |= clSetKernelArg(m_ckSetupBatchesKernel, 0, sizeof(cl_mem), (void *) &m_dPairIds);
	ciErrNum |= clSetKernelArg(m_ckSetupBatchesKernel, 1, sizeof(cl_mem), (void *) &m_dObjUsed);
	ciErrNum |= clSetKernelArg(m_ckSetupBatchesKernel, 2, sizeof(cl_mem), (void *) &m_dSimParams);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	m_ckCheckBatchesKernel = clCreateKernel(m_cpProgram, "kCheckBatches", &ciErrNum);
	ciErrNum |= clSetKernelArg(m_ckCheckBatchesKernel, 0, sizeof(cl_mem), (void *) &m_dPairIds);
	ciErrNum |= clSetKernelArg(m_ckCheckBatchesKernel, 1, sizeof(cl_mem), (void *) &m_dObjUsed);
	ciErrNum |= clSetKernelArg(m_ckCheckBatchesKernel, 2, sizeof(cl_mem), (void *) &m_dSimParams);
	ciErrNum |= clSetKernelArg(m_ckCheckBatchesKernel, 3, sizeof(int), &m_maxBatches);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	m_ckSetupContactsKernel = clCreateKernel(m_cpProgram, "kSetupContacts", &ciErrNum);
	ciErrNum |= clSetKernelArg(m_ckSetupContactsKernel, 0, sizeof(cl_mem), (void *) &m_dPairIds);
	ciErrNum |= clSetKernelArg(m_ckSetupContactsKernel, 1, sizeof(cl_mem), (void *) &m_dPos);
	ciErrNum |= clSetKernelArg(m_ckSetupContactsKernel, 2, sizeof(cl_mem), (void *) &m_dShapeBuf);
	ciErrNum |= clSetKernelArg(m_ckSetupContactsKernel, 3, sizeof(cl_mem), (void *) &m_dContacts);
	ciErrNum |= clSetKernelArg(m_ckSetupContactsKernel, 4, sizeof(cl_mem), (void *) &m_dSimParams);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	m_ckSolveConstraintsKernel = clCreateKernel(m_cpProgram, "kSolveConstraints", &ciErrNum);
	ciErrNum |= clSetKernelArg(m_ckSolveConstraintsKernel, 0, sizeof(cl_mem), (void *) &m_dContacts);
	// parameter 1 will be set later (batchNum)
	ciErrNum |= clSetKernelArg(m_ckSolveConstraintsKernel, 2, sizeof(cl_mem), (void *) &m_dTrans);
	ciErrNum |= clSetKernelArg(m_ckSolveConstraintsKernel, 3, sizeof(cl_mem), (void *) &m_dPairIds);
	ciErrNum |= clSetKernelArg(m_ckSolveConstraintsKernel, 4, sizeof(cl_mem), (void *) &m_dLinVel);
	ciErrNum |= clSetKernelArg(m_ckSolveConstraintsKernel, 5, sizeof(cl_mem), (void *) &m_dAngVel);
	ciErrNum |= clSetKernelArg(m_ckSolveConstraintsKernel, 6, sizeof(cl_mem), (void *) &m_dInvInertiaMass);
	ciErrNum |= clSetKernelArg(m_ckSolveConstraintsKernel, 7, sizeof(cl_mem), (void *) &m_dSimParams);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	m_ckBitonicSortHashKernel = clCreateKernel(m_cpProgram, "kBitonicSortHash", &ciErrNum);
    ciErrNum |= clSetKernelArg(m_ckBitonicSortHashKernel, 0,      sizeof(cl_mem), (void *)&m_dPosHash);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

}

void btSpheresGridDemoDynamicsWorld::runSetSpheresKernel()
{
    cl_int ciErrNum;
    size_t szGlobalWorkSize[2];
    // Set work size and execute the kernel
    szGlobalWorkSize[0] = m_numSpheres;
    ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, m_ckSetSpheresKernel, 1, NULL, szGlobalWorkSize, NULL, 0,0,0 );
    oclCHECKERROR(ciErrNum, CL_SUCCESS);

// check
	int memSize = sizeof(btInt2) * m_hashSize;
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPosHash, CL_TRUE, 0, memSize, &(m_hPosHash[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);

	memSize = sizeof(float) * 4 * m_numSpheres;
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPos, CL_TRUE, 0, memSize, &(m_hPos[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);


    // Explicit Copy (until OpenGL interop will work)
    // map the PBO to copy data from the CL buffer via host
    glBindBufferARB(GL_ARRAY_BUFFER, m_vbo);    
    // map the buffer object into client's memory
    void* ptr = glMapBufferARB(GL_ARRAY_BUFFER, GL_WRITE_ONLY_ARB);
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPos, CL_TRUE, 0, sizeof(float) * 4 * m_numSpheres, ptr, 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
    glUnmapBufferARB(GL_ARRAY_BUFFER); 
}


void btSpheresGridDemoDynamicsWorld::runPredictUnconstrainedMotionKernel()
{
    cl_int ciErrNum;
#if 0
	// CPU version
	// get velocities from GPU
	int memSize = sizeof(btVector3) * m_numObjects;
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dLinVel, CL_TRUE, 0, memSize, &(m_hLinVel[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dAngVel, CL_TRUE, 0, memSize, &(m_hAngVel[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	// execute kernel
	for(int n = 0; n < m_numObjects; n++)
	{
		unsigned int index = n;
		btVector3 mass0 =	m_hInvInertiaMass[index * 3 + 0];
		if(mass0[3] > 0.f)
		{
			btVector3 linVel = m_hLinVel[index];
			btVector3 gravity = *((btVector3*)&m_simParams);
			linVel += gravity * m_timeStep;
			m_hLinVel[index] = linVel;
		}
	}
	// write back to GPU
	memSize = sizeof(btVector3) * m_numObjects;
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dLinVel, CL_TRUE, 0, memSize, &(m_hLinVel[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dAngVel, CL_TRUE, 0, memSize, &(m_hAngVel[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
#else
    size_t szGlobalWorkSize[2];
    // Set work size and execute the kernel
    szGlobalWorkSize[0] = m_numObjects;
	ciErrNum = clSetKernelArg(m_ckPredictUnconstrainedMotionKernel, 5, sizeof(float), &m_timeStep);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
    ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, m_ckPredictUnconstrainedMotionKernel, 1, NULL, szGlobalWorkSize, NULL, 0,0,0 );
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
#endif
}

void btSpheresGridDemoDynamicsWorld::runIntegrateTransformsKernel()
{
    cl_int ciErrNum;
    size_t szGlobalWorkSize[2];
    // Set work size and execute the kernel
    szGlobalWorkSize[0] = m_numObjects;
	ciErrNum = clSetKernelArg(m_ckIntegrateTransformsKernel, 6, sizeof(float), &m_timeStep);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
    ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, m_ckIntegrateTransformsKernel, 1, NULL, szGlobalWorkSize, NULL, 0,0,0 );
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
/*
	// read back for checking	
	int memSize = sizeof(btVector3) * 4 * m_numObjects;
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dTrans, CL_TRUE, 0, memSize, &(m_hTrans[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	memSize = sizeof(btVector3) * m_numObjects;
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPos, CL_TRUE, 0, memSize, &(m_hPos[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
*/
}

void btSpheresGridDemoDynamicsWorld::runSortHashKernel()
{
    cl_int ciErrNum;
	int memSize = m_numSpheres * sizeof(btInt2);
#if defined(CL_PLATFORM_MINI_CL)
	// bitonic sort on MiniCL does not work because barrier() finction is not implemented yet
	// CPU version
	// get hash from GPU
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPosHash, CL_TRUE, 0, memSize, &(m_hPosHash[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	// sort
	class btHashPosKey
	{
	public:
		unsigned int hash;
		unsigned int index;
		void quickSort(btHashPosKey* pData, int lo, int hi)
		{
			int i=lo, j=hi;
			btHashPosKey x = pData[(lo+hi)/2];
			do
			{    
				while(pData[i].hash < x.hash) i++; 
				while(x.hash < pData[j].hash) j--;
				if(i <= j)
				{
					btHashPosKey t = pData[i];
					pData[i] = pData[j];
					pData[j] = t;
					i++; j--;
				}
			} while(i <= j);
			if(lo < j) pData->quickSort(pData, lo, j);
			if(i < hi) pData->quickSort(pData, i, hi);
		}
		void bitonicSort(btHashPosKey* pData, int lo, int n, bool dir)
		{
			if(n > 1)
			{
				int m = n / 2;
				bitonicSort(pData, lo, m, !dir);
				bitonicSort(pData, lo + m, n - m, dir);
				bitonicMerge(pData, lo, n, dir);
			}
		}
		void bitonicMerge(btHashPosKey* pData, int lo, int n, bool dir)
		{
			if(n > 1)
			{
				int m = greatestPowerOfTwoLessThan(n);
				for(int i = lo; i < (lo + n - m); i++)
				{
					compare(pData, i, i + m, dir);
				}
				bitonicMerge(pData, lo, m, dir);
				bitonicMerge(pData, lo + m, n - m, dir);
			}
		}
		void compare(btHashPosKey* pData, int i, int j, bool dir)
		{
			if(dir == (pData[i].hash > pData[j].hash))
			{
				btHashPosKey t = pData[i];
				pData[i] = pData[j];
				pData[j] = t;
			}
		}
		int greatestPowerOfTwoLessThan(int n)
		{
			int k = 1;
			while(k < n)
			{
				k = k << 1;
			}
			return k>>1;
		}
	};
	btHashPosKey* pHash = (btHashPosKey*)(&m_hPosHash[0]);
//	pHash->quickSort(pHash, 0, m_numSpheres - 1);
	pHash->bitonicSort(pHash, 0, m_hashSize, true);
	// write back to GPU
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dPosHash, CL_TRUE, 0, memSize, &(m_hPosHash[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
#else
	{
		// this one sorts up to 1024 elements for now :-(
		size_t localWorkSize[2], globalWorkSize[2];
		int dir = 1;

		int numWorkItems = m_hashSize / 2;

		int workGroupSize  = (numWorkItems < m_workGroupSize) ? numWorkItems : m_workGroupSize;
		int numBatches = numWorkItems / workGroupSize;

		localWorkSize[0]  = workGroupSize;
		globalWorkSize[0] = workGroupSize;
		localWorkSize[1] = 1;
		globalWorkSize[1] = 1;

		ciErrNum  = clSetKernelArg(m_ckBitonicSortHashKernel, 1, sizeof(int), (void *)&numBatches);
		ciErrNum |= clSetKernelArg(m_ckBitonicSortHashKernel, 2, sizeof(int), (void *)&dir);
		oclCHECKERROR (ciErrNum, CL_SUCCESS);

		ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, m_ckBitonicSortHashKernel, 1, NULL, globalWorkSize, localWorkSize, 0, NULL, NULL);
		oclCHECKERROR (ciErrNum, CL_SUCCESS);
		// read results from GPU
		ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPosHash, CL_TRUE, 0, memSize, &(m_hPosHash[0]), 0, NULL, NULL);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	}
#endif
#if 0
	// check order
	for(int i = 1; i < m_hashSize; i++)
	{
		if(m_hPosHash[i-1].x > m_hPosHash[i].x)
		{
			printf("Hash sort error at %d\n", i);
		}
	}
#endif
}


void btSpheresGridDemoDynamicsWorld::runFindCellStartKernel()
{
    cl_int ciErrNum;
#if 1
	// CPU version
	// get hash from GPU
	int memSize = m_numSpheres * sizeof(btInt2);
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPosHash, CL_TRUE, 0, memSize, &(m_hPosHash[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	// clear cells
	for(int i = 0; i < m_numGridCells; i++)
	{
		m_hCellStart[i] = -1;
	}
	// find start of each cell in sorted hash
	btInt2 hash = m_hPosHash[0];
	m_hCellStart[hash.x] = 0;
	for(int i = 1; i < m_numSpheres; i++)
	{
		if(m_hPosHash[i-1].x != m_hPosHash[i].x)
		{
			m_hCellStart[m_hPosHash[i].x] = i;
		}
	}
	// write back to GPU
	memSize = m_numGridCells * sizeof(int);
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dCellStart, CL_TRUE, 0, memSize, &(m_hCellStart[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
#else
#endif
}


void btSpheresGridDemoDynamicsWorld::runBroadphaseCDKernel()
{
    cl_int ciErrNum;
#if 0
	// CPU version : TODO
#else
    size_t szGlobalWorkSize[2];
    // Set work size and execute the kernel
    szGlobalWorkSize[0] = m_numSpheres;

//	int memSize = m_numSpheres * sizeof(btInt2);
//	ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPairBuffStartCurr, CL_TRUE, 0, memSize, &(m_hPairBuffStartCurr[0]), 0, NULL, NULL);

    ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, m_ckBroadphaseCDKernel, 1, NULL, szGlobalWorkSize, NULL, 0,0,0 );
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

//	memSize = m_numSpheres * sizeof(btInt2);
//	ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPairBuffStartCurr, CL_TRUE, 0, memSize, &(m_hPairBuffStartCurr[0]), 0, NULL, NULL);

	#if 0 
		// now check : copy the pair buffer to CPU
		int memSize = m_numSpheres * m_maxNeighbors * sizeof(int);
		ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPairBuff, CL_TRUE, 0, memSize, &(m_hPairBuff[0]), 0, NULL, NULL);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		int numPairs = 0;
		memSize = (m_numSpheres - 1) * m_maxNeighbors;
		for(int i = 0; i < memSize; i++)
		{
			if(m_hPairBuff[i] > (m_numSpheres - 1))
			{
				printf("Pair check error at %d\n", i);
			}
			if(m_hPairBuff[i] >= 0)
			{
				numPairs++;
			}
		}
		printf("pairs : %d\n", numPairs);
	#endif
#endif
}

void btSpheresGridDemoDynamicsWorld::runScanPairsKernel()
{
    cl_int ciErrNum;
#if 1
	// CPU version
	// get data from GPU
	int memSize = m_numSpheres * sizeof(btInt2);
	ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPairBuffStartCurr, CL_TRUE, 0, memSize, &(m_hPairBuffStartCurr[0]), 0, NULL, NULL);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	// do scan
	m_hPairScan[0] = 0;
	for(int i = 1; i < m_numSpheres; i++)
	{
		int count_prev = m_hPairBuffStartCurr[i - 1].y;
		m_hPairScan[i] = m_hPairScan[i-1] + count_prev;
	}
	m_numPairs = m_hPairScan[m_numSpheres - 1];
	// write back to GPU
	memSize = m_numSpheres * sizeof(int);
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dPairScan, CL_TRUE, 0, memSize, &(m_hPairScan[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
#else
#endif
}


void btSpheresGridDemoDynamicsWorld::runCompactPairsKernel()
{
    cl_int ciErrNum;
#if 1
	// CPU version
	// get data from GPU
	int memSize = m_numSpheres * sizeof(btInt2);
	ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPairBuffStartCurr, CL_TRUE, 0, memSize, &(m_hPairBuffStartCurr[0]), 0, NULL, NULL);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	memSize = m_numSpheres * sizeof(int);
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPairScan, CL_TRUE, 0, memSize, &(m_hPairScan[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	memSize = m_numSpheres * m_maxNeighbors * sizeof(int);
	ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPairBuff, CL_TRUE, 0, memSize, &(m_hPairBuff[0]), 0, NULL, NULL);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	int numPairs = 0;
	for(int i = 0; i < m_numSpheres; i++)
	{
		int count = m_hPairBuffStartCurr[i].y;
		int inpStart = m_hPairBuffStartCurr[i].x;
		int outStart = m_hPairScan[i];
		int	objIdA = m_hBodyIds[i];
		for(int j = 0; j < count; j++)
		{
			int sphereIdB = m_hPairBuff[inpStart + j];
			int objIdB = m_hBodyIds[sphereIdB];
			btPairId pairId;
			pairId.m_objA = objIdA;
			pairId.m_sphA = i;
			pairId.m_objB = objIdB;
			pairId.m_sphB = sphereIdB;
//			pairId.m_pair = numPairs;
			pairId.m_pair = 0;
			pairId.m_batch = -1;
			m_hPairIds[outStart + j] = pairId;
			numPairs++;
		}
	}
	if(numPairs != m_numPairs)
	{
		printf("ERROR : number of pairs mismatch %d : %d\n",numPairs, m_numPairs);
	}
	// write pair ids back to GPU
	memSize = m_numPairs * sizeof(btPairId);
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dPairIds, CL_TRUE, 0, memSize, &(m_hPairIds[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
#else
#endif
}



#define BT_PROFILE_SETUP_BATCHES(__arg__) BT_PROFILE(__arg__)
//#define BT_PROFILE_SETUP_BATCHES(__arg__) 

void btSpheresGridDemoDynamicsWorld::runSetupBatchesKernel()
{
    cl_int ciErrNum;
	m_numBatches = 0;
#if 0
	// CPU version
	int memSize = m_numPairs * sizeof(btPairId);
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPairIds, CL_TRUE, 0, memSize, &(m_hPairIds[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	for(int n_batch = 0; n_batch < m_maxBatches; n_batch++)
	{
		bool lastBatch = (n_batch == (m_maxBatches - 1));
		for(int i = 0; i < m_numObjects; i++)
		{
			btVector3 mass0 =	m_hInvInertiaMass[i * 3 + 0];
			if(mass0[3] > 0.f)
			{
				m_hObjUsed[i] = -1;
			}
			else
			{
				m_hObjUsed[i] = -2;
			}
		}
		for(int i = 0; i < m_numPairs; i++)
		{
			btPairId pairId = m_hPairIds[i];

			if((pairId.m_batch < 0)
			 &&(m_hObjUsed[pairId.m_objA] < 0)
			 &&(m_hObjUsed[pairId.m_objB] < 0))
			{
				m_hPairIds[i].m_batch = n_batch;
				m_numBatches = n_batch + 1;
				if((m_hObjUsed[pairId.m_objA] == -1) && !lastBatch) 
				{
					m_hObjUsed[pairId.m_objA] = i;
				}
				if((m_hObjUsed[pairId.m_objB] == -1) && !lastBatch) 
				{
					m_hObjUsed[pairId.m_objB] = i;
				}
			}
		}
	}
	// write results back to GPU
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dPairIds, CL_TRUE, 0, memSize, &(m_hPairIds[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
#else
    size_t szGlobalWorkSize[2];
//  debug breakpoint
//	if(m_numPairs)
//	{
//		ciErrNum = 0;
//	}
	for(int nBatch = 0; nBatch < m_maxBatches; nBatch++)
	{
		{
			BT_PROFILE_SETUP_BATCHES("Init");
			szGlobalWorkSize[0] = m_numObjects;
			ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, m_ckInitObjUsageTabKernel, 1, NULL, szGlobalWorkSize, NULL, 0,0,0 );
			oclCHECKERROR(ciErrNum, CL_SUCCESS);
		}
		{
			BT_PROFILE_SETUP_BATCHES("Setup");
			szGlobalWorkSize[0] = m_numPairs;
			ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, m_ckSetupBatchesKernel, 1, NULL, szGlobalWorkSize, NULL, 0,0,0 );
			oclCHECKERROR(ciErrNum, CL_SUCCESS);
		}
		{
			BT_PROFILE_SETUP_BATCHES("Check");
			ciErrNum = clSetKernelArg(m_ckCheckBatchesKernel, 4, sizeof(int), &nBatch);
			oclCHECKERROR(ciErrNum, CL_SUCCESS);
			ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, m_ckCheckBatchesKernel, 1, NULL, szGlobalWorkSize, NULL, 0,0,0 );
			oclCHECKERROR(ciErrNum, CL_SUCCESS);
		}
	}
	{
		BT_PROFILE_SETUP_BATCHES("Read");
		// read results from GPU
		int memSize = m_numPairs * sizeof(btPairId);
		ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPairIds, CL_TRUE, 0, memSize, &(m_hPairIds[0]), 0, NULL, NULL);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	}
#endif
#if 1
	{
		BT_PROFILE_SETUP_BATCHES("Validate");
		// full check
		int numShared = 0;
		for(int nBatch = 0; nBatch < m_maxBatches; nBatch++)
		{
			bool err = false;
			// clean objUsage table
			for(int i = 0; i < m_numObjects; i++)
			{
				btScalar invMass = m_hInvInertiaMass[i * 3 + 0][3];
				if(invMass > 0.f)
				{
					m_hObjUsed[i] = 0;
				}
				else
				{
					m_hObjUsed[i] = -2;
				}
			}
			// now count object usage for current batch
			for(int i = 0; i < m_numPairs; i++)
			{
				int ojbIdA = m_hPairIds[i].m_objA;
				int ojbIdB = m_hPairIds[i].m_objB;
				int batchId = m_hPairIds[i].m_batch;
				if((batchId < 0) && (!nBatch)) // print only once
				{
					printf("ERROR : pair %d is not dispatched\n", i);
					continue;
				}
				if(batchId != nBatch)
				{
					continue;
				}
				if(m_hObjUsed[ojbIdA] >= 0)
				{
					m_hObjUsed[ojbIdA]++;
				}
				if(m_hObjUsed[ojbIdB] >= 0)
				{
					m_hObjUsed[ojbIdB]++;
				}
			}
			// now see how many pairs use share objects
			for(int i = 0; i < m_numObjects; i++)
			{
				if(m_hObjUsed[i] > 1)
				{
					if(nBatch == (m_maxBatches - 1))
					{
						numShared++;
					}
					else
					{
						printf("ERROR : object %d is dispatched %d times for batch %d\n", i, m_hObjUsed[i], nBatch);
					}
				}
			}
		}
		if(numShared)
		{
			//printf("Shared in last batch : %d\n", numShared); // annoying output
		}
	}
#endif
	{
		// paint contactions spheres in red
		BT_PROFILE_SETUP_BATCHES("Color contacts");
		int numLeft = 0;
		glBindBufferARB(GL_ARRAY_BUFFER, m_colVbo);
		float *data = (float*)glMapBufferARB(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
		float *ptr = data;
		for(int i = 0; i < m_numSpheres; i++)
		{
			int bodyId = m_hBodyIds[i];
			btScalar invMass = m_hInvInertiaMass[bodyId * 3 + 0][3];
			if(invMass > 0.f)
			{
				ptr[0] = 0.f;
				ptr[1] = 1.f;
				ptr[2] = 0.f;
			}
			else
			{
				ptr[0] = 0.f;
				ptr[1] = 0.f;
				ptr[2] = 1.f;
			}
			ptr+=3;
			*ptr++ = 1.0f;
		}
		for(int i = 0; i < m_numPairs; i++)
		{
			//if(m_hPairIds[i].m_batch < 0) 
			//{
			//	m_hPairIds[i].m_batch = m_numBatches - 1;
			//	numLeft++;
			//}
			float* ptr = data + m_hPairIds[i].m_sphA * 4;
			ptr[0] = 1.f;
			ptr[1] = 0.f;
			ptr[2] = 0.f;
			ptr = data + m_hPairIds[i].m_sphB * 4;
			ptr[0] = 1.f;
			ptr[1] = 0.f;
			ptr[2] = 0.f;

		}
		glUnmapBufferARB(GL_ARRAY_BUFFER);
		// printf("pairs : %4d, batches : %d, left : %d\n", m_numPairs, m_numBatches, numLeft);
	}
}


void btSpheresGridDemoDynamicsWorld::runSetupContactsKernel()
{
    cl_int ciErrNum;
#if 0
	// CPU version
	// get pair IDs ...
	int memSize = m_numPairs * sizeof(btPairId);
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPairIds, CL_TRUE, 0, memSize, &(m_hPairIds[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	// ... and sphere positions
	memSize = sizeof(float) * 4 * m_numSpheres;
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPos, CL_TRUE, 0, memSize, &(m_hPos[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	// compute contacts
	for(int i = 0; i < m_numPairs; i++)
	{
		int sphIdA = m_hPairIds[i].m_sphA;
		int sphIdB = m_hPairIds[i].m_sphB;
		btVector3 posA = m_hPos[sphIdA];
		btVector3 posB = m_hPos[sphIdB];
		btScalar radA = m_hShapeBuf[sphIdA][3];
		btScalar radB = m_hShapeBuf[sphIdB][3];
		btVector3 del = posB - posA;
		btScalar dist = del.dot(del);
		dist = btSqrt(dist);
		btScalar maxD = radA + radB;
		if(dist > maxD)
		{ // should never happen
			printf("ERROR : no collision for pair %d\n", i);
			continue;
		}
		btScalar penetration = maxD - dist;
		btVector3 normal;
		if(dist > btScalar(0.f)) 
		{
			btScalar fact = btScalar(-1.0f) / dist;
			normal = del * fact; 
		}
		else
		{	
			normal.setValue(1.f, 0.f, 0.f);
		}
		btVector3 tmp = (normal * radA);
		btVector3 contact = posA - tmp;
		contact.setW(penetration);
		normal.setW(0);
		m_hContacts[i].m_contact = contact;
		m_hContacts[i].m_normal = normal;
	}
	// write back to GPU
	memSize = m_numPairs * sizeof(btSpheresContPair);
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dContacts, CL_TRUE, 0, memSize, &(m_hContacts[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
#else
    size_t szGlobalWorkSize[2];
    // Set work size and execute the kernel
    szGlobalWorkSize[0] = m_numPairs;
    ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, m_ckSetupContactsKernel, 1, NULL, szGlobalWorkSize, NULL, 0,0,0 );
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	// read results from GPU
	int memSize = m_numPairs * sizeof(btSpheresContPair);
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dContacts, CL_TRUE, 0, memSize, &(m_hContacts[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
#endif
}

void btSpheresGridDemoDynamicsWorld::runSolveConstraintsKernel()
{
    cl_int ciErrNum;
#if 0
	// CPU version
	int memSize = m_numPairs * sizeof(btSpheresContPair);
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dContacts, CL_TRUE, 0, memSize, &(m_hContacts[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	// get velocities from GPU
	memSize = sizeof(btVector3) * m_numObjects;
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dLinVel, CL_TRUE, 0, memSize, &(m_hLinVel[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dAngVel, CL_TRUE, 0, memSize, &(m_hAngVel[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	memSize = sizeof(btVector3) * 4 * m_numObjects;
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dTrans, CL_TRUE, 0, memSize, &(m_hTrans[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	for(int nIter = 0; nIter < m_numSolverIterations; nIter++)
	{
		for(int nBatch = 0; nBatch < m_numBatches; nBatch++)
		{
			btSpheresContPair* pPairs = &(m_hContacts[0]);
			for(int nPair = 0; nPair < m_numPairs; nPair++)
			{
				solvePairCPU(pPairs + nPair, nPair, nBatch);
			}
		}
	}
	// write back to GPU
	memSize = sizeof(btVector3) * m_numObjects;
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dLinVel, CL_TRUE, 0, memSize, &(m_hLinVel[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dAngVel, CL_TRUE, 0, memSize, &(m_hAngVel[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
#else
	ciErrNum = clSetKernelArg(m_ckSolveConstraintsKernel, 8, sizeof(float), &m_timeStep);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	for(int nIter = 0; nIter < m_numSolverIterations; nIter++)
	{
		for(int nBatch = 0; nBatch < m_maxBatches; nBatch++)
		{
			size_t szGlobalWorkSize[2];
			// Set work size and execute the kernel
			szGlobalWorkSize[0] = m_numPairs;
			ciErrNum = clSetKernelArg(m_ckSolveConstraintsKernel, 1, sizeof(int), &nBatch);
			oclCHECKERROR(ciErrNum, CL_SUCCESS);
			ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, m_ckSolveConstraintsKernel, 1, NULL, szGlobalWorkSize, NULL, 0,0,0 );
			oclCHECKERROR(ciErrNum, CL_SUCCESS);
		}
	}
	// read results to CPU
	int memSize = sizeof(btVector3) * m_numObjects;
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dLinVel, CL_TRUE, 0, memSize, &(m_hLinVel[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dAngVel, CL_TRUE, 0, memSize, &(m_hAngVel[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
#endif
}

float computeImpulse(btVector3& relVel, float penetration, btVector3& normal, float timeStep)
{
	const float collisionConstant	=	0.1f;
	const float baumgarteConstant	=	0.1f;
	const float penetrationError	=	0.02f;

	float lambdaDt = 0.f;
//	btVector3 impulse(0.f, 0.f, 0.f);

	if(penetration >= 0.f)
	{
		return lambdaDt;
	}

	penetration = btMin(0.0f, penetration + penetrationError);
	lambdaDt	= - normal.dot(relVel) * collisionConstant;
	lambdaDt	-=	(baumgarteConstant/timeStep * penetration);
	return lambdaDt;
}


void btSpheresGridDemoDynamicsWorld::solvePairCPU(btSpheresContPair* pPair, int pairIdx, int batchNum)
{
	int batchId = m_hPairIds[pairIdx].m_batch;
	if(batchId != batchNum)
	{
		return;
	}
	int objIdA = m_hPairIds[pairIdx].m_objA;
	int objIdB = m_hPairIds[pairIdx].m_objB;
	btVector3 posA = m_hTrans[objIdA * 4 + 3];
	btVector3 posB = m_hTrans[objIdB * 4 + 3];
	btVector3 linVelA = m_hLinVel[objIdA];
	btVector3 linVelB = m_hLinVel[objIdB];
	btVector3 angVelA = m_hAngVel[objIdA];
	btVector3 angVelB = m_hAngVel[objIdB];
	btVector3 contPointA = pPair->m_contact - posA;
	btVector3 contPointB = pPair->m_contact - posB;
	btScalar penetration = pPair->m_contact[3];
	if(penetration > 0.f)
	{
		btVector3 contNormal = pPair->m_normal;
		btVector3 velA = linVelA + angVelA.cross(contPointA);
		btVector3 velB = linVelB + angVelB.cross(contPointB);
		btVector3 relVel = velA - velB;
		btScalar lambdaDt = computeImpulse(relVel, -penetration, contNormal, m_timeStep);
		btScalar rLambdaDt = contNormal[3];
		btScalar pLambdaDt = rLambdaDt;
		rLambdaDt = btMax(rLambdaDt + lambdaDt, 0.f);
		lambdaDt = rLambdaDt - pLambdaDt;
		pPair->m_normal[3] = rLambdaDt;
		btVector3 impulse = contNormal * lambdaDt * btScalar(0.5f);
		btScalar invMassA = m_hInvInertiaMass[objIdA * 3 + 0][3];
		btScalar invMassB = m_hInvInertiaMass[objIdB * 3 + 0][3];
		if(invMassA > 0.f)
		{
			linVelA += impulse;
			angVelA += contPointA.cross(impulse);
			linVelA[2] = linVelA[3] = 0.f;
			angVelA[0] = angVelA[1] = angVelA[3] = 0.f;
			m_hLinVel[objIdA] = linVelA;
			m_hAngVel[objIdA] = angVelA;
		}
		if(invMassB > 0.f)
		{
			linVelB -= impulse;
			angVelB -= contPointB.cross(impulse);
			linVelB[2] = linVelB[3] = 0.f;
			angVelB[0] = angVelB[1] = angVelB[3] = 0.f;
			m_hLinVel[objIdB] = linVelB;
			m_hAngVel[objIdB] = angVelB;
		}
	}
}	


