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
	runPredictUnconstrainedMotionKernel();
	runSetSpheresKernel();
	runSortHashKernel();
	runFindCellStartKernel();
	runBroadphaseCDKernel();
	runScanPairsKernel();
	runCompactPairsKernel();
	runSetupBatchesKernel();
	runSortBatchesKernel();
	runSetupContactsKernel();
	runSolveConstraintsKernel();
	runIntegrateTransformsKernel();

	gStepNum++;

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
	m_dPosHashSrc = clCreateBuffer(m_cxMainContext, CL_MEM_READ_ONLY, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	m_dPosHashDst = clCreateBuffer(m_cxMainContext, CL_MEM_READ_ONLY, memSize, NULL, &ciErrNum);
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
//	m_maxBatches = 20;
	m_maxBatches = 40;
	m_hObjUsed.resize((m_numObjects + 1) * m_maxBatches); // last elem used as pair-per-batch counter
	memSize = (m_numObjects + 1) * m_maxBatches * sizeof(int);
	m_dObjUsed = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	m_hNumPairsInBatch.resize(m_maxBatches);
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
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dPosHashSrc, CL_TRUE, 0, memSize, &(m_hPosHash[0]), 0, NULL, NULL);
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
	ciErrNum  = clSetKernelArg(m_ckSetSpheresKernel, 0, sizeof(cl_mem), (void *) &m_dPos);
	ciErrNum |= clSetKernelArg(m_ckSetSpheresKernel, 1, sizeof(cl_mem), (void*) &m_dTrans);
	ciErrNum |= clSetKernelArg(m_ckSetSpheresKernel, 2, sizeof(cl_mem), (void*) &m_dShapeBuf);
	ciErrNum |= clSetKernelArg(m_ckSetSpheresKernel, 3, sizeof(cl_mem), (void*) &m_dBodyIds);
	ciErrNum |= clSetKernelArg(m_ckSetSpheresKernel, 4, sizeof(cl_mem), (void*) &m_dPosHashSrc);
	ciErrNum |= clSetKernelArg(m_ckSetSpheresKernel, 5, sizeof(cl_mem), (void*) &m_dSimParams);
	ciErrNum |= clSetKernelArg(m_ckSetSpheresKernel, 6, sizeof(int), &m_numObjects);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	m_ckPredictUnconstrainedMotionKernel = clCreateKernel(m_cpProgram, "kPredictUnconstrainedMotion", &ciErrNum);
	ciErrNum  = clSetKernelArg(m_ckPredictUnconstrainedMotionKernel, 0, sizeof(cl_mem), (void *) &m_dLinVel);
	ciErrNum |= clSetKernelArg(m_ckPredictUnconstrainedMotionKernel, 1, sizeof(cl_mem), (void *) &m_dAngVel);
	ciErrNum |= clSetKernelArg(m_ckPredictUnconstrainedMotionKernel, 2, sizeof(cl_mem), (void *) &m_dSimParams);
	ciErrNum |= clSetKernelArg(m_ckPredictUnconstrainedMotionKernel, 3, sizeof(cl_mem), (void *) &m_dInvInertiaMass);
	ciErrNum |= clSetKernelArg(m_ckPredictUnconstrainedMotionKernel, 4, sizeof(int), &m_numObjects);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);


	m_ckIntegrateTransformsKernel = clCreateKernel(m_cpProgram, "kIntegrateTransforms", &ciErrNum);
	ciErrNum  = clSetKernelArg(m_ckIntegrateTransformsKernel, 0, sizeof(cl_mem), (void *) &m_dLinVel);
	ciErrNum |= clSetKernelArg(m_ckIntegrateTransformsKernel, 1, sizeof(cl_mem), (void *) &m_dAngVel);
	ciErrNum |= clSetKernelArg(m_ckIntegrateTransformsKernel, 2, sizeof(cl_mem), (void *) &m_dSimParams);
	ciErrNum |= clSetKernelArg(m_ckIntegrateTransformsKernel, 3, sizeof(cl_mem), (void*) &m_dTrans);
	ciErrNum |= clSetKernelArg(m_ckIntegrateTransformsKernel, 4, sizeof(cl_mem), (void *) &m_dInvInertiaMass);
	ciErrNum |= clSetKernelArg(m_ckIntegrateTransformsKernel, 5, sizeof(int), &m_numObjects);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);


	m_ckBroadphaseCDKernel = clCreateKernel(m_cpProgram, "kBroadphaseCD", &ciErrNum);
	ciErrNum  = clSetKernelArg(m_ckBroadphaseCDKernel, 0, sizeof(cl_mem), (void *) &m_dPos);
	ciErrNum |= clSetKernelArg(m_ckBroadphaseCDKernel, 1, sizeof(cl_mem), (void *) &m_dShapeBuf);
	ciErrNum |= clSetKernelArg(m_ckBroadphaseCDKernel, 2, sizeof(cl_mem), (void *) &m_dBodyIds);
	ciErrNum |= clSetKernelArg(m_ckBroadphaseCDKernel, 3, sizeof(cl_mem), (void *) &m_dPosHashDst);
	ciErrNum |= clSetKernelArg(m_ckBroadphaseCDKernel, 4, sizeof(cl_mem), (void *) &m_dCellStart);
	ciErrNum |= clSetKernelArg(m_ckBroadphaseCDKernel, 5, sizeof(cl_mem), (void *) &m_dPairBuff);
	ciErrNum |= clSetKernelArg(m_ckBroadphaseCDKernel, 6, sizeof(cl_mem), (void *) &m_dPairBuffStartCurr);
	ciErrNum |= clSetKernelArg(m_ckBroadphaseCDKernel, 7, sizeof(cl_mem), (void *) &m_dSimParams);
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
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPosHashSrc, CL_TRUE, 0, memSize, &(m_hPosHash[0]), 0, NULL, NULL);
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
//    if (ciErrNum != CL_SUCCESS)
//    {
//        oclLog(LOGBOTH, 0.0, "Error: Failed to copy data:%d\n", ciErrNum);
//    }
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
#if 1
	// CPU version
	// get hash from GPU
	int memSize = m_numSpheres * sizeof(btInt2);
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPosHashSrc, CL_TRUE, 0, memSize, &(m_hPosHash[0]), 0, NULL, NULL);
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
	};
	btHashPosKey* pHash = (btHashPosKey*)(&m_hPosHash[0]);
	pHash->quickSort(pHash, 0, m_numSpheres - 1);
	// check
	for(int i = 1; i < m_numSpheres; i++)
	{
		if(m_hPosHash[i-1].x > m_hPosHash[i].x)
		{
			printf("Hash sort error at %d\n", i);
		}
	}
	// write back to GPU
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dPosHashDst, CL_TRUE, 0, memSize, &(m_hPosHash[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
#else
#endif
}


void btSpheresGridDemoDynamicsWorld::runFindCellStartKernel()
{
    cl_int ciErrNum;
#if 1
	// CPU version
	// get hash from GPU
	int memSize = m_numSpheres * sizeof(btInt2);
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPosHashDst, CL_TRUE, 0, memSize, &(m_hPosHash[0]), 0, NULL, NULL);
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
			pairId.m_pair = numPairs;
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

void btSpheresGridDemoDynamicsWorld::runSetupBatchesKernel()
{
    cl_int ciErrNum;
#if 1
	// CPU version
	int memSize = m_numPairs * sizeof(btPairId);
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPairIds, CL_TRUE, 0, memSize, &(m_hPairIds[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	m_numBatches = 0;
	for(int n_batch = 0; n_batch < m_maxBatches; n_batch++)
	{
		m_hNumPairsInBatch[n_batch] = 0;
		int offset = n_batch * (m_numObjects + 1);
		for(int i = 0; i < m_numObjects; i++)
		{
			btVector3 mass0 =	m_hInvInertiaMass[i * 3 + 0];
			if(mass0[3] > 0.f)
			{
				m_hObjUsed[offset + i] = 0;
			}
			else
			{
				m_hObjUsed[offset + i] = -1;
			}
		}
		m_hObjUsed[offset + m_numObjects] = 0;
		for(int i = 0; i < m_numPairs; i++)
		{
			btPairId pairId = m_hPairIds[i];

			if((pairId.m_batch < 0)
			 &&(m_hObjUsed[offset + pairId.m_objA] <= 0)
			 &&(m_hObjUsed[offset + pairId.m_objB] <= 0))
			{
				m_hPairIds[i].m_batch = n_batch;
				m_numBatches = n_batch + 1;
				m_hNumPairsInBatch[n_batch]++;
				if(!m_hObjUsed[offset + pairId.m_objA]) 
				{
					m_hObjUsed[offset + pairId.m_objA] = 1;
				}
				if(!m_hObjUsed[offset + pairId.m_objB]) 
				{
					m_hObjUsed[offset + pairId.m_objB] = 1;
				}
				m_hObjUsed[offset + m_numObjects]++;
			}
		}
	}
	// print res
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
		if(m_hPairIds[i].m_batch < 0) 
		{ // assign to the last batch (TODO : move to the main loop)
			m_hPairIds[i].m_batch = m_numBatches - 1;
			numLeft++;
		}
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
//	printf("pairs : %4d, batches : %d, left : %d\n", m_numPairs, m_numBatches, numLeft);


	// write results back to GPU
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dPairIds, CL_TRUE, 0, memSize, &(m_hPairIds[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
#else
#endif
}


void btSpheresGridDemoDynamicsWorld::runSortBatchesKernel()
{
    cl_int ciErrNum;
#if 1
	// CPU version
	int memSize = m_numPairs * sizeof(btPairId);
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPairIds, CL_TRUE, 0, memSize, &(m_hPairIds[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	btPairId* pPairId = &(m_hPairIds[0]);
	pPairId->quickSort(pPairId, 0, m_numPairs - 1);
	// check
	for(int i = 1; i < m_numPairs; i++)
	{
		if(m_hPairIds[i-1].m_batch > m_hPairIds[i].m_batch)
		{
			printf("Batch sort error at %d\n", i);
		}
	}
	// write back to GPU
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dPairIds, CL_TRUE, 0, memSize, &(m_hPairIds[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
#else
#endif
}


void btSpheresGridDemoDynamicsWorld::runSetupContactsKernel()
{
    cl_int ciErrNum;
#if 1
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
#endif
}

void btSpheresGridDemoDynamicsWorld::runSolveConstraintsKernel()
{
    cl_int ciErrNum;
#if 1
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
	for(int nIter = 0; nIter < 10; nIter++)
	{
		int totalPairs = 0;
		for(int nBatch = 0; nBatch < m_numBatches; nBatch++)
		{
			int numPairs = m_hNumPairsInBatch[nBatch];
			btSpheresContPair* pPairs = &(m_hContacts[totalPairs]);
			for(int nPair = 0; nPair < numPairs; nPair++)
			{
				solvePairCPU(pPairs + nPair, totalPairs + nPair);
			}
			totalPairs += numPairs;
		}
	}
	// write back to GPU
	memSize = sizeof(btVector3) * m_numObjects;
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dLinVel, CL_TRUE, 0, memSize, &(m_hLinVel[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dAngVel, CL_TRUE, 0, memSize, &(m_hAngVel[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
#else
#endif
}


float computeImpulse(btVector3& relVel, float penetration, btVector3& normal, float timeStep)
{
	const float collisionConstant	=	0.1f;
	const float baumgarteConstant	=	0.1f;
	const float penetrationError	=	0.02f;

	float lambdaDt = 0.f;
	btVector3 impulse(0.f, 0.f, 0.f);

	if(penetration >= 0.f)
	{
		return lambdaDt;
	}

	penetration = btMin(0.0f, penetration + penetrationError);
	lambdaDt	= - normal.dot(relVel) * collisionConstant;
	lambdaDt	-=	(baumgarteConstant/timeStep * penetration);
	return lambdaDt;
}


void btSpheresGridDemoDynamicsWorld::solvePairCPU(btSpheresContPair* pPair, int pairIdx)
{
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

