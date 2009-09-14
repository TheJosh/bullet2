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
#ifndef __APPLE__
#include <GL/glew.h>
#include <CL/cl_platform.h>
#endif //__APPLE__

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

#include "btIntegrationDemoDynamicsWorld.h"

#if BT_USE_CUDA
	extern "C" 
	{
		void btCuda_allocateArray(void** devPtr, unsigned int size);
		void btCuda_copyArrayToDevice(void* device, const void* host, unsigned int size);
		void btCuda_registerGLBufferObject(unsigned int vbo);
		void* btCuda_mapGLBufferObject(unsigned int vbo);
		void btCuda_unmapGLBufferObject(unsigned int vbo);
		void btCuda_integrateMotion(void* pPos, void* pLinVel, int numObjects, void* pParams, float timeStep);
	}
#endif



btIntegrationDemoDynamicsWorld::~btIntegrationDemoDynamicsWorld()
{
}

static int gStepNum = 0;

int	btIntegrationDemoDynamicsWorld::stepSimulation( btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep)
{
	startProfiling(timeStep);
	m_timeStep = timeStep;
	m_dPosInp = m_dPosDB[m_currDB];
	m_dLinVelInp = m_dLinVelDB[m_currDB];
	m_currDB++;
	m_currDB &= 0x01;
	m_dPosOut = m_dPosDB[m_currDB];
	m_dLinVelOut = m_dLinVelDB[m_currDB];
	BT_PROFILE("stepSimulation");
//	printf("Step : %d\n", gStepNum);
	{
//		BT_PROFILE("Integrate");
		runIntegrateMotionKernel();
	}

	gStepNum++;

#ifndef BT_NO_PROFILE
	CProfileManager::Increment_Frame_Counter();
#endif //BT_NO_PROFILE
	return 1;
}


void btIntegrationDemoDynamicsWorld::initDeviceData()
{
	getShapeData();
}



void btIntegrationDemoDynamicsWorld::postInitDeviceData()
{
	#if USE_BULLET_BODIES
		m_numSpheres = getNumCollisionObjects();
	#endif
	createVBO();
	allocateBuffers();
	adjustGrid();
	grabSimulationData();
}


void btIntegrationDemoDynamicsWorld::getShapeData()
{
	int numObjects = getNumCollisionObjects();
	btCollisionObjectArray& collisionObjects = getCollisionObjectArray();
	int totalSpheres = 0;
	for(int i = 0; i < numObjects; i++)
	{
		btCollisionObject* colObj = collisionObjects[i];
		btCollisionShape* pShape = colObj->getCollisionShape();
		int shapeType = pShape->getShapeType();
		if(shapeType == SPHERE_SHAPE_PROXYTYPE)
		{
			btSphereShape* pSph = (btSphereShape*)pShape;
			btScalar sphRad = pSph->getRadius();
			if(!i)
			{
				m_sphereRad = sphRad;
			}
			else
			{
				btAssert(m_sphereRad == sphRad);
			}
			totalSpheres++;
		}
		else
		{
			btAssert(0);
		}
	}
	#if !USE_BULLET_BODIES
		totalSpheres = m_numSpheres;
	#endif
		printf("Integration : total number of spheres : %d\n", totalSpheres);
}

void btIntegrationDemoDynamicsWorld::allocateBuffers()
{
    cl_int ciErrNum;
	// positions of spheres
	m_hPos.resize(m_numSpheres);
	m_hLinVel.resize(m_numSpheres);
    unsigned int memSize;
#if INTEGR_DEMO_USE_IMAGES
	{
		cl_image_format suppForm[256];
		cl_uint numForm = 0;
		ciErrNum = clGetSupportedImageFormats(	m_cxMainContext,	// cl_context context,
												CL_MEM_READ_WRITE,	// cl_mem_flags flags,
												CL_MEM_OBJECT_IMAGE2D,	// cl_mem_object_type image_type,
												256,	// cl_uint num_entries,
												suppForm,	// cl_image_format *image_formats,
												&numForm);	// cl_uint *num_image_formats
		oclCHECKERROR(ciErrNum, CL_SUCCESS);

		cl_image_format imgFormat;
		imgFormat.image_channel_order = CL_RGBA;
		imgFormat.image_channel_data_type = CL_FLOAT;
		m_dPosDB[0] = clCreateImage2D(	m_cxMainContext,	//cl_context context,
										CL_MEM_READ_WRITE,	// cl_mem_flags flags,
										&imgFormat,			// const cl_image_format *image_format,
										m_numSpheres,		// size_t image_width,
										1,					// size_t image_height,
										0,					// size_t image_row_pitch,
										NULL,				// void *host_ptr,
										&ciErrNum);			// cl_int *errcode_ret)
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		m_dPosInp = m_dPosDB[0];
		m_dPosDB[1] = clCreateImage2D(m_cxMainContext, CL_MEM_READ_WRITE, &imgFormat, m_numSpheres, 1, 0, NULL, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		m_dPosOut = m_dPosDB[1];
		m_dLinVelDB[0] = clCreateImage2D(m_cxMainContext, CL_MEM_READ_WRITE, &imgFormat, m_numSpheres, 1, 0, NULL, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		m_dLinVelInp = m_dLinVelDB[0];
		m_dLinVelDB[1] = clCreateImage2D(m_cxMainContext, CL_MEM_READ_WRITE, &imgFormat, m_numSpheres, 1, 0, NULL, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		m_dLinVelOut = m_dLinVelDB[1];
	}
#else
    memSize = sizeof(btVector3) *  m_numSpheres;
    m_dPosDB[0] = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	m_dPosInp = m_dPosDB[0];
    m_dPosDB[1] = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	m_dPosOut = m_dPosDB[1];
	// per object : transform, linear and angular velocity
	memSize = m_numSpheres * sizeof(btVector3);
    m_dLinVelDB[0] = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	m_dLinVelInp = m_dLinVelDB[0];
    m_dLinVelDB[1] = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	m_dLinVelOut = m_dLinVelDB[1];
#endif
	// CUDA
	#if BT_USE_CUDA
		memSize = sizeof(btVector3) *  m_numSpheres;
		btCuda_allocateArray(&m_dCudaLinVel, memSize);
	#endif
	// global simulation parameters
	memSize = sizeof(btSimParams);
	m_dSimParams = clCreateBuffer(m_cxMainContext, CL_MEM_READ_ONLY, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	// CUDA
	#if BT_USE_CUDA
		btCuda_allocateArray(&m_dCudaSimParams, memSize);
	#endif

}

void btIntegrationDemoDynamicsWorld::adjustGrid()
{
	btVector3 wmin( BT_LARGE_FLOAT,  BT_LARGE_FLOAT,  BT_LARGE_FLOAT);
	btVector3 wmax(-BT_LARGE_FLOAT, -BT_LARGE_FLOAT, -BT_LARGE_FLOAT);
	btCollisionObjectArray& collisionObjects = getCollisionObjectArray();
	for(int i = 0; i < m_numSpheres; i++)
	{
		btVector3 boxMin, boxMax;
		#if USE_BULLET_BODIES
			btCollisionObject* colObj = collisionObjects[i];
			btRigidBody* rb = btRigidBody::upcast(colObj);
			rb->getAabb(boxMin, boxMax);
		#else
			btVector3 pos = m_hPos[i];
			boxMin.setValue(pos[0] - m_sphereRad, pos[1] - m_sphereRad, pos[2] - m_sphereRad);
			boxMax.setValue(pos[0] + m_sphereRad, pos[1] + m_sphereRad, pos[2] + m_sphereRad);
		#endif
		wmin.setMin(boxMin);
		wmax.setMax(boxMax);
	}
	m_worldMin = wmin;
	m_worldMax = wmax;
	m_simParams.m_worldMin[0] = m_worldMin[0];
	m_simParams.m_worldMin[1] = m_worldMin[1];
	m_simParams.m_worldMin[2] = m_worldMin[2];

	m_simParams.m_cellSize[0] = btScalar(0.f);
	m_simParams.m_cellSize[1] = btScalar(0.f);
	m_simParams.m_cellSize[2] = btScalar(0.f);
}


void btIntegrationDemoDynamicsWorld::grabSimulationData()
{
	const btVector3& gravity = getGravity();
	m_simParams.m_gravity[0] = gravity[0];
	m_simParams.m_gravity[1] = gravity[1];
	m_simParams.m_gravity[2] = gravity[2];
	
	
	#if USE_BULLET_BODIES
		btCollisionObjectArray& collisionObjects = getCollisionObjectArray();
		for(int i = 0; i < m_numSpheres; i++)
		{
			btCollisionObject* colObj = collisionObjects[i];
			btRigidBody* rb = btRigidBody::upcast(colObj);
			m_hPos[i] = rb->getCenterOfMassPosition();
			m_hLinVel[i] = rb->getLinearVelocity();
		}
	#endif
	// copy to GPU
    cl_int ciErrNum;
	unsigned int memSize = sizeof(btVector3) * m_numSpheres;
#if INTEGR_DEMO_USE_IMAGES
	{
		size_t imgOrg[3];
		imgOrg[0] = imgOrg[1] = imgOrg[2] = 0;
		size_t imgReg[3];
		imgReg[0] = m_numSpheres;
		imgReg[1] = imgReg[2] = 1;
		ciErrNum = clEnqueueWriteImage(	m_cqCommandQue,						// cl_command_queue command_queue,
										m_dPosInp,							// cl_mem image,
										CL_TRUE,							// cl_bool blocking_read,
										imgOrg,								// const size_t origin[3],
										imgReg,								// const size_t region[3],
										m_numSpheres * sizeof(float) * 4,	// size_t row_pitch,
										0,									// size_t slice_pitch,
										&(m_hPos[0]),						// void *ptr,
										0,									// cl_uint num_events_in_wait_list,
										NULL,								// const cl_event *event_wait_list,
										NULL);								// cl_event *event
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		ciErrNum = clEnqueueWriteImage(	m_cqCommandQue,						// cl_command_queue command_queue,
										m_dPosOut,							// cl_mem image,
										CL_TRUE,							// cl_bool blocking_read,
										imgOrg,								// const size_t origin[3],
										imgReg,								// const size_t region[3],
										m_numSpheres * sizeof(float) * 4,	// size_t row_pitch,
										0,									// size_t slice_pitch,
										&(m_hPos[0]),						// void *ptr,
										0,									// cl_uint num_events_in_wait_list,
										NULL,								// const cl_event *event_wait_list,
										NULL);								// cl_event *event
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		ciErrNum = clEnqueueWriteImage(	m_cqCommandQue,						// cl_command_queue command_queue,
										m_dLinVelInp,						// cl_mem image,
										CL_TRUE,							// cl_bool blocking_read,
										imgOrg,								// const size_t origin[3],
										imgReg,								// const size_t region[3],
										m_numSpheres * sizeof(float) * 4,	// size_t row_pitch,
										0,									// size_t slice_pitch,
										&(m_hLinVel[0]),					// void *ptr,
										0,									// cl_uint num_events_in_wait_list,
										NULL,								// const cl_event *event_wait_list,
										NULL);								// cl_event *event
	}
#else
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dPosInp, CL_TRUE, 0, memSize, &(m_hPos[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dLinVelInp, CL_TRUE, 0, memSize, &(m_hLinVel[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
#endif
	// CUDA
	#if BT_USE_CUDA
		btCuda_copyArrayToDevice(m_dCudaLinVel, &(m_hLinVel[0]), memSize);
	#endif
	// params
	memSize = sizeof(btSimParams);
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dSimParams, CL_TRUE, 0, memSize, &m_simParams, 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	// CUDA
	#if BT_USE_CUDA
		btCuda_copyArrayToDevice(m_dCudaSimParams, &m_simParams, memSize);
	#endif
	// VBO
	glBindBufferARB(GL_ARRAY_BUFFER, m_vbo);    
	void* ptr = glMapBufferARB(GL_ARRAY_BUFFER, GL_WRITE_ONLY_ARB);
	memcpy(ptr, &(m_hPos[0]), m_numSpheres * sizeof(btVector3));
	glUnmapBufferARB(GL_ARRAY_BUFFER); 
}


void btIntegrationDemoDynamicsWorld::createVBO()
{
    // create buffer object
    glGenBuffers(1, &m_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	// positions of spheres
    unsigned int memSize = sizeof(btVector3) *  m_numSpheres;
    glBufferData(GL_ARRAY_BUFFER, memSize, 0, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
	#if BT_USE_CUDA
		btCuda_registerGLBufferObject(m_vbo);
	#endif
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



void btIntegrationDemoDynamicsWorld::initCLKernels(int argc, char** argv)
{
    cl_int ciErrNum;

	// create the OpenCL context
    m_cxMainContext = clCreateContextFromType(0, CL_DEVICE_TYPE_ALL, NULL, NULL, &ciErrNum);
//    m_cxMainContext = clCreateContextFromType(0, CL_DEVICE_TYPE_GPU, NULL, NULL, &ciErrNum);
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
#ifndef CL_PLATFORM_MINI_CL
	
	size_t program_length;
	char* fileName = "Integration.cl";
	FILE * fp = fopen(fileName, "rb");
	char newFileName[512];
	
	if (fp == NULL)
	{
		sprintf(newFileName,"Demos//SpheresOpenCL//Shared//%s",fileName);
		fp = fopen(newFileName, "rb");
		if (fp)
			fileName = newFileName;
	}
	if (fp == NULL)
	{
		sprintf(newFileName,"..//..//Demos//SpheresOpenCL//Shared//%s",fileName);
		fp = fopen(newFileName, "rb");
		if (fp)
			fileName = newFileName;
		else
		{
			printf("cannot find %s\n",newFileName);
			exit(0);
		}
	}

//	char *source = oclLoadProgSource(".//Demos//SpheresGrid//SpheresGrid.cl", "", &program_length);
	//char *source = btOclLoadProgSource(".//Demos//SpheresOpenCL//Shared//SpheresGrid.cl", "", &program_length);

	char *source = btOclLoadProgSource(fileName, "", &program_length);
//	oclCHECKERROR (source == NULL, oclFALSE);   
	if(source == NULL)
	{
		printf("ERROR : OpenCL can't load file %s\n", fileName);
	}
	btAssert(source != NULL);

	// create the program
	printf("OpenCL compiles %s ...", fileName);
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
	printf("OK\n");
#else // defined(CL_PLATFORM_MINI_CL)
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
	m_ckIntegrateMotionKernel = clCreateKernel(m_cpProgram, "kIntegrateMotion", &ciErrNum);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	postInitDeviceData();

	// set the args values 
	ciErrNum |= clSetKernelArg(m_ckIntegrateMotionKernel, 0, sizeof(int), &m_numSpheres);
	ciErrNum |= clSetKernelArg(m_ckIntegrateMotionKernel, 1, sizeof(cl_mem), (void *) &m_dPosInp);
	ciErrNum |= clSetKernelArg(m_ckIntegrateMotionKernel, 2, sizeof(cl_mem), (void*) &m_dLinVelInp);
	ciErrNum |= clSetKernelArg(m_ckIntegrateMotionKernel, 3, sizeof(cl_mem), (void *) &m_dPosOut);
	ciErrNum |= clSetKernelArg(m_ckIntegrateMotionKernel, 4, sizeof(cl_mem), (void*) &m_dLinVelOut);
	ciErrNum |= clSetKernelArg(m_ckIntegrateMotionKernel, 5, sizeof(cl_mem), (void*) &m_dSimParams);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
}

void btIntegrationDemoDynamicsWorld::runIntegrateMotionKernel()
{
	{
		BT_PROFILE("Integrate");
		switch(m_usedDevice)
		{
		case 0: 
			{
				for(int i = 0; i < m_numSpheres; i++)
				{
					btVector3 vel = m_hLinVel[i];
		//			btVector3* grav = 
					vel += (*(btVector3*)m_simParams.m_gravity) * m_timeStep;
					btVector3 pos = m_hPos[i];
					pos += vel * m_timeStep;
					m_hPos[i] = pos;
					m_hLinVel[i] = vel;
				}
			}
			break;
		case 1: 
			{
				cl_int ciErrNum;
		ciErrNum  = clSetKernelArg(m_ckIntegrateMotionKernel, 1, sizeof(cl_mem), (void *) &m_dPosInp);
		ciErrNum |= clSetKernelArg(m_ckIntegrateMotionKernel, 2, sizeof(cl_mem), (void*) &m_dLinVelInp);
		ciErrNum |= clSetKernelArg(m_ckIntegrateMotionKernel, 3, sizeof(cl_mem), (void *) &m_dPosOut);
		ciErrNum |= clSetKernelArg(m_ckIntegrateMotionKernel, 4, sizeof(cl_mem), (void*) &m_dLinVelOut);
		ciErrNum |= clSetKernelArg(m_ckIntegrateMotionKernel, 6, sizeof(float), &m_timeStep);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		runKernelWithWorkgroupSize(m_ckIntegrateMotionKernel, m_numSpheres, 128);
		ciErrNum = clFinish(m_cqCommandQue);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
			#if INTEGR_DEMO_USE_IMAGES
			{
				size_t imgOrg[3];
				imgOrg[0] = imgOrg[1] = imgOrg[2] = 0;
				size_t imgReg[3];
				imgReg[0] = m_numSpheres;
				imgReg[1] = imgReg[2] = 1;
				ciErrNum = clEnqueueReadImage(	m_cqCommandQue,						// cl_command_queue command_queue,
												m_dPosOut,							// cl_mem image,
												CL_TRUE,							// cl_bool blocking_read,
												imgOrg,								// const size_t origin[3],
												imgReg,								// const size_t region[3],
												m_numSpheres * sizeof(float) * 4,	// size_t row_pitch,
												0,									// size_t slice_pitch,
												&(m_hPos[0]),						// void *ptr,
												0,									// cl_uint num_events_in_wait_list,
												NULL,								// const cl_event *event_wait_list,
												NULL);								// cl_event *event
				oclCHECKERROR(ciErrNum, CL_SUCCESS);
			}
			#else
			{
			// check
		/*
				int memSize = sizeof(btVector3) * m_numSpheres;
				ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPos, CL_TRUE, 0, memSize, &(m_hPos[0]), 0, NULL, NULL);
				oclCHECKERROR(ciErrNum, CL_SUCCESS);
				ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dLinVel, CL_TRUE, 0, memSize, &(m_hLinVel[0]), 0, NULL, NULL);
				oclCHECKERROR(ciErrNum, CL_SUCCESS);
		*/
			}
			#endif
			}
			break;
		case 2: 
			#if BT_USE_CUDA
			{
				void* dPos = btCuda_mapGLBufferObject(m_vbo);
				btCuda_integrateMotion(dPos, m_dCudaLinVel, m_numSpheres, m_dCudaSimParams, m_timeStep);
				btCuda_unmapGLBufferObject(m_vbo);
			}
			#endif
			break;
		}
	}
	if(m_usedDevice < 2)
//	if(0)
	{
		BT_PROFILE("Copy VBO");
		// Explicit Copy (until OpenGL interop will work)
		// map the PBO to copy data from the CL buffer via host
		glBindBufferARB(GL_ARRAY_BUFFER, m_vbo);    
		// map the buffer object into client's memory
		void* ptr = glMapBufferARB(GL_ARRAY_BUFFER, GL_WRITE_ONLY_ARB);
		if(!m_usedDevice)
		{
			memcpy(ptr, &(m_hPos[0]), m_numSpheres * sizeof(btVector3));
		}
		else
		{
			cl_int ciErrNum;
			#if INTEGR_DEMO_USE_IMAGES
			{
				size_t imgOrg[3];
				imgOrg[0] = imgOrg[1] = imgOrg[2] = 0;
				size_t imgReg[3];
				imgReg[0] = m_numSpheres;
				imgReg[1] = imgReg[2] = 1;
				ciErrNum = clEnqueueReadImage(	m_cqCommandQue,						// cl_command_queue command_queue,
												m_dPosOut,							// cl_mem image,
												CL_TRUE,							// cl_bool blocking_read,
												imgOrg,								// const size_t origin[3],
												imgReg,								// const size_t region[3],
												m_numSpheres * sizeof(float) * 4,	// size_t row_pitch,
												0,									// size_t slice_pitch,
												ptr,								// void *ptr,
												0,									// cl_uint num_events_in_wait_list,
												NULL,								// const cl_event *event_wait_list,
												NULL);								// cl_event *event
			}
			#else
				ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPosOut, CL_TRUE, 0, sizeof(float) * 4 * m_numSpheres, ptr, 0, NULL, NULL);
			#endif
			oclCHECKERROR(ciErrNum, CL_SUCCESS);
		}
		glUnmapBufferARB(GL_ARRAY_BUFFER); 
	}
}

void btIntegrationDemoDynamicsWorld::runKernelWithWorkgroupSize(cl_kernel kernelFunc, int globalSize, int workgroupSize)
{
	if(globalSize <= 0)
	{
		return;
	}
	cl_int ciErrNum = clSetKernelArg(kernelFunc, 0, sizeof(int), (void*)&globalSize);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	if(workgroupSize <= 0)
	{ // let OpenCL library calculate workgroup size
		size_t globalWorkSize[2];
		globalWorkSize[0] = globalSize;
		globalWorkSize[1] = 1;
		ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, kernelFunc, 1, NULL, globalWorkSize, NULL, 0,0,0 );
	}
	else
	{
		#if defined(CL_PLATFORM_MINI_CL)
			workgroupSize = 4;
		#else
			int maxWorkgroupSize;
			clGetDeviceInfo(m_cdDevice, CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof(maxWorkgroupSize), &maxWorkgroupSize, NULL);
			#if defined(CL_PLATFORM_NVIDIA)
				// use given value
			#else
				workgroupSize = maxWorkgroupSize;
			#endif // CL_PLATFORM_NVIDIA
		#endif
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
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
}
