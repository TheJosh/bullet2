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

#include <stdio.h>
#include <stdlib.h>

#ifdef __APPLE__
//CL_PLATFORM_MINI_CL could be defined in build system
#else
#include <GL/glew.h>
#ifdef USE_MINICL
#include <MiniCL/cl_platform.h> //for CL_PLATFORM_MINI_CL definition
#else
#include <CL/cl_platform.h> //for CL_PLATFORM_MINI_CL definition
#endif
#endif //__APPLE__

#include "LinearMath/btScalar.h"
#include "LinearMath/btMinMax.h"
#include "btOclCommon.h"
#include "btOclUtils.h"

#include "btGpuDemo3dOCLWrap.h"


cl_context			btGpuDemo3dOCLWrap::m_cxMainContext;
cl_device_id		btGpuDemo3dOCLWrap::m_cdDevice;
cl_command_queue	btGpuDemo3dOCLWrap::m_cqCommandQue;
cl_program			btGpuDemo3dOCLWrap::m_cpProgram;
btKernelInfo		btGpuDemo3dOCLWrap::m_kernels[GPUDEMO3D_KERNEL_TOTAL];

cl_mem btGpuDemo3dOCLWrap::m_dTrans;
cl_mem btGpuDemo3dOCLWrap::m_dVel;
cl_mem btGpuDemo3dOCLWrap::m_dAngVel;
cl_mem btGpuDemo3dOCLWrap::m_dIds;
cl_mem btGpuDemo3dOCLWrap::m_dBatchIds;
cl_mem btGpuDemo3dOCLWrap::m_dLambdaDtBox;
cl_mem btGpuDemo3dOCLWrap::m_dPositionConstraint;
cl_mem btGpuDemo3dOCLWrap::m_dNormal;
cl_mem btGpuDemo3dOCLWrap::m_dContact;
cl_mem btGpuDemo3dOCLWrap::m_dForceTorqueDamp;
cl_mem btGpuDemo3dOCLWrap::m_dInvInertiaMass;
cl_mem btGpuDemo3dOCLWrap::m_dParams;

int	btGpuDemo3dOCLWrap::m_maxObj;
int	btGpuDemo3dOCLWrap::m_maxNeihbors;
int	btGpuDemo3dOCLWrap::m_maxConstr;
int	btGpuDemo3dOCLWrap::m_maxPointsPerConstr;
int	btGpuDemo3dOCLWrap::m_numObj;
int	btGpuDemo3dOCLWrap::m_maxBatches;
int	btGpuDemo3dOCLWrap::m_numBatches;
int	btGpuDemo3dOCLWrap::m_numConstraints;


void btGpuDemo3dOCLWrap::initCL(int argc, char** argv)
{
    cl_int ciErrNum;

//	m_cxMainContext = clCreateContextFromType(0, CL_DEVICE_TYPE_ALL, NULL, NULL, &ciErrNum);
	m_cxMainContext = btOclCommon::createContextFromType(CL_DEVICE_TYPE_ALL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
 
	m_cdDevice = btOclGetMaxFlopsDev(m_cxMainContext);

	// create a command-queue
	m_cqCommandQue = clCreateCommandQueue(m_cxMainContext, m_cdDevice, 0, &ciErrNum);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	// Program Setup
	size_t program_length;
	char* fileName = "Gpu3dDemoOCL.cl";
	FILE * fp = fopen(fileName, "rb");
	char newFileName[512];
	
	
	if (fp == NULL)
	{
		sprintf(newFileName,"..//%s",fileName);
		fp = fopen(newFileName, "rb");
		if (fp)
			fileName = newFileName;
	}
	
	if (fp == NULL)
	{
		sprintf(newFileName,"Demos//Gpu3dDemo//%s",fileName);
		fp = fopen(newFileName, "rb");
		if (fp)
			fileName = newFileName;
	}

	if (fp == NULL)
	{
		sprintf(newFileName,"..//..//..//..//..//Demos//Gpu3dDemo//%s",fileName);
		fp = fopen(newFileName, "rb");
		if (fp)
			fileName = newFileName;
		else
		{
			printf("cannot find %s\n",newFileName);
			exit(0);
		}
	}

	char *source = btOclLoadProgSource(fileName, "", &program_length);
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
		char cBuildLog[10240];
		clGetProgramBuildInfo(m_cpProgram, btOclGetFirstDev(m_cxMainContext), CL_PROGRAM_BUILD_LOG, 
							  sizeof(cBuildLog), cBuildLog, NULL );
		printf("\n\n%s\n\n\n", cBuildLog);
		printf("Press ENTER key to terminate the program\n");
		getchar();
		exit(-1); 
	}
	printf("OK\n");

}



void btGpuDemo3dOCLWrap::initKernel(int kernelId, char* pName)
{
	
	cl_int ciErrNum;
	cl_kernel kernel = clCreateKernel(m_cpProgram, pName, &ciErrNum);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	size_t wgSize;
	ciErrNum = clGetKernelWorkGroupInfo(kernel, m_cdDevice, CL_KERNEL_WORK_GROUP_SIZE, sizeof(size_t), &wgSize, NULL);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	m_kernels[kernelId].m_Id = kernelId;
	m_kernels[kernelId].m_kernel = kernel;
	m_kernels[kernelId].m_name = pName;
	m_kernels[kernelId].m_workgroupSize = (int)wgSize;
	return;
}

void btGpuDemo3dOCLWrap::runKernelWithWorkgroupSize(int kernelId, int globalSize)
{
	if(globalSize <= 0)
	{
		return;
	}
	cl_kernel kernelFunc = m_kernels[kernelId].m_kernel;
	cl_int ciErrNum = clSetKernelArg(kernelFunc, 0, sizeof(int), (void*)&globalSize);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
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
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	ciErrNum = clFlush(m_cqCommandQue);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
}



void btGpuDemo3dOCLWrap::allocateBuffers(int maxObjs, int maxConstr, int maxPointsPerConstr, int maxBatches)
{
    cl_int ciErrNum;

	m_maxObj = maxObjs;
	m_maxConstr = maxConstr;
	m_maxPointsPerConstr = maxPointsPerConstr;

    unsigned int memSize = sizeof(float)* 4 * m_maxObj * 4;
    m_dTrans = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);

	memSize = sizeof(float)* 4 * m_maxObj;
	m_dVel = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);

	m_dAngVel = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);

	memSize = sizeof(int) * m_maxConstr;
	m_dIds = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);

	memSize = sizeof(int) * 2 * m_maxConstr;
	m_dBatchIds = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);

	memSize = sizeof(float) * m_maxConstr * m_maxPointsPerConstr;
	m_dLambdaDtBox = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	m_dPositionConstraint = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);

	memSize = sizeof(float) * 4 * m_maxConstr * m_maxPointsPerConstr;
	m_dNormal = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	m_dContact = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);

	memSize = sizeof(float) * m_maxObj * 4 * 2;
	m_dForceTorqueDamp = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);

	memSize = sizeof(float) * m_maxObj * 4 * 3;
	m_dInvInertiaMass = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);

	memSize = sizeof(float) * 4 * 2;
	m_dParams = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);

}

void btGpuDemo3dOCLWrap::initKernels()
{
	initKernel(GPUDEMO3D_KERNEL_CLEAR_ACCUM_IMPULSE, "kClearAccumImpulse");
	setKernelArg(GPUDEMO3D_KERNEL_CLEAR_ACCUM_IMPULSE, 1, sizeof(cl_mem),	(void*)&m_dLambdaDtBox);
	setKernelArg(GPUDEMO3D_KERNEL_CLEAR_ACCUM_IMPULSE, 2, sizeof(int),		(void*)&m_maxPointsPerConstr);

	initKernel(GPUDEMO3D_KERNEL_COLLISION_WITH_WALL, "kCollisionWithWallBox");
	setKernelArg(GPUDEMO3D_KERNEL_COLLISION_WITH_WALL, 1, sizeof(cl_mem),	(void*)&m_dTrans);
	setKernelArg(GPUDEMO3D_KERNEL_COLLISION_WITH_WALL, 2, sizeof(cl_mem),	(void*)&m_dVel);
	setKernelArg(GPUDEMO3D_KERNEL_COLLISION_WITH_WALL, 3, sizeof(cl_mem),	(void*)&m_dAngVel);
	setKernelArg(GPUDEMO3D_KERNEL_COLLISION_WITH_WALL, 4, sizeof(cl_mem),	(void*)&m_dParams);

	initKernel(GPUDEMO3D_KERNEL_SOLVE_CONSTRAINTS, "kSolveConstraint");
	setKernelArg(GPUDEMO3D_KERNEL_SOLVE_CONSTRAINTS, 1, sizeof(cl_mem),	(void*)&m_dIds);
	setKernelArg(GPUDEMO3D_KERNEL_SOLVE_CONSTRAINTS, 2, sizeof(cl_mem),	(void*)&m_dBatchIds);
	setKernelArg(GPUDEMO3D_KERNEL_SOLVE_CONSTRAINTS, 3, sizeof(cl_mem),	(void*)&m_dTrans);
	setKernelArg(GPUDEMO3D_KERNEL_SOLVE_CONSTRAINTS, 4, sizeof(cl_mem),	(void*)&m_dVel);
	setKernelArg(GPUDEMO3D_KERNEL_SOLVE_CONSTRAINTS, 5, sizeof(cl_mem),	(void*)&m_dAngVel);
	setKernelArg(GPUDEMO3D_KERNEL_SOLVE_CONSTRAINTS, 6, sizeof(cl_mem),	(void*)&m_dLambdaDtBox);
	setKernelArg(GPUDEMO3D_KERNEL_SOLVE_CONSTRAINTS, 7, sizeof(cl_mem),	(void*)&m_dPositionConstraint);
	setKernelArg(GPUDEMO3D_KERNEL_SOLVE_CONSTRAINTS, 8, sizeof(cl_mem),	(void*)&m_dNormal);
	setKernelArg(GPUDEMO3D_KERNEL_SOLVE_CONSTRAINTS, 9, sizeof(cl_mem),	(void*)&m_dContact);

	initKernel(GPUDEMO3D_KERNEL_INTEGRATE_VELOCITIES, "kIntegrateVelocities");
	setKernelArg(GPUDEMO3D_KERNEL_INTEGRATE_VELOCITIES, 1, sizeof(cl_mem),	(void*)&m_dForceTorqueDamp);
	setKernelArg(GPUDEMO3D_KERNEL_INTEGRATE_VELOCITIES, 2, sizeof(cl_mem),	(void*)&m_dInvInertiaMass);
	setKernelArg(GPUDEMO3D_KERNEL_INTEGRATE_VELOCITIES, 3, sizeof(cl_mem),	(void*)&m_dVel);	
	setKernelArg(GPUDEMO3D_KERNEL_INTEGRATE_VELOCITIES, 4, sizeof(cl_mem),	(void*)&m_dAngVel);	

	initKernel(GPUDEMO3D_KERNEL_INTEGRATE_TRANSFORMS, "kIntegrateTransforms");
	setKernelArg(GPUDEMO3D_KERNEL_INTEGRATE_TRANSFORMS, 1, sizeof(cl_mem),	(void*)&m_dTrans);
	setKernelArg(GPUDEMO3D_KERNEL_INTEGRATE_TRANSFORMS, 2, sizeof(cl_mem),	(void*)&m_dVel);
	setKernelArg(GPUDEMO3D_KERNEL_INTEGRATE_TRANSFORMS, 3, sizeof(cl_mem),	(void*)&m_dAngVel);
	setKernelArg(GPUDEMO3D_KERNEL_INTEGRATE_TRANSFORMS, 4, sizeof(cl_mem),	(void*)&m_dInvInertiaMass);

}

void btGpuDemo3dOCLWrap::setKernelArg(int kernelId, int argNum, int argSize, void* argPtr)
{
    cl_int ciErrNum;
	ciErrNum  = clSetKernelArg(m_kernels[kernelId].m_kernel, argNum, argSize, argPtr);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
}



void btGpuDemo3dOCLWrap::copyArrayToDevice(cl_mem device, const void* host, unsigned int size, int devOffs, int hostOffs)
{
    cl_int ciErrNum;
	char* pHost = (char*)host + hostOffs;
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, device, CL_TRUE, devOffs, size, pHost, 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
}

void btGpuDemo3dOCLWrap::copyArrayFromDevice(void* host, const cl_mem device, unsigned int size, int hostOffs, int devOffs)
{
    cl_int ciErrNum;
	char* pHost = (char*)host + hostOffs;
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, device, CL_TRUE, devOffs, size, pHost, 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
}

#ifdef CL_PLATFORM_MINI_CL

#include <MiniCL/cl_MiniCL_Defs.h>

extern "C"
{
	#include "Gpu3dDemoOCL.cl"
}

MINICL_REGISTER(kClearAccumImpulse)
MINICL_REGISTER(kCollisionWithWallBox)
MINICL_REGISTER(kSolveConstraint)
MINICL_REGISTER(kIntegrateVelocities)
MINICL_REGISTER(kIntegrateTransforms)

#endif