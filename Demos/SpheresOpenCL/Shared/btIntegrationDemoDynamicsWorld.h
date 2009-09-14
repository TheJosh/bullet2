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


#ifndef BT_INTEGRATION_DEMO_DYNAMICS_WORLD_H
#define BT_INTEGRATION_DEMO_DYNAMICS_WORLD_H

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


#define INTEGR_DEMO_USE_IMAGES 0

#define SPHERES_GRID_MAX_OBJS (65536)

// REAL number of threads executing in parallel
#if defined(CL_PLATFORM_MINI_CL)
#define SPHERES_GRID_MAX_WORKGROUP_SIZE  (4) // TODO : get from device
#else
// CUDA 1.0
#define SPHERES_GRID_MAX_WORKGROUP_SIZE  (512) // TODO : get from device
#endif


class btIntegrationDemoDynamicsWorld : public btDiscreteDynamicsWorld
{
protected:
	int			m_workGroupSize;
public:
	int			m_numSpheres;
	// CPU side data
	btAlignedObjectArray<btVector3>	m_hPos;
	btAlignedObjectArray<btVector3>	m_hLinVel;
protected:
	// GPU side data
	cl_mem		m_dPosInp;
	cl_mem		m_dLinVelInp;
	cl_mem		m_dPosOut;
	cl_mem		m_dLinVelOut;
	cl_mem		m_dPosDB[2];
	cl_mem		m_dLinVelDB[2];
	int			m_currDB;
	cl_mem		m_dSimParams; // copy of m_simParams : global simulation paramerers such as gravity, etc. 
	// CUDA
	// CUDA will use VBO buffer for positions
	void*		m_dCudaSimParams;
	void*		m_dCudaLinVel;

	// OpenCL 
public:
	cl_context			m_cxMainContext;
	cl_device_id		m_cdDevice;
	cl_command_queue	m_cqCommandQue;
	cl_program			m_cpProgram;
protected:
	cl_kernel			m_ckIntegrateMotionKernel;

	btVector3			m_worldMin;
	btVector3			m_worldMax;

public:
	btScalar		m_sphereRad;
	int				m_usedDevice;
	// vbo variables
	GLuint			m_vbo;
	unsigned int	m_posVbo;
	unsigned int	m_colVbo;
	btSimParams		m_simParams;
	float			m_timeStep;

	int getNumSpheres() { return m_numSpheres; }
	float* getPosBuffer() { return (float*)&(m_hPos[0]); }


	btIntegrationDemoDynamicsWorld(btDispatcher* dispatcher,btBroadphaseInterface* pairCache,btConstraintSolver* constraintSolver,btCollisionConfiguration* collisionConfiguration,
			int maxObjs = SPHERES_GRID_MAX_OBJS)
		: btDiscreteDynamicsWorld(dispatcher, pairCache, constraintSolver, collisionConfiguration)
	{ 
//		m_useCPU = true;
		m_usedDevice = 1; // OpenCL GPU
		m_simParams.m_gravity[0] = 0.f;
		m_simParams.m_gravity[1] = -10.f;
		m_simParams.m_gravity[2] = 0.f;
		m_simParams.m_gravity[3] = 0.f;
		m_workGroupSize = SPHERES_GRID_MAX_WORKGROUP_SIZE;
		m_currDB = 0;
	}
	virtual ~btIntegrationDemoDynamicsWorld();
	virtual int	stepSimulation( btScalar timeStep,int maxSubSteps=1, btScalar fixedTimeStep=btScalar(1.)/btScalar(60.));

	void initDeviceData();
	void initCLKernels(int argc, char** argv);
	void createVBO();
	void postInitDeviceData();
	void getShapeData();
	void allocateBuffers();
	void grabSimulationData();
	void adjustGrid();
	void setPointers();
	void runIntegrateMotionKernel();
	void runKernelWithWorkgroupSize(cl_kernel kernelFunc, int globalSize, int workgroupSize);
};


#endif //BT_INTEGRATION_DEMO_DYNAMICS_WORLD_H
