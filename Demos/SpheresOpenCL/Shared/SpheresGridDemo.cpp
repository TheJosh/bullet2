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

#define START_POS_X btScalar(0.f)
#define START_POS_Y btScalar(140.f)
#define START_POS_Z btScalar(0.f)
#define ARRAY_SIZE_X 20
#define ARRAY_SIZE_Y 50
//#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 80
//#define ARRAY_SIZE_Z 1
#define DIST btScalar(2.f)

#define STRESS_X  60
//#define STRESS_Y  200
#define STRESS_Y  40


		

///The 3 following lines include the CPU implementation of the kernels, keep them in this order.
#include "BulletMultiThreaded/btGpuDefines.h"
#include "BulletMultiThreaded/btGpuUtilsSharedDefs.h"
#include "BulletMultiThreaded/btGpuUtilsSharedCode.h"
#ifndef __APPLE__
#include <GL/glew.h>
#endif


#include "GL_DialogDynamicsWorld.h"
#include "GL_DialogWindow.h"



#include "BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"
#include "GLDebugFont.h"
#include "GlutStuff.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include <stdio.h> //printf debugging
#include "oecakeLoader.h"
#include "shaders.h"

#include "SpheresGridDemo.h"





btScalar gTimeStep = btScalar(1./60.);

#define SCALING btScalar(1.f)

class	SpheresGridDemoOecakeLoader : public BasicOECakeReader
{

	SpheresGridDemo* m_demo;

public:

	SpheresGridDemoOecakeLoader(SpheresGridDemo* demo)
	:m_demo(demo)
	{

	}

	virtual void createBodyForCompoundShape(btCompoundShape* compoundTmpShape,bool addConstraint, const btTransform& worldTransform, btScalar mass)
	{

		btDefaultMotionState* myMotionState= 0;
		
		btVector3 aabbMin,aabbMax;
		compoundTmpShape->getAabb(btTransform::getIdentity(),aabbMin,aabbMax);
		int numSpheres = compoundTmpShape->getNumChildShapes();
		btAssert(numSpheres>0);
		if (0 ) // numSpheres>8)
		{
//			printf("error: exceeded 8 spheres\n");
			return;
		}
			
		btVector3* positions = new btVector3[numSpheres];
		btScalar* radii = new btScalar[numSpheres];

		for (int i=0;i<numSpheres;i++)
		{
			btAssert(compoundTmpShape->getChildShape(i)->getShapeType()== SPHERE_SHAPE_PROXYTYPE);
			btSphereShape* sphereShape = (btSphereShape*)compoundTmpShape->getChildShape(i);
			radii[i]=sphereShape->getRadius();
			positions[i] = compoundTmpShape->getChildTransform(i).getOrigin();
		}

		btMultiSphereShape* multiSphere = new btMultiSphereShape(positions,radii,numSpheres);
		
			btVector3 localInertia(0,0,0);
			if (mass)
			{
				myMotionState = new btDefaultMotionState(worldTransform);
				multiSphere->calculateLocalInertia(mass,localInertia);
			}

				//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
			btRigidBody* body = new btRigidBody(mass,myMotionState,multiSphere,localInertia);	
			body->setLinearFactor(btVector3(1,1,0));
			body->setAngularFactor(btVector3(0,0,1));

			body->setWorldTransform(worldTransform);


			m_demo->getDynamicsWorld()->addRigidBody(body);

			if (addConstraint)
			{
				btVector3 pivotInA(0,0,0);
				btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body,pivotInA);
				m_demo->getDynamicsWorld()->addConstraint(p2p);
			}
	}

};


void SpheresGridDemo::clientMoveAndDisplay()
{


	updateCamera();
	glDisable(GL_LIGHTING);
	glColor3f(1.f, 1.f, 1.f);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	glDisable(GL_TEXTURE_2D); // we always draw wireframe in this demo

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();

	renderme(); 

	if (m_dialogDynamicsWorld)
		m_dialogDynamicsWorld->draw(gTimeStep);

	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(gTimeStep,0);//ms / 1000000.f);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}

	

	ms = getDeltaTimeMicroseconds();

	glFlush();

	glutSwapBuffers();

}



void SpheresGridDemo::displayCallback(void) {

	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	//if (m_dialogDynamicsWorld)
	//	m_dialogDynamicsWorld->draw(gTimeStep);

	glFlush();
	glutSwapBuffers();
}

class btNullBroadphase : public btBroadphaseInterface
{
public:
	btNullBroadphase()
	{
	}
	virtual ~btNullBroadphase() 
	{
	}
	virtual btBroadphaseProxy*	createProxy(  const btVector3& aabbMin,  const btVector3& aabbMax,int shapeType,void* userPtr, short int collisionFilterGroup,short int collisionFilterMask, btDispatcher* dispatcher,void* multiSapProxy)
	{
		return NULL;
	}
	virtual void	destroyProxy(btBroadphaseProxy* proxy,btDispatcher* dispatcher)
	{
	}
	virtual void	setAabb(btBroadphaseProxy* proxy,const btVector3& aabbMin,const btVector3& aabbMax, btDispatcher* dispatcher)
	{
	}
	virtual void	getAabb(btBroadphaseProxy* proxy,btVector3& aabbMin, btVector3& aabbMax ) const
	{
	}
	virtual void	rayTest(const btVector3& rayFrom,const btVector3& rayTo, btBroadphaseRayCallback& rayCallback, const btVector3& aabbMin=btVector3(0,0,0), const btVector3& aabbMax = btVector3(0,0,0))
	{
	}
	virtual void	calculateOverlappingPairs(btDispatcher* dispatcher)
	{
	}
	virtual	btOverlappingPairCache*	getOverlappingPairCache()
	{
		return NULL;
	}
	virtual	const btOverlappingPairCache*	getOverlappingPairCache() const
	{
		return NULL;
	}
	virtual void getBroadphaseAabb(btVector3& aabbMin,btVector3& aabbMax) const
	{
	}
	virtual void resetPool(btDispatcher* dispatcher)
	{
	}
	virtual void	printStats()
	{
	}
};



void	SpheresGridDemo::initPhysics()
{
	
	setTexturing(false);
	setShadows(false);

	setCameraDistance(80.);
	m_cameraTargetPosition.setValue(50, 10, 0);
	m_azi = btScalar(0.f);
	m_ele = btScalar(0.f);

	///collision configuration contains default setup for memory, collision setup

	btDefaultCollisionConstructionInfo dci;
	dci.m_defaultMaxPersistentManifoldPoolSize=50000;
	dci.m_defaultMaxCollisionAlgorithmPoolSize=50000;

	m_collisionConfiguration = new btDefaultCollisionConfiguration(dci);

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_pairCache = new (btAlignedAlloc(sizeof(btHashedOverlappingPairCache),16))btHashedOverlappingPairCache(); 


//	m_broadphase = new btDbvtBroadphase(m_pairCache);
	m_broadphase = new btNullBroadphase();

	///the default constraint solver
	m_solver = new btSequentialImpulseConstraintSolver();

#ifdef INTEGRATION_DEMO
	m_pWorldI = new btIntegrationDemoDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration, 65536);
#endif

#ifdef SPHERES_DEMO
	m_pWorldS = new btSpheresGridDemoDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration, 65536);
#endif

#ifdef SPHERES_DEMO
	m_dialogDynamicsWorld = new GL_DialogDynamicsWorld();
	GL_DialogWindow* settings = m_dialogDynamicsWorld->createDialog(50,0,280,280,"CPU fallback");
	
	m_pWorldS->m_useCpuControls[0] = 0;
	GL_ToggleControl* ctrl = 0;
	m_pWorldS->m_useCpuControls[SIMSTAGE_APPLY_GRAVITY] = m_dialogDynamicsWorld->createToggle(settings,"Apply Gravity");
	m_pWorldS->m_useCpuControls[SIMSTAGE_COMPUTE_CELL_ID] = m_dialogDynamicsWorld->createToggle(settings,"Compute Cell ID");
	m_pWorldS->m_useCpuControls[SIMSTAGE_SORT_CELL_ID] = m_dialogDynamicsWorld->createToggle(settings,"Sort Cell ID");
	m_pWorldS->m_useCpuControls[SIMSTAGE_FIND_CELL_START] = m_dialogDynamicsWorld->createToggle(settings,"Find Cell Start");
	m_pWorldS->m_useCpuControls[SIMSTAGE_FIND_PAIRS] = m_dialogDynamicsWorld->createToggle(settings,"Find Pairs");
	m_pWorldS->m_useCpuControls[SIMSTAGE_SCAN_PAIRS] = m_dialogDynamicsWorld->createToggle(settings,"Scan Pairs");
	m_pWorldS->m_useCpuControls[SIMSTAGE_COMPACT_PAIRS] = m_dialogDynamicsWorld->createToggle(settings,"Compact Pairs");
	m_pWorldS->m_useCpuControls[SIMSTAGE_COMPUTE_BATCHES] = m_dialogDynamicsWorld->createToggle(settings,"Compute Batches");
	m_pWorldS->m_useCpuControls[SIMSTAGE_COMPUTE_CONTACTS] = m_dialogDynamicsWorld->createToggle(settings,"Compute Contacts");
	m_pWorldS->m_useCpuControls[SIMSTAGE_SOLVE_CONSTRAINTS] = m_dialogDynamicsWorld->createToggle(settings,"Solve Constraints");
	m_pWorldS->m_useCpuControls[SIMSTAGE_INTEGRATE_TRANSFORMS] = m_dialogDynamicsWorld->createToggle(settings,"Integrate Transforms");
	m_pWorldS->m_useCpuControls[SIMSTAGE_KERNEL_COLLIDE_SPHERE_WALLS] = m_dialogDynamicsWorld->createToggle(settings,"Collide Sphere Walls");
	

	for(int i = 1; i < SIMSTAGE_TOTAL; i++)
	{
		m_pWorldS->m_useCpuControls[i]->m_active = false;
	}
#if defined(CL_PLATFORM_MINI_CL)
	m_pWorldS->m_useCpuControls[SIMSTAGE_SCAN_PAIRS]->m_active = true; 
	m_pWorldS->m_useCpuControls[SIMSTAGE_SORT_CELL_ID]->m_active = true; 
	m_pWorldS->m_useCpuControls[SIMSTAGE_COMPUTE_CONTACTS]->m_active = true; 
	

#endif
#if defined(CL_PLATFORM_AMD)
	m_pWorldS->m_useCpuControls[SIMSTAGE_SORT_CELL_ID]->m_active = true; // sloooow, incorrect, crashes application
	m_pWorldS->m_useCpuControls[SIMSTAGE_FIND_CELL_START]->m_active = true; // run-time error "Unimplemented"
	m_pWorldS->m_useCpuControls[SIMSTAGE_SCAN_PAIRS]->m_active = true; // works, but very slow (up to 100 times)
	m_pWorldS->m_useCpuControls[SIMSTAGE_COMPUTE_BATCHES]->m_active = true; // run-time error "Unimplemented"
#endif

#endif //#ifdef SPHERES_DEMO


	if(m_demoType == DEMO_INTEGRATION)
	{
#ifdef INTEGRATION_DEMO
		m_dynamicsWorld = m_pWorldI;
#endif
	}
	else
	{
#ifdef SPHERES_DEMO
		m_dynamicsWorld = m_pWorldS;
#endif //#ifdef SPHERES_DEMO
	}
#ifdef INTEGRATION_DEMO
	m_pWorldI->getSimulationIslandManager()->setSplitIslands(true);
	m_pWorldI->setGravity(btVector3(0,-10.,0));
	m_pWorldI->getSolverInfo().m_numIterations = 4;
#endif //INTEGRATION_DEMO

#ifdef SPHERES_DEMO
	m_pWorldS->getSimulationIslandManager()->setSplitIslands(true);
	m_pWorldS->setGravity(btVector3(0,-10.,0));
	m_pWorldS->getSolverInfo().m_numIterations = 4;
#endif //#ifdef SPHERES_DEMO

	{
		btCollisionShape* colShape = new btSphereShape(btScalar(1.0f));
		m_collisionShapes.push_back(colShape);
		btTransform startTransform;
		startTransform.setIdentity();
		btScalar	mass(1.f);
		btVector3 localInertia(0,0,0);
		colShape->calculateLocalInertia(mass,localInertia);
		float start_x = START_POS_X - ARRAY_SIZE_X * DIST * btScalar(0.5f);
		float start_y = START_POS_Y - ARRAY_SIZE_Y * DIST * btScalar(0.5f);
		float start_z = START_POS_Z - ARRAY_SIZE_Z * DIST * btScalar(0.5f);
	#if USE_BULLET_BODIES
		// may be very slow for > 10K bodies
		printf("\nGenerating bodies ...\n");
		int total = ARRAY_SIZE_X * ARRAY_SIZE_Y * ARRAY_SIZE_Z;
		int done = 0;
		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					startTransform.setOrigin(btVector3(
										DIST*i + start_x,
										DIST*k + start_y,
										DIST*j + start_z));
					btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,0,colShape,localInertia);
					rbInfo.m_startWorldTransform = startTransform;
					btRigidBody* body = new btRigidBody(rbInfo);
					m_pWorldI->addRigidBody(body);
					done++;
				}
			}
			printf("%6d of %6d\r", done, total);
			fflush(stdout);
		}
		printf("\n... Done!\n");
	#else
		// add just one sphere
#ifdef INTEGRATION_DEMO
		startTransform.setOrigin(btVector3(start_x, start_y, start_z));
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,0,colShape,localInertia);
		rbInfo.m_startWorldTransform = startTransform;
		btRigidBody* body = new btRigidBody(rbInfo);
		m_pWorldI->addRigidBody(body);
#endif
		// now fill m_hPos and m_hLinVel directly
		init_scene_directly();
	#endif
	}
	btDynamicsWorld* tmpW = m_dynamicsWorld;
#ifdef SPHERES_DEMO
	m_dynamicsWorld = m_pWorldS;
	SpheresGridDemoOecakeLoader	loader(this);
	//loader.processFile("test1.oec");
	#if 1 /// stress test
		btCompoundShape* compound = new btCompoundShape();
		btSphereShape* sphere = new btSphereShape(1.f);
		btTransform localTrans;
		localTrans.setIdentity();
		localTrans.setOrigin(btVector3(0,0,0));
		compound->addChildShape(localTrans,sphere);
		//localTrans.setOrigin(btVector3(-1,0,0));
		//compound->addChildShape(localTrans,sphere);

		btTransform trans;
		trans.setIdentity();

		btVector3 offset(0.00001,0.00001,0.00001);
		for (int j=0;j<STRESS_X;j++)
		for (int i=0;i<STRESS_Y;i++)
		{
			trans.setOrigin(offset*i+btVector3(25+j*6,30+i*3,0.));
			loader.createBodyForCompoundShape(compound,false,trans,1.);
		}
	#endif
	m_dynamicsWorld = tmpW;
#else
	m_dynamicsWorld = m_pWorldI;
#endif//#ifdef SPHERES_DEMO


	clientResetScene();
#ifdef INTEGRATION_DEMO
	m_pWorldI->initDeviceData();
#endif

#ifdef SPHERES_DEMO
	m_pWorldS->initDeviceData();
#endif
	print_used_device();
}

void SpheresGridDemo::init_scene_directly()
{
#ifdef INTEGRATION_DEMO
	float start_x = START_POS_X - ARRAY_SIZE_X * DIST * btScalar(0.5f);
	float start_y = START_POS_Y - ARRAY_SIZE_Y * DIST * btScalar(0.5f);
	float start_z = START_POS_Z - ARRAY_SIZE_Z * DIST * btScalar(0.5f);
	int total = ARRAY_SIZE_X * ARRAY_SIZE_Y * ARRAY_SIZE_Z;
	m_pWorldI->m_hPos.resize(total);
	m_pWorldI->m_hLinVel.resize(total);
	total = 0;
	for (int k=0;k<ARRAY_SIZE_Y;k++)
	{
		for (int i=0;i<ARRAY_SIZE_X;i++)
		{
			for(int j = 0;j<ARRAY_SIZE_Z;j++)
			{
				m_pWorldI->m_hLinVel[total] = btVector3(0., 0., 0.); 
				m_pWorldI->m_hPos[total] = btVector3(DIST*i + start_x, DIST*k + start_y, DIST*j + start_z);
				total++;
			}
		}
	}
	m_pWorldI->m_numSpheres = total;
#endif
}


void SpheresGridDemo::clientResetScene()
{
	static bool bFirstCall = true;
	DemoApplication::clientResetScene();
	if(m_demoType != DEMO_INTEGRATION)
	{
		setRandomZCoordinate((m_demoType == DEMO_OE_CAKE_3D) ?  1.f : 0.f);
	}
	else
	{
		#if(!USE_BULLET_BODIES)
			init_scene_directly();
		#endif
	}
	if(bFirstCall)
	{
		bFirstCall = false;
	}
	else
	{
#ifdef SPHERES_DEMO
		m_pWorldS->grabSimulationData();
#endif//#ifdef SPHERES_DEMO
#ifdef INTEGRATION_DEMO
		m_pWorldI->grabSimulationData();
#endif

	}
}


void SpheresGridDemo::setRandomZCoordinate(btScalar range)
{
#ifdef SPHERES_DEMO
	int numObjects = m_pWorldS->getNumCollisionObjects();
	btCollisionObjectArray objArray = m_pWorldS->getCollisionObjectArray();
	for(int i = 0; i < numObjects; i++)
	{
		btCollisionObject* colObj = objArray[i];
		btRigidBody* body = btRigidBody::upcast(colObj);
		if (body)
		{
			btTransform trans = body->getCenterOfMassTransform();
			btVector3 org = trans.getOrigin();
			org[2] = range * ((btScalar)rand() / (btScalar)RAND_MAX - btScalar(0.5f));
			trans.setOrigin(org);
			body->setCenterOfMassTransform(trans);
		}
	}
#endif
}

void	SpheresGridDemo::exitPhysics()
{
	delete m_dialogDynamicsWorld;
	m_dialogDynamicsWorld = 0;

	//cleanup in the reverse order of creation/initialization
	int i;

#ifdef SPHERES_DEMO

	//remove the rigidbodies from the dynamics world and delete them
	for (i=m_pWorldS->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_pWorldS->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_pWorldS->removeCollisionObject( obj );
		delete obj;
	}
#endif
#ifdef INTEGRATION_DEMO

	for (i=m_pWorldI->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_pWorldI->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_pWorldI->removeCollisionObject( obj );
		delete obj;
	}
#endif

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

#ifdef INTEGRATION_DEMO
	delete m_pWorldI;
#endif

#ifdef SPHERES_DEMO
	delete m_pWorldS;
#endif

	delete m_solver;
	
	delete m_broadphase;
	
	delete m_dispatcher;

	delete m_collisionConfiguration;
}

void SpheresGridDemo::print_used_device()
{
	if(m_demoType != DEMO_INTEGRATION)
	{
		return;
	}
	printf("\nIntegration demo : ");
#ifdef INTEGRATION_DEMO
	switch(m_pWorldI->m_usedDevice)
	{
		case 0 : 
			printf("Using CPU\n");
			break;
		case 1 : 
			printf("Using OpenCL GPU\n");
			break;
		case 2 : 
			printf("Using CUDA GPU\n");
			break;
		default : 
			printf("Using unknown device\n");
			break;
	}
#endif

}


void SpheresGridDemo::keyboardCallback(unsigned char key, int x, int y)
{
	(void)x;
	(void)y;
	switch (key) 
	{
		case 'D' :
			if(m_demoType != DEMO_INTEGRATION)
			{
				m_GpuCpuTogglePtr++;
				m_GpuCpuTogglePtr %= SIMSTAGE_TOTAL;
			}
			break;
		case 'G' :
			if(m_demoType != DEMO_INTEGRATION)
			{
				m_drawGridMode++;
				m_drawGridMode %= 3;
			}
			break;
		case 'j' : 
			m_demoType++;
			m_demoType %= DEMO_TOTAL_NUM;
			if(m_demoType == DEMO_INTEGRATION)
			{
#ifdef INTEGRATION_DEMO
				m_dynamicsWorld = m_pWorldI;
#endif

			}
			else
			{
#ifdef SPHERES_DEMO
				m_dynamicsWorld = m_pWorldS;
#endif
			}
			clientResetScene();
			break;
		case 'q' : 
			exitPhysics();
			exit(0);
			break;
		case '\t' : 
			if(m_demoType == DEMO_INTEGRATION)
			{
#ifdef INTEGRATION_DEMO
				m_pWorldI->m_usedDevice++;
				#if BT_USE_CUDA
					m_pWorldI->m_usedDevice %= 3;
				#else
					m_pWorldI->m_usedDevice %= 2;
				#endif
#endif

				print_used_device();
			}
			break;
		default : 
			{
				DemoApplication::keyboardCallback(key, x, y);
			}
			break;
	}
	
	if(key == ' ')
	{
	}
}



void SpheresGridDemo::renderme()
{


	glPointSize(5.0f);
	glEnable(GL_POINT_SPRITE_ARB);
	glTexEnvi(GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE);
#ifndef __APPLE__
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE_NV);
#endif //__APPLE__
	
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);

	glUseProgram(m_shaderProgram);

	btScalar dist = (m_glutScreenWidth > m_glutScreenHeight) ? m_glutScreenHeight : m_glutScreenWidth;
	glUniform1f( glGetUniformLocation(m_shaderProgram, "pointScale"), dist  );
//	glUniform1f( glGetUniformLocation(m_shaderProgram, "pointRadius"), 0.5f );
#ifdef SPHERES_DEMO
#ifdef INTEGRATION_DEMO
	int numSpheres = (m_demoType == DEMO_INTEGRATION) ? m_pWorldI->getNumSpheres() : m_pWorldS->getNumSpheres();
	int	col_vbo = (m_demoType == DEMO_INTEGRATION) ? m_pWorldI->m_colVbo : m_pWorldS->m_colVbo;
	int curr_vbo = (m_demoType == DEMO_INTEGRATION) ? m_pWorldI->m_vbo : m_pWorldS->m_vbo;
	float sphere_rad = (m_demoType == DEMO_INTEGRATION) ? m_pWorldI->m_sphereRad : m_pWorldS->m_sphereRad;
#else
	int numSpheres = m_pWorldS->getNumSpheres();
	int	col_vbo = m_pWorldS->m_colVbo;
	int curr_vbo =  m_pWorldS->m_vbo;
	float sphere_rad = m_pWorldS->m_sphereRad;
#endif
#else
	int numSpheres = m_pWorldI->getNumSpheres();
	int	col_vbo = m_pWorldI->m_colVbo;
	int curr_vbo =  m_pWorldI->m_vbo;
	float sphere_rad = m_pWorldI->m_sphereRad;
#endif

	

	glUniform1f( glGetUniformLocation(m_shaderProgram, "pointRadius"), sphere_rad );
	glColor3f(1, 1, 1);

	// render from the vbo
    glBindBuffer(GL_ARRAY_BUFFER, curr_vbo);
    glVertexPointer(4, GL_FLOAT, 0, 0);
    glEnableClientState(GL_VERTEX_ARRAY);
    if(col_vbo) 
	{
		glBindBufferARB(GL_ARRAY_BUFFER_ARB, col_vbo);
		glColorPointer(4, GL_FLOAT, 0, 0);
		glEnableClientState(GL_COLOR_ARRAY);
	}
	glDrawArrays(GL_POINTS, 0, numSpheres);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY); 
	glUseProgram(0);
	glDisable(GL_POINT_SPRITE_ARB);
	glBindBufferARB(GL_ARRAY_BUFFER,0);
#ifdef SPHERES_DEMO
	if((m_demoType != DEMO_INTEGRATION) && m_drawGridMode)
	{
		btVector3& wmin =  m_pWorldS->m_worldMin;
		btVector3& wmax =  m_pWorldS->m_worldMax;
		glBegin(GL_LINE_LOOP);
		glVertex3f(wmin[0], wmin[1], wmin[2]);
		glVertex3f(wmin[0], wmax[1], wmin[2]);
		glVertex3f(wmax[0], wmax[1], wmin[2]);
		glVertex3f(wmax[0], wmin[1], wmin[2]);
		glVertex3f(wmax[0], wmin[1], wmax[2]);
		glVertex3f(wmax[0], wmax[1], wmax[2]);
		glVertex3f(wmin[0], wmax[1], wmax[2]);
		glVertex3f(wmin[0], wmin[1], wmax[2]);
		glEnd();
		glBegin(GL_LINES);
		glVertex3f(wmin[0], wmin[1], wmin[2]);
		glVertex3f(wmax[0], wmin[1], wmin[2]);
		glVertex3f(wmin[0], wmin[1], wmax[2]);
		glVertex3f(wmax[0], wmin[1], wmax[2]);
		glVertex3f(wmin[0], wmax[1], wmin[2]);
		glVertex3f(wmin[0], wmax[1], wmax[2]);
		glVertex3f(wmax[0], wmax[1], wmin[2]);
		glVertex3f(wmax[0], wmax[1], wmax[2]);
		glEnd();
		if(m_drawGridMode == 2)
		{
			int szx = m_pWorldS->m_simParams.m_gridSize[0];
			int szy = m_pWorldS->m_simParams.m_gridSize[1];
			glBegin(GL_LINES);
			for(int i = 1; i < (szx-1); i++)
			{
				float wgt = (float)i / (float)(szx-1);
				btVector3 vtx = wmax * wgt + wmin * (1.0f - wgt); 
				glVertex3f(vtx[0], wmin[1], wmin[2]);
				glVertex3f(vtx[0], wmax[1], wmin[2]);
			}
			for(int i = 1; i < (szy-1); i++)
			{
				float wgt = (float)i / (float)(szy-1);
				btVector3 vtx = wmax * wgt + wmin * (1.0f - wgt); 
				glVertex3f(wmin[0], vtx[1], wmin[2]);
				glVertex3f(wmax[0], vtx[1], wmin[2]);
			}
		glEnd();
		}
	}
#endif

	if ((m_debugMode & btIDebugDraw::DBG_NoHelpText)==0)
	{
		setOrthographicProjection();
		int  xOffset = 10.f;
		int  yStart = 20.f;
		int  yIncr = 20.f;
		if(m_demoType == DEMO_INTEGRATION)
		{
#ifdef INTEGRATION_DEMO
			switch(m_pWorldI->m_usedDevice)
			{
				case 0:
					GLDebugDrawString(xOffset,yStart,"CPU");
					break;
				case 1:
					GLDebugDrawString(xOffset,yStart,"OpenCL");
					break;
				case 2:
					GLDebugDrawString(xOffset,yStart,"CUDA");
					break;
			}
			yStart += 30.f;
#endif

		}
		showProfileInfo(xOffset, yStart, yIncr);
		outputDebugInfo(xOffset, yStart, yIncr);
		resetPerspectiveProjection();
	}
}



void SpheresGridDemo::outputDebugInfo(int & xOffset,int & yStart, int  yIncr)
{
	char buf[124];
	glDisable(GL_LIGHTING);
	glColor3f(0, 0, 0);
	
	sprintf(buf,"mouse move+buttons to interact");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"space to reset");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"cursor keys and z,x to navigate");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"i to toggle simulation, s single step");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"q to quit");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"h to toggle help text");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"p to toggle profiling (+results to file)");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	sprintf(buf,"j to toggle between demos (integration/OECake2D/OECake3D)");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	if(m_demoType == DEMO_INTEGRATION)
	{
		sprintf(buf,"TAB to toggle between CPU  and GPU solvers");
		GLDebugDrawString(xOffset,yStart,buf);
		yStart += yIncr;
	}
	else
	{
		sprintf(buf,"G to draw broadphase grid");
		GLDebugDrawString(xOffset,yStart,buf);
		yStart += yIncr;
		sprintf(buf,"D and U to toggle between GPU and CPU");
		GLDebugDrawString(xOffset,yStart,buf);
		yStart += yIncr;
		
	}
	
}


GLuint _compileProgram(const char *vsource, const char *fsource)
{
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

    glShaderSource(vertexShader, 1, &vsource, 0);
    glShaderSource(fragmentShader, 1, &fsource, 0);
    
    glCompileShader(vertexShader);
    glCompileShader(fragmentShader);

    GLuint program = glCreateProgram();

    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);

    glLinkProgram(program);

    // check if program linked
    GLint success = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &success);

    if (!success) {
        char temp[256];
        glGetProgramInfoLog(program, 256, 0, temp);
        printf("Failed to link program:\n%s\n", temp);
        glDeleteProgram(program);
        program = 0;
    }
    return program;
}


void SpheresGridDemo::myinit()
{
	DemoApplication::myinit();
#ifndef __APPLE__
    glewInit();
    if (!glewIsSupported("GL_VERSION_2_0 GL_VERSION_1_5 GL_ARB_multitexture GL_ARB_vertex_buffer_object")) {
        fprintf(stderr, "Required OpenGL extensions missing.");
        exit(-1);
    }
#endif //__APPLE__
	
	m_shaderProgram = _compileProgram(vertexShader, spherePixelShader);
#ifdef INTEGRATION_DEMO
	m_pWorldI->initCLKernels(m_argc, m_argv);
#endif

#ifdef SPHERES_DEMO
#ifdef INTEGRATION_DEMO
	m_pWorldS->m_cxMainContext = m_pWorldI->m_cxMainContext;
	m_pWorldS->m_cdDevice = m_pWorldI->m_cdDevice;
	m_pWorldS->m_cqCommandQue = m_pWorldI->m_cqCommandQue;
	m_pWorldS->m_cpProgram = m_pWorldI->m_cpProgram;
	m_pWorldS->initCLKernels(m_argc, m_argv);
#else
	m_pWorldS->initCLKernels(m_argc, m_argv);
#endif
#endif
}






void SpheresGridDemo::mouseFunc(int button, int state, int x, int y)
{

	if (!m_dialogDynamicsWorld->mouseFunc(button,state,x,y))
	{
		DemoApplication::mouseFunc(button,state,x,y);
	}
}

void	SpheresGridDemo::mouseMotionFunc(int x,int y)
{
	m_dialogDynamicsWorld->mouseMotionFunc(x,y);
	DemoApplication::mouseMotionFunc(x,y);
}



void SpheresGridDemo::reshape(int w, int h)
{
	if (m_dialogDynamicsWorld)
		m_dialogDynamicsWorld->setScreenSize(w,h);
	GlutDemoApplication::reshape(w,h);
}

