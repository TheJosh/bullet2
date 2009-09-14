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

///The 3 following lines include the CPU implementation of the kernels, keep them in this order.
#include "BulletMultiThreaded/btGpuDefines.h"
#include "BulletMultiThreaded/btGpuUtilsSharedDefs.h"
#include "BulletMultiThreaded/btGpuUtilsSharedCode.h"

#include <GL/glew.h>


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


#define START_POS_X btScalar(0.f)
#define START_POS_Y btScalar(140.f)
#define START_POS_Z btScalar(0.f)
#define ARRAY_SIZE_X 20
#define ARRAY_SIZE_Y 500
//#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 80
//#define ARRAY_SIZE_Z 1
#define DIST btScalar(2.f)


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

	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(gTimeStep,0);//ms / 1000000.f);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}

	renderme(); 

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

	m_pWorldI = new btIntegrationDemoDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration, 65536);
	m_pWorldS = new btSpheresGridDemoDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration, 65536);
	
	if(m_demoType == DEMO_INTEGRATION)
	{
		m_dynamicsWorld = m_pWorldI;
	}
	else
	{
		m_dynamicsWorld = m_pWorldS;
	}
	m_pWorldI->getSimulationIslandManager()->setSplitIslands(true);
	m_pWorldI->setGravity(btVector3(0,-10.,0));
	m_pWorldI->getSolverInfo().m_numIterations = 4;
	m_pWorldS->getSimulationIslandManager()->setSplitIslands(true);
	m_pWorldS->setGravity(btVector3(0,-10.,0));
	m_pWorldS->getSolverInfo().m_numIterations = 4;

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
		startTransform.setOrigin(btVector3(start_x, start_y, start_z));
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,0,colShape,localInertia);
		rbInfo.m_startWorldTransform = startTransform;
		btRigidBody* body = new btRigidBody(rbInfo);
		m_pWorldI->addRigidBody(body);
		// now fill m_hPos and m_hLinVel directly
		init_scene_directly();
	#endif
	}
	btDynamicsWorld* tmpW = m_dynamicsWorld;
	m_dynamicsWorld = m_pWorldS;
	SpheresGridDemoOecakeLoader	loader(this);
	loader.processFile("test1.oec");
	#if 1 /// stress test
		btCompoundShape* compound = new btCompoundShape();
		btSphereShape* sphere = new btSphereShape(1.f);
		btTransform localTrans;
		localTrans.setIdentity();
		localTrans.setOrigin(btVector3(1,0,0));
		compound->addChildShape(localTrans,sphere);
		localTrans.setOrigin(btVector3(-1,0,0));
		compound->addChildShape(localTrans,sphere);

		btTransform trans;
		trans.setIdentity();

//		for (int j=0;j<800;j++)
		for (int j=0;j<50;j++)
		for (int i=0;i<200;i++)
		{
			trans.setOrigin(btVector3(25+j*6,30+i*3,0.));
			loader.createBodyForCompoundShape(compound,false,trans,1.);
		}
	#endif
	m_dynamicsWorld = tmpW;

	clientResetScene();

	m_pWorldI->initDeviceData();
	m_pWorldS->initDeviceData();
	print_used_device();
}

void SpheresGridDemo::init_scene_directly()
{
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
		m_pWorldS->grabSimulationData();
		m_pWorldI->grabSimulationData();
	}
}


void SpheresGridDemo::setRandomZCoordinate(btScalar range)
{
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
}

void	SpheresGridDemo::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
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

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	delete m_pWorldI;
	delete m_pWorldS;
	
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
}


void SpheresGridDemo::keyboardCallback(unsigned char key, int x, int y)
{
	(void)x;
	(void)y;
	switch (key) 
	{
		case 'j' : 
			m_demoType++;
			m_demoType %= DEMO_TOTAL_NUM;
			if(m_demoType == DEMO_INTEGRATION)
			{
				m_dynamicsWorld = m_pWorldI;
			}
			else
			{
				m_dynamicsWorld = m_pWorldS;
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
				m_pWorldI->m_usedDevice++;
				#if BT_USE_CUDA
					m_pWorldI->m_usedDevice %= 3;
				#else
					m_pWorldI->m_usedDevice %= 2;
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
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE_NV);
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);

	glUseProgram(m_shaderProgram);

	btScalar dist = (m_glutScreenWidth > m_glutScreenHeight) ? m_glutScreenHeight : m_glutScreenWidth;
	glUniform1f( glGetUniformLocation(m_shaderProgram, "pointScale"), dist  );
//	glUniform1f( glGetUniformLocation(m_shaderProgram, "pointRadius"), 0.5f );
	float sphere_rad = (m_demoType == DEMO_INTEGRATION) ? m_pWorldI->m_sphereRad : m_pWorldS->m_sphereRad;
	glUniform1f( glGetUniformLocation(m_shaderProgram, "pointRadius"), sphere_rad );
	glColor3f(1, 1, 1);

	// render from the vbo
	int curr_vbo = (m_demoType == DEMO_INTEGRATION) ? m_pWorldI->m_vbo : m_pWorldS->m_vbo;
    glBindBuffer(GL_ARRAY_BUFFER, curr_vbo);
    glVertexPointer(4, GL_FLOAT, 0, 0);
    glEnableClientState(GL_VERTEX_ARRAY);
	curr_vbo = (m_demoType == DEMO_INTEGRATION) ? m_pWorldI->m_colVbo : m_pWorldS->m_colVbo;
    if(curr_vbo) 
	{
		glBindBufferARB(GL_ARRAY_BUFFER_ARB, curr_vbo);
		glColorPointer(4, GL_FLOAT, 0, 0);
		glEnableClientState(GL_COLOR_ARRAY);
	}
	int numSpheres = (m_demoType == DEMO_INTEGRATION) ? m_pWorldI->getNumSpheres() : m_pWorldS->getNumSpheres();
	glDrawArrays(GL_POINTS, 0, numSpheres);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY); 
	glUseProgram(0);
	glDisable(GL_POINT_SPRITE_ARB);

	if ((m_debugMode & btIDebugDraw::DBG_NoHelpText)==0)
	{
		setOrthographicProjection();
		int  xOffset = 10.f;
		int  yStart = 20.f;
		int  yIncr = 20.f;
		if(m_demoType == DEMO_INTEGRATION)
		{
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
    glewInit();
    if (!glewIsSupported("GL_VERSION_2_0 GL_VERSION_1_5 GL_ARB_multitexture GL_ARB_vertex_buffer_object")) {
        fprintf(stderr, "Required OpenGL extensions missing.");
        exit(-1);
    }
	m_shaderProgram = _compileProgram(vertexShader, spherePixelShader);
	m_pWorldI->initCLKernels(m_argc, m_argv);
	m_pWorldS->m_cxMainContext = m_pWorldI->m_cxMainContext;
	m_pWorldS->m_cdDevice = m_pWorldI->m_cdDevice;
	m_pWorldS->m_cqCommandQue = m_pWorldI->m_cqCommandQue;
	m_pWorldS->m_cpProgram = m_pWorldI->m_cpProgram;
	m_pWorldS->initCLKernels(m_argc, m_argv);
}

