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


	m_broadphase = new btDbvtBroadphase(m_pairCache);

	///the default constraint solver
	m_solver = new btSequentialImpulseConstraintSolver();

	m_pWorld = new btSpheresGridDemoDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration, 65536);
	m_dynamicsWorld = m_pWorld;
	m_pWorld->getSimulationIslandManager()->setSplitIslands(true);

	m_pWorld->setGravity(btVector3(0,-10.,0));
	m_pWorld->getSolverInfo().m_numIterations = 4;

	SpheresGridDemoOecakeLoader	loader(this);
	loader.processFile("test1.oec");

	clientResetScene();

	m_pWorld->initDeviceData();
}

void SpheresGridDemo::clientResetScene()
{
	static bool bFirstCall = true;
	DemoApplication::clientResetScene();
	if(bFirstCall)
	{
		bFirstCall = false;
	}
	else
	{
		m_pWorld->grabSimulationData();
	}
}

	

void	SpheresGridDemo::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	delete m_dynamicsWorld;
	
	delete m_solver;
	
	delete m_broadphase;
	
	delete m_dispatcher;

	delete m_collisionConfiguration;
}



void SpheresGridDemo::keyboardCallback(unsigned char key, int x, int y)
{
	(void)x;
	(void)y;
	switch (key) 
	{
		case 'q' : 
			exitPhysics();
			exit(0);
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
	glUniform1f( glGetUniformLocation(m_shaderProgram, "pointRadius"), 0.5f );
	glColor3f(1, 1, 1);

	// render from the vbo
    glBindBuffer(GL_ARRAY_BUFFER, m_pWorld->m_vbo);
    glVertexPointer(4, GL_FLOAT, 0, 0);
    glEnableClientState(GL_VERTEX_ARRAY);
    if(m_pWorld->m_colVbo) 
	{
		glBindBufferARB(GL_ARRAY_BUFFER_ARB, m_pWorld->m_colVbo);
		glColorPointer(4, GL_FLOAT, 0, 0);
		glEnableClientState(GL_COLOR_ARRAY);
	}
	int numSpheres = m_pWorld->getNumSpheres();
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

	sprintf(buf,"u to toggle between CPU  and CUDA solvers");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
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
	m_pWorld->initCLKernels(m_argc, m_argv);
}

