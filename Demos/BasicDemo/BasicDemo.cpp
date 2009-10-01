/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


///create 125 (5x5x5) dynamic object
#define ARRAY_SIZE_X 45
#define ARRAY_SIZE_Y 45
#define ARRAY_SIZE_Z 45
#define VOXEL_SIZE_START .05
//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024)

///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.
#define START_POS_X -5
#define START_POS_Y -5
#define START_POS_Z -3

#include "BulletCollision/CollisionDispatch/SphereTriangleDetector.h"

#include "BunnyMesh.h"
#include "BasicDemo.h"
#include "GlutStuff.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include <stdio.h> //printf debugging
#include "BulletCollision/CollisionShapes/btTriangleShape.h"

void BasicDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();
	
	GL_ShapeDrawer::drawCoordSystem();

	///step the simulation
	if (m_dynamicsWorld)
	{
		//m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		m_dynamicsWorld->stepSimulation(ms / 1000000.f,10,1./240.);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}
		
	renderme(); 

	glFlush();

	glutSwapBuffers();

}


static bool CustomMaterialCombinerCallback(btManifoldPoint& cp,	const btCollisionObject* colObj0,int partId0,int index0,const btCollisionObject* colObj1,int partId1,int index1)
{
	cp.m_lateralFrictionInitialized = true;
	cp.m_lateralFrictionDir1.setValue(1,0,0);
	///downscale the friction direction 2 for lower (anisotropic) friction (rather than a unit vector)
	cp.m_lateralFrictionDir2.setValue(0,0,0.1);
	///choose a target velocity in the friction dir1 direction, for a conveyor belt effect
	//cp.m_contactMotion1 = 1.f;
	//cp.m_contactCFM2 = 0.1;
	//cp.m_combinedFriction = 10;
	//cp.m_combinedRestitution = calculateCombinedRestitution(restitution0,restitution1);
	return true;
}


void BasicDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	
	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}


struct  btVoxelizationCallback : public btTriangleCallback
	{
		btCompoundShape* m_bunnyCompound;
		btSphereShape*	m_sphereChildShape;
		btVector3	m_curSpherePos;
		bool	m_oncePerSphere;

		virtual void processTriangle(btVector3* triangleVerts, int partId, int triangleIndex)
		{
			if (!m_oncePerSphere)
			{
				btTransform tr;
				tr.setIdentity();
				tr.setOrigin(m_curSpherePos);

				btTriangleShape triangleShape(triangleVerts[0],triangleVerts[1],triangleVerts[2]);
				SphereTriangleDetector detector(m_sphereChildShape,&triangleShape,0.);
				btVector3 hitPos,hitNormal;
				btScalar hitDepth,timeOfImpact;
				if (detector.collide(m_curSpherePos,hitPos,hitNormal,hitDepth,timeOfImpact,0.))
				{
					m_oncePerSphere = true;
					m_bunnyCompound->addChildShape(tr,m_sphereChildShape);
				}
			}
		}
	};




void	BasicDemo::initPhysics()
{
	
	gContactAddedCallback = CustomMaterialCombinerCallback;

	setTexturing(true);
	setShadows(false);

	setCameraDistance(btScalar(SCALING*20.));
	this->m_azi = 90;

	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

	m_dynamicsWorld->setGravity(btVector3(0,-10,0));
	m_dynamicsWorld->getSolverInfo().m_solverMode |= SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION+SOLVER_USE_2_FRICTION_DIRECTIONS;
	m_dynamicsWorld->getSolverInfo().m_solverMode |= SOLVER_ENABLE_FRICTION_DIRECTION_CACHING;


	///create a few basic rigid bodies
#if 1
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-60,0));

	//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//enable custom material callback
		body->setCollisionFlags(body->getCollisionFlags()  | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}
#endif


	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		//btCollisionShape* colShape = new btBoxShape(btVector3(SCALING*1,SCALING*1,SCALING*1));
		//btCollisionShape* colShape = new btBoxShape(btVector3(SCALING*.1,SCALING*.1,SCALING*.1));
		btCollisionShape* colShape = new btSphereShape(SCALING*btScalar(1.));
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);

	
		float start_x = -ARRAY_SIZE_X;
		float start_y = -ARRAY_SIZE_Y;
		float start_z = - ARRAY_SIZE_Z;


		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					startTransform.setOrigin(1.*SCALING*btVector3(
										2.0*i + start_x,
										2.0*k + start_y,
										2.0*j + start_z));

			
					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
					btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
					btRigidBody* body = new btRigidBody(rbInfo);
					//body->setContactProcessingThreshold(colShape->getContactBreakingThreshold());
					body->setActivationState(ISLAND_SLEEPING);
					body->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);

				//	m_dynamicsWorld->addRigidBody(body);
					body->setActivationState(ISLAND_SLEEPING);
				}
			}
		}
	}

	btTriangleIndexVertexArray* meshInterface = new btTriangleIndexVertexArray();
	btIndexedMesh indexMesh;
	
	indexMesh.m_numTriangles = BUNNY_NUM_TRIANGLES ;
	indexMesh.m_numVertices = BUNNY_NUM_VERTICES;
	indexMesh.m_vertexBase = (const unsigned char*) &gVerticesBunny[0];
	indexMesh.m_vertexStride = 3*sizeof(REAL);
	indexMesh.m_triangleIndexBase = (const unsigned char*)&gIndicesBunny[0];
	indexMesh.m_triangleIndexStride = 3*sizeof(int);
	meshInterface->addIndexedMesh(indexMesh);
	btBvhTriangleMeshShape* bunny = new btBvhTriangleMeshShape(meshInterface,true);
	bunny->setLocalScaling(btVector3(2,2,2));
	btCollisionObject* obj = new btCollisionObject();
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0,2,-15));
	obj ->setWorldTransform(tr);
	obj->setCollisionShape(bunny);
	m_dynamicsWorld->addCollisionObject(obj);



	
	
	//btDiscreteCollisionDetectorInterface::ClosestPointInput input;
	//input.m_maximumDistanceSquared = btScalar(BT_LARGE_FLOAT);///@todo: tighter bounds
	//input.m_transformA = sphereObj->getWorldTransform();
	//input.m_transformB = triObj->getWorldTransform();
	//bool swapResults = m_swapped;
	//detector.getClosestPoints(input,*resultOut,dispatchInfo.m_debugDraw,swapResults);


	
	for (int v=1;v<10;v++)
	{

		float VOXEL_SIZE = VOXEL_SIZE_START * v;
		
		btVoxelizationCallback voxelizationCallback;

		btCompoundShape* compoundBunny = new btCompoundShape();
		voxelizationCallback.m_bunnyCompound = compoundBunny;
		voxelizationCallback.m_sphereChildShape = new btSphereShape(VOXEL_SIZE);

	#if 1
		float start_x = -ARRAY_SIZE_X;
			float start_y = -ARRAY_SIZE_Y;
			float start_z = - ARRAY_SIZE_Z;


		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					btVector3 pos =VOXEL_SIZE*SCALING*btVector3(
										2.0*i + start_x,
										2.0*k + start_y,
										2.0*j + start_z);
					btVector3 aabbMin(pos-btVector3(VOXEL_SIZE,VOXEL_SIZE,VOXEL_SIZE));
					btVector3 aabbMax(pos+btVector3(VOXEL_SIZE,VOXEL_SIZE,VOXEL_SIZE));
					voxelizationCallback.m_curSpherePos = pos;
					voxelizationCallback.m_oncePerSphere = false;

					bunny->processAllTriangles(&voxelizationCallback,aabbMin,aabbMax);
				}
			}
		}
		
		//btCollisionObject* obj2 = new btCollisionObject();
		//obj2->setCollisionShape(compoundBunny);
		//m_dynamicsWorld->addCollisionObject(obj2);

		btVector3 localInertia;
		compoundBunny->calculateLocalInertia(1,localInertia);
		btRigidBody* body = new btRigidBody(1,0,compoundBunny,localInertia);
		//m_dynamicsWorld->addRigidBody(body);
		btTransform start;
		start.setIdentity();
		start.setOrigin(btVector3(0,2,-12+6*v));
		localCreateRigidBody(1.,start,compoundBunny);
		printf("compoundBunny with %d spheres\n",compoundBunny->getNumChildShapes());

	#endif
}

	clientResetScene();
}
	

void	BasicDemo::exitPhysics()
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




