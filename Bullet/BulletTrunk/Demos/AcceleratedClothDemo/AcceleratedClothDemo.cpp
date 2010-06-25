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

//#define TEST_SERIALIZATION 1

///create 125 (5x5x5) dynamic object
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5

//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024)

///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.
#define START_POS_X -5
#define START_POS_Y -5
#define START_POS_Z -3

#include "AcceleratedClothDemo.h"
#include "GlutStuff.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "BulletAcceleratedSoftBody/btAcceleratedSoftRigidDynamicsWorld.h"
#include "BulletAcceleratedSoftBody/btAcceleratedSoftBody_Devices.h"
#include <cmath>

using Vectormath::Aos::Vector3;


#if 0
btVector3 operator ()( Vectormath::Aos::Vector3 &input )
{
	btVector3 returnVector;
	returnVector.setX( input.getX() );
	returnVector.setY( input.getY() );
	returnVector.setZ( input.getZ() );
	return returnVector;
}
#endif


#ifdef TEST_SERIALIZATION
#include "LinearMath/btSerializer.h"
#endif //TEST_SERIALIZATION

#include <stdio.h> //printf debugging
#include <iostream>

void AcceleratedClothDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();
	
	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		//optional but useful: debug drawing
		//m_dynamicsWorld->debugDrawWorld();
	}
		
	renderme(); 

	glFlush();

	swapBuffers();

}
void AcceleratedClothDemo::keyboardCallback(unsigned char key, int x, int y)
{

	switch (key) 
	{
	case '0' : 
		m_displayBendLinks = !m_displayBendLinks; 
		reinterpret_cast< btAcceleratedSoftRigidDynamicsWorld* >(m_dynamicsWorld)->debugDisplayBendLinks(m_displayBendLinks); 
		break;

	default:
		//        std::cout << "unused key : " << key << std::endl;
		break;
	}

	DemoApplication::keyboardCallback(key, x, y);

}

void AcceleratedClothDemo::renderme()
{
	static int counter = 0;
	m_dynamicsWorld->debugDrawWorld();	

	// Change wind velocity a bit based on a frame counter
	if( (counter % 120) == 0 )
	{
		m_windAngle = (m_windAngle + 0.05);
		if( m_windAngle > 3.141 )
			m_windAngle = 0;

		for( int flagIndex = 0; flagIndex < m_flags.size(); ++flagIndex )
		{		
			BTAcceleratedSoftBody::Cloth *cloth = 0;
			cloth = m_flags[flagIndex];

			float localWind = m_windAngle + 0.5*(((float(rand())/RAND_MAX))-0.1);
			float xCoordinate = cos(localWind)*m_windStrength;
			float zCoordinate = sin(localWind)*m_windStrength;

			cloth->setWindVelocity( Vectormath::Aos::Vector3(xCoordinate, 0, zCoordinate) );
		}
	}

	counter++;

	DemoApplication::renderme();
}


void AcceleratedClothDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	
	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}


/**
 * Create a flag x nodes wide and y nodes tall.
 * Coordinate range will be -1.0 to 1.0 around the origin.
 * device is the Bullet physics simulation device that will initially simulate the flag.
 */
btAlignedObjectArray<BTAcceleratedSoftBody::Cloth *> AcceleratedClothDemo::createFlag( const BTAcceleratedSoftBody::BulletPhysicsDevice &device, int width, int height )
{
	// First create a triangle mesh to represent a flag

	using namespace BTAcceleratedSoftBody;
	std::cout << "Vector3 size: " << sizeof(Vector3) << " btVector3 size: " << sizeof(btVector3) << "\n";
	btIndexedMesh mesh;
	mesh.m_numVertices = width*height;
	mesh.m_numTriangles = 2*(width-1)*(height-1);
	Vector3 *vertexArray = new Vector3[mesh.m_numVertices];
	mesh.m_vertexBase = reinterpret_cast<const unsigned char*>(vertexArray);
	int *triangleVertexIndexArray = new int[3*mesh.m_numTriangles];	
	mesh.m_triangleIndexBase = reinterpret_cast<const unsigned char*>(triangleVertexIndexArray);
	mesh.m_triangleIndexStride = sizeof(int)*3;
	mesh.m_vertexStride = sizeof(Vector3);

	// Generate normalised object space vertex coordinates

	// TEMPORARILY *10 on the values!
	float zCoordinate = 0.0f;
	for( int y = 0; y < height; ++y )
	{
		float yCoordinate = y*2.0f/float(height) - 1.0f;
		for( int x = 0; x < width; ++x )
		{			
			float xCoordinate = x*2.0f/float(width) - 1.0f;
			Vector3 vertex(xCoordinate, yCoordinate, zCoordinate);
			vertexArray[y*width + x] = vertex;
		}
	}

	// Generate vertex indices for triangles
	for( int y = 0; y < (height-1); ++y )
	{
		for( int x = 0; x < (width-1); ++x )
		{	
			// Triangle 0
			// Top left of square on mesh
			{
				int vertex0 = y*width + x;
				int vertex1 = vertex0 + 1;
				int vertex2 = vertex0 + width;
				int triangleIndex = 2*y*(width-1) + 2*x;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex)/sizeof(int)] = vertex0;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex+1)/sizeof(int)+1] = vertex1;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex+2)/sizeof(int)+2] = vertex2;
			}

			// Triangle 1
			// Bottom right of square on mesh
			{
				int vertex0 = y*width + x + 1;
				int vertex1 = vertex0 + width;
				int vertex2 = vertex1 - 1;
				int triangleIndex = 2*y*(width-1) + 2*x + 1;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex)/sizeof(int)] = vertex0;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex)/sizeof(int)+1] = vertex1;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex)/sizeof(int)+2] = vertex2;
			}
		}
	}
	
	std::cout << "Original triangle indices: ";
	for( int i = 0; i < 10; ++i )
		std::cout << " " << triangleVertexIndexArray[i] << " ";
	std::cout << "\n";

	ClothMeshDescription clothMeshDescription;
	clothMeshDescription.addIndexedMesh( mesh );
	clothMeshDescription.setStretchLinkLinearStiffness( 1.0f );
	clothMeshDescription.setBendLinkLinearStiffness( 0.7f );
	float *vertexMasses = new float[mesh.m_numVertices];
	for( int i = 0; i < mesh.m_numVertices; ++i )
	{
		vertexMasses[i] = 10.f/mesh.m_numVertices;
	}
	vertexMasses[(height-1)*width] = 0.f;
	vertexMasses[(height-1)*width + width/2] = 0.f;
	vertexMasses[(height-1)*width + width-1] = 0.f;

	clothMeshDescription.setVertexMasses(reinterpret_cast< const unsigned char* >(vertexMasses), sizeof(float));

	ClothDescription clothDesc( clothMeshDescription );
	
	
	// Appropriately transform the cloth
	using Vectormath::Aos::Matrix3;
	using Vectormath::Aos::Vector3;

	float rotateAngleRoundZ = 0.5;
	float rotateAngleRoundX = 0.5;
	Matrix3 defaultScale(Vector3(5.f, 0.f, 0.f), Vector3(0.f, 20.f, 0.f), Vector3(0.f, 0.f, 1.f));
	Matrix3 defaultRotate(
		Vector3(cos(rotateAngleRoundZ), sin(rotateAngleRoundZ), 0.f), 
		Vector3(-sin(rotateAngleRoundZ), cos(rotateAngleRoundZ), 0.f), 
		Vector3(0.f, 0.f, 1.f));
	Matrix3 defaultRotateX(
		Vector3(1.f, 0.f, 0.f), 
		Vector3( 0.f, cos(rotateAngleRoundX), sin(rotateAngleRoundX)), 
		Vector3(0.f, -sin(rotateAngleRoundX), cos(rotateAngleRoundX)));
	Matrix3 defaultRotateAndScale( (defaultRotateX*defaultRotate) * defaultScale );

	btAlignedObjectArray<BTAcceleratedSoftBody::Cloth *> flags;

	{
		Vector3 defaultTranslate(0.f, 30.f, -60.f);
		Vectormath::Aos::Transform3 transform( defaultRotateAndScale, defaultTranslate );
		clothDesc.setMeshTransform( transform );
		BTAcceleratedSoftBody::Cloth &newFlag( reinterpret_cast< btAcceleratedSoftRigidDynamicsWorld*  >(m_dynamicsWorld)->createSoftBody(clothDesc, device) );	
		flags.push_back( &newFlag );
	}
	{
		Vector3 defaultTranslate(0.f, 30.f, -30.f);
		Vectormath::Aos::Transform3 transform( defaultRotateAndScale, defaultTranslate );
		clothDesc.setMeshTransform( transform );
		BTAcceleratedSoftBody::Cloth &newFlag( reinterpret_cast< btAcceleratedSoftRigidDynamicsWorld*  >(m_dynamicsWorld)->createSoftBody(clothDesc, device) );	
		flags.push_back( &newFlag );
	}
	{
		Vector3 defaultTranslate(0.f, 30.f, 0.f);
		Vectormath::Aos::Transform3 transform( defaultRotateAndScale, defaultTranslate );
		clothDesc.setMeshTransform( transform );
		BTAcceleratedSoftBody::Cloth &newFlag( reinterpret_cast< btAcceleratedSoftRigidDynamicsWorld*  >(m_dynamicsWorld)->createSoftBody(clothDesc, device) );	
		flags.push_back( &newFlag );
	}
	{
		Vector3 defaultTranslate(0.f, 30.f, 30.f);
		Vectormath::Aos::Transform3 transform( defaultRotateAndScale, defaultTranslate );
		clothDesc.setMeshTransform( transform );
		BTAcceleratedSoftBody::Cloth &newFlag( reinterpret_cast< btAcceleratedSoftRigidDynamicsWorld*  >(m_dynamicsWorld)->createSoftBody(clothDesc, device) );	
		flags.push_back( &newFlag );
	}
	{
		Vector3 defaultTranslate(0.f, 30.f, 60.f);
		Vectormath::Aos::Transform3 transform( defaultRotateAndScale, defaultTranslate );
		clothDesc.setMeshTransform( transform );
		BTAcceleratedSoftBody::Cloth &newFlag( reinterpret_cast< btAcceleratedSoftRigidDynamicsWorld*  >(m_dynamicsWorld)->createSoftBody(clothDesc, device) );	
		flags.push_back( &newFlag );
	}

	delete [] vertexArray;
	delete [] triangleVertexIndexArray;

	return flags;
}


void	AcceleratedClothDemo::initPhysics()
{
	setTexturing(true);
	setShadows(true);

	setCameraDistance(btScalar(SCALING*50.));

	// Initialise CPU physics device
	m_cpuDevice = new BTAcceleratedSoftBody::CPUDevice;

	if( m_dxSupport.InitComputeShaderDevice() )
	{
		m_dx11Device = new BTAcceleratedSoftBody::DX11Device(m_dxSupport.getDevice(), m_dxSupport.getContext());
	} else {
		std::cerr << "No DX11 support\n";
		exit(1);
	}
	
	
	//m_dx11Device = new BTAcceleratedSoftBody::DX11Device;

	// TODO: Create function to initialise DX11 and create a device appropriately
//	m_dx11Device = 0;

	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btAcceleratedSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);

	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	///create a few basic rigid bodies
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
	
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0));

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

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}

#if 0
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btCollisionShape* colShape = new btBoxShape(btVector3(SCALING*1,SCALING*1,SCALING*1));
		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
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

		float start_x = START_POS_X - ARRAY_SIZE_X/2;
		float start_y = START_POS_Y;
		float start_z = START_POS_Z - ARRAY_SIZE_Z/2;

		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					startTransform.setOrigin(SCALING*btVector3(
										btScalar(2.0*i + start_x),
										btScalar(20+2.0*k + start_y),
										btScalar(2.0*j + start_z)));

			
					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
					btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
					btRigidBody* body = new btRigidBody(rbInfo);
					
					body->setActivationState(ISLAND_SLEEPING);

					m_dynamicsWorld->addRigidBody(body);
					body->setActivationState(ISLAND_SLEEPING);
				}
			}
		}
	}
#endif

	
	// Obtain a soft body description for a flag

	{
	// Create a series of flags on the CPU device
	//btAlignedObjectArray<BTAcceleratedSoftBody::Cloth *> flags = createFlag( *m_cpuDevice, 20, 40 );
	//btAlignedObjectArray<BTAcceleratedSoftBody::Cloth *> flags = createFlag( *m_cpuDevice, 4,8 );
 	m_flags = createFlag( *m_dx11Device, 20, 40 );
	//btAlignedObjectArray<BTAcceleratedSoftBody::Cloth *> flags = createFlag( *m_dx11Device, 4, 8 );

	}

	for( int flagIndex = 0; flagIndex < m_flags.size(); ++flagIndex )
	{		
		m_flags[flagIndex]->setWindVelocity( Vectormath::Aos::Vector3( 0.f, 0.f, 15.f ) );
	}


	clientResetScene();


}
	

void	AcceleratedClothDemo::exitPhysics()
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




