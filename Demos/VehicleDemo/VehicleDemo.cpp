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

//Ignore this USE_PARALLEL_DISPATCHER define, it is for future optimizations
//#define USE_PARALLEL_DISPATCHER 1

/// September 2006: VehicleDemo is work in progress, this file is mostly just a placeholder
/// This VehicleDemo file is very early in development, please check it later

#include "CcdPhysicsEnvironment.h"
#include "ParallelPhysicsEnvironment.h"
#include "CcdPhysicsController.h"
#include "btBulletDynamicsCommon.h"

#include "PHY_IVehicle.h"

#include "ParallelIslandDispatcher.h"

#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"

#include "PHY_Pro.h"
#include "BMF_Api.h"
#include <stdio.h> //printf debugging

float deltaTime = 1.f/60.f;


#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"
#include "VehicleDemo.h"

const int maxProxies = 32766;
const int maxOverlap = 65535;


DefaultMotionState wheelMotionState[4];

///PHY_IVehicle is the interface behind the constraint that implements the raycast vehicle (WrapperVehicle which holds a btRaycastVehicle)
///notice that for higher-quality slow-moving vehicles, another approach might be better
///implementing explicit hinged-wheel constraints with cylinder collision, rather then raycasts
PHY_IVehicle* gVehicleConstraint=0;
float	gEngineForce = 0.f;
float	maxEngineForce = 1000.f;
float	gVehicleSteering = 0.f;
float	steeringIncrement = 0.1f;
float	steeringClamp = 0.3f;
float	wheelRadius = 0.5f;
float	wheelWidth = 0.2f;
float	wheelFriction = 100.f;
float	suspensionStiffness = 10.f;
float	suspensionDamping = 1.3f;
float	suspensionCompression = 2.4f;
float	rollInfluence = 0.1f;
btVector3 wheelDirectionCS0(0,-1,0);
btVector3 wheelAxleCS(1,0,0);
btScalar suspensionRestLength(0.6);

#define CUBE_HALF_EXTENTS 1



////////////////////////////////////



GLDebugDrawer debugDrawer;

int main(int argc,char** argv)
{

	VehicleDemo* vehicleDemo = new VehicleDemo;

	vehicleDemo->setupPhysics();

	return glutmain(argc, argv,640,480,"Bullet Vehicle Demo. http://www.continuousphysics.com/Bullet/phpBB2/", vehicleDemo);
}

VehicleDemo::VehicleDemo()
:
m_carChassis(0),
m_cameraHeight(4.f),
m_minCameraDistance(3.f),
m_maxCameraDistance(10.f)
{
	m_cameraPosition = btVector3(30,30,30);
}

void VehicleDemo::setupPhysics()
{

	btCollisionDispatcher* dispatcher = new	btCollisionDispatcher();
	ParallelIslandDispatcher* dispatcher2 = new	ParallelIslandDispatcher();
	
	btVector3 worldAabbMin(-30000,-30000,-30000);
	btVector3 worldAabbMax(30000,30000,30000);

	btOverlappingPairCache* broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);
	//OverlappingPairCache* broadphase = new btSimpleBroadphase(maxProxies,maxOverlap);

#ifdef USE_PARALLEL_DISPATCHER
	m_physicsEnvironmentPtr = new ParallelPhysicsEnvironment(dispatcher2,broadphase);
#else
	m_physicsEnvironmentPtr = new CcdPhysicsEnvironment(dispatcher,broadphase);
#endif
	m_physicsEnvironmentPtr->setDeactivationTime(2.f);

	m_physicsEnvironmentPtr->setDebugDrawer(&debugDrawer);

	m_physicsEnvironmentPtr->setGravity(0,-10,0);//0,0);//-10,0);
	int i;

	btCollisionShape* groundShape = new btBoxShape(btVector3(50,3,50));

#define  USE_TRIMESH_GROUND 1
#ifdef USE_TRIMESH_GROUND


const float TRIANGLE_SIZE=20.f;

	//create a triangle-mesh ground
	int vertStride = sizeof(btVector3);
	int indexStride = 3*sizeof(int);

	const int NUM_VERTS_X = 50;
	const int NUM_VERTS_Y = 50;
	const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;
	
	const int totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);

	btVector3*	gVertices = new btVector3[totalVerts];
	int*	gIndices = new int[totalTriangles*3];

	

	for ( i=0;i<NUM_VERTS_X;i++)
	{
		for (int j=0;j<NUM_VERTS_Y;j++)
		{
			gVertices[i+j*NUM_VERTS_X].setValue((i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,2.f*sinf((float)i)*cosf((float)j)+10.f,(j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);
		}
	}

	int index=0;
	for ( i=0;i<NUM_VERTS_X-1;i++)
	{
		for (int j=0;j<NUM_VERTS_Y-1;j++)
		{
			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = j*NUM_VERTS_X+i+1;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;

			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i;
		}
	}
	
	btTriangleIndexVertexArray* indexVertexArrays = new btTriangleIndexVertexArray(totalTriangles,
		gIndices,
		indexStride,
		totalVerts,(float*) &gVertices[0].x(),vertStride);

	groundShape = new btBvhTriangleMeshShape(indexVertexArrays);
	
#endif //

	btTransform tr;
	tr.setIdentity();

	tr.setOrigin(btVector3(0,-20.f,0));

	//create ground object
	LocalCreatePhysicsObject(false,0,tr,groundShape);

	btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f,0.5f,2.f));
	tr.setOrigin(btVector3(0,0.f,0));

	m_carChassis = LocalCreatePhysicsObject(true,800,tr,chassisShape);
	
	clientResetScene();

	m_physicsEnvironmentPtr->SyncMotionStates(0.f);

	/// create vehicle
	{
		int constraintId;

		constraintId =m_physicsEnvironmentPtr->createConstraint(
		m_carChassis,0,
			PHY_VEHICLE_CONSTRAINT,
			0,0,0,
			0,0,0);

		///never deactivate the vehicle
		m_carChassis->GetRigidBody()->SetActivationState(DISABLE_DEACTIVATION);
		
		gVehicleConstraint = m_physicsEnvironmentPtr->getVehicleConstraint(constraintId);

		btVector3 connectionPointCS0(CUBE_HALF_EXTENTS-(0.3*wheelWidth),0,2*CUBE_HALF_EXTENTS-wheelRadius);
		btRaycastVehicle::btVehicleTuning tuning;
		bool isFrontWheel=true;
		int rightIndex = 0;
		int upIndex = 1;
		int forwardIndex = 2;

		gVehicleConstraint->SetCoordinateSystem(rightIndex,upIndex,forwardIndex);

		gVehicleConstraint->AddWheel(&wheelMotionState[0],
			(PHY__Vector3&)connectionPointCS0,
			(PHY__Vector3&)wheelDirectionCS0,(PHY__Vector3&)wheelAxleCS,suspensionRestLength,wheelRadius,isFrontWheel);

		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),0,2*CUBE_HALF_EXTENTS-wheelRadius);
		gVehicleConstraint->AddWheel(&wheelMotionState[1],
			(PHY__Vector3&)connectionPointCS0,
			(PHY__Vector3&)wheelDirectionCS0,(PHY__Vector3&)wheelAxleCS,suspensionRestLength,wheelRadius,isFrontWheel);

		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),0,-2*CUBE_HALF_EXTENTS+wheelRadius);
		isFrontWheel = false;
		gVehicleConstraint->AddWheel(&wheelMotionState[2],
			(PHY__Vector3&)connectionPointCS0,
			(PHY__Vector3&)wheelDirectionCS0,(PHY__Vector3&)wheelAxleCS,suspensionRestLength,wheelRadius,isFrontWheel);
		
		connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3*wheelWidth),0,-2*CUBE_HALF_EXTENTS+wheelRadius);
		gVehicleConstraint->AddWheel(&wheelMotionState[3],
			(PHY__Vector3&)connectionPointCS0,
			(PHY__Vector3&)wheelDirectionCS0,(PHY__Vector3&)wheelAxleCS,suspensionRestLength,wheelRadius,isFrontWheel);
		
	

		gVehicleConstraint->SetSuspensionStiffness(suspensionStiffness,0);
		gVehicleConstraint->SetSuspensionStiffness(suspensionStiffness,1);
		gVehicleConstraint->SetSuspensionStiffness(suspensionStiffness,2);
		gVehicleConstraint->SetSuspensionStiffness(suspensionStiffness,3);
		
		gVehicleConstraint->SetSuspensionDamping(suspensionDamping,0);
		gVehicleConstraint->SetSuspensionDamping(suspensionDamping,1);
		gVehicleConstraint->SetSuspensionDamping(suspensionDamping,2);
		gVehicleConstraint->SetSuspensionDamping(suspensionDamping,3);
		
		gVehicleConstraint->SetSuspensionCompression(suspensionCompression,0);
		gVehicleConstraint->SetSuspensionCompression(suspensionCompression,1);
		gVehicleConstraint->SetSuspensionCompression(suspensionCompression,2);
		gVehicleConstraint->SetSuspensionCompression(suspensionCompression,3);

		gVehicleConstraint->SetWheelFriction(wheelFriction,0);
		gVehicleConstraint->SetWheelFriction(wheelFriction,1);
		gVehicleConstraint->SetWheelFriction(wheelFriction,2);
		gVehicleConstraint->SetWheelFriction(wheelFriction,3);

		
	}

	
	setCameraDistance(26.f);

}


//to be implemented by the demo
void VehicleDemo::renderme()
{
	updateCamera();

	debugDrawer.SetDebugMode(getDebugMode());
	float m[16];
	int i;

	btCylinderShapeX wheelShape(btVector3(wheelWidth,wheelRadius,wheelRadius));
	btVector3 wheelColor(1,0,0);

	for (i=0;i<4;i++)
	{
		//draw wheels (cylinders)
		wheelMotionState[i].m_worldTransform.getOpenGLMatrix(m);
		GL_ShapeDrawer::DrawOpenGL(m,&wheelShape,wheelColor,getDebugMode());
	}

	DemoApplication::renderme();

}

void VehicleDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 


{			
		int steerWheelIndex = 2;
		gVehicleConstraint->ApplyEngineForce(gEngineForce,steerWheelIndex);
		steerWheelIndex = 3;
		gVehicleConstraint->ApplyEngineForce(gEngineForce,steerWheelIndex);

		steerWheelIndex = 0;
		gVehicleConstraint->SetSteeringValue(gVehicleSteering,steerWheelIndex);
		steerWheelIndex = 1;
		gVehicleConstraint->SetSteeringValue(gVehicleSteering,steerWheelIndex);

	}

	m_physicsEnvironmentPtr->proceedDeltaTime(0.f,deltaTime);

	


#ifdef USE_QUICKPROF 
        btProfiler::beginBlock("render"); 
#endif //USE_QUICKPROF 


	renderme(); 

#ifdef USE_QUICKPROF 
        btProfiler::endBlock("render"); 
#endif 
	glFlush();
	glutSwapBuffers();

}



void VehicleDemo::displayCallback(void) 
{

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 


	m_physicsEnvironmentPtr->UpdateAabbs(deltaTime);
	//draw contactpoints
	m_physicsEnvironmentPtr->CallbackTriggers();


	renderme();


	glFlush();
	glutSwapBuffers();
}



void VehicleDemo::clientResetScene()
{
	gEngineForce = 0.f;
	gVehicleSteering = 0.f;
	m_carChassis->setPosition(0,0,0);
	m_carChassis->setOrientation(0,0,0,1);
}



void VehicleDemo::specialKeyboard(int key, int x, int y)
{
	printf("key = %i x=%i y=%i\n",key,x,y);

    switch (key) 
    {
    case GLUT_KEY_LEFT : 
		{
			gVehicleSteering += steeringIncrement;
			if (	gVehicleSteering > steeringClamp)
					gVehicleSteering = steeringClamp;

		break;
		}
    case GLUT_KEY_RIGHT : 
		{
			gVehicleSteering -= steeringIncrement;
			if (	gVehicleSteering < -steeringClamp)
					gVehicleSteering = -steeringClamp;

		break;
		}
    case GLUT_KEY_UP :
		{
			gEngineForce = -maxEngineForce;
		break;
		}
	case GLUT_KEY_DOWN :
		{			
			gEngineForce = maxEngineForce;
		break;
		}
	default:
		DemoApplication::specialKeyboard(key,x,y);
        break;
    }

//	glutPostRedisplay();

}



void	VehicleDemo::updateCamera()
{

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	//look at the vehicle
	m_cameraTargetPosition = m_carChassis->GetRigidBody()->m_worldTransform.getOrigin();

	//interpolate the camera height
	m_cameraPosition[1] = (15.0*m_cameraPosition[1] + m_cameraTargetPosition[1] + m_cameraHeight)/16.0;

	btVector3 camToObject = m_cameraTargetPosition - m_cameraPosition;

	//keep distance between min and max distance
	float cameraDistance = camToObject.length();
	float correctionFactor = 0.f;
	if (cameraDistance < m_minCameraDistance)
	{
		correctionFactor = 0.15*(m_minCameraDistance-cameraDistance)/cameraDistance;
	}
	if (cameraDistance > m_maxCameraDistance)
	{
		correctionFactor = 0.15*(m_maxCameraDistance-cameraDistance)/cameraDistance;
	}
	m_cameraPosition -= correctionFactor*camToObject;
	
	//update OpenGL camera settings
    glFrustum(-1.0, 1.0, -1.0, 1.0, 1.0, 10000.0);

    gluLookAt(m_cameraPosition[0],m_cameraPosition[1],m_cameraPosition[2],
		      m_cameraTargetPosition[0],m_cameraTargetPosition[1], m_cameraTargetPosition[2],
			  m_cameraUp.getX(),m_cameraUp.getY(),m_cameraUp.getZ());
    glMatrixMode(GL_MODELVIEW);

}

