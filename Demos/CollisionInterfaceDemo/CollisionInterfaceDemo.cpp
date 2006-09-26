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


///
/// CollisionInterfaceDemo shows high level usage of the Collision Detection.
///

#include "GL_Simplex1to4.h"

//include common Bullet Collision Detection headerfiles
#include "btBulletCollisionCommon.h"

#include "LinearMath/GenIDebugDraw.h"
#include "GL_ShapeDrawer.h"
#include "CollisionInterfaceDemo.h"
#include "GlutStuff.h"


float yaw=0.f,pitch=0.f,roll=0.f;
const int maxNumObjects = 4;
const int numObjects = 2;

GL_Simplex1to4 simplex;


btCollisionObject	objects[maxNumObjects];
btCollisionWorld*	collisionWorld = 0;

int screenWidth = 640;
int screenHeight = 480;


int main(int argc,char** argv)
{
	CollisionInterfaceDemo* collisionInterfaceDemo = new CollisionInterfaceDemo();

	collisionInterfaceDemo->initPhysics();

	collisionInterfaceDemo->clientResetScene();

	return glutmain(argc, argv,screenWidth,screenHeight,"Collision Interface Demo",collisionInterfaceDemo);
}

void	CollisionInterfaceDemo::initPhysics()
{
			
	m_debugMode |= btIDebugDraw::DBG_DrawWireframe;
	
	btSimdMatrix3x3 basisA;
	basisA.setIdentity();

	btSimdMatrix3x3 basisB;
	basisB.setIdentity();

	objects[0].m_worldTransform.setBasis(basisA);
	objects[1].m_worldTransform.setBasis(basisB);

	//SimdPoint3	points0[3]={SimdPoint3(1,0,0),SimdPoint3(0,1,0),SimdPoint3(0,0,1)};
	//SimdPoint3	points1[5]={SimdPoint3(1,0,0),SimdPoint3(0,1,0),SimdPoint3(0,0,1),SimdPoint3(0,0,-1),SimdPoint3(-1,-1,0)};
	
	btBoxShape* boxA = new btBoxShape(btSimdVector3(1,1,1));
	btBoxShape* boxB = new btBoxShape(btSimdVector3(0.5,0.5,0.5));
	//ConvexHullShape	hullA(points0,3);
	//hullA.setLocalScaling(btSimdVector3(3,3,3));
	//ConvexHullShape	hullB(points1,4);
	//hullB.setLocalScaling(btSimdVector3(4,4,4));

	objects[0].m_collisionShape = boxA;//&hullA;
	objects[1].m_collisionShape = boxB;//&hullB;

	btCollisionDispatcher* dispatcher = new btCollisionDispatcher;
	btSimdVector3	worldAabbMin(-1000,-1000,-1000);
	btSimdVector3	worldAabbMax(1000,1000,1000);

	btAxisSweep3*	broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax);
	
	//SimpleBroadphase is a brute force alternative, performing N^2 aabb overlap tests
	//SimpleBroadphase*	broadphase = new btSimpleBroadphase;

	collisionWorld = new btCollisionWorld(dispatcher,broadphase);
	
	collisionWorld->AddCollisionObject(&objects[0]);
	collisionWorld->AddCollisionObject(&objects[1]);

}


//to be implemented by the demo

void CollisionInterfaceDemo::clientMoveAndDisplay()
{
	
	displayCallback();
}


static btVoronoiSimplexSolver sGjkSimplexSolver;
btSimplexSolverInterface& gGjkSimplexSolver = sGjkSimplexSolver;



void CollisionInterfaceDemo::displayCallback(void) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	glDisable(GL_LIGHTING);

	if (collisionWorld)
		collisionWorld->PerformDiscreteCollisionDetection();
	
	int i;

	///one way to draw all the contact points is iterating over contact manifolds / points:

	int numManifolds = collisionWorld->GetDispatcher()->GetNumManifolds();
	for (i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold = collisionWorld->GetDispatcher()->GetManifoldByIndexInternal(i);
		btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->GetBody0());
		btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->GetBody1());
		contactManifold->RefreshContactPoints(obA->m_worldTransform,obB->m_worldTransform);

		int numContacts = contactManifold->GetNumContacts();
		for (int j=0;j<numContacts;j++)
		{
			btManifoldPoint& pt = contactManifold->GetContactPoint(j);

			glBegin(GL_LINES);
			glColor3f(1, 0, 1);
			
			btSimdVector3 ptA = pt.GetPositionWorldOnA();
			btSimdVector3 ptB = pt.GetPositionWorldOnB();

			glVertex3d(ptA.x(),ptA.y(),ptA.z());
			glVertex3d(ptB.x(),ptB.y(),ptB.z());
			glEnd();
		}

		//you can un-comment out this line, and then all points are removed
		//contactManifold->ClearManifold();	
	}

	//GL_ShapeDrawer::DrawCoordSystem();

	float m[16];
	


	for (i=0;i<numObjects;i++)
	{
		
		objects[i].m_worldTransform.getOpenGLMatrix( m );
		GL_ShapeDrawer::DrawOpenGL(m,objects[i].m_collisionShape,btSimdVector3(1,1,1),getDebugMode());

	}


	btSimdQuaternion orn;
	orn.setEuler(yaw,pitch,roll);
	objects[1].m_worldTransform.setOrigin(objects[1].m_worldTransform.getOrigin()+btSimdVector3(0,-0.01,0));

	objects[0].m_worldTransform.setRotation(orn);

	pitch += 0.005f;
	yaw += 0.01f;

	glFlush();
    glutSwapBuffers();
}

void CollisionInterfaceDemo::clientResetScene()
{
	objects[0].m_worldTransform.setOrigin(btSimdVector3(0.0f,3.f,0.f));
	objects[1].m_worldTransform.setOrigin(btSimdVector3(0.0f,9.f,0.f));
}


