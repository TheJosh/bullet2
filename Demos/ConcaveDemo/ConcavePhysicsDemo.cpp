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

#include "CcdPhysicsEnvironment.h"
#include "CcdPhysicsController.h"
#include "MyMotionState.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btTetrahedronShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletCollision/BroadphaseCollision/btAxisSweep3.h"

#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/CollisionShapes/btTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "CollisionShapes/TriangleMesh.h"
#include "LinearMath/GenIDebugDraw.h"
#include "GLDebugDrawer.h"
#include "PHY_Pro.h"
#include "ConcaveDemo.h"
#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"


GLDebugDrawer	debugDrawer;

static const int NUM_VERTICES = 5;
static const int NUM_TRIANGLES=4;

SimdVector3	gVertices[NUM_VERTICES];
int	gIndices[NUM_TRIANGLES*3];
const float TRIANGLE_SIZE=80.f;


///User can override this material combiner by implementing gContactAddedCallback and setting body0->m_collisionFlags |= CollisionObject::customMaterialCallback;
inline SimdScalar	calculateCombinedFriction(float friction0,float friction1)
{
	SimdScalar friction = friction0 * friction1;

	const SimdScalar MAX_FRICTION  = 10.f;
	if (friction < -MAX_FRICTION)
		friction = -MAX_FRICTION;
	if (friction > MAX_FRICTION)
		friction = MAX_FRICTION;
	return friction;

}

inline SimdScalar	calculateCombinedRestitution(float restitution0,float restitution1)
{
	return restitution0 * restitution1;
}



bool CustomMaterialCombinerCallback(ManifoldPoint& cp,	const CollisionObject* colObj0,int partId0,int index0,const CollisionObject* colObj1,int partId1,int index1)
{

	float friction0 = colObj0->getFriction();
	float friction1 = colObj1->getFriction();
	float restitution0 = colObj0->getRestitution();
	float restitution1 = colObj1->getRestitution();

	if (colObj0->m_collisionFlags & CollisionObject::customMaterialCallback)
	{
		friction0 = 1.0;//partId0,index0
		restitution0 = 0.f;
	}
	if (colObj1->m_collisionFlags & CollisionObject::customMaterialCallback)
	{
		if (index1&1)
		{
			friction1 = 1.0f;//partId1,index1
		} else
		{
			friction1 = 0.f;
		}
		restitution1 = 0.f;
	}

	cp.m_combinedFriction = calculateCombinedFriction(friction0,friction1);
	cp.m_combinedRestitution = calculateCombinedRestitution(restitution0,restitution1);

	//this return value is currently ignored, but to be on the safe side: return false if you don't calculate friction
	return true;
}

extern ContactAddedCallback		gContactAddedCallback;



int main(int argc,char** argv)
{
	gContactAddedCallback = CustomMaterialCombinerCallback;

	ConcaveDemo* concaveDemo = new ConcaveDemo();
	concaveDemo->initPhysics();
	concaveDemo->setCameraDistance(30.f);

	return glutmain(argc, argv,640,480,"Static Concave Mesh Demo",concaveDemo);
}

void	ConcaveDemo::initPhysics()
{
	#define TRISIZE 10.f

	int vertStride = sizeof(SimdVector3);
	int indexStride = 3*sizeof(int);

	const int NUM_VERTS_X = 50;
	const int NUM_VERTS_Y = 50;
	const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;
	
	const int totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);

	SimdVector3*	gVertices = new SimdVector3[totalVerts];
	int*	gIndices = new int[totalTriangles*3];

	int i;

	for ( i=0;i<NUM_VERTS_X;i++)
	{
		for (int j=0;j<NUM_VERTS_Y;j++)
		{
			gVertices[i+j*NUM_VERTS_X].setValue((i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,2.f*sinf((float)i)*cosf((float)j),(j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);
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
	
	TriangleIndexVertexArray* indexVertexArrays = new TriangleIndexVertexArray(totalTriangles,
		gIndices,
		indexStride,
		totalVerts,(float*) &gVertices[0].x(),vertStride);

	CollisionShape* trimeshShape  = new BvhTriangleMeshShape(indexVertexArrays);
		

	
	// ConstraintSolver* solver = new SequentialImpulseConstraintSolver;
	
	CollisionDispatcher* dispatcher = new	CollisionDispatcher();
		
	OverlappingPairCache* broadphase = new SimpleBroadphase();


	m_physicsEnvironmentPtr = new CcdPhysicsEnvironment(dispatcher,broadphase);
	
		bool isDynamic = false;
	float mass = 0.f;
	SimdTransform	startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(SimdVector3(0,-2,0));

	CcdPhysicsController* staticTrimesh = LocalCreatePhysicsObject(isDynamic, mass, startTransform,trimeshShape);
	//enable custom material callback
	staticTrimesh->GetRigidBody()->m_collisionFlags |= CollisionObject::customMaterialCallback;

	{
		for (int i=0;i<10;i++)
		{
			CollisionShape* boxShape = new BoxShape(SimdVector3(1,1,1));
			startTransform.setOrigin(SimdVector3(2*i,1,1));
			LocalCreatePhysicsObject(true, 1, startTransform,boxShape);
		}
	}
	m_physicsEnvironmentPtr->setGravity(-1,-10,1);
	
	m_physicsEnvironmentPtr->setDebugDrawer(&debugDrawer);
}

void ConcaveDemo::clientMoveAndDisplay()
{
	 glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	float deltaTime = 1.f/60.f;

	m_physicsEnvironmentPtr->proceedDeltaTime(0.f,deltaTime);
	
	renderme();

    glFlush();
    glutSwapBuffers();

}

void	ConcaveDemo::clientResetScene()
{
	int numObj = m_physicsEnvironmentPtr->GetNumControllers();

	//skip ground
	for (int i=1;i<numObj;i++)
	{
		CcdPhysicsController* ctrl = m_physicsEnvironmentPtr->GetPhysicsController(i);
		ctrl->setPosition(2*i,1,1);
		ctrl->setOrientation(0,0,0,1);
		ctrl->SetLinearVelocity(0,0,0,0);
		ctrl->SetAngularVelocity(0,0,0,0);
	}
}




void ConcaveDemo::displayCallback(void) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

    glFlush();
    glutSwapBuffers();
}


