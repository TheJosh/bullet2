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


//#define USER_DEFINED_FRICTION_MODEL 1
//#define PRINT_CONTACT_STATISTICS 1
#define REGISTER_CUSTOM_COLLISION_ALGORITHM 1

#include "CcdPhysicsEnvironment.h"
#include "ParallelPhysicsEnvironment.h"

#include "CcdPhysicsController.h"
#include "MyMotionState.h"
//#include "GL_LineSegmentShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btTetrahedronShape.h"
#include "BulletCollision/CollisionShapes/btEmptyShape.h"
#include "BulletCollision/CollisionShapes/btTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btTriangleMesh.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"

#include "ParallelIslandDispatcher.h"

#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/BroadphaseCollision/btAxisSweep3.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"

#include "LinearMath/GenQuickprof.h"
#include "LinearMath/GenIDebugDraw.h"

#include "GLDebugDrawer.h"
#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"



#include "PHY_Pro.h"
#include "BMF_Api.h"
#include <stdio.h> //printf debugging

float deltaTime = 1.f/60.f;

#include "CcdPhysicsDemo.h"
#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"


extern float eye[3];
extern int glutScreenWidth;
extern int glutScreenHeight;

const int maxProxies = 32766;
const int maxOverlap = 65535;

bool createConstraint = true;//false;
bool useCompound = false;//true;//false;


#ifdef _DEBUG
const int gNumObjects = 120;
#else
const int gNumObjects = 120;//try this in release mode: 3000. never go above 16384, unless you increate maxNumObjects  value in DemoApplication.cp
#endif

const int maxNumObjects = 32760;

int	shapeIndex[maxNumObjects];

#ifdef USE_PARALLEL_DISPATCHER
ParallelPhysicsEnvironment* m_physicsEnvironmentPtr = 0;
#else
CcdPhysicsEnvironment* m_physicsEnvironmentPtr = 0;
#endif

#define CUBE_HALF_EXTENTS 1

#define EXTRA_HEIGHT -20.f
//GL_LineSegmentShape shapeE(SimdPoint3(-50,0,0),
//						   SimdPoint3(50,0,0));
static const int numShapes = 4;

CollisionShape* shapePtr[numShapes] = 
{
	///Please don't make the box sizes larger then 1000: the collision detection will be inaccurate.
	///See http://www.continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=346

//#define USE_GROUND_PLANE 1
#ifdef USE_GROUND_PLANE
	new StaticPlaneShape(SimdVector3(0,1,0),10),
#else
	new BoxShape (SimdVector3(50,10,50)),
#endif
		
		new BoxShape (SimdVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)),
		new SphereShape (CUBE_HALF_EXTENTS- 0.05f),

		//new ConeShape(CUBE_HALF_EXTENTS,2.f*CUBE_HALF_EXTENTS),
		//new BU_Simplex1to4(SimdPoint3(-1,-1,-1),SimdPoint3(1,-1,-1),SimdPoint3(-1,1,-1),SimdPoint3(0,0,1)),

		//new EmptyShape(),

		new BoxShape (SimdVector3(0.4,1,0.8))

};



////////////////////////////////////






GLDebugDrawer debugDrawer;

int main(int argc,char** argv)
{

	CcdPhysicsDemo* ccdDemo = new CcdPhysicsDemo();

	ccdDemo->initPhysics();

	ccdDemo->setCameraDistance(26.f);

	return glutmain(argc, argv,640,480,"Bullet Physics Demo. http://www.continuousphysics.com/Bullet/phpBB2/",ccdDemo);
}



extern int gNumManifold;
extern int gOverlappingPairs;
extern int gTotalContactPoints;

void CcdPhysicsDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 



	if (m_physicsEnvironmentPtr)
		m_physicsEnvironmentPtr->proceedDeltaTime(0.f,deltaTime);

#ifdef USE_QUICKPROF 
        Profiler::beginBlock("render"); 
#endif //USE_QUICKPROF 
	
	renderme(); 

#ifdef USE_QUICKPROF 
        Profiler::endBlock("render"); 
#endif 
	glFlush();
	//some additional debugging info
#ifdef PRINT_CONTACT_STATISTICS
	printf("num manifolds: %i\n",gNumManifold);
	printf("num gOverlappingPairs: %i\n",gOverlappingPairs);
	printf("num gTotalContactPoints : %i\n",gTotalContactPoints );
#endif //PRINT_CONTACT_STATISTICS

	gTotalContactPoints = 0;
	glutSwapBuffers();

}



void CcdPhysicsDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	if (m_physicsEnvironmentPtr)
	{
		m_physicsEnvironmentPtr->UpdateAabbs(deltaTime);
		//draw contactpoints
		m_physicsEnvironmentPtr->CallbackTriggers();
	}

	renderme();

	glFlush();
	glutSwapBuffers();
}



///make this positive to show stack falling from a distance
///this shows the penalty tresholds in action, springy/spungy look

void CcdPhysicsDemo::clientResetScene()
{

	
	int i;
	int numObjects = m_physicsEnvironmentPtr->GetNumControllers();

	for (i=0;i<numObjects;i++)
	{
		//skip the first object (static ground)
		if (i>0)
		{
			CcdPhysicsController* ctrl = m_physicsEnvironmentPtr->GetPhysicsController(i);

			if ((getDebugMode() & IDebugDraw::DBG_NoHelpText))
			{
				if (ctrl->GetRigidBody()->GetCollisionShape()->GetShapeType() != SPHERE_SHAPE_PROXYTYPE)
				{
					ctrl->GetRigidBody()->SetCollisionShape(shapePtr[2]);
				} else
				{
					ctrl->GetRigidBody()->SetCollisionShape(shapePtr[1]);
				}

				BroadphaseProxy* bpproxy = ctrl->GetRigidBody()->m_broadphaseHandle;
				m_physicsEnvironmentPtr->GetBroadphase()->CleanProxyFromPairs(bpproxy);
			}

			//stack them
			int colsize = 10;
			int row = (i*CUBE_HALF_EXTENTS*2)/(colsize*2*CUBE_HALF_EXTENTS);
			int row2 = row;
			int col = (i)%(colsize)-colsize/2;


			if (col>3)
			{
				col=11;
				row2 |=1;
			}
			ctrl->setPosition(col*2*CUBE_HALF_EXTENTS + (row2%2)*CUBE_HALF_EXTENTS,
				row*2*CUBE_HALF_EXTENTS+CUBE_HALF_EXTENTS+EXTRA_HEIGHT,0);
			ctrl->setOrientation(0,0,0,1);
			ctrl->SetLinearVelocity(0,0,0,false);
			ctrl->SetAngularVelocity(0,0,0,false);
		} 
	}
}


///User-defined friction model, the most simple friction model available: no friction
float myFrictionModel(	RigidBody& body1,	RigidBody& body2,	ManifoldPoint& contactPoint,	const ContactSolverInfo& solverInfo	)
{
	//don't do any friction
	return 0.f;
}

void	CcdPhysicsDemo::initPhysics()
{

	CollisionDispatcher* dispatcher = new	CollisionDispatcher();

	SimdVector3 worldAabbMin(-10000,-10000,-10000);
	SimdVector3 worldAabbMax(10000,10000,10000);

	OverlappingPairCache* broadphase = new AxisSweep3(worldAabbMin,worldAabbMax,maxProxies);
//	OverlappingPairCache* broadphase = new SimpleBroadphase;
	
#ifdef REGISTER_CUSTOM_COLLISION_ALGORITHM
	dispatcher->RegisterCollisionCreateFunc(SPHERE_SHAPE_PROXYTYPE,SPHERE_SHAPE_PROXYTYPE,new SphereSphereCollisionAlgorithm::CreateFunc);
#endif //REGISTER_CUSTOM_COLLISION_ALGORITHM


#ifdef USE_PARALLEL_DISPATCHER
	m_physicsEnvironmentPtr = new ParallelPhysicsEnvironment(dispatcher2,broadphase);
#else
	m_physicsEnvironmentPtr = new CcdPhysicsEnvironment(dispatcher,broadphase);
#endif
	m_physicsEnvironmentPtr->setDeactivationTime(2.f);

	m_physicsEnvironmentPtr->setGravity(0,-10,0);

	m_physicsEnvironmentPtr->setDebugDrawer(&debugDrawer);

#ifdef USER_DEFINED_FRICTION_MODEL
	SequentialImpulseConstraintSolver* solver = (SequentialImpulseConstraintSolver*) m_physicsEnvironmentPtr->GetConstraintSolver();
	//solver->SetContactSolverFunc(ContactSolverFunc func,USER_CONTACT_SOLVER_TYPE1,DEFAULT_CONTACT_SOLVER_TYPE);
	solver->SetFrictionSolverFunc(myFrictionModel,USER_CONTACT_SOLVER_TYPE1,DEFAULT_CONTACT_SOLVER_TYPE);
	solver->SetFrictionSolverFunc(myFrictionModel,DEFAULT_CONTACT_SOLVER_TYPE,USER_CONTACT_SOLVER_TYPE1);
	solver->SetFrictionSolverFunc(myFrictionModel,USER_CONTACT_SOLVER_TYPE1,USER_CONTACT_SOLVER_TYPE1);
	//m_physicsEnvironmentPtr->setNumIterations(2);
#endif //USER_DEFINED_FRICTION_MODEL


	int i;

	SimdTransform tr;
	tr.setIdentity();

	
	for (i=0;i<gNumObjects;i++)
	{
		if (i>0)
		{
			shapeIndex[i] = 1;//sphere
		}
		else
			shapeIndex[i] = 0;
	}

	if (useCompound)
	{
		CompoundShape* compoundShape = new CompoundShape();
		CollisionShape* oldShape = shapePtr[1];
		shapePtr[1] = compoundShape;

		SimdTransform ident;
		ident.setIdentity();
		ident.setOrigin(SimdVector3(0,0,0));	
		compoundShape->AddChildShape(ident,oldShape);//
		ident.setOrigin(SimdVector3(0,0,2));	
		compoundShape->AddChildShape(ident,new SphereShape(0.9));//
	}

	for (i=0;i<gNumObjects;i++)
	{
		CollisionShape* shape = shapePtr[shapeIndex[i]];
		shape->SetMargin(0.05f);

		bool isDyna = i>0;

		SimdTransform trans;
		trans.setIdentity();
		
		if (i>0)
		{
			//stack them
			int colsize = 10;
			int row = (i*CUBE_HALF_EXTENTS*2)/(colsize*2*CUBE_HALF_EXTENTS);
			int row2 = row;
			int col = (i)%(colsize)-colsize/2;


			if (col>3)
			{
				col=11;
				row2 |=1;
			}

			SimdVector3 pos(col*2*CUBE_HALF_EXTENTS + (row2%2)*CUBE_HALF_EXTENTS,
				row*2*CUBE_HALF_EXTENTS+CUBE_HALF_EXTENTS+EXTRA_HEIGHT,0);

			trans.setOrigin(pos);
		} else
		{
			trans.setOrigin(SimdVector3(0,-30,0));
		}

		float mass = 1.f;

		if (!isDyna)
			mass = 0.f;
	
		CcdPhysicsController* ctrl = LocalCreatePhysicsObject(isDyna,mass,trans,shape);

		// Only do CCD if  motion in one timestep (1.f/60.f) exceeds CUBE_HALF_EXTENTS
		ctrl->GetRigidBody()->m_ccdSquareMotionTreshold = CUBE_HALF_EXTENTS;
		
		//Experimental: better estimation of CCD Time of Impact:
		ctrl->GetRigidBody()->m_ccdSweptShereRadius = 0.2*CUBE_HALF_EXTENTS;
#ifdef USER_DEFINED_FRICTION_MODEL	
		///Advanced use: override the friction solver
		ctrl->GetRigidBody()->m_frictionSolverType = USER_CONTACT_SOLVER_TYPE1;
#endif //USER_DEFINED_FRICTION_MODEL

	}


	clientResetScene();

	m_physicsEnvironmentPtr->SyncMotionStates(0.f);

}
	



