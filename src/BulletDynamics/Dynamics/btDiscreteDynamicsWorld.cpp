
#include "btDiscreteDynamicsWorld.h"

//collision detection
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"

//rigidbody & constraints
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"

//vehicle
#include "BulletDynamics/Vehicle/btRaycastVehicle.h"
#include "BulletDynamics/Vehicle/btVehicleRaycaster.h"
#include "BulletDynamics/Vehicle/btWheelInfo.h"

#include <algorithm>

btDiscreteDynamicsWorld::btDiscreteDynamicsWorld()
:btDynamicsWorld(new	btCollisionDispatcher(),new btSimpleBroadphase()),
m_constraintSolver(new btSequentialImpulseConstraintSolver)
{
	m_islandManager = new btSimulationIslandManager();
}

btDiscreteDynamicsWorld::btDiscreteDynamicsWorld(btDispatcher* dispatcher,btOverlappingPairCache* pairCache,btConstraintSolver* constraintSolver)
:btDynamicsWorld(dispatcher,pairCache),
m_constraintSolver(constraintSolver)
{
	m_islandManager = new btSimulationIslandManager();
}


btDiscreteDynamicsWorld::~btDiscreteDynamicsWorld()
{
	delete m_islandManager ;

	delete m_constraintSolver;

	//delete the dispatcher and paircache
	delete m_dispatcher1;
	m_dispatcher1 = 0;
	delete m_pairCache;
	m_pairCache = 0;
}

void	btDiscreteDynamicsWorld::stepSimulation(float timeStep)
{
	///update aabbs information
	updateAabbs();

	///apply gravity, predict motion
	predictUnconstraintMotion(timeStep);

	///perform collision detection
	PerformDiscreteCollisionDetection();

	calculateSimulationIslands();

	btContactSolverInfo infoGlobal;
	infoGlobal.m_timeStep = timeStep;
	
	///solve non-contact constraints
	solveNoncontactConstraints(infoGlobal);
	
	///solve contact constraints
	solveContactConstraints(infoGlobal);

	///update vehicle simulation
	updateVehicles(timeStep);
	
	///CallbackTriggers();

	///integrate transforms
	integrateTransforms(timeStep);
		
	updateActivationState( timeStep );

	

}

void	btDiscreteDynamicsWorld::updateVehicles(float timeStep)
{
	for (int i=0;i<m_vehicles.size();i++)
	{
		btRaycastVehicle* vehicle = m_vehicles[i];
		vehicle->UpdateVehicle( timeStep);
	}
}

void	btDiscreteDynamicsWorld::updateActivationState(float timeStep)
{
	for (int i=0;i<m_collisionObjects.size();i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		if (colObj->m_internalOwner)
		{
			btRigidBody* body = (btRigidBody*)colObj->m_internalOwner;
	
			body->updateDeactivation(timeStep);

			if (body->wantsSleeping())
			{
				if (body->GetActivationState() == ACTIVE_TAG)
					body->SetActivationState( WANTS_DEACTIVATION );
			} else
			{
				if (body->GetActivationState() != DISABLE_DEACTIVATION)
					body->SetActivationState( ACTIVE_TAG );
			}
		}
	}
}

void	btDiscreteDynamicsWorld::addConstraint(btTypedConstraint* constraint)
{
	m_constraints.push_back(constraint);
}

void	btDiscreteDynamicsWorld::removeConstraint(btTypedConstraint* constraint)
{
	std::vector<btTypedConstraint*>::iterator cit = std::find(m_constraints.begin(),m_constraints.end(),constraint);
	if (!(cit==m_constraints.end()))
	{
		m_constraints.erase(cit);
	}
}

void	btDiscreteDynamicsWorld::addVehicle(btRaycastVehicle* vehicle)
{
	m_vehicles.push_back(vehicle);
}

void	btDiscreteDynamicsWorld::removeVehicle(btRaycastVehicle* vehicle)
{
	std::vector<btRaycastVehicle*>::iterator vit = std::find(m_vehicles.begin(),m_vehicles.end(),vehicle);
	if (!(vit==m_vehicles.end()))
	{
		m_vehicles.erase(vit);
	}
}


void	btDiscreteDynamicsWorld::solveContactConstraints(btContactSolverInfo& solverInfo)
{
	
	struct InplaceSolverIslandCallback : public btSimulationIslandManager::IslandCallback
	{

		btContactSolverInfo& m_solverInfo;
		btConstraintSolver*	m_solver;
		btIDebugDraw*	m_debugDrawer;

		InplaceSolverIslandCallback(
			btContactSolverInfo& solverInfo,
			btConstraintSolver*	solver,
			btIDebugDraw*	debugDrawer)
			:m_solverInfo(solverInfo),
			m_solver(solver),
			m_debugDrawer(debugDrawer)
		{

		}

		virtual	void	ProcessIsland(btPersistentManifold**	manifolds,int numManifolds)
		{
			m_solver->SolveGroup( manifolds, numManifolds,m_solverInfo,m_debugDrawer);
		}

	};

	btIDebugDraw* debugDraw = 0;
	InplaceSolverIslandCallback	solverCallback(	solverInfo,	m_constraintSolver,	debugDraw);

	
	/// solve all the contact points and contact friction
	m_islandManager->BuildAndProcessIslands(GetCollisionWorld()->GetDispatcher(),GetCollisionWorld()->GetCollisionObjectArray(),&solverCallback);


}


void	btDiscreteDynamicsWorld::solveNoncontactConstraints(btContactSolverInfo& solverInfo)
{
	#ifdef USE_QUICKPROF
	Profiler::beginBlock("SolveConstraint");
#endif //USE_QUICKPROF



	int i;
	int numConstraints = m_constraints.size();

	///constraint preparation: building jacobians
	for (i=0;i< numConstraints ; i++ )
	{
		btTypedConstraint* constraint = m_constraints[i];
		constraint->BuildJacobian();
	}

	//solve the regular non-contact constraints (point 2 point, hinge, generic d6)
	for (int g=0;g<solverInfo.m_numIterations;g++)
	{
		//
		// constraint solving
		//
		for (i=0;i< numConstraints ; i++ )
		{
			btTypedConstraint* constraint = m_constraints[i];
			constraint->SolveConstraint( solverInfo.m_timeStep );
		}
	}

#ifdef USE_QUICKPROF
	Profiler::endBlock("SolveConstraint");
#endif //USE_QUICKPROF

}

void	btDiscreteDynamicsWorld::calculateSimulationIslands()
{
	
#ifdef USE_QUICKPROF
	Profiler::beginBlock("IslandUnionFind");
#endif //USE_QUICKPROF

	GetSimulationIslandManager()->UpdateActivationState(GetCollisionWorld(),GetCollisionWorld()->GetDispatcher());

	{
		int i;
		int numConstraints = m_constraints.size();
		for (i=0;i< numConstraints ; i++ )
		{
			btTypedConstraint* constraint = m_constraints[i];

			const btRigidBody* colObj0 = &constraint->GetRigidBodyA();
			const btRigidBody* colObj1 = &constraint->GetRigidBodyB();

			if (((colObj0) && ((colObj0)->mergesSimulationIslands())) &&
				((colObj1) && ((colObj1)->mergesSimulationIslands())))
			{
				if (colObj0->IsActive() || colObj1->IsActive())
				{

					GetSimulationIslandManager()->GetUnionFind().unite((colObj0)->m_islandTag1,
						(colObj1)->m_islandTag1);
				}
			}
		}
	}

	//Store the island id in each body
	GetSimulationIslandManager()->StoreIslandActivationState(GetCollisionWorld());

#ifdef USE_QUICKPROF
	Profiler::endBlock("IslandUnionFind");
#endif //USE_QUICKPROF

}

void	btDiscreteDynamicsWorld::updateAabbs()
{
	btTransform predictedTrans;
	for (int i=0;i<m_collisionObjects.size();i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		if (colObj->m_internalOwner)
		{
			btRigidBody* body = (btRigidBody*)colObj->m_internalOwner;
			if (body->IsActive() && (!body->IsStatic()))
			{
				btPoint3 minAabb,maxAabb;
				colObj->m_collisionShape->GetAabb(colObj->m_worldTransform, minAabb,maxAabb);
				btSimpleBroadphase* bp = (btSimpleBroadphase*)m_pairCache;
				bp->SetAabb(body->m_broadphaseHandle,minAabb,maxAabb);
			}
		}
	}
}

void	btDiscreteDynamicsWorld::integrateTransforms(float timeStep)
{
	btTransform predictedTrans;
	for (int i=0;i<m_collisionObjects.size();i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		if (colObj->m_internalOwner)
		{
			btRigidBody* body = (btRigidBody*)colObj->m_internalOwner;
			if (body->IsActive() && (!body->IsStatic()))
			{
				body->predictIntegratedTransform(timeStep, predictedTrans);
				body->proceedToTransform( predictedTrans);
			}
		}
	}
}



void	btDiscreteDynamicsWorld::predictUnconstraintMotion(float timeStep)
{
	for (int i=0;i<m_collisionObjects.size();i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		if (colObj->m_internalOwner)
		{
			btRigidBody* body = (btRigidBody*)colObj->m_internalOwner;
			body->m_cachedInvertedWorldTransform = body->m_worldTransform.inverse();
			if (body->IsActive() && (!body->IsStatic()))
			{
				body->applyForces( timeStep);
				body->integrateVelocities( timeStep);
				body->predictIntegratedTransform(timeStep,body->m_interpolationWorldTransform);

			}
		}
	}
}