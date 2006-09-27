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

#ifndef BT_DISCRETE_DYNAMICS_WORLD_H
#define BT_DISCRETE_DYNAMICS_WORLD_H

#include "btDynamicsWorld.h"

class btDispatcher;
class btOverlappingPairCache;
class btConstraintSolver;
class btSimulationIslandManager;
class btTypedConstraint;
struct btContactSolverInfo;
class btRaycastVehicle;

#include <vector>

///btDiscreteDynamicsWorld provides discrete rigid body simulation
///those classes replace the obsolete CcdPhysicsEnvironment/CcdPhysicsController
class btDiscreteDynamicsWorld : public btDynamicsWorld
{
protected:

	btConstraintSolver*	m_constraintSolver;

	btSimulationIslandManager*	m_islandManager;

	std::vector<btTypedConstraint*> m_constraints;

	std::vector<btRaycastVehicle*>	m_vehicles;

	void	predictUnconstraintMotion(float timeStep);
	
	void	integrateTransforms(float timeStep);
		
	void	updateAabbs();

	void	calculateSimulationIslands();

	void	solveNoncontactConstraints(btContactSolverInfo& solverInfo);

	void	solveContactConstraints(btContactSolverInfo& solverInfo);

	void	updateActivationState(float timeStep);

	void	updateVehicles(float timeStep);

public:


	btDiscreteDynamicsWorld(btDispatcher* dispatcher,btOverlappingPairCache* pairCache,btConstraintSolver* constraintSolver);

	btDiscreteDynamicsWorld();
		
	virtual ~btDiscreteDynamicsWorld();
		
	virtual void	stepSimulation( float timeStep);

	void	addConstraint(btTypedConstraint* constraint);

	void	removeConstraint(btTypedConstraint* constraint);

	void	addVehicle(btRaycastVehicle* vehicle);

	void	removeVehicle(btRaycastVehicle* vehicle);

	btSimulationIslandManager*	GetSimulationIslandManager()
	{
		return m_islandManager;
	}

	const btSimulationIslandManager*	GetSimulationIslandManager() const 
	{
		return m_islandManager;
	}

	btCollisionWorld*	GetCollisionWorld()
	{
		return this;
	}

};

#endif //BT_DISCRETE_DYNAMICS_WORLD_H
