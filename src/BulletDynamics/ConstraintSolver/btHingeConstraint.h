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

#ifndef HINGECONSTRAINT_H
#define HINGECONSTRAINT_H

#include "LinearMath/SimdVector3.h"

#include "BulletDynamics/ConstraintSolver/btJacobianEntry.h"
#include "btTypedConstraint.h"

class btRigidBody;


/// hinge constraint between two rigidbodies each with a pivotpoint that descibes the axis location in local space
/// axis defines the orientation of the hinge axis
class btHingeConstraint : public btTypedConstraint
{
	btJacobianEntry	m_jac[3]; //3 orthogonal linear constraints
	btJacobianEntry	m_jacAng[2]; //2 orthogonal angular constraints

	btSimdVector3	m_pivotInA;
	btSimdVector3	m_pivotInB;
	btSimdVector3	m_axisInA;
	btSimdVector3	m_axisInB;

	bool	m_angularOnly;
	
public:

	btHingeConstraint(btRigidBody& rbA,btRigidBody& rbB, const btSimdVector3& pivotInA,const btSimdVector3& pivotInB,btSimdVector3& axisInA,btSimdVector3& axisInB);

	btHingeConstraint(btRigidBody& rbA,const btSimdVector3& pivotInA,btSimdVector3& axisInA);

	btHingeConstraint();

	virtual void	BuildJacobian();

	virtual	void	SolveConstraint(SimdScalar	timeStep);

	void	UpdateRHS(SimdScalar	timeStep);

	const btRigidBody& GetRigidBodyA() const
	{
		return m_rbA;
	}
	const btRigidBody& GetRigidBodyB() const
	{
		return m_rbB;
	}

	void	setAngularOnly(bool angularOnly)
	{
		m_angularOnly = angularOnly;
	}



};

#endif //HINGECONSTRAINT_H
