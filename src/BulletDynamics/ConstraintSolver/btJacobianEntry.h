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

#ifndef JACOBIAN_ENTRY_H
#define JACOBIAN_ENTRY_H

#include "LinearMath/SimdVector3.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"


//notes:
// Another memory optimization would be to store m_1MinvJt in the remaining 3 w components
// which makes the btJacobianEntry memory layout 16 bytes
// if you only are interested in angular part, just feed massInvA and massInvB zero

/// Jacobian entry is an abstraction that allows to describe constraints
/// it can be used in combination with a constraint solver
/// Can be used to relate the effect of an impulse to the constraint error
class btJacobianEntry
{
public:
	btJacobianEntry() {};
	//constraint between two different rigidbodies
	btJacobianEntry(
		const btSimdMatrix3x3& world2A,
		const btSimdMatrix3x3& world2B,
		const btSimdVector3& rel_pos1,const btSimdVector3& rel_pos2,
		const btSimdVector3& jointAxis,
		const btSimdVector3& inertiaInvA, 
		const SimdScalar massInvA,
		const btSimdVector3& inertiaInvB,
		const SimdScalar massInvB)
		:m_linearJointAxis(jointAxis)
	{
		m_aJ = world2A*(rel_pos1.cross(m_linearJointAxis));
		m_bJ = world2B*(rel_pos2.cross(-m_linearJointAxis));
		m_0MinvJt	= inertiaInvA * m_aJ;
		m_1MinvJt = inertiaInvB * m_bJ;
		m_Adiag = massInvA + m_0MinvJt.dot(m_aJ) + massInvB + m_1MinvJt.dot(m_bJ);

		ASSERT(m_Adiag > 0.0f);
	}

	//angular constraint between two different rigidbodies
	btJacobianEntry(const btSimdVector3& jointAxis,
		const btSimdMatrix3x3& world2A,
		const btSimdMatrix3x3& world2B,
		const btSimdVector3& inertiaInvA,
		const btSimdVector3& inertiaInvB)
		:m_linearJointAxis(btSimdVector3(0.f,0.f,0.f))
	{
		m_aJ= world2A*jointAxis;
		m_bJ = world2B*-jointAxis;
		m_0MinvJt	= inertiaInvA * m_aJ;
		m_1MinvJt = inertiaInvB * m_bJ;
		m_Adiag =  m_0MinvJt.dot(m_aJ) + m_1MinvJt.dot(m_bJ);

		ASSERT(m_Adiag > 0.0f);
	}

	//angular constraint between two different rigidbodies
	btJacobianEntry(const btSimdVector3& axisInA,
		const btSimdVector3& axisInB,
		const btSimdVector3& inertiaInvA,
		const btSimdVector3& inertiaInvB)
		: m_linearJointAxis(btSimdVector3(0.f,0.f,0.f))
		, m_aJ(axisInA)
		, m_bJ(-axisInB)
	{
		m_0MinvJt	= inertiaInvA * m_aJ;
		m_1MinvJt = inertiaInvB * m_bJ;
		m_Adiag =  m_0MinvJt.dot(m_aJ) + m_1MinvJt.dot(m_bJ);

		ASSERT(m_Adiag > 0.0f);
	}

	//constraint on one rigidbody
	btJacobianEntry(
		const btSimdMatrix3x3& world2A,
		const btSimdVector3& rel_pos1,const btSimdVector3& rel_pos2,
		const btSimdVector3& jointAxis,
		const btSimdVector3& inertiaInvA, 
		const SimdScalar massInvA)
		:m_linearJointAxis(jointAxis)
	{
		m_aJ= world2A*(rel_pos1.cross(jointAxis));
		m_bJ = world2A*(rel_pos2.cross(-jointAxis));
		m_0MinvJt	= inertiaInvA * m_aJ;
		m_1MinvJt = btSimdVector3(0.f,0.f,0.f);
		m_Adiag = massInvA + m_0MinvJt.dot(m_aJ);

		ASSERT(m_Adiag > 0.0f);
	}

	SimdScalar	getDiagonal() const { return m_Adiag; }

	// for two constraints on the same rigidbody (for example vehicle friction)
	SimdScalar	getNonDiagonal(const btJacobianEntry& jacB, const SimdScalar massInvA) const
	{
		const btJacobianEntry& jacA = *this;
		SimdScalar lin = massInvA * jacA.m_linearJointAxis.dot(jacB.m_linearJointAxis);
		SimdScalar ang = jacA.m_0MinvJt.dot(jacB.m_aJ);
		return lin + ang;
	}

	

	// for two constraints on sharing two same rigidbodies (for example two contact points between two rigidbodies)
	SimdScalar	getNonDiagonal(const btJacobianEntry& jacB,const SimdScalar massInvA,const SimdScalar massInvB) const
	{
		const btJacobianEntry& jacA = *this;
		btSimdVector3 lin = jacA.m_linearJointAxis * jacB.m_linearJointAxis;
		btSimdVector3 ang0 = jacA.m_0MinvJt * jacB.m_aJ;
		btSimdVector3 ang1 = jacA.m_1MinvJt * jacB.m_bJ;
		btSimdVector3 lin0 = massInvA * lin ;
		btSimdVector3 lin1 = massInvB * lin;
		btSimdVector3 sum = ang0+ang1+lin0+lin1;
		return sum[0]+sum[1]+sum[2];
	}

	SimdScalar getRelativeVelocity(const btSimdVector3& linvelA,const btSimdVector3& angvelA,const btSimdVector3& linvelB,const btSimdVector3& angvelB)
	{
		btSimdVector3 linrel = linvelA - linvelB;
		btSimdVector3 angvela  = angvelA * m_aJ;
		btSimdVector3 angvelb  = angvelB * m_bJ;
		linrel *= m_linearJointAxis;
		angvela += angvelb;
		angvela += linrel;
		SimdScalar rel_vel2 = angvela[0]+angvela[1]+angvela[2];
		return rel_vel2 + SIMD_EPSILON;
	}
//private:

	btSimdVector3	m_linearJointAxis;
	btSimdVector3	m_aJ;
	btSimdVector3	m_bJ;
	btSimdVector3	m_0MinvJt;
	btSimdVector3	m_1MinvJt;
	//Optimization: can be stored in the w/last component of one of the vectors
	SimdScalar	m_Adiag;

};

#endif //JACOBIAN_ENTRY_H
