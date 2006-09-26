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


#ifndef BU_STATIC_MOTIONSTATE
#define BU_STATIC_MOTIONSTATE


#include <btCollisionShapes/BU_MotionStateInterface.h>

class BU_StaticMotionState :public BU_MotionStateInterface
{
public:
	virtual ~BU_StaticMotionState(){};

	virtual void	SetTransform(const btSimdTransform& trans)
	{
		m_trans = trans;
	}
	virtual void	GetTransform(btSimdTransform& trans) const
	{
		trans = m_trans;
	}
	virtual void	SetPosition(const SimdPoint3& position)
	{
		m_trans.setOrigin( position );
	}
	virtual void	GetPosition(SimdPoint3& position) const
	{
		position = m_trans.getOrigin();
	}

	virtual void	SetOrientation(const btSimdQuaternion& orientation)
	{
		m_trans.setRotation( orientation);
	}
	virtual void	GetOrientation(btSimdQuaternion& orientation) const
	{
		orientation = m_trans.getRotation();
	}

	virtual void	SetBasis(const btSimdMatrix3x3& basis)
	{
		m_trans.setBasis( basis);
	}
	virtual void	GetBasis(btSimdMatrix3x3& basis) const
	{ 
		basis = m_trans.getBasis();
	}

	virtual void	SetLinearVelocity(const btSimdVector3& linvel)
	{
		m_linearVelocity = linvel;
	}
	virtual void	GetLinearVelocity(btSimdVector3& linvel) const
	{
		linvel = m_linearVelocity;
	}
	
	virtual void	SetAngularVelocity(const btSimdVector3& angvel)
	{
		m_angularVelocity = angvel;
	}
	virtual void	GetAngularVelocity(btSimdVector3& angvel) const
	{
		angvel = m_angularVelocity;
	}



protected:

	btSimdTransform	m_trans;
	btSimdVector3		m_angularVelocity;
	btSimdVector3		m_linearVelocity;

};

#endif //BU_STATIC_MOTIONSTATE
