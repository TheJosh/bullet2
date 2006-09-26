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

#ifndef COLLISION_SHAPE_H
#define COLLISION_SHAPE_H

#include "LinearMath/SimdTransform.h"
#include "LinearMath/SimdVector3.h"
#include <LinearMath/SimdMatrix3x3.h>
#include "LinearMath/SimdPoint3.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" //for the shape types

///CollisionShape provides generic interface for collidable objects
class btCollisionShape
{
public:

	btCollisionShape() :m_tempDebug(0)
	{
	}
	virtual ~btCollisionShape()
	{
	}

	virtual void GetAabb(const btSimdTransform& t,btSimdVector3& aabbMin,btSimdVector3& aabbMax) const =0;

	virtual void	GetBoundingSphere(btSimdVector3& center,SimdScalar& radius) const;

	virtual float	GetAngularMotionDisc() const;

	virtual int		GetShapeType() const=0;

	///CalculateTemporalAabb calculates the enclosing aabb for the moving object over interval [0..timeStep)
	///result is conservative
	void CalculateTemporalAabb(const btSimdTransform& curTrans,const btSimdVector3& linvel,const btSimdVector3& angvel,SimdScalar timeStep, btSimdVector3& temporalAabbMin,btSimdVector3& temporalAabbMax);

	inline bool	IsPolyhedral() const
	{
		return btBroadphaseProxy::IsPolyhedral(GetShapeType());
	}

	inline bool	IsConvex() const
	{
		return btBroadphaseProxy::IsConvex(GetShapeType());
	}
	inline bool	IsConcave() const
	{
		return btBroadphaseProxy::IsConcave(GetShapeType());
	}
	inline bool	IsCompound() const
	{
		return btBroadphaseProxy::IsCompound(GetShapeType());
	}

	virtual void	setLocalScaling(const btSimdVector3& scaling) =0;
	virtual const btSimdVector3& getLocalScaling() const =0;

	virtual void	CalculateLocalInertia(SimdScalar mass,btSimdVector3& inertia) = 0;

//debugging support
	virtual char*	GetName()const =0 ;
	const char* GetExtraDebugInfo() const { return m_tempDebug;}
	void  SetExtraDebugInfo(const char* extraDebugInfo) { m_tempDebug = extraDebugInfo;}
	const char * m_tempDebug;
//endif debugging support

	virtual void	SetMargin(float margin) = 0;
	virtual float	GetMargin() const = 0;

};	

#endif //COLLISION_SHAPE_H

