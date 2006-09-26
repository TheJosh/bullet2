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

#ifndef CONVEX_SHAPE_INTERFACE1
#define CONVEX_SHAPE_INTERFACE1

#include "btCollisionShape.h"

#include "LinearMath/SimdVector3.h"
#include "LinearMath/SimdTransform.h"
#include "LinearMath/SimdMatrix3x3.h"
#include <vector>
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"

//todo: get rid of this btConvexCastResult thing!
struct btConvexCastResult;


/// btConvexShape is an abstract shape interface.
/// The explicit part provides plane-equations, the implicit part provides GetClosestPoint interface.
/// used in combination with GJK or btConvexCast
class btConvexShape : public btCollisionShape
{
public:
	btConvexShape();

	virtual btSimdVector3	LocalGetSupportingVertex(const btSimdVector3& vec)const;
	virtual btSimdVector3	LocalGetSupportingVertexWithoutMargin(const btSimdVector3& vec) const= 0;
	
	//notice that the vectors should be unit length
	virtual void	BatchedUnitVectorGetSupportingVertexWithoutMargin(const btSimdVector3* vectors,btSimdVector3* supportVerticesOut,int numVectors) const= 0;

	// testing for hullnode code

	///GetAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	void GetAabb(const btSimdTransform& t,btSimdVector3& aabbMin,btSimdVector3& aabbMax) const
	{
		GetAabbSlow(t,aabbMin,aabbMax);
	}


	
	virtual void GetAabbSlow(const btSimdTransform& t,btSimdVector3& aabbMin,btSimdVector3& aabbMax) const;


	virtual void	setLocalScaling(const btSimdVector3& scaling);
	virtual const btSimdVector3& getLocalScaling() const 
	{
		return m_localScaling;
	}


	virtual void	SetMargin(float margin)
	{
		m_collisionMargin = margin;
	}
	virtual float	GetMargin() const
	{
		return m_collisionMargin;
	}
private:
	SimdScalar	m_collisionMargin;
	//local scaling. collisionMargin is not scaled !
protected:
	btSimdVector3	m_localScaling;

};



#endif //CONVEX_SHAPE_INTERFACE1
