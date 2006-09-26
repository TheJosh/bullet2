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

#ifndef SPHERE_MINKOWSKI_H
#define SPHERE_MINKOWSKI_H

#include "btConvexShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types

///SphereShape implements an implicit (getSupportingVertex) Sphere
class btSphereShape : public btConvexShape

{
	SimdScalar m_radius;
	
public:
	btSphereShape (SimdScalar radius);
	
	
	virtual btSimdVector3	LocalGetSupportingVertex(const btSimdVector3& vec)const;
	virtual btSimdVector3	LocalGetSupportingVertexWithoutMargin(const btSimdVector3& vec)const;
	//notice that the vectors should be unit length
	virtual void	BatchedUnitVectorGetSupportingVertexWithoutMargin(const btSimdVector3* vectors,btSimdVector3* supportVerticesOut,int numVectors) const;


	virtual void	CalculateLocalInertia(SimdScalar mass,btSimdVector3& inertia);

	virtual void GetAabb(const btSimdTransform& t,btSimdVector3& aabbMin,btSimdVector3& aabbMax) const;

	virtual int	GetShapeType() const { return SPHERE_SHAPE_PROXYTYPE; }

	SimdScalar	GetRadius() const { return m_radius;}

	//debugging
	virtual char*	GetName()const {return "SPHERE";}

	virtual void	SetMargin(float margin)
	{
		btConvexShape::SetMargin(margin);
	}
	virtual float	GetMargin() const
	{
		//to improve gjk behaviour, use radius+margin as the full margin, so never get into the penetration case
		//this means, non-uniform scaling is not supported anymore
		return m_localScaling[0] * m_radius + btConvexShape::GetMargin();
	}


};


#endif //SPHERE_MINKOWSKI_H
