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

#include "btSphereShape.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"

#include "LinearMath/SimdQuaternion.h"


btSphereShape ::btSphereShape (SimdScalar radius)
: m_radius(radius)
{	
}

btSimdVector3	btSphereShape::LocalGetSupportingVertexWithoutMargin(const btSimdVector3& vec)const
{
	return btSimdVector3(0.f,0.f,0.f);
}

void	btSphereShape::BatchedUnitVectorGetSupportingVertexWithoutMargin(const btSimdVector3* vectors,btSimdVector3* supportVerticesOut,int numVectors) const
{
	for (int i=0;i<numVectors;i++)
	{
		supportVerticesOut[i].setValue(0.f,0.f,0.f);
	}
}


btSimdVector3	btSphereShape::LocalGetSupportingVertex(const btSimdVector3& vec)const
{
	btSimdVector3 supVertex;
	supVertex = LocalGetSupportingVertexWithoutMargin(vec);

	btSimdVector3 vecnorm = vec;
	if (vecnorm .length2() < (SIMD_EPSILON*SIMD_EPSILON))
	{
		vecnorm.setValue(-1.f,-1.f,-1.f);
	} 
	vecnorm.normalize();
	supVertex+= GetMargin() * vecnorm;
	return supVertex;
}


//broken due to scaling
void btSphereShape::GetAabb(const btSimdTransform& t,btSimdVector3& aabbMin,btSimdVector3& aabbMax) const
{
	const btSimdVector3& center = t.getOrigin();
	btSimdVector3 extent(GetMargin(),GetMargin(),GetMargin());
	aabbMin = center - extent;
	aabbMax = center + extent;
}



void	btSphereShape::CalculateLocalInertia(SimdScalar mass,btSimdVector3& inertia)
{
	SimdScalar elem = 0.4f * mass * GetMargin()*GetMargin();
	inertia[0] = inertia[1] = inertia[2] = elem;

}

