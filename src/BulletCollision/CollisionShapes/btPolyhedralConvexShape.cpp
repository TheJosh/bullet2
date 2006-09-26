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

#include <BulletCollision/CollisionShapes/btPolyhedralConvexShape.h>

btPolyhedralConvexShape::btPolyhedralConvexShape()
:m_optionalHull(0)
{

}



btSimdVector3	btPolyhedralConvexShape::LocalGetSupportingVertexWithoutMargin(const btSimdVector3& vec0)const
{
	int i;
	btSimdVector3 supVec(0,0,0);

	SimdScalar maxDot(-1e30f);

	btSimdVector3 vec = vec0;
	SimdScalar lenSqr = vec.length2();
	if (lenSqr < 0.0001f)
	{
		vec.setValue(1,0,0);
	} else
	{
		float rlen = 1.f / SimdSqrt(lenSqr );
		vec *= rlen;
	}

	btSimdVector3 vtx;
	SimdScalar newDot;

	for (i=0;i<GetNumVertices();i++)
	{
		GetVertex(i,vtx);
		newDot = vec.dot(vtx);
		if (newDot > maxDot)
		{
			maxDot = newDot;
			supVec = vtx;
		}
	}

	return supVec;

}

void	btPolyhedralConvexShape::BatchedUnitVectorGetSupportingVertexWithoutMargin(const btSimdVector3* vectors,btSimdVector3* supportVerticesOut,int numVectors) const
{
	int i;

	btSimdVector3 vtx;
	SimdScalar newDot;

	for (int i=0;i<numVectors;i++)
	{
		supportVerticesOut[i][3] = -1e30f;
	}

	for (int j=0;j<numVectors;j++)
	{
	
		const btSimdVector3& vec = vectors[j];

		for (i=0;i<GetNumVertices();i++)
		{
			GetVertex(i,vtx);
			newDot = vec.dot(vtx);
			if (newDot > supportVerticesOut[j][3])
			{
				//WARNING: don't swap next lines, the w component would get overwritten!
				supportVerticesOut[j] = vtx;
				supportVerticesOut[j][3] = newDot;
			}
		}
	}
}



void	btPolyhedralConvexShape::CalculateLocalInertia(SimdScalar mass,btSimdVector3& inertia)
{
	//not yet, return box inertia

	float margin = GetMargin();

	btSimdTransform ident;
	ident.setIdentity();
	btSimdVector3 aabbMin,aabbMax;
	GetAabb(ident,aabbMin,aabbMax);
	btSimdVector3 halfExtents = (aabbMax-aabbMin)*0.5f;

	SimdScalar lx=2.f*(halfExtents.x()+margin);
	SimdScalar ly=2.f*(halfExtents.y()+margin);
	SimdScalar lz=2.f*(halfExtents.z()+margin);
	const SimdScalar x2 = lx*lx;
	const SimdScalar y2 = ly*ly;
	const SimdScalar z2 = lz*lz;
	const SimdScalar scaledmass = mass * 0.08333333f;

	inertia = scaledmass * (btSimdVector3(y2+z2,x2+z2,x2+y2));

}

