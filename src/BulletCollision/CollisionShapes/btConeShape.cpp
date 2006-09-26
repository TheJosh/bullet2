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

#include "btConeShape.h"
#include "LinearMath/SimdPoint3.h"

#ifdef WIN32
static int coneindices[3] = {1,2,0};
#else
static int coneindices[3] = {2,1,0};
#endif

btConeShape::btConeShape (SimdScalar radius,SimdScalar height):
m_radius (radius),
m_height(height)
{
	btSimdVector3 halfExtents;
	m_sinAngle = (m_radius / sqrt(m_radius * m_radius + m_height * m_height));
}


btSimdVector3 btConeShape::ConeLocalSupport(const btSimdVector3& v) const
{
	
	float halfHeight = m_height * 0.5f;

 if (v[coneindices[1]] > v.length() * m_sinAngle)
 {
	btSimdVector3 tmp;

	tmp[coneindices[0]] = 0.f;
	tmp[coneindices[1]] = halfHeight;
	tmp[coneindices[2]] = 0.f;
	return tmp;
 }
  else {
    SimdScalar s = SimdSqrt(v[coneindices[0]] * v[coneindices[0]] + v[coneindices[2]] * v[coneindices[2]]);
    if (s > SIMD_EPSILON) {
      SimdScalar d = m_radius / s;
	  btSimdVector3 tmp;
	  tmp[coneindices[0]] = v[coneindices[0]] * d;
	  tmp[coneindices[1]] = -halfHeight;
	  tmp[coneindices[2]] = v[coneindices[2]] * d;
	  return tmp;
    }
    else  {
		btSimdVector3 tmp;
		tmp[coneindices[0]] = 0.f;
		tmp[coneindices[1]] = -halfHeight;
		tmp[coneindices[2]] = 0.f;
		return tmp;
	}
  }

}

btSimdVector3	btConeShape::LocalGetSupportingVertexWithoutMargin(const btSimdVector3& vec) const
{
		return ConeLocalSupport(vec);
}

void	btConeShape::BatchedUnitVectorGetSupportingVertexWithoutMargin(const btSimdVector3* vectors,btSimdVector3* supportVerticesOut,int numVectors) const
{
	for (int i=0;i<numVectors;i++)
	{
		const btSimdVector3& vec = vectors[i];
		supportVerticesOut[i] = ConeLocalSupport(vec);
	}
}


btSimdVector3	btConeShape::LocalGetSupportingVertex(const btSimdVector3& vec)  const
{
	btSimdVector3 supVertex = ConeLocalSupport(vec);
	if ( GetMargin()!=0.f )
	{
		btSimdVector3 vecnorm = vec;
		if (vecnorm .length2() < (SIMD_EPSILON*SIMD_EPSILON))
		{
			vecnorm.setValue(-1.f,-1.f,-1.f);
		} 
		vecnorm.normalize();
		supVertex+= GetMargin() * vecnorm;
	}
	return supVertex;
}


