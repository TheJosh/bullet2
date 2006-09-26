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
#include "btCylinderShape.h"
#include "LinearMath/SimdPoint3.h"

btCylinderShape::btCylinderShape (const btSimdVector3& halfExtents)
:btBoxShape(halfExtents)
{

}


btCylinderShapeX::btCylinderShapeX (const btSimdVector3& halfExtents)
:btCylinderShape(halfExtents)
{
}


btCylinderShapeZ::btCylinderShapeZ (const btSimdVector3& halfExtents)
:btCylinderShape(halfExtents)
{
}



inline btSimdVector3 CylinderLocalSupportX(const btSimdVector3& halfExtents,const btSimdVector3& v) 
{
const int cylinderUpAxis = 0;
const int XX = 1;
const int YY = 0;
const int ZZ = 2;

	//mapping depends on how cylinder local orientation is
	// extents of the cylinder is: X,Y is for radius, and Z for height


	float radius = halfExtents[XX];
	float halfHeight = halfExtents[cylinderUpAxis];


    btSimdVector3 tmp;
	SimdScalar d ;

    SimdScalar s = SimdSqrt(v[XX] * v[XX] + v[ZZ] * v[ZZ]);
    if (s != SimdScalar(0.0))
	{
        d = radius / s;  
		tmp[XX] = v[XX] * d;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = v[ZZ] * d;
		return tmp;
	}
    else
	{
	    tmp[XX] = radius;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = SimdScalar(0.0);
		return tmp;
    }


}






inline  btSimdVector3 CylinderLocalSupportY(const btSimdVector3& halfExtents,const btSimdVector3& v) 
{

const int cylinderUpAxis = 1;
const int XX = 0;
const int YY = 1;
const int ZZ = 2;


	float radius = halfExtents[XX];
	float halfHeight = halfExtents[cylinderUpAxis];


    btSimdVector3 tmp;
	SimdScalar d ;

    SimdScalar s = SimdSqrt(v[XX] * v[XX] + v[ZZ] * v[ZZ]);
    if (s != SimdScalar(0.0))
	{
        d = radius / s;  
		tmp[XX] = v[XX] * d;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = v[ZZ] * d;
		return tmp;
	}
    else
	{
	    tmp[XX] = radius;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = SimdScalar(0.0);
		return tmp;
    }

}

inline btSimdVector3 CylinderLocalSupportZ(const btSimdVector3& halfExtents,const btSimdVector3& v) 
{
const int cylinderUpAxis = 2;
const int XX = 0;
const int YY = 2;
const int ZZ = 1;

	//mapping depends on how cylinder local orientation is
	// extents of the cylinder is: X,Y is for radius, and Z for height


	float radius = halfExtents[XX];
	float halfHeight = halfExtents[cylinderUpAxis];


    btSimdVector3 tmp;
	SimdScalar d ;

    SimdScalar s = SimdSqrt(v[XX] * v[XX] + v[ZZ] * v[ZZ]);
    if (s != SimdScalar(0.0))
	{
        d = radius / s;  
		tmp[XX] = v[XX] * d;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = v[ZZ] * d;
		return tmp;
	}
    else
	{
	    tmp[XX] = radius;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = SimdScalar(0.0);
		return tmp;
    }


}

btSimdVector3	btCylinderShapeX::LocalGetSupportingVertexWithoutMargin(const btSimdVector3& vec)const
{
	return CylinderLocalSupportX(GetHalfExtents(),vec);
}


btSimdVector3	btCylinderShapeZ::LocalGetSupportingVertexWithoutMargin(const btSimdVector3& vec)const
{
	return CylinderLocalSupportZ(GetHalfExtents(),vec);
}
btSimdVector3	btCylinderShape::LocalGetSupportingVertexWithoutMargin(const btSimdVector3& vec)const
{
	return CylinderLocalSupportY(GetHalfExtents(),vec);
}

void	btCylinderShape::BatchedUnitVectorGetSupportingVertexWithoutMargin(const btSimdVector3* vectors,btSimdVector3* supportVerticesOut,int numVectors) const
{
	for (int i=0;i<numVectors;i++)
	{
		supportVerticesOut[i] = CylinderLocalSupportY(GetHalfExtents(),vectors[i]);
	}
}

void	btCylinderShapeZ::BatchedUnitVectorGetSupportingVertexWithoutMargin(const btSimdVector3* vectors,btSimdVector3* supportVerticesOut,int numVectors) const
{
	for (int i=0;i<numVectors;i++)
	{
		supportVerticesOut[i] = CylinderLocalSupportZ(GetHalfExtents(),vectors[i]);
	}
}




void	btCylinderShapeX::BatchedUnitVectorGetSupportingVertexWithoutMargin(const btSimdVector3* vectors,btSimdVector3* supportVerticesOut,int numVectors) const
{
	for (int i=0;i<numVectors;i++)
	{
		supportVerticesOut[i] = CylinderLocalSupportX(GetHalfExtents(),vectors[i]);
	}
}


