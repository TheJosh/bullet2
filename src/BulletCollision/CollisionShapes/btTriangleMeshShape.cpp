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

#include "btTriangleMeshShape.h"
#include "LinearMath/SimdVector3.h"
#include "LinearMath/SimdQuaternion.h"
#include "btStridingMeshInterface.h"
#include "LinearMath/GenAabbUtil2.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"

#include "stdio.h"

btTriangleMeshShape::btTriangleMeshShape(btStridingMeshInterface* meshInterface)
: m_meshInterface(meshInterface)
{
	RecalcLocalAabb();
}


btTriangleMeshShape::~btTriangleMeshShape()
{
		
}




void btTriangleMeshShape::GetAabb(const btSimdTransform& trans,btSimdVector3& aabbMin,btSimdVector3& aabbMax) const
{

	btSimdVector3 localHalfExtents = 0.5f*(m_localAabbMax-m_localAabbMin);
	btSimdVector3 localCenter = 0.5f*(m_localAabbMax+m_localAabbMin);
	
	btSimdMatrix3x3 abs_b = trans.getBasis().absolute();  

	SimdPoint3 center = trans(localCenter);

	btSimdVector3 extent = btSimdVector3(abs_b[0].dot(localHalfExtents),
		   abs_b[1].dot(localHalfExtents),
		  abs_b[2].dot(localHalfExtents));
	extent += btSimdVector3(GetMargin(),GetMargin(),GetMargin());

	aabbMin = center - extent;
	aabbMax = center + extent;

	
}

void	btTriangleMeshShape::RecalcLocalAabb()
{
	for (int i=0;i<3;i++)
	{
		btSimdVector3 vec(0.f,0.f,0.f);
		vec[i] = 1.f;
		btSimdVector3 tmp = LocalGetSupportingVertex(vec);
		m_localAabbMax[i] = tmp[i]+m_collisionMargin;
		vec[i] = -1.f;
		tmp = LocalGetSupportingVertex(vec);
		m_localAabbMin[i] = tmp[i]-m_collisionMargin;
	}
}



class SupportVertexCallback : public btTriangleCallback
{

	btSimdVector3 m_supportVertexLocal;
public:

	btSimdTransform	m_worldTrans;
	SimdScalar m_maxDot;
	btSimdVector3 m_supportVecLocal;

	SupportVertexCallback(const btSimdVector3& supportVecWorld,const btSimdTransform& trans)
		: m_supportVertexLocal(0.f,0.f,0.f), m_worldTrans(trans) ,m_maxDot(-1e30f)
		
	{
		m_supportVecLocal = supportVecWorld * m_worldTrans.getBasis();
	}

	virtual void ProcessTriangle( btSimdVector3* triangle,int partId, int triangleIndex)
	{
		for (int i=0;i<3;i++)
		{
			SimdScalar dot = m_supportVecLocal.dot(triangle[i]);
			if (dot > m_maxDot)
			{
				m_maxDot = dot;
				m_supportVertexLocal = triangle[i];
			}
		}
	}

	btSimdVector3 GetSupportVertexWorldSpace()
	{
		return m_worldTrans(m_supportVertexLocal);
	}

	btSimdVector3	GetSupportVertexLocal()
	{
		return m_supportVertexLocal;
	}

};

	
void btTriangleMeshShape::setLocalScaling(const btSimdVector3& scaling)
{
	m_meshInterface->setScaling(scaling);
	RecalcLocalAabb();
}

const btSimdVector3& btTriangleMeshShape::getLocalScaling() const
{
	return m_meshInterface->getScaling();
}






//#define DEBUG_TRIANGLE_MESH


void	btTriangleMeshShape::ProcessAllTriangles(btTriangleCallback* callback,const btSimdVector3& aabbMin,const btSimdVector3& aabbMax) const
{

	struct FilteredCallback : public btInternalTriangleIndexCallback
	{
		btTriangleCallback* m_callback;
		btSimdVector3 m_aabbMin;
		btSimdVector3 m_aabbMax;

		FilteredCallback(btTriangleCallback* callback,const btSimdVector3& aabbMin,const btSimdVector3& aabbMax)
			:m_callback(callback),
			m_aabbMin(aabbMin),
			m_aabbMax(aabbMax)
		{
		}

		virtual void InternalProcessTriangleIndex(btSimdVector3* triangle,int partId,int triangleIndex)
		{
			if (TestTriangleAgainstAabb2(&triangle[0],m_aabbMin,m_aabbMax))
			{
				//check aabb in triangle-space, before doing this
				m_callback->ProcessTriangle(triangle,partId,triangleIndex);
			}
			
		}

	};

	FilteredCallback filterCallback(callback,aabbMin,aabbMax);

	m_meshInterface->InternalProcessAllTriangles(&filterCallback,aabbMin,aabbMax);

}





void	btTriangleMeshShape::CalculateLocalInertia(SimdScalar mass,btSimdVector3& inertia)
{
	//moving concave objects not supported
	assert(0);
	inertia.setValue(0.f,0.f,0.f);
}


btSimdVector3 btTriangleMeshShape::LocalGetSupportingVertex(const btSimdVector3& vec) const
{
	btSimdVector3 supportVertex;

	btSimdTransform ident;
	ident.setIdentity();

	SupportVertexCallback supportCallback(vec,ident);

	btSimdVector3 aabbMax(1e30f,1e30f,1e30f);
	
	ProcessAllTriangles(&supportCallback,-aabbMax,aabbMax);
		
	supportVertex = supportCallback.GetSupportVertexLocal();

	return supportVertex;
}
