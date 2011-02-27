/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "SpuContactManifoldCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btPolyhedralConvexShape.h"




void SpuContactManifoldCollisionAlgorithm::processCollision (const btCollisionProcessInfo& processInfo)
{
	btAssert(0);
}

btScalar SpuContactManifoldCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
	btAssert(0);
	return 1.f;
}

#ifndef __SPU__
SpuContactManifoldCollisionAlgorithm::SpuContactManifoldCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci)
:btCollisionAlgorithm(ci)
#ifdef USE_SEPDISTANCE_UTIL
,m_sepDistance(body0->getCollisionShape()->getAngularMotionDisc(),body1->getCollisionShape()->getAngularMotionDisc())
#endif //USE_SEPDISTANCE_UTIL
{
	m_manifoldPtr = ci.m_dispatcher1->getNewManifold(ci.m_colObj0->getCollisionObject(),ci.m_colObj1->getCollisionObject());
	m_shapeType0 = ci.m_colObj0->getCollisionShape()->getShapeType();
	m_shapeType1 = ci.m_colObj1->getCollisionShape()->getShapeType();
	m_collisionMargin0 = ci.m_colObj0->getCollisionShape()->getMargin();
	m_collisionMargin1 = ci.m_colObj1->getCollisionShape()->getMargin();
	m_collisionObject0 = ci.m_colObj0->getCollisionObject();
	m_collisionObject1 = ci.m_colObj1->getCollisionObject();

	if (ci.m_colObj0->getCollisionShape()->isPolyhedral())
	{
		btPolyhedralConvexShape* convex0 = (btPolyhedralConvexShape*)ci.m_colObj0->getCollisionShape();
		m_shapeDimensions0 = convex0->getImplicitShapeDimensions();
	}
	if (ci.m_colObj1->getCollisionShape()->isPolyhedral())
	{
		btPolyhedralConvexShape* convex1 = (btPolyhedralConvexShape*)ci.m_colObj1->getCollisionShape();
		m_shapeDimensions1 = convex1->getImplicitShapeDimensions();
	}
}
#endif //__SPU__

void SpuContactManifoldCollisionAlgorithm::nihilize(btDispatcher* dispatcher)
{
	if (m_manifoldPtr)
		dispatcher->releaseManifold(m_manifoldPtr);
}
