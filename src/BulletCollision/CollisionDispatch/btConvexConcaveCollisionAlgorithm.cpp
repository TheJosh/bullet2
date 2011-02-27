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

//#pragma optimize("", off)
//#pragma inline_depth(0)

#include "btConvexConcaveCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionShapes/btConcaveShape.h"
#include "BulletCollision/CollisionDispatch/btManifoldResult.h"
#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "LinearMath/btIDebugDraw.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"

btConvexTriangleCallback::btConvexTriangleCallback(btDispatcher* dispatcher, const btCollider* body0, const btCollider* body1, bool isSwapped)
:	m_convexObject(0)
,	m_triObject(0)
,	m_dispatcher(dispatcher)
,	m_dispatchInfoPtr(0)
{
	m_convexObject = isSwapped ? body1->getCollisionObject() : body0->getCollisionObject();
	m_triObject = isSwapped ? body0->getCollisionObject() : body1->getCollisionObject();

	// Create the manifold from the dispatcher 'manifold pool'.
	m_manifoldPtr = m_dispatcher->getNewManifold(m_convexObject, m_triObject);

	clearCache();
}

btConvexTriangleCallback::~btConvexTriangleCallback()
{
	clearCache();
	m_dispatcher->releaseManifold(m_manifoldPtr);
}

void btConvexTriangleCallback::clearCache()
{
	m_dispatcher->clearManifold(m_manifoldPtr);
}

void btConvexTriangleCallback::processTriangle(btVector3* triangle,int partId, int triangleIndex)
{
	const btCollisionObject* ob = m_triObject;

	if (m_convexObject->getCollisionShape()->isConvex())
	{
		btTriangleShape tm(triangle[0],triangle[1],triangle[2]);	
		tm.setMargin(m_collisionMarginTriangle);

		const btCollisionShape* tmpShape = ob->getCollisionShape();

		btCollider obColl(0, &tm, ob, ob->getWorldTransform());
		btCollider convexColl(0, m_convexObject->getCollisionShape(), m_convexObject, m_convexObject->getWorldTransform());

		btCollisionAlgorithm* colAlgo = m_dispatcher->findAlgorithm(&convexColl, &obColl, m_manifoldPtr);

		if (m_resultOut->getBody0Internal() == m_triObject)
		{
			m_resultOut->setShapeIdentifiersA(partId,triangleIndex);
		}
		else
		{
			m_resultOut->setShapeIdentifiersB(partId,triangleIndex);
		}

		btCollisionProcessInfo processInfo(convexColl, obColl, *m_dispatchInfoPtr, m_resultOut, m_dispatcher);
		colAlgo->processCollision(processInfo);
		colAlgo->nihilize(m_dispatcher);
		colAlgo->~btCollisionAlgorithm();

		m_dispatcher->freeCollisionAlgorithm(colAlgo);
	}
}

void btConvexTriangleCallback::setTimeStepAndCounters(btScalar collisionMarginTriangle,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
	m_dispatchInfoPtr = &dispatchInfo;
	m_collisionMarginTriangle = collisionMarginTriangle;
	m_resultOut = resultOut;

	//recalc aabbs
	btTransform convexInTriangleSpace;
	convexInTriangleSpace = m_triObject->getWorldTransform().inverse() * m_convexObject->getWorldTransform();
	const btCollisionShape* convexShape = static_cast<const btCollisionShape*>(m_convexObject->getCollisionShape());
	//CollisionShape* triangleShape = static_cast<btCollisionShape*>(triBody->m_collisionShape);
	convexShape->getAabb(convexInTriangleSpace,m_aabbMin,m_aabbMax);
	btScalar extraMargin = collisionMarginTriangle;
	btVector3 extra(extraMargin,extraMargin,extraMargin);

	m_aabbMax += extra;
	m_aabbMin -= extra;
}

btConvexConcaveCollisionAlgorithm::btConvexConcaveCollisionAlgorithm( const btCollisionAlgorithmConstructionInfo& ci)
:	btActivatingCollisionAlgorithm(ci)
,	m_isSwapped(ci.m_isSwapped)
,	m_btConvexTriangleCallback(ci.m_dispatcher1,ci.m_colObj0,ci.m_colObj1,ci.m_isSwapped)
{
}

btConvexConcaveCollisionAlgorithm::~btConvexConcaveCollisionAlgorithm()
{
}

void btConvexConcaveCollisionAlgorithm::getAllContactManifolds(btManifoldArray&	manifoldArray)
{
	if (m_btConvexTriangleCallback.m_manifoldPtr)
	{
		manifoldArray.push_back(m_btConvexTriangleCallback.m_manifoldPtr);
	}
}

void btConvexConcaveCollisionAlgorithm::clearCache()
{
	m_btConvexTriangleCallback.clearCache();
}

void btConvexConcaveCollisionAlgorithm::processCollision (const btCollisionProcessInfo& processInfo)
{
	const btCollider& convexBody = m_isSwapped ? processInfo.m_body1 : processInfo.m_body0;
	const btCollider& triBody = m_isSwapped ? processInfo.m_body0 : processInfo.m_body1;

	if (triBody.getCollisionShape()->isConcave())
	{
		const btConcaveShape* concaveShape = static_cast<const btConcaveShape*>( triBody.getCollisionShape());
		
		if (convexBody.getCollisionShape()->isConvex())
		{
			btScalar collisionMarginTriangle = concaveShape->getMargin();
					
			processInfo.m_result->setPersistentManifold(m_btConvexTriangleCallback.m_manifoldPtr);
			m_btConvexTriangleCallback.setTimeStepAndCounters(collisionMarginTriangle,processInfo.m_dispatchInfo,processInfo.m_result);

			//Disable persistency. previously, some older algorithm calculated all contacts in one go, so you can clear it here.
			//m_dispatcher->clearManifold(m_btConvexTriangleCallback.m_manifoldPtr);

			m_btConvexTriangleCallback.m_manifoldPtr->setBodies(
				convexBody.getCollisionObject(),
				triBody.getCollisionObject()
			);

			concaveShape->processAllTriangles( &m_btConvexTriangleCallback,m_btConvexTriangleCallback.getAabbMin(),m_btConvexTriangleCallback.getAabbMax());
			
			processInfo.m_result->refreshContactPoints();
	
		}
	
	}

}


btScalar btConvexConcaveCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	btCollisionObject* convexbody = m_isSwapped ? body1 : body0;
	btCollisionObject* triBody = m_isSwapped ? body0 : body1;


	//quick approximation using raycast, todo: hook up to the continuous collision detection (one of the btConvexCast)

	//only perform CCD above a certain threshold, this prevents blocking on the long run
	//because object in a blocked ccd state (hitfraction<1) get their linear velocity halved each frame...
	btScalar squareMot0 = (convexbody->getInterpolationWorldTransform().getOrigin() - convexbody->getWorldTransform().getOrigin()).length2();
	if (squareMot0 < convexbody->getCcdSquareMotionThreshold())
	{
		return btScalar(1.);
	}

	//const btVector3& from = convexbody->m_worldTransform.getOrigin();
	//btVector3 to = convexbody->m_interpolationWorldTransform.getOrigin();
	//todo: only do if the motion exceeds the 'radius'

	btTransform triInv = triBody->getWorldTransform().inverse();
	btTransform convexFromLocal = triInv * convexbody->getWorldTransform();
	btTransform convexToLocal = triInv * convexbody->getInterpolationWorldTransform();

	struct LocalTriangleSphereCastCallback	: public btTriangleCallback
	{
		btTransform m_ccdSphereFromTrans;
		btTransform m_ccdSphereToTrans;
		btTransform	m_meshTransform;

		btScalar	m_ccdSphereRadius;
		btScalar	m_hitFraction;
	

		LocalTriangleSphereCastCallback(const btTransform& from,const btTransform& to,btScalar ccdSphereRadius,btScalar hitFraction)
			:m_ccdSphereFromTrans(from),
			m_ccdSphereToTrans(to),
			m_ccdSphereRadius(ccdSphereRadius),
			m_hitFraction(hitFraction)
		{			
		}
		
		
		virtual void processTriangle(btVector3* triangle, int partId, int triangleIndex)
		{
			(void)partId;
			(void)triangleIndex;
			//do a swept sphere for now
			btTransform ident;
			ident.setIdentity();
			btConvexCast::CastResult castResult;
			castResult.m_fraction = m_hitFraction;
			btSphereShape	pointShape(m_ccdSphereRadius);
			btTriangleShape	triShape(triangle[0],triangle[1],triangle[2]);
			btVoronoiSimplexSolver	simplexSolver;
			btSubsimplexConvexCast convexCaster(&pointShape,&triShape,&simplexSolver);
			//GjkConvexCast	convexCaster(&pointShape,convexShape,&simplexSolver);
			//ContinuousConvexCollision convexCaster(&pointShape,convexShape,&simplexSolver,0);
			//local space?

			if (convexCaster.calcTimeOfImpact(m_ccdSphereFromTrans,m_ccdSphereToTrans,
				ident,ident,castResult))
			{
				if (m_hitFraction > castResult.m_fraction)
					m_hitFraction = castResult.m_fraction;
			}

		}

	};


	

	
	if (triBody->getCollisionShape()->isConcave())
	{
		btVector3 rayAabbMin = convexFromLocal.getOrigin();
		rayAabbMin.setMin(convexToLocal.getOrigin());
		btVector3 rayAabbMax = convexFromLocal.getOrigin();
		rayAabbMax.setMax(convexToLocal.getOrigin());
		btScalar ccdRadius0 = convexbody->getCcdSweptSphereRadius();
		rayAabbMin -= btVector3(ccdRadius0,ccdRadius0,ccdRadius0);
		rayAabbMax += btVector3(ccdRadius0,ccdRadius0,ccdRadius0);

		btScalar curHitFraction = btScalar(1.); //is this available?
		LocalTriangleSphereCastCallback raycastCallback(convexFromLocal,convexToLocal,
			convexbody->getCcdSweptSphereRadius(),curHitFraction);

		raycastCallback.m_hitFraction = convexbody->getHitFraction();

		btCollisionObject* concavebody = triBody;

		btConcaveShape* triangleMesh = (btConcaveShape*) concavebody->getCollisionShape();
		
		if (triangleMesh)
		{
			triangleMesh->processAllTriangles(&raycastCallback,rayAabbMin,rayAabbMax);
		}
	


		if (raycastCallback.m_hitFraction < convexbody->getHitFraction())
		{
			convexbody->setHitFraction( raycastCallback.m_hitFraction);
			return raycastCallback.m_hitFraction;
		}
	}

	return btScalar(1.);

}
