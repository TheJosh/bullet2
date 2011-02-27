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


#include "btSoftBodyConcaveCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionShapes/btConcaveShape.h"
#include "BulletCollision/CollisionDispatch/btManifoldResult.h"
#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btTetrahedronShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"



#include "LinearMath/btIDebugDraw.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletSoftBody/btSoftBody.h"

#define BT_SOFTBODY_TRIANGLE_EXTRUSION btScalar(0.06)//make this configurable

btSoftBodyConcaveCollisionAlgorithm::btSoftBodyConcaveCollisionAlgorithm( const btCollisionAlgorithmConstructionInfo& ci, bool isSwapped)
: btCollisionAlgorithm(ci),
m_isSwapped(isSwapped),
m_btSoftBodyTriangleCallback(ci.m_dispatcher1, ci.m_colObj0, ci.m_colObj1, isSwapped)
{
}



btSoftBodyConcaveCollisionAlgorithm::~btSoftBodyConcaveCollisionAlgorithm()
{
}
void btSoftBodyConcaveCollisionAlgorithm::nihilize(btDispatcher* dispatcher)
{
	
}


btSoftBodyTriangleCallback::btSoftBodyTriangleCallback(btDispatcher*  dispatcher, const btCollider* body0, const btCollider* body1,bool isSwapped):
m_dispatcher(dispatcher),
m_dispatchInfoPtr(0),
m_softBody(isSwapped? body1:body0),
m_triBody(isSwapped? body0:body1)
{
	//
	// create the manifold from the dispatcher 'manifold pool'
	//
	//	  m_manifoldPtr = m_dispatcher->getNewManifold(m_convexBody,m_triBody);

	clearCache();
}

btSoftBodyTriangleCallback::~btSoftBodyTriangleCallback()
{
	clearCache();
	//	m_dispatcher->releaseManifold( m_manifoldPtr );

}


void	btSoftBodyTriangleCallback::clearCache()
{
	for (int i=0;i<m_shapeCache.size();i++)
	{
		btTriIndex* tmp = m_shapeCache.getAtIndex(i);
		btAssert(tmp);
		btAssert(tmp->m_childShape);
		// Ugly hack. Where is this reference acquired? Shouldn't dnew shapes in "processTriangle". That's not acceptable for realtime performance.
		if(m_softBody)
			const_cast<btSoftBody*>(static_cast<const btSoftBody*>(m_softBody->getCollisionObject()))->getWorldInfo()->m_sparsesdf.RemoveReferences(tmp->m_childShape);//necessary?
		delete tmp->m_childShape;
	}
	m_shapeCache.clear();
}


void btSoftBodyTriangleCallback::processTriangle(btVector3* triangle,int partId, int triangleIndex)
{
	//just for debugging purposes
	//printf("triangle %d",m_triangleCount++);

	///debug drawing of the overlapping triangles
	if (m_dispatchInfoPtr && m_dispatchInfoPtr->m_debugDraw && m_dispatchInfoPtr->m_debugDraw->getDebugMode() &btIDebugDraw::DBG_DrawWireframe)
	{
		btVector3 color(1,1,0);
		const btTransform& tr = m_triBody->getWorldTransform();
		m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[0]),tr(triangle[1]),color);
		m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[1]),tr(triangle[2]),color);
		m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[2]),tr(triangle[0]),color);
	}

	btTriIndex	triIndex(partId,triangleIndex,0);
	btHashKey<btTriIndex> triKey(triIndex.getUid());


	btTriIndex* shapeIndex = m_shapeCache[triKey];
	if (shapeIndex)
	{
		btCollisionShape* tm = shapeIndex->m_childShape;
		btAssert(tm);

		//copy over user pointers to temporary shape
		tm->setUserPointer(m_triBody->getCollisionShape()->getUserPointer());

		btTransform triTrans;
		triTrans.setIdentity();
		btCollider triCol(m_triBody, tm, m_triBody->getCollisionObject(), triTrans);

		btCollisionProcessInfo processInfo(*m_softBody,triCol,*m_dispatchInfoPtr,m_resultOut, m_dispatcher);
		
		btCollisionAlgorithm* colAlgo = m_dispatcher->findAlgorithm(m_softBody,&triCol,0);

		colAlgo->processCollision(processInfo);
		colAlgo->nihilize(m_dispatcher);
		colAlgo->~btCollisionAlgorithm();
		m_dispatcher->freeCollisionAlgorithm(colAlgo);
		return;
	}

	//aabb filter is already applied!	

	//btCollisionObject* colObj = static_cast<btCollisionObject*>(m_convexProxy->m_clientObject);

	//	if (m_softBody->getCollisionShape()->getShapeType()==
	{
		//		btVector3 other;
		btVector3 normal = (triangle[1]-triangle[0]).cross(triangle[2]-triangle[0]);
		normal.normalize();
		normal*= BT_SOFTBODY_TRIANGLE_EXTRUSION;
		//		other=(triangle[0]+triangle[1]+triangle[2])*0.333333f;
		//		other+=normal*22.f;
		btVector3	pts[6] = {triangle[0]+normal,
			triangle[1]+normal,
			triangle[2]+normal,
			triangle[0]-normal,
			triangle[1]-normal,
			triangle[2]-normal};

		btConvexHullShape* tm = new btConvexHullShape(&pts[0].getX(),6);

		//copy over user pointers to temporary shape
		tm->setUserPointer(m_triBody->getCollisionShape()->getUserPointer());

		btTransform triTrans;
		triTrans.setIdentity();
		btCollider triCol(m_triBody, tm, m_triBody->getCollisionObject(), triTrans);
		btCollisionProcessInfo processInfo(*m_softBody,triCol,*m_dispatchInfoPtr,m_resultOut, m_dispatcher);

		btCollisionAlgorithm* colAlgo = m_dispatcher->findAlgorithm(m_softBody,&triCol,0);

		colAlgo->processCollision(processInfo);
		colAlgo->nihilize(m_dispatcher);
		colAlgo->~btCollisionAlgorithm();
		m_dispatcher->freeCollisionAlgorithm(colAlgo);


		triIndex.m_childShape = tm;
		m_shapeCache.insert(triKey,triIndex);

	}



}



void	btSoftBodyTriangleCallback::setTimeStepAndCounters(btScalar collisionMarginTriangle,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
	m_dispatchInfoPtr = &dispatchInfo;
	m_collisionMarginTriangle = collisionMarginTriangle+btScalar(BT_SOFTBODY_TRIANGLE_EXTRUSION);
	m_resultOut = resultOut;


	btVector3	aabbWorldSpaceMin,aabbWorldSpaceMax;
	static_cast<const btSoftBody*>(m_softBody->getCollisionObject())->getAabb(aabbWorldSpaceMin,aabbWorldSpaceMax);
	btVector3 halfExtents = (aabbWorldSpaceMax-aabbWorldSpaceMin)*btScalar(0.5);
	btVector3 softBodyCenter = (aabbWorldSpaceMax+aabbWorldSpaceMin)*btScalar(0.5);

	btTransform softTransform;
	softTransform.setIdentity();
	softTransform.setOrigin(softBodyCenter);

	btTransform convexInTriangleSpace;
	convexInTriangleSpace = m_triBody->getWorldTransform().inverse() * softTransform;
	btTransformAabb(halfExtents,m_collisionMarginTriangle,convexInTriangleSpace,m_aabbMin,m_aabbMax);
}

void btSoftBodyTriangleCallback::setColliders( const btCollider* triBody, const btCollider* softBody )
{
	m_triBody = triBody; 
	m_softBody = softBody;
}

void btSoftBodyConcaveCollisionAlgorithm::clearCache()
{
	m_btSoftBodyTriangleCallback.clearCache();

}

void btSoftBodyConcaveCollisionAlgorithm::processCollision (const btCollisionProcessInfo& processInfo)
{
	const btCollider& triColl = m_isSwapped ? processInfo.m_body0 : processInfo.m_body1;

	if (triColl.getCollisionShape()->isConcave())
	{
		const btConcaveShape* concaveShape = static_cast<const btConcaveShape*>( triColl.getCollisionShape());

		//	if (convexBody->getCollisionShape()->isConvex())
		{
			btScalar collisionMarginTriangle = concaveShape->getMargin();

			//			resultOut->setPersistentManifold(m_btSoftBodyTriangleCallback.m_manifoldPtr);
			m_btSoftBodyTriangleCallback.setColliders(&triColl, m_isSwapped ? &processInfo.m_body1 : &processInfo.m_body0);
			m_btSoftBodyTriangleCallback.setTimeStepAndCounters(collisionMarginTriangle, processInfo.m_dispatchInfo, processInfo.m_result);
			//Disable persistency. previously, some older algorithm calculated all contacts in one go, so you can clear it here.
			//m_dispatcher->clearManifold(m_btSoftBodyTriangleCallback.m_manifoldPtr);

			concaveShape->processAllTriangles( &m_btSoftBodyTriangleCallback, m_btSoftBodyTriangleCallback.getAabbMin(), m_btSoftBodyTriangleCallback.getAabbMax());


			m_btSoftBodyTriangleCallback.setColliders(0x0, 0x0);
			//	resultOut->refreshContactPoints();

		}

	}

}


btScalar btSoftBodyConcaveCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
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
