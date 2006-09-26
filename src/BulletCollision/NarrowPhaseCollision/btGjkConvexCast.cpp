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



#include "btGjkConvexCast.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btMinkowskiSumShape.h"
#include "btGjkPairDetector.h"
#include "btPointCollector.h"


btGjkConvexCast::btGjkConvexCast(btConvexShape* convexA,btConvexShape* convexB,btSimplexSolverInterface* simplexSolver)
:m_simplexSolver(simplexSolver),
m_convexA(convexA),
m_convexB(convexB)
{
}

bool	btGjkConvexCast::calcTimeOfImpact(
					const btSimdTransform& fromA,
					const btSimdTransform& toA,
					const btSimdTransform& fromB,
					const btSimdTransform& toB,
					CastResult& result)
{


	btMinkowskiSumShape combi(m_convexA,m_convexB);
	btMinkowskiSumShape* convex = &combi;

	btSimdTransform	rayFromLocalA;
	btSimdTransform	rayToLocalA;

	rayFromLocalA = fromA.inverse()* fromB;
	rayToLocalA = toA.inverse()* toB;


	btSimdTransform trA,trB;
	trA = btSimdTransform(fromA);
	trB = btSimdTransform(fromB);
	trA.setOrigin(SimdPoint3(0,0,0));
	trB.setOrigin(SimdPoint3(0,0,0));

	convex->SetTransformA(trA);
	convex->SetTransformB(trB);




	float radius = 0.01f;

	SimdScalar lambda = 0.f;
	btSimdVector3 s = rayFromLocalA.getOrigin();
	btSimdVector3 r = rayToLocalA.getOrigin()-rayFromLocalA.getOrigin();
	btSimdVector3 x = s;
	btSimdVector3 n;
	n.setValue(0,0,0);
	bool hasResult = false;
	btSimdVector3 c;

	float lastLambda = lambda;

	//first solution, using GJK

	//no penetration support for now, perhaps pass a pointer when we really want it
	btConvexPenetrationDepthSolver* penSolverPtr = 0;

	btSimdTransform identityTrans;
	identityTrans.setIdentity();

	btSphereShape	raySphere(0.0f);
	raySphere.SetMargin(0.f);

	btSimdTransform sphereTr;
	sphereTr.setIdentity();
	sphereTr.setOrigin( rayFromLocalA.getOrigin());

	result.DrawCoordSystem(sphereTr);
	{
		btPointCollector	pointCollector1;
		btGjkPairDetector gjk(&raySphere,convex,m_simplexSolver,penSolverPtr);		

		btGjkPairDetector::ClosestPointInput input;
		input.m_transformA = sphereTr;
		input.m_transformB = identityTrans;
		gjk.GetClosestPoints(input,pointCollector1,0);

		hasResult = pointCollector1.m_hasResult;
		c = pointCollector1.m_pointInWorld;
		n = pointCollector1.m_normalOnBInWorld;
	}

	

	if (hasResult)
	{
		SimdScalar dist;
		dist = (c-x).length();
		if (dist < radius)
		{
			//penetration
			lastLambda = 1.f;
		}

		//not close enough
		while (dist > radius)
		{
			
			n = x - c;
			SimdScalar nDotr = n.dot(r);

			if (nDotr >= -(SIMD_EPSILON*SIMD_EPSILON))
				return false;
			
			lambda = lambda - n.dot(n) / nDotr;
			if (lambda <= lastLambda)
				break;

			lastLambda = lambda;

			x = s + lambda * r;

			sphereTr.setOrigin( x );
			result.DrawCoordSystem(sphereTr);
			btPointCollector	pointCollector;
			btGjkPairDetector gjk(&raySphere,convex,m_simplexSolver,penSolverPtr);
			btGjkPairDetector::ClosestPointInput input;
			input.m_transformA = sphereTr;
			input.m_transformB = identityTrans;
			gjk.GetClosestPoints(input,pointCollector,0);
			if (pointCollector.m_hasResult)
			{
				if (pointCollector.m_distance < 0.f)
				{
					//degeneracy, report a hit
					result.m_fraction = lastLambda;
					result.m_normal = n;
					return true;
				}
				c = pointCollector.m_pointInWorld;			
				dist = (c-x).length();
			} else
			{
				//??
				return false;
			}

		}

		if (lastLambda < 1.f)
		{
		
			result.m_fraction = lastLambda;
			result.m_normal = n;
			return true;
		}
	}

	return false;
}

