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


#ifndef CONVEX_CAST_H
#define CONVEX_CAST_H

#include <LinearMath/SimdTransform.h>
#include <LinearMath/SimdVector3.h>
#include <LinearMath/SimdScalar.h>
class btMinkowskiSumShape;
#include "LinearMath/GenIDebugDraw.h"

/// btConvexCast is an interface for Casting
class btConvexCast
{
public:


	virtual ~btConvexCast();

	///RayResult stores the closest result
	/// alternatively, add a callback method to decide about closest/all results
	struct	CastResult
	{
		//virtual bool	addRayResult(const btSimdVector3& normal,SimdScalar	fraction) = 0;
				
		virtual void	DebugDraw(SimdScalar	fraction) {}
		virtual void	DrawCoordSystem(const btSimdTransform& trans) {}

		CastResult()
			:m_fraction(1e30f),
			m_debugDrawer(0)
		{
		}


		virtual ~CastResult() {};

		btSimdVector3	m_normal;
		SimdScalar	m_fraction;
		btSimdTransform	m_hitTransformA;
		btSimdTransform	m_hitTransformB;

		btIDebugDraw* m_debugDrawer;

	};


	/// cast a convex against another convex object
	virtual bool	calcTimeOfImpact(
					const btSimdTransform& fromA,
					const btSimdTransform& toA,
					const btSimdTransform& fromB,
					const btSimdTransform& toB,
					CastResult& result) = 0;
};

#endif //CONVEX_CAST_H
