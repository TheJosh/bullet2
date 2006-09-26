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

#ifndef SOLVE_2LINEAR_CONSTRAINT_H
#define SOLVE_2LINEAR_CONSTRAINT_H

#include "LinearMath/SimdMatrix3x3.h"
#include "LinearMath/SimdVector3.h"


class btRigidBody;



/// constraint class used for lateral tyre friction.
class	btSolve2LinearConstraint
{
	SimdScalar	m_tau;
	SimdScalar	m_damping;

public:

	btSolve2LinearConstraint(SimdScalar tau,SimdScalar damping)
	{
		m_tau = tau;
		m_damping = damping;
	}
	//
	// solve unilateral constraint (equality, direct method)
	//
	void resolveUnilateralPairConstraint(		
														   btRigidBody* body0,
		btRigidBody* body1,

		const btSimdMatrix3x3& world2A,
						const btSimdMatrix3x3& world2B,
						
						const btSimdVector3& invInertiaADiag,
						const SimdScalar invMassA,
						const btSimdVector3& linvelA,const btSimdVector3& angvelA,
						const btSimdVector3& rel_posA1,
						const btSimdVector3& invInertiaBDiag,
						const SimdScalar invMassB,
						const btSimdVector3& linvelB,const btSimdVector3& angvelB,
						const btSimdVector3& rel_posA2,

					  SimdScalar depthA, const btSimdVector3& normalA, 
					  const btSimdVector3& rel_posB1,const btSimdVector3& rel_posB2,
					  SimdScalar depthB, const btSimdVector3& normalB, 
					  SimdScalar& imp0,SimdScalar& imp1);


	//
	// solving 2x2 lcp problem (inequality, direct solution )
	//
	void resolveBilateralPairConstraint(
			btRigidBody* body0,
						btRigidBody* body1,
		const btSimdMatrix3x3& world2A,
						const btSimdMatrix3x3& world2B,
						
						const btSimdVector3& invInertiaADiag,
						const SimdScalar invMassA,
						const btSimdVector3& linvelA,const btSimdVector3& angvelA,
						const btSimdVector3& rel_posA1,
						const btSimdVector3& invInertiaBDiag,
						const SimdScalar invMassB,
						const btSimdVector3& linvelB,const btSimdVector3& angvelB,
						const btSimdVector3& rel_posA2,

					  SimdScalar depthA, const btSimdVector3& normalA, 
					  const btSimdVector3& rel_posB1,const btSimdVector3& rel_posB2,
					  SimdScalar depthB, const btSimdVector3& normalB, 
					  SimdScalar& imp0,SimdScalar& imp1);


	void resolveAngularConstraint(	const btSimdMatrix3x3& invInertiaAWS,
						const SimdScalar invMassA,
						const btSimdVector3& linvelA,const btSimdVector3& angvelA,
						const btSimdVector3& rel_posA1,
						const btSimdMatrix3x3& invInertiaBWS,
						const SimdScalar invMassB,
						const btSimdVector3& linvelB,const btSimdVector3& angvelB,
						const btSimdVector3& rel_posA2,

					  SimdScalar depthA, const btSimdVector3& normalA, 
					  const btSimdVector3& rel_posB1,const btSimdVector3& rel_posB2,
					  SimdScalar depthB, const btSimdVector3& normalB, 
					  SimdScalar& imp0,SimdScalar& imp1);


};

#endif //SOLVE_2LINEAR_CONSTRAINT_H
