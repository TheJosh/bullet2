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

#ifndef BT_SOFT_BODY_DEFAULT_SOLVER_H
#define BT_SOFT_BODY_DEFAULT_SOLVER_H

#include "BulletMultiThreaded/vectormath/scalar/cpp/vectormath_aos.h"
#include "BulletMultiThreaded/vectormath/scalar/cpp/mat_aos.h"
#include "BulletMultiThreaded/vectormath/scalar/cpp/vec_aos.h"

#include "BulletSoftBody/btSoftBodySolvers.h"
#include "BulletSoftBody/VertexBuffers/btSoftBodySolverVertexBuffer.h"


class btDefaultSoftBodySolver : public btSoftBodySolver
{
protected:		
	/** Variable to define whether we need to update solver constants on the next iteration */
	bool m_updateSolverConstants;

	btAlignedObjectArray< btSoftBody * > m_softBodySet;


public:
	btDefaultSoftBodySolver();
	
	virtual ~btDefaultSoftBodySolver();



#if 0
	virtual void addVelocity( Vectormath::Aos::Vector3 velocity );

	virtual btSoftBodyLinkData &getLinkData();

	virtual btSoftBodyVertexData &getVertexData();

	virtual btSoftBodyTriangleData &getTriangleData();

	virtual void setPerClothAcceleration( int clothIdentifier, Vectormath::Aos::Vector3 acceleration );

	virtual void setPerClothWindVelocity( int clothIdentifier, Vectormath::Aos::Vector3 windVelocity );

	virtual void setPerClothMediumDensity( int clothIdentifier, float mediumDensity );

	virtual void setPerClothDampingFactor( int clothIdentifier, float dampingFactor );

	virtual void setPerClothVelocityCorrectionCoefficient( int clothIdentifier, float velocityCorrectionCoefficient );

	virtual void setPerClothLiftFactor( int clothIdentifier, float liftFactor );

	/** Drag parameter for wind action on cloth. */
	virtual void setPerClothDragFactor( int clothIdentifier, float dragFactor );

	Vectormath::Aos::Vector3 ProjectOnAxis( const Vectormath::Aos::Vector3 &v, const Vectormath::Aos::Vector3 &a );

	void ApplyClampedForce( float solverdt, const Vectormath::Aos::Vector3 &force, const Vectormath::Aos::Vector3 &vertexVelocity, float inverseMass, Vectormath::Aos::Vector3 &vertexForce );

	virtual void applyForces( float solverdt );

	float computeTriangleArea( 
		const Vectormath::Aos::Point3 &vertex0,
		const Vectormath::Aos::Point3 &vertex1,
		const Vectormath::Aos::Point3 &vertex2 );
#endif




	virtual bool checkInitialized();

	virtual void updateSoftBodies( );

	virtual void optimize( btAlignedObjectArray< btSoftBody * > &softBodies );

	virtual void solveConstraints( float solverdt );

	virtual void predictMotion( float solverdt );

	virtual void copySoftBodyToVertexBuffer( const btSoftBody *const softBody, btVertexBufferDescriptor *vertexBuffer );
};

#endif // #ifndef BT_ACCELERATED_SOFT_BODY_CPU_SOLVER_H