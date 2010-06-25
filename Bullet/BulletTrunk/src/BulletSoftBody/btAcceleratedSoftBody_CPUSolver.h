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

#ifndef BT_ACCELERATED_SOFT_BODY_CPU_SOLVER_H
#define BT_ACCELERATED_SOFT_BODY_CPU_SOLVER_H

#include "BulletMultiThreaded/vectormath/scalar/cpp/vectormath_aos.h"
#include "BulletMultiThreaded/vectormath/scalar/cpp/mat_aos.h"
#include "BulletMultiThreaded/vectormath/scalar/cpp/vec_aos.h"

#include <utility>

#include "BulletSoftBody/btAcceleratedSoftBody_Solvers.h"
#include "BulletSoftBody/btAcceleratedSoftBody_VertexBuffers.h"

#include "BulletSoftBody/btAcceleratedSoftBodyData.h"


class btCPUSoftBodySolver : public btSoftBodySolver
{
private:
	btSoftBodyLinkData m_linkData;
	btSoftBodyVertexData m_vertexData;
	btSoftBodyTriangleData m_triangleData;
		
	/** Variable to define whether we need to update solver constants on the next iteration */
	bool m_updateSolverConstants;

	/** 
	 * Cloths owned by this solver.
	 * Only our cloths are in this array.
	 */
	btAlignedObjectArray< btAcceleratedSoftBodyInterface * > m_cloths;
	
	/** Acceleration value to be applied to all non-static vertices in the solver. 
	 * Index n is cloth n, array sized by number of cloths in the world not the solver. 
	 */
	btAlignedObjectArray< Vectormath::Aos::Vector3 > m_perClothAcceleration;

	/** Wind velocity to be applied normal to all non-static vertices in the solver. 
	 * Index n is cloth n, array sized by number of cloths in the world not the solver. 
	 */
	btAlignedObjectArray< Vectormath::Aos::Vector3 > m_perClothWindVelocity;

	/** Velocity damping factor */
	btAlignedObjectArray< float > m_perClothDampingFactor;

	/** Velocity correction coefficient */
	btAlignedObjectArray< float > m_perClothVelocityCorrectionCoefficient;

	/** Lift parameter for wind effect on cloth. */
	btAlignedObjectArray< float > m_perClothLiftFactor;
	
	/** Drag parameter for wind effect on cloth. */
	btAlignedObjectArray< float > m_perClothDragFactor;

	/** Density of the medium in which each cloth sits */
	btAlignedObjectArray< float > m_perClothMediumDensity;


public:
	btCPUSoftBodySolver();
	
	virtual ~btCPUSoftBodySolver();

	virtual int ownCloth( btAcceleratedSoftBodyInterface *cloth );
	virtual void removeCloth( btAcceleratedSoftBodyInterface *cloth );

	virtual btSoftBodyLinkData &getLinkData();

	virtual btSoftBodyVertexData &getVertexData();

	virtual btSoftBodyTriangleData &getTriangleData();

	virtual void addVelocity( Vectormath::Aos::Vector3 velocity );

	virtual void setPerClothAcceleration( int clothIdentifier, Vectormath::Aos::Vector3 acceleration );

	virtual void setPerClothWindVelocity( int clothIdentifier, Vectormath::Aos::Vector3 windVelocity );

	virtual void setPerClothMediumDensity( int clothIdentifier, float mediumDensity );

	virtual void setPerClothDampingFactor( int clothIdentifier, float dampingFactor );

	virtual void setPerClothVelocityCorrectionCoefficient( int clothIdentifier, float velocityCorrectionCoefficient );

	virtual void setPerClothLiftFactor( int clothIdentifier, float liftFactor );

	/** Drag parameter for wind action on cloth. */
	virtual void setPerClothDragFactor( int clothIdentifier, float dragFactor );

	virtual void updateSoftBodies();

	Vectormath::Aos::Vector3 ProjectOnAxis( const Vectormath::Aos::Vector3 &v, const Vectormath::Aos::Vector3 &a );

	void ApplyClampedForce( float solverdt, const Vectormath::Aos::Vector3 &force, const Vectormath::Aos::Vector3 &vertexVelocity, float inverseMass, Vectormath::Aos::Vector3 &vertexForce );

	virtual bool checkInitialized();

	virtual void applyForces( float solverdt );

	virtual void optimize();

	/** Return the softbody object represented by softBodyIndex */
	virtual btAcceleratedSoftBodyInterface *getSoftBody( int softBodyIndex );

	/**
	 * Integrate motion on the solver.
	 */
	virtual void integrate( float solverdt );

	float computeTriangleArea( 
		const Vectormath::Aos::Point3 &vertex0,
		const Vectormath::Aos::Point3 &vertex1,
		const Vectormath::Aos::Point3 &vertex2 );

	virtual void updateConstants( float timeStep );

	virtual void solveConstraints( float solverdt );

	virtual void outputToVertexBuffers();
};

#endif // #ifndef BT_ACCELERATED_SOFT_BODY_CPU_SOLVER_H