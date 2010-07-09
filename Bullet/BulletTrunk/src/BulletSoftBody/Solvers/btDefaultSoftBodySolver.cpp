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

#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletMultiThreaded/vectormath/scalar/cpp/vectormath_aos.h"
#include "BulletMultiThreaded/vectormath/scalar/cpp/mat_aos.h"
#include "BulletMultiThreaded/vectormath/scalar/cpp/vec_aos.h"

#include "BulletSoftBody/solvers/btDefaultSoftBodySolver.h"
#include "BulletCollision/CollisionShapes/btCapsuleShape.h"
#include "BulletSoftBody/btSoftBody.h"


btDefaultSoftBodySolver::btDefaultSoftBodySolver()
{
	// Initial we will clearly need to update solver constants
	// For now this is global for the cloths linked with this solver - we should probably make this body specific 
	// for performance in future once we understand more clearly when constants need to be updated
	m_updateSolverConstants = true;
}

btDefaultSoftBodySolver::~btDefaultSoftBodySolver()
{
}


#if 0 // grab all these direct from the soft body and update the array


void btDefaultSoftBodySolver::addVelocity( Vectormath::Aos::Vector3 velocity )
{
	int numVertices = m_vertexData.getNumVertices();
	for( int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex )
	{
		if( m_vertexData.getInverseMass( vertexIndex ) > 0 )
			m_vertexData.getVelocity( vertexIndex ) += velocity;
	}
}

void btDefaultSoftBodySolver::setPerClothAcceleration( int clothIdentifier, Vectormath::Aos::Vector3 acceleration )
{
	m_perClothAcceleration[clothIdentifier] = acceleration;
}

void btDefaultSoftBodySolver::setPerClothWindVelocity( int clothIdentifier, Vectormath::Aos::Vector3 windVelocity )
{
	m_perClothWindVelocity[clothIdentifier] = windVelocity;
}

void btDefaultSoftBodySolver::setPerClothMediumDensity( int clothIdentifier, float mediumDensity )
{
	m_perClothMediumDensity[clothIdentifier] = mediumDensity;
}

void btDefaultSoftBodySolver::setPerClothDampingFactor( int clothIdentifier, float dampingFactor )
{
	m_perClothDampingFactor[clothIdentifier] = dampingFactor;
}

void btDefaultSoftBodySolver::setPerClothVelocityCorrectionCoefficient( int clothIdentifier, float velocityCorrectionCoefficient )
{
	m_perClothVelocityCorrectionCoefficient[clothIdentifier] = velocityCorrectionCoefficient;
}		

void btDefaultSoftBodySolver::setPerClothLiftFactor( int clothIdentifier, float liftFactor )
{
	m_perClothLiftFactor[clothIdentifier] = liftFactor;
}

/** Drag parameter for wind action on cloth. */
void btDefaultSoftBodySolver::setPerClothDragFactor( int clothIdentifier, float dragFactor )
{
	m_perClothDragFactor[clothIdentifier] = dragFactor;
}
#endif



void btDefaultSoftBodySolver::optimize( btAlignedObjectArray< btSoftBody * > &softBodies )
{
	if( m_softBodySet.size() != softBodies.size() )
	{
		// Have a change in the soft body set so store that here
		m_softBodySet.copyFromArray( softBodies );
	}
}

void btDefaultSoftBodySolver::updateSoftBodies( )
{
	for ( int i=0; i < m_softBodySet.size(); i++)
	{
		btSoftBody*	psb=(btSoftBody*)m_softBodySet[i];
		psb->integrateMotion();	
	}
} // updateSoftBodies

bool btDefaultSoftBodySolver::checkInitialized()
{
	return true;
}

void btDefaultSoftBodySolver::solveConstraints( float solverdt )
{
	// Solve constraints for non-solver softbodies
	for(int i=0; i < m_softBodySet.size(); ++i)
	{
		btSoftBody*	psb = static_cast<btSoftBody*>(m_softBodySet[i]);
		psb->solveConstraints();
	}	
} // btDefaultSoftBodySolver::solveConstraints


void btDefaultSoftBodySolver::copySoftBodyToVertexBuffer( const btSoftBody *const softBody, btVertexBufferDescriptor *vertexBuffer )
{
	// Currently only support CPU output buffers
	// TODO: check for DX11 buffers. Take all offsets into the same DX11 buffer
	// and use them together on a single kernel call if possible by setting up a
	// per-cloth target buffer array for the copy kernel.

	if( vertexBuffer->getBufferType() == btVertexBufferDescriptor::CPU_BUFFER )
	{
		const btAlignedObjectArray<btSoftBody::Node> &clothVertices( softBody->m_nodes );
		int numVertices = clothVertices.size();

		const btCPUVertexBufferDescriptor *cpuVertexBuffer = static_cast< btCPUVertexBufferDescriptor* >(vertexBuffer);						
		float *basePointer = cpuVertexBuffer->getBasePointer();						

		if( vertexBuffer->hasVertexPositions() )
		{
			const int vertexOffset = cpuVertexBuffer->getVertexOffset();
			const int vertexStride = cpuVertexBuffer->getVertexStride();
			float *vertexPointer = basePointer + vertexOffset;

			for( int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex )
			{
				btVector3 position = clothVertices[vertexIndex].m_x;
				*(vertexPointer + 0) = position.getX();
				*(vertexPointer + 1) = position.getY();
				*(vertexPointer + 2) = position.getZ();
				vertexPointer += vertexStride;
			}
		}
		if( vertexBuffer->hasNormals() )
		{
			const int normalOffset = cpuVertexBuffer->getNormalOffset();
			const int normalStride = cpuVertexBuffer->getNormalStride();
			float *normalPointer = basePointer + normalOffset;

			for( int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex )
			{
				btVector3 normal = clothVertices[vertexIndex].m_n;
				*(normalPointer + 0) = normal.getX();
				*(normalPointer + 1) = normal.getY();
				*(normalPointer + 2) = normal.getZ();
				normalPointer += normalStride;
			}
		}
	}
} // btDefaultSoftBodySolver::copySoftBodyToVertexBuffer


void btDefaultSoftBodySolver::predictMotion( float timeStep )
{
	for ( int i=0; i < m_softBodySet.size(); ++i)
	{
		btSoftBody*	psb = m_softBodySet[i];

		psb->predictMotion(timeStep);		
	}
}

static Vectormath::Aos::Vector3 toVector3( const btVector3 &vec )
{
	Vectormath::Aos::Vector3 outVec( vec.getX(), vec.getY(), vec.getZ() );
	return outVec;
}

static Vectormath::Aos::Transform3 toTransform3( const btTransform &transform )
{
	Vectormath::Aos::Transform3 outTransform;
	outTransform.setCol(0, toVector3(transform.getBasis().getColumn(0)));
	outTransform.setCol(1, toVector3(transform.getBasis().getColumn(1)));
	outTransform.setCol(2, toVector3(transform.getBasis().getColumn(2)));
	outTransform.setCol(3, toVector3(transform.getOrigin()));
	return outTransform;	
}
