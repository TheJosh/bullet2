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

#ifndef BT_ACCELERATED_SOFT_BODY_SOLVERS_H
#define BT_ACCELERATED_SOFT_BODY_SOLVERS_H

#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"
#include "BulletMultiThreaded/vectormath/scalar/cpp/vectormath_aos.h"
#include "BulletMultiThreaded/vectormath/scalar/cpp/mat_aos.h"
#include "BulletMultiThreaded/vectormath/scalar/cpp/vec_aos.h"

class btSoftBodyTriangleData;
class btSoftBodyLinkData;
class btSoftBodyVertexData;
class btVertexBufferDescriptor;
class btAcceleratedSoftBodyInterface;
class btCollisionObject;


class btSoftBodySolver
{

protected:
	int m_numberOfPositionIterations;
	// Simulation timescale
	float m_timeScale;

public:
	btSoftBodySolver() :
		m_numberOfPositionIterations( 10 ),
		m_timeScale( 1 )

	{
	}

	virtual ~btSoftBodySolver()
	{
	}

	/** 
	 * Ensure that data structures are ready for this cloth to be owned by the solver.
	 * Returns the identifier of the cloth within the solver.
	 */
	virtual int ownCloth( btAcceleratedSoftBodyInterface *cloth ) = 0;

	/** Do any cleanup work required to remove a cloth from ownership by this solver */
	virtual void removeCloth( btAcceleratedSoftBodyInterface *cloth ) = 0;

	/** Return a reference to the solver's link data structure */
	virtual btSoftBodyLinkData &getLinkData() = 0;
	/** Return a reference to the solver's vertex data structure */
	virtual btSoftBodyVertexData &getVertexData() = 0;
	/** Return a reference to the solver's triangle data structure */
	virtual btSoftBodyTriangleData &getTriangleData() = 0;

	/** Acceleration for all cloths in the solver. Can be used to efficiently apply gravity. */
	virtual void setPerClothAcceleration( int clothIdentifier, Vectormath::Aos::Vector3 acceleration ) = 0;

	/** A wind velocity applied normal to the cloth for all cloths in the solver. */
	virtual void setPerClothWindVelocity( int clothIdentifier, Vectormath::Aos::Vector3 windVelocity ) = 0;

	/** Set the density of the medium a given cloth is situated in. This could be air or possibly water. */
	virtual void setPerClothMediumDensity( int clothIdentifier, float mediumDensity ) = 0;		

	/** A damping factor specific to each cloth applied for all cloths. */
	virtual void setPerClothDampingFactor( int clothIdentifier, float dampingFactor ) = 0;

	/** A damping factor specific to each cloth applied for all cloths. */
	virtual void setPerClothVelocityCorrectionCoefficient( int clothIdentifier, float velocityCorrectionCoefficient ) = 0;

	/** Lift parameter for wind action on cloth. */
	virtual void setPerClothLiftFactor( int clothIdentifier, float liftFactor ) = 0;

	/** Drag parameter for wind action on cloth. */
	virtual void setPerClothDragFactor( int clothIdentifier, float dragFactor ) = 0;

	/** Ensure that this solver is initialized. */
	virtual bool checkInitialized() = 0;

	/* Setup constants necessary for the current simulation */
	virtual void updateConstants( float timeStep ) = 0;

	/** Apply configured forces to the set of cloths */
	virtual void applyForces( float solverdt ) = 0;

	/* Integrate motion for all soft bodies in the solver updating positions etc. */
	virtual void integrate( float solverdt ) = 0;

	/** Perform necessary per-step updates of soft bodies such as recomputing normals and bounding boxes */
	virtual void updateSoftBodies() = 0;

	/** Output current computed vertex data to the vertex buffers for all cloths in the solver. */
	virtual void outputToVertexBuffers() = 0;

	/** Optimize soft bodies in this solver. */
	virtual void optimize() = 0;

	/**
	 * Add a velocity to all soft bodies in the solver - useful for doing world-wide velocities such as a change due to gravity 
	 * Only add a velocity to nodes with a non-zero inverse mass.
	 */
	virtual void addVelocity( Vectormath::Aos::Vector3 velocity ) = 0;

	/** Solver steps */
	virtual void solveConstraints( float solverdt ) = 0;

	/** Set the number of velocity constraint solver iterations this solver uses. */
	virtual void setNumberOfPositionIterations( int iterations )
	{
		m_numberOfPositionIterations = iterations;
	}

	/** Get the number of velocity constraint solver iterations this solver uses. */
	virtual int getNumberOfPositionIterations()
	{
		return m_numberOfPositionIterations;
	}

	/** Return the timescale that the simulation is using */
	float getTimeScale()
	{
		return m_timeScale;
	}

	/** Return the softbody object represented by softBodyIndex */
	virtual btAcceleratedSoftBodyInterface *getSoftBody( int softBodyIndex ) = 0;

	/**
	 * Add a collision object to be used by the indicated softbody.
	 */
	virtual void addCollisionObjectForSoftBody( int clothIdentifier, btCollisionObject *collisionObject ) = 0;
};


/**
 * SoftBody class to maintain information about a soft body instance
 * within a solver.
 * This data addresses the main solver arrays.
 */
class btAcceleratedSoftBodyInterface
{
protected:
	/** Identifier for this cloth */
	int m_clothIdentifier;

	/** Current number of vertices that are part of this cloth */
	int m_numVertices;
	/** Maximum number of vertices allocated to be part of this cloth */
	int m_maxVertices;
	/** Current number of triangles that are part of this cloth */
	int m_numTriangles;
	/** Maximum number of triangles allocated to be part of this cloth */
	int m_maxTriangles;
	/** Index of first vertex in the world allocated to this cloth */
	int m_firstVertex;
	/** Index of first triangle in the world allocated to this cloth */
	int m_firstTriangle;
	/** Index of first link in the world allocated to this cloth */
	int m_firstLink;
	/** Maximum number of links allocated to this cloth */
	int m_maxLinks;
	/** Current number of links allocated to this cloth */
	int m_numLinks;

	/** Current solver in which this cloth resides */
	btSoftBodySolver *m_currentSolver;

	/** Vertex buffer output to send node data into */
	btVertexBufferDescriptor *m_vertexBuffer;

public:
	btAcceleratedSoftBodyInterface( )
	{
		
		m_numVertices = 0;
		m_maxVertices = 0;
		m_numTriangles = 0;
		m_maxTriangles = 0;
		m_firstVertex = 0;
		m_firstTriangle = 0;
		m_firstLink = 0;
		m_maxLinks = 0;
		m_numLinks = 0;
		m_currentSolver = 0;
		m_vertexBuffer = 0;
	}

	int getIdentifier()
	{
		return m_clothIdentifier;
	}

	btSoftBodySolver *getSolver()
	{
		return m_currentSolver;
	}

	void setSolver( btSoftBodySolver *solver )
	{
		m_currentSolver = solver;
		m_clothIdentifier = m_currentSolver->ownCloth( this );
		m_currentSolver->setPerClothAcceleration(m_clothIdentifier, Vectormath::Aos::Vector3(0, 0, 0));
		m_currentSolver->setPerClothDampingFactor(m_clothIdentifier, 0);
		m_currentSolver->setPerClothVelocityCorrectionCoefficient(m_clothIdentifier, 0.001f);
		m_currentSolver->setPerClothLiftFactor(m_clothIdentifier, 0.0005f);
		m_currentSolver->setPerClothDragFactor(m_clothIdentifier, 0.f);
		m_currentSolver->setPerClothWindVelocity(m_clothIdentifier, Vectormath::Aos::Vector3(0.f, 0.f, 0.f));
	}

	int getNumVertices()
	{
		return m_numVertices;
	}

	int getNumTriangles()
	{
		return m_numTriangles;
	}

	int getMaxVertices()
	{
		return m_maxVertices;
	}

	int getMaxTriangles()
	{
		return m_maxTriangles;
	}

	int getFirstVertex()
	{
		return m_firstVertex;
	}

	int getFirstTriangle()
	{
		return m_firstTriangle;
	}

	// TODO: All of these set functions will have to do checks and
	// update the world because restructuring of the arrays will be necessary
	// Reasonable use of "friend"?
	void setNumVertices( int numVertices )
	{
		m_numVertices = numVertices;
	}	
	
	void setNumTriangles( int numTriangles )
	{
		m_numTriangles = numTriangles;
	}

	void setMaxVertices( int maxVertices )
	{
		m_maxVertices = maxVertices;
	}

	void setMaxTriangles( int maxTriangles )
	{
		m_maxTriangles = maxTriangles;
	}

	void setFirstVertex( int firstVertex )
	{
		m_firstVertex = firstVertex;
	}

	void setFirstTriangle( int firstTriangle )
	{
		m_firstTriangle = firstTriangle;
	}

	void setMaxLinks( int maxLinks )
	{
		m_maxLinks = maxLinks;
	}

	void setNumLinks( int numLinks )
	{
		m_numLinks = numLinks;
	}

	void setFirstLink( int firstLink )
	{
		m_firstLink = firstLink;
	}

	int getMaxLinks()
	{
		return m_maxLinks;
	}

	int getNumLinks()
	{
		return m_numLinks;
	}

	int getFirstLink()
	{
		return m_firstLink;
	}

	void setAcceleration( Vectormath::Aos::Vector3 acceleration )
	{
		m_currentSolver->setPerClothAcceleration( m_clothIdentifier, acceleration );
	}

	void setWindVelocity( Vectormath::Aos::Vector3 windVelocity )
	{
		m_currentSolver->setPerClothWindVelocity( m_clothIdentifier, windVelocity );
	}

	/** 
	 * Set the density of the air in which the cloth is situated.
	 */
	void setAirDensity( btScalar density )
	{
		m_currentSolver->setPerClothMediumDensity( m_clothIdentifier, static_cast<float>(density) );
	}

	/** 
	 * Set the vertex buffer to output to from this cloth.
	 */
	void setVertexBufferTarget( btVertexBufferDescriptor *vertexBuffer )
	{
		m_vertexBuffer = vertexBuffer;
	}	

	/** 
	 * Obtain the vertex buffer to output to from this cloth.
	 */
	btVertexBufferDescriptor *getVertexBufferTarget()
	{
		return m_vertexBuffer;
	}

	/**
	 * Add a collision object to this soft body.
	 */
	void addCollisionObject( btCollisionObject *collisionObject )
	{
		m_currentSolver->addCollisionObjectForSoftBody( m_clothIdentifier, collisionObject );
	}
};

#endif // #ifndef BT_ACCELERATED_SOFT_BODY_H