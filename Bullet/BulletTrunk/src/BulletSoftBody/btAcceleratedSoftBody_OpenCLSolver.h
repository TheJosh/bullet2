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

#ifndef BT_ACCELERATED_SOFT_BODY_OPENCL_SOLVER_H
#define BT_ACCELERATED_SOFT_BODY_OPENCL_SOLVER_H

#include "BulletSoftBody/btAcceleratedSoftBody_Settings.h"

#ifdef BULLET_USE_OPENCL

#include "BulletMultiThreaded/vectormath/scalar/cpp/vectormath_aos.h"
#include "BulletMultiThreaded/vectormath/scalar/cpp/mat_aos.h"
#include "BulletMultiThreaded/vectormath/scalar/cpp/vec_aos.h"

#include "BulletSoftBody/btAcceleratedSoftBody_Solvers.h"
#include "BulletSoftBody/btAcceleratedSoftBody_OpenCLBuffer.h"
#include "BulletSoftBody/btAcceleratedSoftBody_LinkDataOpenCL.h"
#include "BulletSoftBody/btAcceleratedSoftBody_VertexDataOpenCL.h"
#include "BulletSoftBody/btAcceleratedSoftBody_TriangleDataOpenCL.h"


class btOpenCLSoftBodySolver : public btSoftBodySolver
{
private:
	class KernelDesc
	{
	protected:
	public:
		cl::Kernel kernel;

		KernelDesc()
		{
		}

		virtual ~KernelDesc()
		{
		}
	}; 

	btSoftBodyLinkDataOpenCL m_linkData;
	btSoftBodyVertexDataOpenCL m_vertexData;
	btSoftBodyTriangleDataOpenCL m_triangleData;

	/** Variable to define whether we need to update solver constants on the next iteration */
	bool m_updateSolverConstants;

	bool m_shadersInitialized;

	/** 
	 * Cloths owned by this solver.
	 * Only our cloths are in this array.
	 */
	btAlignedObjectArray< SoftBody * > m_cloths;

	/** Acceleration value to be applied to all non-static vertices in the solver. 
	 * Index n is cloth n, array sized by number of cloths in the world not the solver. 
	 */
	btAlignedObjectArray< Vectormath::Aos::Vector3 >	m_perClothAcceleration;
	btOpenCLBuffer<Vectormath::Aos::Vector3>				m_clPerClothAcceleration;

	/** Wind velocity to be applied normal to all non-static vertices in the solver. 
	 * Index n is cloth n, array sized by number of cloths in the world not the solver. 
	 */
	btAlignedObjectArray< Vectormath::Aos::Vector3 >	m_perClothWindVelocity;
	btOpenCLBuffer<Vectormath::Aos::Vector3>				m_clPerClothWindVelocity;

	/** Velocity damping factor */
	btAlignedObjectArray< float >						m_perClothDampingFactor;
	btOpenCLBuffer<float>									m_clPerClothDampingFactor;

	/** Velocity correction coefficient */
	btAlignedObjectArray< float >						m_perClothVelocityCorrectionCoefficient;
	btOpenCLBuffer<float>									m_clPerClothVelocityCorrectionCoefficient;

	/** Lift parameter for wind effect on cloth. */
	btAlignedObjectArray< float >						m_perClothLiftFactor;
	btOpenCLBuffer<float>									m_clPerClothLiftFactor;
	
	/** Drag parameter for wind effect on cloth. */
	btAlignedObjectArray< float >						m_perClothDragFactor;
	btOpenCLBuffer<float>									m_clPerClothDragFactor;

	/** Density of the medium in which each cloth sits */
	btAlignedObjectArray< float >						m_perClothMediumDensity;
	btOpenCLBuffer<float>									m_clPerClothMediumDensity;

	KernelDesc		prepareLinksKernel;
	KernelDesc		solvePositionsFromLinksKernel;
	KernelDesc		updateConstantsKernel;
	KernelDesc		integrateKernel;
	KernelDesc		addVelocityKernel;
	KernelDesc		updatePositionsFromVelocitiesKernel;
	KernelDesc		updateVelocitiesFromPositionsWithoutVelocitiesKernel;
	KernelDesc		updateVelocitiesFromPositionsWithVelocitiesKernel;
	KernelDesc		vSolveLinksKernel;
	KernelDesc		resetNormalsAndAreasKernel;
	KernelDesc		normalizeNormalsAndAreasKernel;
	KernelDesc		updateSoftBodiesKernel;
	KernelDesc		outputToVertexArrayWithNormalsKernel;
	KernelDesc		outputToVertexArrayWithoutNormalsKernel;

	KernelDesc		outputToVertexArrayKernel;
	KernelDesc		applyForcesKernel;
	KernelDesc		collideSphereKernel;
	KernelDesc		collideCylinderKernel;

	static const int workGroupSize = 128;

	cl::CommandQueue m_queue;
	cl::Context context;
	cl::Device device;

public:
	btOpenCLSoftBodySolver(const cl::CommandQueue &queue);

	virtual ~btOpenCLSoftBodySolver();

	/**
	 * Compile a compute shader kernel from a string and return the appropriate KernelDesc object.
	 */
	KernelDesc compileCLKernelFromString( const char *shaderString, const char *shaderName );

	bool buildShaders();

	void optimize();

	virtual int ownCloth( SoftBody *cloth );

	virtual void removeCloth( BTAcceleratedSoftBody::Cloth *cloth );

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
	virtual bool checkInitialized();

	void resetNormalsAndAreas( int numVertices );

	void normalizeNormalsAndAreas( int numVertices );

	void executeUpdateSoftBodies( int firstTriangle, int numTriangles );

	virtual void updateSoftBodies();

	/** Return the softbody object represented by softBodyIndex */
	virtual SoftBody *getSoftBody( int softBodyIndex );


	Vectormath::Aos::Vector3 ProjectOnAxis( const Vectormath::Aos::Vector3 &v, const Vectormath::Aos::Vector3 &a );

	void ApplyClampedForce( float solverdt, const Vectormath::Aos::Vector3 &force, const Vectormath::Aos::Vector3 &vertexVelocity, float inverseMass, Vectormath::Aos::Vector3 &vertexForce );

	virtual void applyForces( float solverdt );
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

	//////////////////////////////////////
	// Kernel dispatches
	void prepareLinks();

	void solveLinksForVelocity( int startLink, int numLinks, float kst );

	void updatePositionsFromVelocities( float solverdt );

	void solveLinksForPosition( int startLink, int numLinks, float kst, float ti );
	
	void updateVelocitiesFromPositionsWithVelocities( float isolverdt );

	void updateVelocitiesFromPositionsWithoutVelocities( float isolverdt );

	// End kernel dispatches
	/////////////////////////////////////



	virtual void outputToVertexBuffers();
}; // btOpenCLSoftBodySolver

#endif // #ifdef BULLET_USE_OPENCL

#endif #ifndef BT_ACCELERATED_SOFT_BODY_OPENCL_SOLVER_H