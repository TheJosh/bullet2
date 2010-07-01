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

#include "BulletSoftBody/btAcceleratedSoftBody_Settings.h"

#ifdef BULLET_USE_DX11

#include "BulletMultiThreaded/vectormath/scalar/cpp/vectormath_aos.h"
#include "BulletMultiThreaded/vectormath/scalar/cpp/mat_aos.h"
#include "BulletMultiThreaded/vectormath/scalar/cpp/vec_aos.h"

#include "BulletSoftBody/btAcceleratedSoftBody_Solvers.h"
#include "BulletSoftBody/btAcceleratedSoftBody_DX11Buffer.h"
#include "BulletSoftBody/btAcceleratedSoftBody_LinkDataDX11.h"
#include "BulletSoftBody/btAcceleratedSoftBody_VertexDataDX11.h"
#include "BulletSoftBody/btAcceleratedSoftBody_TriangleDataDX11.h"


#ifndef BT_ACCELERATED_SOFT_BODY_DX11_SOLVER_H
#define BT_ACCELERATED_SOFT_BODY_DX11_SOLVER_H

class btDX11SoftBodySolver : public btSoftBodySolver
{
public:

	class KernelDesc
	{
	protected:
		

	public:
		ID3D11ComputeShader* kernel;
		ID3D11Buffer* constBuffer;

		KernelDesc()
		{
			kernel = 0;
			constBuffer = 0;
		}

		virtual ~KernelDesc()
		{
			// TODO: this should probably destroy its kernel but we need to be careful
			// in case KernelDescs are copied
		}
	}; 


	struct PrepareLinksCB
	{		
		int numLinks;
		int padding0;
		int padding1;
		int padding2;
	};

	struct UpdateConstantsCB
	{		
		int numLinks;
		int padding0;
		int padding1;
		int padding2;
	};

	struct SolvePositionsFromLinksKernelCB
	{		
		int startLink;
		int numLinks;
		float kst;
		float ti;
	};

	struct IntegrateCB
	{
		int numNodes;
		float solverdt;
		int padding1;
		int padding2;
	};

	struct UpdatePositionsFromVelocitiesCB
	{
		int numNodes;
		float solverSDT;
		int padding1;
		int padding2;
	};

	struct UpdateVelocitiesFromPositionsWithoutVelocitiesCB
	{
		int numNodes;
		float isolverdt;
		int padding1;
		int padding2;
	};

	struct UpdateVelocitiesFromPositionsWithVelocitiesCB
	{
		int numNodes;
		float isolverdt;
		int padding1;
		int padding2;
	};

	struct UpdateSoftBodiesCB
	{
		int numNodes;
		int startFace;
		int numFaces;
		float epsilon;
	};


	struct OutputToVertexArrayCB
	{
		int startNode;
		int numNodes;
		int positionOffset;
		int positionStride;
		
		int normalOffset;	
		int normalStride;
		int padding1;
		int padding2;
	};


	struct ApplyForcesCB
	{
		unsigned int numNodes;
		float solverdt;
		float epsilon;
		int padding3;
	};

	struct AddVelocityCB
	{
		int startNode;
		int lastNode;
		float velocityX;
		float velocityY;
		float velocityZ;
		int padding1;
		int padding2;
		int padding3;
	};


private:
	ID3D11Device *		 m_dx11Device;
	ID3D11DeviceContext* m_dx11Context;


	/** Link data for all cloths. Note that this will be sorted batch-wise for efficient computation and m_linkAddresses will maintain the addressing. */
	btSoftBodyLinkDataDX11 m_linkData;
	btSoftBodyVertexDataDX11 m_vertexData;
	btSoftBodyTriangleDataDX11 m_triangleData;
		
	/** Variable to define whether we need to update solver constants on the next iteration */
	bool m_updateSolverConstants;

	bool m_shadersInitialized;

	/** 
	 * Cloths owned by this solver.
	 * Only our cloths are in this array.
	 */
	btAlignedObjectArray< btAcceleratedSoftBodyInterface * > m_cloths;

	/** Acceleration value to be applied to all non-static vertices in the solver. 
	 * Index n is cloth n, array sized by number of cloths in the world not the solver. 
	 */
	btAlignedObjectArray< Vectormath::Aos::Vector3 >	m_perClothAcceleration;
	btDX11Buffer<Vectormath::Aos::Vector3>				m_dx11PerClothAcceleration;

	/** Wind velocity to be applied normal to all non-static vertices in the solver. 
	 * Index n is cloth n, array sized by number of cloths in the world not the solver. 
	 */
	btAlignedObjectArray< Vectormath::Aos::Vector3 >	m_perClothWindVelocity;
	btDX11Buffer<Vectormath::Aos::Vector3>				m_dx11PerClothWindVelocity;

	/** Velocity damping factor */
	btAlignedObjectArray< float >						m_perClothDampingFactor;
	btDX11Buffer<float>									m_dx11PerClothDampingFactor;

	/** Velocity correction coefficient */
	btAlignedObjectArray< float >						m_perClothVelocityCorrectionCoefficient;
	btDX11Buffer<float>									m_dx11PerClothVelocityCorrectionCoefficient;

	/** Lift parameter for wind effect on cloth. */
	btAlignedObjectArray< float >						m_perClothLiftFactor;
	btDX11Buffer<float>									m_dx11PerClothLiftFactor;
	
	/** Drag parameter for wind effect on cloth. */
	btAlignedObjectArray< float >						m_perClothDragFactor;
	btDX11Buffer<float>									m_dx11PerClothDragFactor;

	/** Density of the medium in which each cloth sits */
	btAlignedObjectArray< float >						m_perClothMediumDensity;
	btDX11Buffer<float>									m_dx11PerClothMediumDensity;

	KernelDesc		prepareLinksKernel;
	KernelDesc		solvePositionsFromLinksKernel;
	KernelDesc		updateConstantsKernel;
	KernelDesc		integrateKernel;
	KernelDesc		addVelocityKernel;
	KernelDesc		updatePositionsFromVelocitiesKernel;
	KernelDesc		updateVelocitiesFromPositionsWithoutVelocitiesKernel;
	KernelDesc		updateVelocitiesFromPositionsWithVelocitiesKernel;
	KernelDesc		resetNormalsAndAreasKernel;
	KernelDesc		normalizeNormalsAndAreasKernel;
	KernelDesc		updateSoftBodiesKernel;
	KernelDesc		outputToVertexArrayWithNormalsKernel;
	KernelDesc		outputToVertexArrayWithoutNormalsKernel;

	KernelDesc		outputToVertexArrayKernel;
	KernelDesc		applyForcesKernel;
	KernelDesc		collideSphereKernel;
	KernelDesc		collideCylinderKernel;

public:
	btDX11SoftBodySolver(ID3D11Device * dx11Device, ID3D11DeviceContext* dx11Context);

	virtual ~btDX11SoftBodySolver();

	/**
	 * Compile a compute shader kernel from a string and return the appropriate KernelDesc object.
	 */
	KernelDesc compileComputeShaderFromString( const char* shaderString, const char* shaderName, int constBufferSize );

	bool buildShaders();

	virtual void optimize();

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

	virtual bool checkInitialized();

	void resetNormalsAndAreas( int numVertices );

	void normalizeNormalsAndAreas( int numVertices );

	void executeUpdateSoftBodies( int firstTriangle, int numTriangles );
	virtual void updateSoftBodies();

	/** Return the softbody object represented by softBodyIndex */
	virtual btAcceleratedSoftBodyInterface *getSoftBody( int softBodyIndex );

	/**
	 * Add a collision object to be used by the indicated softbody.
	 */
	virtual void addCollisionObjectForSoftBody( int clothIdentifier, btCollisionObject *collisionObject );


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

	void updatePositionsFromVelocities( float solverdt );
	void solveLinksForPosition( int startLink, int numLinks, float kst, float ti );
	
	void updateVelocitiesFromPositionsWithVelocities( float isolverdt );
	void updateVelocitiesFromPositionsWithoutVelocities( float isolverdt );

	// End kernel dispatches
	/////////////////////////////////////

	virtual void outputToVertexBuffers();
};

#endif // #ifndef BT_ACCELERATED_SOFT_BODY_DX11_SOLVER_H


#endif // #ifdef BULLET_USE_DX11
