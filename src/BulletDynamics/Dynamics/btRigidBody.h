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

#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <vector>
#include <LinearMath/SimdPoint3.h>
#include <LinearMath/SimdTransform.h>
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"


#include "BulletCollision/CollisionDispatch/btCollisionObject.h"

class btCollisionShape;
struct btMassProps;
typedef SimdScalar dMatrix3[4*3];

extern float gLinearAirDamping;
extern bool gUseEpa;



/// btRigidBody class for btRigidBody Dynamics
/// 
class btRigidBody  : public btCollisionObject
{
public:

	btRigidBody(const btMassProps& massProps,SimdScalar linearDamping,SimdScalar angularDamping,SimdScalar friction,SimdScalar restitution);

	void			proceedToTransform(const btSimdTransform& newTrans); 
	
	
	/// continuous collision detection needs prediction
	void			predictIntegratedTransform(SimdScalar step, btSimdTransform& predictedTransform) const;
	
	void			saveKinematicState(SimdScalar step);
	

	void			applyForces(SimdScalar step);
	
	void			setGravity(const btSimdVector3& acceleration);  
	
	void			setDamping(SimdScalar lin_damping, SimdScalar ang_damping);
	
	inline const btCollisionShape*	GetCollisionShape() const {
		return m_collisionShape;
	}

	inline btCollisionShape*	GetCollisionShape() {
			return m_collisionShape;
	}
	
	void			setMassProps(SimdScalar mass, const btSimdVector3& inertia);
	
	SimdScalar		getInvMass() const { return m_inverseMass; }
	const btSimdMatrix3x3& getInvInertiaTensorWorld() const { 
		return m_invInertiaTensorWorld; 
	}
		
	void			integrateVelocities(SimdScalar step);

	void			setCenterOfMassTransform(const btSimdTransform& xform);

	void			applyCentralForce(const btSimdVector3& force)
	{
		m_totalForce += force;
	}
    
	const btSimdVector3& getInvInertiaDiagLocal()
	{
		return m_invInertiaLocal;
	};

	void	setInvInertiaDiagLocal(const btSimdVector3& diagInvInertia)
	{
		m_invInertiaLocal = diagInvInertia;
	}

	void	applyTorque(const btSimdVector3& torque)
	{
		m_totalTorque += torque;
	}
	
	void	applyForce(const btSimdVector3& force, const btSimdVector3& rel_pos) 
	{
		applyCentralForce(force);
		applyTorque(rel_pos.cross(force));
	}
	
	void applyCentralImpulse(const btSimdVector3& impulse)
	{
		m_linearVelocity += impulse * m_inverseMass;
	}
	
  	void applyTorqueImpulse(const btSimdVector3& torque)
	{
		if (!IsStatic())
			m_angularVelocity += m_invInertiaTensorWorld * torque;

	}
	
	void applyImpulse(const btSimdVector3& impulse, const btSimdVector3& rel_pos) 
	{
		if (m_inverseMass != 0.f)
		{
			applyCentralImpulse(impulse);
			applyTorqueImpulse(rel_pos.cross(impulse));
		}
	}
	
	void clearForces() 
	{
		m_totalForce.setValue(0.0f, 0.0f, 0.0f);
		m_totalTorque.setValue(0.0f, 0.0f, 0.0f);
	}
	
	void updateInertiaTensor();    
	
	const SimdPoint3&     getCenterOfMassPosition() const { 
		return m_worldTransform.getOrigin(); 
	}
	btSimdQuaternion getOrientation() const;
	
	const btSimdTransform&  getCenterOfMassTransform() const { 
		return m_worldTransform; 
	}
	const btSimdVector3&   getLinearVelocity() const { 
		return m_linearVelocity; 
	}
	const btSimdVector3&    getAngularVelocity() const { 
		return m_angularVelocity; 
	}
	

	void setLinearVelocity(const btSimdVector3& lin_vel);
	void setAngularVelocity(const btSimdVector3& ang_vel) { 
		if (!IsStatic())
		{
			m_angularVelocity = ang_vel; 
		}
	}

	btSimdVector3 getVelocityInLocalPoint(const btSimdVector3& rel_pos) const
	{
		//we also calculate lin/ang velocity for kinematic objects
		return m_linearVelocity + m_angularVelocity.cross(rel_pos);

		//for kinematic objects, we could also use use:
		//		return 	(m_worldTransform(rel_pos) - m_interpolationWorldTransform(rel_pos)) / m_kinematicTimeStep;
	}

	void translate(const btSimdVector3& v) 
	{
		m_worldTransform.getOrigin() += v; 
	}

	
	void	getAabb(btSimdVector3& aabbMin,btSimdVector3& aabbMax) const;




	
	inline float ComputeImpulseDenominator(const SimdPoint3& pos, const btSimdVector3& normal) const
	{
		btSimdVector3 r0 = pos - getCenterOfMassPosition();

		btSimdVector3 c0 = (r0).cross(normal);

		btSimdVector3 vec = (c0 * getInvInertiaTensorWorld()).cross(r0);

		return m_inverseMass + normal.dot(vec);

	}

	inline float ComputeAngularImpulseDenominator(const btSimdVector3& axis) const
	{
		btSimdVector3 vec = axis * getInvInertiaTensorWorld();
		return axis.dot(vec);
	}



private:
	
	btSimdMatrix3x3	m_invInertiaTensorWorld;
	btSimdVector3		m_gravity;	
	btSimdVector3		m_invInertiaLocal;
	btSimdVector3		m_totalForce;
	btSimdVector3		m_totalTorque;
//	btSimdQuaternion	m_orn1;
	
	btSimdVector3		m_linearVelocity;
	
	btSimdVector3		m_angularVelocity;
	
	SimdScalar		m_linearDamping;
	SimdScalar		m_angularDamping;
	SimdScalar		m_inverseMass;


	SimdScalar		m_kinematicTimeStep;

	btBroadphaseProxy*	m_broadphaseProxy;


	


	
public:
	const btBroadphaseProxy*	GetBroadphaseProxy() const
	{
		return m_broadphaseProxy;
	}
	btBroadphaseProxy*	GetBroadphaseProxy() 
	{
		return m_broadphaseProxy;
	}
	void	SetBroadphaseProxy(btBroadphaseProxy* broadphaseProxy)
	{
		m_broadphaseProxy = broadphaseProxy;
	}
	
	//for experimental overriding of friction/contact solver func
	int	m_contactSolverType;
	int	m_frictionSolverType;


	/// for ode solver-binding
	dMatrix3		m_R;//temp
	dMatrix3		m_I;
	dMatrix3		m_invI;

	int				m_odeTag;
	
	btSimdVector3		m_tacc;//temp
	btSimdVector3		m_facc;



	int	m_debugBodyId;
};



#endif
