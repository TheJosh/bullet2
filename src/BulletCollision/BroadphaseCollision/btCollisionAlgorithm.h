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

#ifndef COLLISION_ALGORITHM_H
#define COLLISION_ALGORITHM_H

#include "LinearMath/btScalar.h"
#include "LinearMath/btAlignedObjectArray.h"

struct btBroadphaseProxy;
class btDispatcher;
class btManifoldResult;
class btCollisionObject;
class btCollisionShape;
struct btDispatcherInfo;
class btPersistentManifold;
class btTransform;
struct btCollider;
class btVoronoiSimplexSolver;
class btConvexPenetrationDepthSolver;

typedef btAlignedObjectArray<btPersistentManifold*>	btManifoldArray;

struct btCollisionAlgorithmConstructionInfo
{
private:
	btCollisionAlgorithmConstructionInfo()
		: m_dispatcher1(0)
		, m_isSwapped(false)
		, m_manifold(0)
		, m_colObj0(0x0)
		, m_colObj1(0x0)
	{
	}
public:
	btCollisionAlgorithmConstructionInfo(
		btDispatcher* dispatcher, 
		bool isSwapped, 
		btPersistentManifold* manifold, 
		const btCollider* colObj0, 
		const btCollider* colObj1,
		btVoronoiSimplexSolver* simplexSolver,
		btConvexPenetrationDepthSolver* pdSolver)
		: m_dispatcher1(dispatcher)
		, m_isSwapped(isSwapped)
		, m_manifold(manifold)
		, m_colObj0(colObj0)
		, m_colObj1(colObj1)
		, m_simplexSolver(simplexSolver)
		, m_pdSolver(pdSolver)
	{
	}

	btDispatcher*			m_dispatcher1;
	bool					m_isSwapped;
	btPersistentManifold*	m_manifold;
	const btCollider*		m_colObj0;
	const btCollider*		m_colObj1;
	mutable btVoronoiSimplexSolver*	m_simplexSolver;
	mutable btConvexPenetrationDepthSolver* m_pdSolver;
};

#define BT_DECLARE_STACK_ONLY_OBJECT \
	private: \
		void* operator new(size_t size); \
		void operator delete(void*);

struct btCollider;
struct btCollider
{
BT_DECLARE_STACK_ONLY_OBJECT

private:
	btCollider(const btCollider&); // not implemented. Not allowed.
	btCollider* operator=(const btCollider&);

public:
	const btCollider* m_parent;
	const btCollisionShape* m_shape;
	const btCollisionObject* m_collisionObject;
	const btTransform& m_worldTransform;

	btCollider(const btCollider* parent, const btCollisionShape* shape, const btCollisionObject* collisionObject, const btTransform& worldTransform)
	: m_parent(parent), m_shape(shape), m_collisionObject(collisionObject), m_worldTransform(worldTransform)
	{}

	SIMD_FORCE_INLINE const btTransform& getWorldTransform() const { return m_worldTransform; }
	SIMD_FORCE_INLINE const btCollisionObject* getCollisionObject() const { return m_collisionObject; }
	SIMD_FORCE_INLINE const btCollisionShape* getCollisionShape() const { return m_shape; }
};

struct btCollisionProcessInfo
{
BT_DECLARE_STACK_ONLY_OBJECT

private:
	btCollisionProcessInfo(const btCollisionProcessInfo&); // not implemented. Not allowed.
	btCollisionProcessInfo* operator=(const btCollisionProcessInfo&);

public:
	const btCollider& m_body0;
	const btCollider& m_body1;
	const btDispatcherInfo& m_dispatchInfo;
	mutable btManifoldResult* m_result;
	mutable btDispatcher* m_dispatcher;

	btCollisionProcessInfo(const btCollider& body0, const btCollider& body1, const btDispatcherInfo& dispatchInfo, btManifoldResult* result, btDispatcher* dispatcher)
	: m_body0(body0), m_body1(body1), m_dispatchInfo(dispatchInfo), m_result(result), m_dispatcher(dispatcher)
	{}
};

///btCollisionAlgorithm is an collision interface that is compatible with the Broadphase and btDispatcher.
///It is persistent over frames
class btCollisionAlgorithm
{
public:

	btCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci);

	virtual ~btCollisionAlgorithm() { }

	virtual void processCollision (const btCollisionProcessInfo& processInfo) = 0;

	virtual btScalar calculateTimeOfImpact(btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut) = 0;

	virtual	void	getAllContactManifolds(btManifoldArray&	manifoldArray) = 0;

	virtual void nihilize(btDispatcher* dispatcher) =0;
};


#endif //COLLISION_ALGORITHM_H
