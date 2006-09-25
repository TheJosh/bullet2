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

#ifndef COLLISION__DISPATCHER_H
#define COLLISION__DISPATCHER_H

#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"

#include "BulletCollision/CollisionDispatch/btManifoldResult.h"

#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"


class IDebugDraw;
class OverlappingPairCache;


#include "btCollisionCreateFunc.h"




///CollisionDispatcher supports algorithms that handle ConvexConvex and ConvexConcave collision pairs.
///Time of Impact, Closest Points and Penetration Depth.
class CollisionDispatcher : public Dispatcher
{
	
	std::vector<PersistentManifold*>	m_manifoldsPtr;

	bool m_useIslands;
	
	ManifoldResult	m_defaultManifoldResult;
	
	CollisionAlgorithmCreateFunc* m_doubleDispatch[MAX_BROADPHASE_COLLISION_TYPES][MAX_BROADPHASE_COLLISION_TYPES];
	
	CollisionAlgorithmCreateFunc* InternalFindCreateFunc(int proxyType0,int proxyType1);

	//default CreationFunctions, filling the m_doubleDispatch table
	CollisionAlgorithmCreateFunc*	m_convexConvexCreateFunc;
	CollisionAlgorithmCreateFunc*	m_convexConcaveCreateFunc;
	CollisionAlgorithmCreateFunc*	m_swappedConvexConcaveCreateFunc;
	CollisionAlgorithmCreateFunc*	m_compoundCreateFunc;
	CollisionAlgorithmCreateFunc*	m_swappedCompoundCreateFunc;
	CollisionAlgorithmCreateFunc*   m_emptyCreateFunc;

public:

	///RegisterCollisionCreateFunc allows registration of custom/alternative collision create functions
	void	RegisterCollisionCreateFunc(int proxyType0,int proxyType1, CollisionAlgorithmCreateFunc* createFunc);

	int	GetNumManifolds() const
	{ 
		return m_manifoldsPtr.size();
	}

	 PersistentManifold* GetManifoldByIndexInternal(int index)
	{
		return m_manifoldsPtr[index];
	}

	 const PersistentManifold* GetManifoldByIndexInternal(int index) const
	{
		return m_manifoldsPtr[index];
	}

	int m_count;
	
	CollisionDispatcher ();
	virtual ~CollisionDispatcher();

	virtual PersistentManifold*	GetNewManifold(void* b0,void* b1);
	
	virtual void ReleaseManifold(PersistentManifold* manifold);

	
	///allows the user to get contact point callbacks 
	virtual	ManifoldResult*	GetNewManifoldResult(CollisionObject* obj0,CollisionObject* obj1,PersistentManifold* manifold);

	///allows the user to get contact point callbacks 
	virtual	void	ReleaseManifoldResult(ManifoldResult*);

	virtual void ClearManifold(PersistentManifold* manifold);

			
	CollisionAlgorithm* FindAlgorithm(BroadphaseProxy& proxy0,BroadphaseProxy& proxy1);
	
	CollisionAlgorithm* InternalFindAlgorithm(BroadphaseProxy& proxy0,BroadphaseProxy& proxy1);
	
	virtual bool	NeedsCollision(BroadphaseProxy& proxy0,BroadphaseProxy& proxy1);
	
	virtual bool	NeedsResponse(const CollisionObject& colObj0,const CollisionObject& colObj1);

	virtual int GetUniqueId() { return RIGIDBODY_DISPATCHER;}
	
	virtual void	DispatchAllCollisionPairs(OverlappingPairCache* pairCache,DispatcherInfo& dispatchInfo);

	

};

#endif //COLLISION__DISPATCHER_H

