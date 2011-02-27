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

#include "btSoftRigidCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "btSoftBody.h"
///TODO: include all the shapes that the softbody can collide with
///alternatively, implement special case collision algorithms (just like for rigid collision shapes)

//#include <stdio.h>

btSoftRigidCollisionAlgorithm::btSoftRigidCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci, bool isSwapped)
: btCollisionAlgorithm(ci),
m_isSwapped(isSwapped)
{
}


void btSoftRigidCollisionAlgorithm::nihilize(btDispatcher* dispatcher)
{
}


#include <stdio.h>

void btSoftRigidCollisionAlgorithm::processCollision (const btCollisionProcessInfo& processInfo)
{
	btSoftBody* softBody =  m_isSwapped? (btSoftBody*)processInfo.m_body1.getCollisionObject(): (btSoftBody*)processInfo.m_body0.getCollisionObject();
	const btCollider* rigidCollisionObject = &(m_isSwapped? processInfo.m_body0 : processInfo.m_body1);
	
	if (softBody->m_collisionDisabledObjects.findLinearSearch(rigidCollisionObject->getCollisionObject())==softBody->m_collisionDisabledObjects.size())
	{
		softBody->defaultCollisionHandler(rigidCollisionObject);
	}


}

btScalar btSoftRigidCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* col0,btCollisionObject* col1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	(void)col0;
	(void)col1;

	//not yet
	return btScalar(1.);
}



