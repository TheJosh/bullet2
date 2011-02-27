/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btActivatingCollisionAlgorithm.h"
#include "btCollisionDispatcher.h"
#include "btCollisionObject.h"

btActivatingCollisionAlgorithm::btActivatingCollisionAlgorithm (const btCollisionAlgorithmConstructionInfo& ci, bool allocateManifold)
:btCollisionAlgorithm(ci)
, m_ownManifold(false)
, m_manifoldPtr(ci.m_manifold)
{
	const btCollisionObject* collider0 = ci.m_colObj0->getCollisionObject();
	const btCollisionObject* collider1 = ci.m_colObj1->getCollisionObject();
	if( ci.m_isSwapped )
	{
		collider0 = ci.m_colObj1->getCollisionObject();
		collider1 = ci.m_colObj0->getCollisionObject();
	}

	if (!m_manifoldPtr && allocateManifold)
	{
		m_manifoldPtr = ci.m_dispatcher1->getNewManifold(collider0,collider1);
		m_ownManifold = true;
	}
}

btActivatingCollisionAlgorithm::~btActivatingCollisionAlgorithm()
{
	btAssert(m_manifoldPtr == 0x0);
}

void btActivatingCollisionAlgorithm::nihilize( btDispatcher* dispatcher )
{
	if (m_ownManifold && m_manifoldPtr)
	{
		dispatcher->releaseManifold(m_manifoldPtr);
	}
	m_manifoldPtr = 0x0;
}
