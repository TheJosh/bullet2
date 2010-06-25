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
#ifndef ACCELERATED_CLOTH_DEMO_H
#define ACCELERATED_CLOTH_DEMO_H

#ifdef _WINDOWS
#include "Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
#include "GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
#endif

#include "LinearMath/btAlignedObjectArray.h"

#include "BulletAcceleratedSoftBody/btDirectComputeSupport.hpp"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;
namespace BTAcceleratedSoftBody
{
	class Cloth;
	class BulletPhysicsDevice;
	class CPUDevice;
	class DX11Device;	
}
namespace Vectormath
{
	namespace Aos
	{
		class Transform3;
	}
}

///BasicDemo is good starting point for learning the code base and porting.

class AcceleratedClothDemo : public PlatformDemoApplication
{

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	bool m_displayBendLinks;

	BTAcceleratedSoftBody::CPUDevice *m_cpuDevice;
	BTAcceleratedSoftBody::DX11Device *m_dx11Device;
	

	BTAcceleratedSoftBody::DX11SupportHelper m_dxSupport;

	
	btAlignedObjectArray<BTAcceleratedSoftBody::Cloth *> m_flags;
	float m_windAngle;
	float m_windStrength;

	public:

	AcceleratedClothDemo()
	{
		m_displayBendLinks = false;
		m_cpuDevice = 0;
		m_dx11Device = 0;

		m_windAngle = 0.4;
		m_windStrength = 20;
	}

	virtual ~AcceleratedClothDemo()
	{
		exitPhysics();
	}

	virtual void keyboardCallback(unsigned char key, int x, int y);

	btAlignedObjectArray<BTAcceleratedSoftBody::Cloth *> createFlag( const BTAcceleratedSoftBody::BulletPhysicsDevice &device, int width, int height );

	void	initPhysics();

	void	exitPhysics();

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();
	
	static DemoApplication* Create()
	{
		AcceleratedClothDemo* demo = new AcceleratedClothDemo;
		demo->myinit();
		demo->initPhysics();
		return demo;
	}

	virtual void renderme();
};

#endif //BASIC_DEMO_H

