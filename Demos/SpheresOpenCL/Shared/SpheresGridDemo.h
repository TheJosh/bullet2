/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006 - 2009 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef SPHERES_GRID_DEMO_H
#define SPHERES_GRID_DEMO_H

#define USE_BULLET_BODIES 0

#include "DemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

	#include "btIntegrationDemoDynamicsWorld.h"
	#include "btSpheresGridDemoDynamicsWorld.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;
#include "GlutDemoApplication.h"


enum 
{
	DEMO_INTEGRATION = 0,
	DEMO_OE_CAKE_2D,
	DEMO_OE_CAKE_3D,
	DEMO_TOTAL_NUM
};
///BasicDemo is good starting point for learning the code base and porting.
class SpheresGridDemo : public GlutDemoApplication
{

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	btOverlappingPairCache* m_pairCache;

	int m_mouseButtons;
	int m_mouseOldX;
	int m_mouseOldY;

	int m_argc;
	char** m_argv;

	public:

		int m_demoType;

	btIntegrationDemoDynamicsWorld* m_pWorldI;
	btSpheresGridDemoDynamicsWorld* m_pWorldS;
	// shader
	GLuint				m_shaderProgram;

	SpheresGridDemo(int argc, char** argv)
	{
		m_argc = argc;
		m_argv = argv;
		m_demoType = DEMO_INTEGRATION;
	}
	virtual ~SpheresGridDemo()
	{
		exitPhysics();
	}
	void	initPhysics();

	void	setRandomZCoordinate(btScalar range);

	void	exitPhysics();

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void keyboardCallback(unsigned char key, int x, int y);

	virtual void clientResetScene();

/*
	static DemoApplication* Create()
	{
		SpheresGridDemo* demo = new SpheresGridDemo;
		demo->myinit();
		demo->initPhysics();
		demo->m_mouseButtons = 0;
		demo->m_mouseOldX = 0;
		demo->m_mouseOldY = 0;
		return demo;
	}
*/

	void outputDebugInfo(int & xOffset,int & yStart, int  yIncr);
	virtual void renderme();
	virtual void myinit();
	void init_scene_directly();
	void print_used_device();


};


#endif // SPHERES_GRID_DEMO_H

