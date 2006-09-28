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

#include "CcdPhysicsEnvironment.h"
#include "CcdPhysicsController.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"
#include "GLDebugDrawer.h"



//COLLADA_DOM and LibXML source code are included in Extras/ folder.
//COLLADA_DOM should compile under all platforms, and is enabled by default.

#include "ColladaConverter.h"
#include "PHY_Pro.h"
#include "BMF_Api.h"
#include <stdio.h> //printf debugging

float deltaTime = 1.f/60.f;

#include "ColladaDemo.h"
#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"

///custom version of the converter, that creates physics objects/constraints
class MyColladaConverter : public ColladaConverter
{
	DemoApplication*	m_demoApp;
	
	public:
		MyColladaConverter(DemoApplication* demoApp)
		:m_demoApp(demoApp)
		{
		}
		
		///those 2 virtuals are called for each constraint/physics object
	virtual int			createUniversalD6Constraint(
		class PHY_IPhysicsController* ctrlRef,class PHY_IPhysicsController* ctrlOther,
			btTransform& localAttachmentFrameRef,
			btTransform& localAttachmentOther,
			const btVector3& linearMinLimits,
			const btVector3& linearMaxLimits,
			const btVector3& angularMinLimits,
			const btVector3& angularMaxLimits
			)
		{
			return m_demoApp->getPhysicsEnvironment()->createUniversalD6Constraint(
					ctrlRef,ctrlOther,
					localAttachmentFrameRef,
					localAttachmentOther,
					linearMinLimits,
					linearMaxLimits,
					angularMinLimits,
					angularMaxLimits
				);
		}

	virtual CcdPhysicsController*  createPhysicsObject(bool isDynamic, 
		float mass, 
		const btTransform& startTransform,
		btCollisionShape* shape)
	{
		CcdPhysicsController*  ctrl = m_demoApp->localCreatePhysicsObject(isDynamic, mass, startTransform,shape);
		return ctrl;
	}


	virtual	void	setGravity(const btVector3& grav)
	{
		m_demoApp->getPhysicsEnvironment()->setGravity(grav.getX(),grav.getY(),grav.getZ());
	}
	virtual void	setCameraInfo(const btVector3& camUp,int forwardAxis) 
	{
		m_demoApp->setCameraUp(camUp);
		m_demoApp->setCameraForwardAxis(forwardAxis);
	}

};

MyColladaConverter* gColladaConverter = 0;





////////////////////////////////////



GLDebugDrawer debugDrawer;




int main(int argc,char** argv)
{

	/// Import Collada 1.4 Physics objects
	/// also can pass filename in as argument
	char* filename = "jenga.dae";
	printf("argc=%i\n",argc);
	{
		for (int i=0;i<argc;i++)
		{
			printf("argv[%i]=%s\n",i,argv[i]);
		}
	}
	if (argc>1)
	{
		filename = argv[1];
	}

	ColladaDemo* colladaDemo = new ColladaDemo();

	colladaDemo->initPhysics(filename);

	
	colladaDemo->clientResetScene();

	colladaDemo->setCameraDistance(26.f);

	return glutmain(argc, argv,640,480,"Bullet COLLADA Physics Viewer http://bullet.sourceforge.net",colladaDemo);
}

void	ColladaDemo::initPhysics(const char* filename)
{
	m_cameraUp = btVector3(0,0,1);
	m_forwardAxis = 1;

	///Setup a Physics Simulation Environment
	btCollisionDispatcher* dispatcher = new	btCollisionDispatcher();
	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	btOverlappingPairCache* broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax);
	//BroadphaseInterface* broadphase = new btSimpleBroadphase();
	m_physicsEnvironmentPtr = new CcdPhysicsEnvironment(dispatcher,broadphase);
	m_physicsEnvironmentPtr->setDeactivationTime(2.f);
	m_physicsEnvironmentPtr->setGravity(0,0,-10);
	m_physicsEnvironmentPtr->setDebugDrawer(&debugDrawer);

	MyColladaConverter* converter = new MyColladaConverter(this);

	bool result = converter->load(filename);

	if (result)
	{
		result = converter->convert();
	}
	if (result)
	{
		gColladaConverter = converter;
	} else
	{
		gColladaConverter = 0;
	}
}


void ColladaDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	m_physicsEnvironmentPtr->proceedDeltaTime(0.f,deltaTime);

	renderme();

	glFlush();
	glutSwapBuffers();

}



void ColladaDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 


	m_physicsEnvironmentPtr->UpdateAabbs(deltaTime);

	renderme();


	glFlush();
	glutSwapBuffers();
}






void ColladaDemo::keyboardCallback(unsigned char key, int x, int y)
{
	if (key =='e')
	{
		//save a COLLADA .dae physics snapshot
		if (gColladaConverter)
			gColladaConverter->saveAs();
	} else
	{
	DemoApplication::keyboardCallback(key,x,y);
	}
}

