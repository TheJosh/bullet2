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

#ifndef CD_TEST_DEMO_H
#define CD_TEST_DEMO_H

#include "DemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../OpenGL/GlutDemoApplication.h"


enum
{
CDTESTDEMO_BROADPHASE_DBVT = 0,
CDTESTDEMO_BROADPHASE_AXISSWEEP3,
CDTESTDEMO_BROADPHASE_3DGRIDCPU,
CDTESTDEMO_BROADPHASE_3DGRIDOCL,
CDTESTDEMO_BROADPHASE_3DHIERGRID,
CDTESTDEMO_BROADPHASE_NUM
};

class CDTest
{
public:
	int m_numBoxes;
	btAlignedObjectArray<struct  btBroadphaseProxy*> m_proxies;
	btAlignedObjectArray<float> m_boxTime;
	btAlignedObjectArray<bool> m_boxFlags;
	btAlignedObjectArray<btVector3> m_boxCenter;
	btAlignedObjectArray<btVector3> m_boxExtents;
	btScalar	m_amplitude;
	btScalar	m_objectSpeed;
	int		m_percentUpdate;
	class btBroadphaseInterface*	m_broadphase;

	void initBoxes(int numBoxes);
	void updateBoxes(int numBoxesToUpdate);
	void drawBox(int index);
};

class CDTestDemo : public GlutDemoApplication
{
	class GL_DialogDynamicsWorld* m_dialogDynamicsWorld;

	int m_mouseButtons;
	int m_mouseOldX;
	int m_mouseOldY;

	bool m_renderEnabled;
	int m_argc;
	char** m_argv;

	int m_numDetectedPairs;

	CDTest m_tests[CDTESTDEMO_BROADPHASE_NUM];
	CDTest*	m_pSelectedTest;
	int m_selectedTestIndex;
	struct GL_ToggleControl* m_testSelector[CDTESTDEMO_BROADPHASE_NUM];
	struct GL_SliderControl* m_amplitudeSlider;
	struct GL_SliderControl* m_objSpeedSlider;
	struct GL_SliderControl* m_pctUpdateSlider;
	struct GL_ToggleControl* m_renderEnabledToggle;


	public:

	CDTestDemo(int argc, char** argv)
	{
		m_argc = argc;
		m_argv = argv;
	}
	virtual ~CDTestDemo()
	{
	}

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void keyboardCallback(unsigned char key, int x, int y);

	virtual void clientResetScene();

	virtual void mouseFunc(int button, int state, int x, int y);
	virtual void mouseMotionFunc(int x,int y);
	virtual	void reshape(int w, int h);

	void outputDebugInfo(int & xOffset,int & yStart, int  yIncr);
	

	virtual void renderme();
	virtual void myinit();
	virtual void initPhysics();
	void exitPhysics();

	void performTest();
	void checkSelection();
};


#endif // CD_TEST_DEMO_H

