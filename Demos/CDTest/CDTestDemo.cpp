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

		

#ifndef __APPLE__
#include <GL/glew.h>
#endif

#include "GL_DialogDynamicsWorld.h"
#include "GL_DialogWindow.h"

#include "GLDebugFont.h"
#include "GlutStuff.h"

///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "BulletMultiThreaded/btGpu3DGridBroadphase.h"
#include "../Extras/OpenCL/3dGridBroadphase/Shared/bt3dGridBroadphaseOCL.h"
#include "../Extras/OpenCL/Hier3dGridBroadphase/Shared/btHier3dGridBroadphaseOCL.h"

#include <stdio.h> //printf debugging

#include "CDTestDemo.h"

btScalar gTimeStep = btScalar(1./60.);


#define DEF_AMPLITUDE btScalar(100.f)
#define DEF_OBJ_SPEED btScalar(0.01f)



void CDTestDemo::clientMoveAndDisplay()
{
	if(m_dialogDynamicsWorld)
	{
		checkSelection();
	}


	updateCamera();
	glDisable(GL_LIGHTING);
	glColor3f(1.f, 1.f, 1.f);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	glDisable(GL_TEXTURE_2D); // we always draw wireframe in this demo

	{
		CProfileManager::EnableProfiler(true);
		CProfileManager::Reset();
		{
			BT_PROFILE("performTest");
			performTest();
		}
		CProfileManager::Increment_Frame_Counter();
		renderme(); 
	}


	CProfileManager::EnableProfiler(false);
	if (m_dialogDynamicsWorld)
		m_dialogDynamicsWorld->draw(gTimeStep);

	glFlush();

	glutSwapBuffers();
}



void CDTestDemo::displayCallback(void) {

	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

	glFlush();
	glutSwapBuffers();
}

void CDTestDemo::checkSelection()
{
	int newSelection = -1;
	for(int i = 0; i < CDTESTDEMO_BROADPHASE_NUM; i++)
	{
		if(m_testSelector[i]->m_active)
		{
			if(i != m_selectedTestIndex)
			{
				newSelection = i;
			}
		}
	}
	if(newSelection >= 0)
	{
		for(int i = 0; i < CDTESTDEMO_BROADPHASE_NUM; i++)
		{
			m_testSelector[i]->m_active = (i == newSelection);
		}
		m_selectedTestIndex = newSelection;
		m_pSelectedTest = m_tests + m_selectedTestIndex;
	}
	if(m_pSelectedTest)
	{
		m_pSelectedTest->m_amplitude = m_amplitudeSlider->btGetFraction() * DEF_AMPLITUDE;
		m_pSelectedTest->m_objectSpeed = m_objSpeedSlider->btGetFraction() * DEF_OBJ_SPEED;
		m_pSelectedTest->m_percentUpdate = (int)(m_pctUpdateSlider->btGetFraction() * btScalar(100.f));
	}
	m_renderEnabled = m_renderEnabledToggle->m_active;
}


void CDTestDemo::initPhysics()
{
	
	setTexturing(false);
	setShadows(false);
	m_renderEnabled = true;
	m_dialogDynamicsWorld = NULL;
	m_pSelectedTest = NULL;
	m_numDetectedPairs = 0;

//	setCameraDistance(80.f);
	setCameraDistance(300.0f);
//	m_cameraTargetPosition.setValue(50, 10, 0);
	m_cameraTargetPosition.setValue(0, 0, 0);
//	m_azi = btScalar(0.f);
//	m_ele = btScalar(0.f);
	m_azi = btScalar(45.f);
	m_ele = btScalar(30.f);
	setFrustumZPlanes(0.1f, 10.f);
	m_dynamicsWorld = NULL;

	btDbvtBroadphase*	pbp  = new btDbvtBroadphase();
	pbp->m_deferedcollide	=	true;	/* Faster initialization, set to false after.	*/ 
	m_tests[CDTESTDEMO_BROADPHASE_DBVT].m_broadphase = pbp;

	btVector3 aabbMin(-200,-200,-200);
	btVector3 aabbMax(200,200,200);
	btBroadphaseInterface* bpif;
	bpif = new btAxisSweep3(aabbMin,aabbMax,8192,0,true);
	m_tests[CDTESTDEMO_BROADPHASE_AXISSWEEP3].m_broadphase = bpif;

	bpif	 = new btGpu3DGridBroadphase(btVector3(10.f, 10.f, 10.f), 32, 32, 32, 8192, 8192, 64, 100.f, 16);
	m_tests[CDTESTDEMO_BROADPHASE_3DGRIDCPU].m_broadphase = bpif;

	bpif	 = new bt3dGridBroadphaseOCL(NULL, btVector3(10.f, 10.f, 10.f), 32, 32, 32,8192,8192, 64, 100.f, 16);
	m_tests[CDTESTDEMO_BROADPHASE_3DGRIDOCL].m_broadphase = bpif;

	bpif	 = new btHier3dGridBroadphaseOCL(NULL, btVector3(8.f, 8.f, 8.f), 32, 32, 32, 8192, 64);
	m_tests[CDTESTDEMO_BROADPHASE_3DHIERGRID].m_broadphase = bpif;

	clientResetScene();
	for(int i = 0; i < CDTESTDEMO_BROADPHASE_NUM; i++)
	{
		m_tests[i].initBoxes(8192);
	}
}

void CDTestDemo::clientResetScene()
{
	DemoApplication::clientResetScene();
}


void CDTestDemo::exitPhysics()
{
	delete m_dialogDynamicsWorld;
	m_dialogDynamicsWorld = 0;
}


void CDTestDemo::keyboardCallback(unsigned char key, int x, int y)
{
	(void)x;
	(void)y;
	switch (key) 
	{
		case 'q' : 
			exitPhysics();
			exit(0);
			break;
		default : 
			{
				DemoApplication::keyboardCallback(key, x, y);
			}
			break;
	}
}



void CDTestDemo::renderme()
{

    glColor3f(1.0, 1.0, 1.0);
//    glutWireCube(2.0);

	if((m_pSelectedTest != NULL) && m_renderEnabled)
	{
		glDepthMask(GL_TRUE);
		glEnable(GL_DEPTH_TEST);

		glEnable(GL_COLOR_MATERIAL);
		glDisable(GL_TEXTURE_2D);
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
		for(int i = 0; i < m_pSelectedTest->m_numBoxes; i++)
		{
			m_pSelectedTest->drawBox(i);
		}
	}

	char buf[128];
	sprintf(buf,"Num pairs : %6d", m_numDetectedPairs);
	GLDebugDrawString(10.f, 20.f, buf);

	if ((m_debugMode & btIDebugDraw::DBG_NoHelpText)==0)
	{
		setOrthographicProjection();
		int  xOffset = 10.f;
		int  yStart = 40.f;
		int  yIncr = 20.f;
		showProfileInfo(xOffset, yStart, yIncr);
		outputDebugInfo(xOffset, yStart, yIncr);
		resetPerspectiveProjection();
	}
}



void CDTestDemo::outputDebugInfo(int & xOffset,int & yStart, int  yIncr)
{
	char buf[124];
	glDisable(GL_LIGHTING);
	glColor3f(0, 0, 0);
	
	sprintf(buf,"mouse move+buttons to interact");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"space to reset");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"cursor keys and z,x to navigate");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"q to quit");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"h to toggle help text");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	sprintf(buf,"p to toggle profiling (+results to file)");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

}


void CDTestDemo::myinit()
{
	DemoApplication::myinit();
	glDisable(GL_LIGHT1);

	float AmbientColor[] = { 0.0f, 0.0f, 0.0f, 0.0f };		
	glLightfv(GL_LIGHT0, GL_AMBIENT, AmbientColor);
	float DiffuseColor[] = { 1.f, 1.f, 1.f, 0.0f };			
	glLightfv(GL_LIGHT0, GL_DIFFUSE, DiffuseColor);
	float SpecularColor[] = { 0.f, 0.f, 0.f, 0.0f };		
	glLightfv(GL_LIGHT0, GL_SPECULAR, SpecularColor);
	float Position[] = { 100.0f, 100.0f, 100.0f, 1.0f };	
	glLightfv(GL_LIGHT0, GL_POSITION, Position);
	glEnable(GL_LIGHT0);
	setFrustumZPlanes(0.1f, 600.f);

#ifndef __APPLE__
    glewInit();
    if (!glewIsSupported("GL_VERSION_2_0 GL_VERSION_1_5 GL_ARB_multitexture GL_ARB_vertex_buffer_object")) {
        fprintf(stderr, "Required OpenGL extensions missing.");
        exit(-1);
    }
#endif //__APPLE__



}






void CDTestDemo::mouseFunc(int button, int state, int x, int y)
{

	if (!m_dialogDynamicsWorld->mouseFunc(button,state,x,y))
	{
		DemoApplication::mouseFunc(button,state,x,y);
	}
}

void	CDTestDemo::mouseMotionFunc(int x,int y)
{
	m_dialogDynamicsWorld->mouseMotionFunc(x,y);
	DemoApplication::mouseMotionFunc(x,y);
}



void CDTestDemo::reshape(int w, int h)
{
static bool bFirstCall = true;
if(bFirstCall)
{
	m_dialogDynamicsWorld = new GL_DialogDynamicsWorld();
}
	if (m_dialogDynamicsWorld)
		m_dialogDynamicsWorld->setScreenSize(w,h);
	
if(bFirstCall)
{
#ifdef CL_PLATFORM_MINI_CL
	GL_DialogWindow* settings = m_dialogDynamicsWorld->createDialog(50,0,280,280,"Broadphase (MiniCL)");
#elif defined(CL_PLATFORM_NVIDIA)
	GL_DialogWindow* settings = m_dialogDynamicsWorld->createDialog(50,0,280,280,"Broadphase (NVIDIA)");
#else
#error "OpenCL platform not supported"
#endif
	m_dialogDynamicsWorld->setScreenSize(w,h);
	m_selectedTestIndex = CDTESTDEMO_BROADPHASE_DBVT;
	m_testSelector[CDTESTDEMO_BROADPHASE_DBVT] = m_dialogDynamicsWorld->createToggle(settings,"DBVT");
	m_testSelector[CDTESTDEMO_BROADPHASE_AXISSWEEP3] = m_dialogDynamicsWorld->createToggle(settings,"AxisSweep3");
	m_testSelector[CDTESTDEMO_BROADPHASE_3DGRIDCPU] = m_dialogDynamicsWorld->createToggle(settings,"3D Grid CPU");
	m_testSelector[CDTESTDEMO_BROADPHASE_3DGRIDOCL] = m_dialogDynamicsWorld->createToggle(settings,"3D Grid OpenCL");
	m_testSelector[CDTESTDEMO_BROADPHASE_3DHIERGRID] = m_dialogDynamicsWorld->createToggle(settings,"Hierarhical 3D Grid");
	m_testSelector[m_selectedTestIndex]->m_active = true;
	m_selectedTestIndex = -1;

	m_amplitudeSlider = m_dialogDynamicsWorld->createSlider(settings, "Amplitude");
	m_objSpeedSlider = m_dialogDynamicsWorld->createSlider(settings, "Speed");
	m_pctUpdateSlider = m_dialogDynamicsWorld->createSlider(settings, "Update pct");

	m_renderEnabledToggle = m_dialogDynamicsWorld->createToggle(settings,"Rendering");
	m_renderEnabledToggle->m_active = true;
	checkSelection();
	bFirstCall = false;
}

	GlutDemoApplication::reshape(w,h);
}

inline float frand()	{ return float(rand()) * (1.f / (float)RAND_MAX);	}

void CDTest::initBoxes(int numBoxes)
{
	btClock		clock;
	srand(0);
	m_numBoxes = numBoxes;
	m_proxies.resize(m_numBoxes);
	m_boxTime.resize(m_numBoxes);
	m_boxFlags.resize(m_numBoxes);
	m_boxCenter.resize(m_numBoxes);
	m_boxExtents.resize(m_numBoxes);

//	m_amplitude = 100.f;
//	m_objectSpeed = 0.01f;
//	m_percentUpdate = 100;

	for(int i = 0; i < m_numBoxes; i++)
	{
		btVector3 center, extents;

		center[0] = (frand()-0.5f) * 100.0f;
		center[1] = (frand()-0.5f) * 10.0f;
		center[2] = (frand()-0.5f) * 100.0f;
		extents[0] = 2.0f + frand() * 2.0f;
		extents[1] = 2.0f + frand() * 2.0f;
		extents[2] = 2.0f + frand() * 2.0f;

		m_boxCenter[i] = center;
		m_boxExtents[i] = extents;

		btVector3	aabbMin(center[0]-extents[0],center[1]-extents[1],center[2]-extents[2]);
		btVector3	aabbMax(center[0]+extents[0],center[1]+extents[1],center[2]+extents[2]);
		int shapeType =0;
		btBroadphaseProxy* proxy = m_broadphase->createProxy(aabbMin,aabbMax,shapeType,&m_boxCenter[i],1,1,0,0);//m_dispatcher);
		m_proxies[i] = proxy;
		m_boxTime[i] = 2000.0f*frand();
	}
//	printf("Initialization of %s with %u boxes: %ums\r\n",methodname,mNbBoxes,clock.getTimeMilliseconds());
}

void CDTest::updateBoxes(int numBoxesToUpdate)
{
	for(int i = 0; i < numBoxesToUpdate; i++)
	{
		m_boxTime[i] += m_objectSpeed;
		btVector3 center;
		center[0] = cosf(m_boxTime[i]*2.17f)*m_amplitude + sinf(m_boxTime[i])*m_amplitude*0.5f;
		center[1] = cosf(m_boxTime[i]*1.38f)*m_amplitude + sinf(m_boxTime[i]*m_amplitude);
		center[2] = sinf(m_boxTime[i]*0.777f)*m_amplitude;
		m_boxCenter[i] = center;
	}
	return;
}


void CDTestDemo::performTest()
{
	if(!m_pSelectedTest) return;
	int numUpdatedBoxes = (m_pSelectedTest->m_numBoxes * m_pSelectedTest->m_percentUpdate)/100;
	{
		BT_PROFILE("updateBoxes");
		m_pSelectedTest->updateBoxes(numUpdatedBoxes);
		for (int i=0;i<numUpdatedBoxes;i++)
		{
			btVector3& center = m_pSelectedTest->m_boxCenter[i];
			btVector3& extents = m_pSelectedTest->m_boxExtents[i];
			btVector3	aabbMin(center[0]-extents[0],center[1]-extents[1],center[2]-extents[2]);
			btVector3	aabbMax(center[0]+extents[0],center[1]+extents[1],center[2]+extents[2]);
			m_pSelectedTest->m_broadphase->setAabb(m_pSelectedTest->m_proxies[i],aabbMin,aabbMax,0);
		}
	}
	{
		BT_PROFILE("calculateOverlappingPairs");
		m_pSelectedTest->m_broadphase->calculateOverlappingPairs(0);
	}

	
	for(int i = 0; i < m_pSelectedTest->m_numBoxes; i++)
	{
		m_pSelectedTest->m_boxFlags[i] = false;
	}

	btOverlappingPairCache* pairCache = m_pSelectedTest->m_broadphase->getOverlappingPairCache();
	const btBroadphasePair* pairPtr = pairCache->getOverlappingPairArrayPtr();

	m_numDetectedPairs = pairCache->getNumOverlappingPairs();
	for(int i = 0; i < m_numDetectedPairs; i++)
	{
		int	j;
		j = int((btVector3*)(pairPtr[i].m_pProxy0->m_clientObject) - &(m_pSelectedTest->m_boxCenter[0]));
		m_pSelectedTest->m_boxFlags[j] = true;
		j = int((btVector3*)(pairPtr[i].m_pProxy1->m_clientObject) - &(m_pSelectedTest->m_boxCenter[0]));
		m_pSelectedTest->m_boxFlags[j] = true;
	}
/*	
	char Buffer[4096];
	sprintf_s(Buffer, sizeof(Buffer), "Bullet %s: %5.1f us (%d cycles) : %d pairs\n", methodname, mProfiler.mMsTime, mProfiler.mCycles, 
			m_broadphase->getOverlappingPairCache()->getNumOverlappingPairs());

//	m_broadphase)->printStats();

	GLFontRenderer::print(10.0f, 10.0f, 0.02f, Buffer);
*/
}

void CDTest::drawBox(int index)
{
	static int indices[36] = {
		0, 2, 1,
		3, 1, 2,
		4, 6, 0,
		2, 0, 6,
		5, 4, 1,
		0, 1, 4,
		7, 5, 3,
		1, 3, 5,
		5, 7, 4,
		6, 4, 7,
		7, 3, 6,
		2, 6, 3
	};
	const btVector3& wmin = m_proxies[index]->m_aabbMin;
	const btVector3& wmax = m_proxies[index]->m_aabbMax;
	if(m_boxFlags[index])
	{
		glColor3f(1.f, 0.2f, 0.2f);
	}
	else
	{
		glColor3f(0.2f, 1.f, 0.2f);
	}
	 btVector3 vertices[8] = {	
		btVector3(wmax[0],wmax[1],wmax[2]),
		btVector3(wmin[0],wmax[1],wmax[2]),
		btVector3(wmax[0],wmin[1],wmax[2]),	
		btVector3(wmin[0],wmin[1],wmax[2]),	
		btVector3(wmax[0],wmax[1],wmin[2]),
		btVector3(wmin[0],wmax[1],wmin[2]),	
		btVector3(wmax[0],wmin[1],wmin[2]),	
		btVector3(wmin[0],wmin[1],wmin[2])
	 };
	glBegin (GL_TRIANGLES);
	int si=36;
	for (int i=0;i<si;i+=3)
	{
		const btVector3& v1 = vertices[indices[i]];;
		const btVector3& v2 = vertices[indices[i+1]];
		const btVector3& v3 = vertices[indices[i+2]];
		btVector3 normal = (v3-v1).cross(v2-v1);
		normal.normalize ();
		glNormal3f(normal.getX(),normal.getY(),normal.getZ());
		glVertex3f (v1.x(), v1.y(), v1.z());
		glVertex3f (v2.x(), v2.y(), v2.z());
		glVertex3f (v3.x(), v3.y(), v3.z());
	}
	glEnd();
}

