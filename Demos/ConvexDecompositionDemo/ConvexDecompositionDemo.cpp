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

#include "cd_wavefront.h"
#include "ConvexBuilder.h"


#include "CcdPhysicsEnvironment.h"
#include "CcdPhysicsController.h"

//#include "GL_LineSegmentShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btTriangleMesh.h"


#include "BulletCollision/CollisionShapes/btTetrahedronShape.h"
#include "BulletCollision/CollisionShapes/btEmptyShape.h"

#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/BroadphaseCollision/btAxisSweep3.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"

#include "LinearMath/GenQuickprof.h"
#include "LinearMath/GenIDebugDraw.h"

#include "GLDebugDrawer.h"


#include "PHY_Pro.h"
#include "BMF_Api.h"
#include <stdio.h> //printf debugging

float deltaTime = 1.f/60.f;

#include "ConvexDecompositionDemo.h"
#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"

const int maxNumObjects = 450;

int	shapeIndex[maxNumObjects];

SimdVector3	centroid;

#define CUBE_HALF_EXTENTS 4

CollisionShape* shapePtr[maxNumObjects];



////////////////////////////////////

unsigned int tcount = 0;


GLDebugDrawer debugDrawer;

int main(int argc,char** argv)
{
	char* filename = "file.obj";


	ConvexDecompositionDemo* convexDecompDemo = new ConvexDecompositionDemo();

	convexDecompDemo->initPhysics(filename);



	convexDecompDemo->clientResetScene();

	convexDecompDemo->setCameraDistance(26.f);

	return glutmain(argc, argv,640,480,"Bullet Physics Demo. http://www.continuousphysics.com/Bullet/phpBB2/",convexDecompDemo);
}

void ConvexDecompositionDemo::initPhysics(const char* filename)
{
	ConvexDecomposition::WavefrontObj wo;

	tcount = wo.loadObj(filename);

	CollisionDispatcher* dispatcher = new	CollisionDispatcher();


	SimdVector3 worldAabbMin(-10000,-10000,-10000);
	SimdVector3 worldAabbMax(10000,10000,10000);

	OverlappingPairCache* broadphase = new AxisSweep3(worldAabbMin,worldAabbMax);
	//OverlappingPairCache* broadphase = new SimpleBroadphase();

	m_physicsEnvironmentPtr = new CcdPhysicsEnvironment(dispatcher,broadphase);
	m_physicsEnvironmentPtr->setDeactivationTime(2.f);

	m_physicsEnvironmentPtr->setGravity(0,-10,0);

	SimdTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(SimdVector3(0,-4,0));

	LocalCreatePhysicsObject(false,0,startTransform,new BoxShape(SimdVector3(30,2,30)));

	class MyConvexDecomposition : public ConvexDecomposition::ConvexDecompInterface
	{

		ConvexDecompositionDemo*	m_convexDemo;
		public:

		MyConvexDecomposition (FILE* outputFile,ConvexDecompositionDemo* demo)
			:m_convexDemo(demo),
				mBaseCount(0),
			mHullCount(0),
			mOutputFile(outputFile)

		{
		}
		
			virtual void ConvexDecompResult(ConvexDecomposition::ConvexResult &result)
			{

				TriangleMesh* trimesh = new TriangleMesh();

				SimdVector3 localScaling(6.f,6.f,6.f);

				//export data to .obj
				printf("ConvexResult\n");
				if (mOutputFile)
				{
					fprintf(mOutputFile,"## Hull Piece %d with %d vertices and %d triangles.\r\n", mHullCount, result.mHullVcount, result.mHullTcount );

					fprintf(mOutputFile,"usemtl Material%i\r\n",mBaseCount);
					fprintf(mOutputFile,"o Object%i\r\n",mBaseCount);

					for (unsigned int i=0; i<result.mHullVcount; i++)
					{
						const float *p = &result.mHullVertices[i*3];
						fprintf(mOutputFile,"v %0.9f %0.9f %0.9f\r\n", p[0], p[1], p[2] );
					}

					//calc centroid, to shift vertices around center of mass
					centroid.setValue(0,0,0);
					if ( 1 )
					{
						const unsigned int *src = result.mHullIndices;
						for (unsigned int i=0; i<result.mHullTcount; i++)
						{
							unsigned int index0 = *src++;
							unsigned int index1 = *src++;
							unsigned int index2 = *src++;
							SimdVector3 vertex0(result.mHullVertices[index0*3], result.mHullVertices[index0*3+1],result.mHullVertices[index0*3+2]);
							SimdVector3 vertex1(result.mHullVertices[index1*3], result.mHullVertices[index1*3+1],result.mHullVertices[index1*3+2]);
							SimdVector3 vertex2(result.mHullVertices[index2*3], result.mHullVertices[index2*3+1],result.mHullVertices[index2*3+2]);
							vertex0 *= localScaling;
							vertex1 *= localScaling;
							vertex2 *= localScaling;
							centroid += vertex0;
							centroid += vertex1;
							centroid += vertex2;
							
						}
					}

					centroid *= 1.f/(float(result.mHullTcount) * 3);

					if ( 1 )
					{
						const unsigned int *src = result.mHullIndices;
						for (unsigned int i=0; i<result.mHullTcount; i++)
						{
							unsigned int index0 = *src++;
							unsigned int index1 = *src++;
							unsigned int index2 = *src++;


							SimdVector3 vertex0(result.mHullVertices[index0*3], result.mHullVertices[index0*3+1],result.mHullVertices[index0*3+2]);
							SimdVector3 vertex1(result.mHullVertices[index1*3], result.mHullVertices[index1*3+1],result.mHullVertices[index1*3+2]);
							SimdVector3 vertex2(result.mHullVertices[index2*3], result.mHullVertices[index2*3+1],result.mHullVertices[index2*3+2]);
							vertex0 *= localScaling;
							vertex1 *= localScaling;
							vertex2 *= localScaling;
							
							vertex0 -= centroid;
							vertex1 -= centroid;
							vertex2 -= centroid;

							trimesh->AddTriangle(vertex0,vertex1,vertex2);

							index0+=mBaseCount;
							index1+=mBaseCount;
							index2+=mBaseCount;
							
							fprintf(mOutputFile,"f %d %d %d\r\n", index0+1, index1+1, index2+1 );
						}
					}

					bool isDynamic = true;
					float mass = 1.f;
					CollisionShape* convexShape = new ConvexTriangleMeshShape(trimesh);
					SimdTransform trans;
					trans.setIdentity();
					trans.setOrigin(centroid);
					m_convexDemo->LocalCreatePhysicsObject(isDynamic, mass, trans,convexShape);

					mBaseCount+=result.mHullVcount; // advance the 'base index' counter.


				}
			}

			int   	mBaseCount;
  			int		mHullCount;
			FILE*	mOutputFile;

	};

	if (tcount)
	{
		TriangleMesh* trimesh = new TriangleMesh();

		SimdVector3 localScaling(6.f,6.f,6.f);
		
		for (int i=0;i<wo.mTriCount;i++)
		{
			int index0 = wo.mIndices[i*3];
			int index1 = wo.mIndices[i*3+1];
			int index2 = wo.mIndices[i*3+2];

			SimdVector3 vertex0(wo.mVertices[index0*3], wo.mVertices[index0*3+1],wo.mVertices[index0*3+2]);
			SimdVector3 vertex1(wo.mVertices[index1*3], wo.mVertices[index1*3+1],wo.mVertices[index1*3+2]);
			SimdVector3 vertex2(wo.mVertices[index2*3], wo.mVertices[index2*3+1],wo.mVertices[index2*3+2]);
			
			vertex0 *= localScaling;
			vertex1 *= localScaling;
			vertex2 *= localScaling;

			trimesh->AddTriangle(vertex0,vertex1,vertex2);
		}

		CollisionShape* convexShape = new ConvexTriangleMeshShape(trimesh);
		bool isDynamic = true;
		float mass = 1.f;
		
		SimdTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(SimdVector3(20,2,0));

		LocalCreatePhysicsObject(isDynamic, mass, startTransform,convexShape);

	}
			

	if (tcount)
	{

		char outputFileName[512];
  		strcpy(outputFileName,filename);
  		char *dot = strstr(outputFileName,".");
  		if ( dot ) 
			*dot = 0;
		strcat(outputFileName,"_convex.obj");
  		FILE* outputFile = fopen(outputFileName,"wb");
				
		unsigned int depth = 7;
		float cpercent     = 5;
		float ppercent     = 15;
		unsigned int maxv  = 16;
		float skinWidth    = 0.01;

		printf("WavefrontObj num triangles read %i",tcount);
		ConvexDecomposition::DecompDesc desc;
		desc.mVcount       =	wo.mVertexCount;
		desc.mVertices     = wo.mVertices;
		desc.mTcount       = wo.mTriCount;
		desc.mIndices      = (unsigned int *)wo.mIndices;
		desc.mDepth        = depth;
		desc.mCpercent     = cpercent;
		desc.mPpercent     = ppercent;
		desc.mMaxVertices  = maxv;
		desc.mSkinWidth    = skinWidth;

		MyConvexDecomposition	convexDecomposition(outputFile,this);
		desc.mCallback = &convexDecomposition;
		
		

		//convexDecomposition.performConvexDecomposition(desc);

		ConvexBuilder cb(desc.mCallback);
		cb.process(desc);
		
		if (outputFile)
			fclose(outputFile);


	}


	m_physicsEnvironmentPtr->setDebugDrawer(&debugDrawer);

}

void ConvexDecompositionDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 



	m_physicsEnvironmentPtr->proceedDeltaTime(0.f,deltaTime);

	renderme();

	glFlush();
	glutSwapBuffers();

}



void ConvexDecompositionDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 


	m_physicsEnvironmentPtr->UpdateAabbs(deltaTime);

	renderme();


	glFlush();
	glutSwapBuffers();
}

