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

#ifdef WIN32 //needed for glut.h
#include <windows.h>
#endif

//think different
#if defined(__APPLE__) && !defined (VMDMESA)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "GL_ShapeDrawer.h"
#include "BulletCollision/CollisionShapes/btPolyhedralConvexShape.h"
#include "BulletCollision/CollisionShapes/btTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btTetrahedronShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"

#include "BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h"


#include "LinearMath/btIDebugDraw.h"
//for debugmodes
#include "BMF_Api.h"
#include <stdio.h> //printf debugging

void GL_ShapeDrawer::DrawCoordSystem()  {
    glBegin(GL_LINES);
    glColor3f(1, 0, 0);
    glVertex3d(0, 0, 0);
    glVertex3d(1, 0, 0);
    glColor3f(0, 1, 0);
    glVertex3d(0, 0, 0);
    glVertex3d(0, 1, 0);
    glColor3f(0, 0, 1);
    glVertex3d(0, 0, 0);
    glVertex3d(0, 0, 1);
    glEnd();
	
}





class GlDrawcallback : public btTriangleCallback
{
public:

	virtual void ProcessTriangle(btVector3* triangle,int partId, int triangleIndex)
	{
		glBegin(GL_LINES);
		glColor3f(1, 0, 0);
		glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
		glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
		glColor3f(0, 1, 0);
		glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
		glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
		glColor3f(0, 0, 1);
		glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
		glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
		glEnd();

	}
};

class TriangleGlDrawcallback : public btInternalTriangleIndexCallback
{
public:
	virtual void InternalProcessTriangleIndex(btVector3* triangle,int partId,int  triangleIndex)
	{
		glBegin(GL_TRIANGLES);//LINES);
		glColor3f(1, 0, 0);
		glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
		glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
		glColor3f(0, 1, 0);
		glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
		glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
		glColor3f(0, 0, 1);
		glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
		glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
		glEnd();
	}
};


void GL_ShapeDrawer::DrawOpenGL(float* m, const btCollisionShape* shape, const btVector3& color,int	debugMode)
{

	
	glPushMatrix(); 
    glMultMatrixf(m);

	if (shape->GetShapeType() == COMPOUND_SHAPE_PROXYTYPE)
	{
		const btCompoundShape* compoundShape = static_cast<const btCompoundShape*>(shape);
		for (int i=compoundShape->GetNumChildShapes()-1;i>=0;i--)
		{
			btTransform childTrans = compoundShape->GetChildTransform(i);
			const btCollisionShape* colShape = compoundShape->GetChildShape(i);
			float childMat[16];
			childTrans.getOpenGLMatrix(childMat);
			DrawOpenGL(childMat,colShape,color,debugMode);
		}

	} else
	{
		//DrawCoordSystem();
	    
		//glPushMatrix();
		glEnable(GL_COLOR_MATERIAL);
		glColor3f(color.x(),color.y(), color.z());

		

		bool useWireframeFallback = true;

		if (!(debugMode & btIDebugDraw::DBG_DrawWireframe))
		{
			switch (shape->GetShapeType())
			{
			case BOX_SHAPE_PROXYTYPE:
				{
					const btBoxShape* boxShape = static_cast<const btBoxShape*>(shape);
					btVector3 halfExtent = boxShape->GetHalfExtents();
					glScaled(2*halfExtent[0], 2*halfExtent[1], 2*halfExtent[2]);
					glutSolidCube(1.0);
					useWireframeFallback = false;
					break;
				}
			case TRIANGLE_SHAPE_PROXYTYPE:
			case TETRAHEDRAL_SHAPE_PROXYTYPE:
				{
					//todo:	
//					useWireframeFallback = false;
					break;
				}
			case CONVEX_HULL_SHAPE_PROXYTYPE:
				break;
			case SPHERE_SHAPE_PROXYTYPE:
				{
					const btSphereShape* sphereShape = static_cast<const btSphereShape*>(shape);
					float radius = sphereShape->GetMargin();//radius doesn't include the margin, so draw with margin
					glutSolidSphere(radius,10,10);
					useWireframeFallback = false;
					break;
				}
			case MULTI_SPHERE_SHAPE_PROXYTYPE:
			case CONE_SHAPE_PROXYTYPE:
				{
					const btConeShape* coneShape = static_cast<const btConeShape*>(shape);
					float radius = coneShape->GetRadius();//+coneShape->GetMargin();
					float height = coneShape->GetHeight();//+coneShape->GetMargin();
					//glRotatef(-90.0, 1.0, 0.0, 0.0);
					glTranslatef(0.0, 0.0, -0.5*height);
					glutSolidCone(radius,height,10,10);
					useWireframeFallback = false;
					break;

				}
			case CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE:
				{
					useWireframeFallback = false;
					break;
				}

			case CONVEX_SHAPE_PROXYTYPE:
			case CYLINDER_SHAPE_PROXYTYPE:
				{
					const btCylinderShape* cylinder = static_cast<const btCylinderShape*>(shape);
					int upAxis = cylinder->GetUpAxis();
					
					GLUquadricObj *quadObj = gluNewQuadric();
					float radius = cylinder->GetRadius();
					float halfHeight = cylinder->GetHalfExtents()[upAxis];

					glPushMatrix();
					switch (upAxis)
					{
					case 0:
						glRotatef(-90.0, 0.0, 1.0, 0.0);
						glTranslatef(0.0, 0.0, -halfHeight);
						break;
					case 1:
						glRotatef(-90.0, 1.0, 0.0, 0.0);
						glTranslatef(0.0, 0.0, -halfHeight);
						break;
					case 2:
						
						glTranslatef(0.0, 0.0, -halfHeight);
						break;
					default:
						{
							assert(0);
						}

					}
					
					//The gluCylinder subroutine draws a cylinder that is oriented along the z axis. 
					//The base of the cylinder is placed at z = 0; the top of the cylinder is placed at z=height. 
					//Like a sphere, the cylinder is subdivided around the z axis into slices and along the z axis into stacks.
					
					gluQuadricDrawStyle(quadObj, (GLenum)GLU_FILL);
					gluQuadricNormals(quadObj, (GLenum)GLU_SMOOTH);
					
					
					gluCylinder(quadObj, radius, radius, 2.f*halfHeight, 15, 10);
					glPopMatrix();
					glEndList();

					break;
				}
			default:
				{
				}

			};

		}
		

		

		if (useWireframeFallback)
		{
			/// for polyhedral shapes
			if (shape->IsPolyhedral())
			{
				btPolyhedralConvexShape* polyshape = (btPolyhedralConvexShape*) shape;
				
				
				glBegin(GL_LINES);


				int i;
				for (i=0;i<polyshape->GetNumEdges();i++)
				{
					btPoint3 a,b;
					polyshape->GetEdge(i,a,b);

					glVertex3f(a.getX(),a.getY(),a.getZ());
					glVertex3f(b.getX(),b.getY(),b.getZ());


				}
				glEnd();

				
				if (debugMode==btIDebugDraw::DBG_DrawFeaturesText)
				{
					glRasterPos3f(0.0,  0.0,  0.0);
					BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),polyshape->GetExtraDebugInfo());

					glColor3f(1.f, 1.f, 1.f);
					for (i=0;i<polyshape->GetNumVertices();i++)
					{
						btPoint3 vtx;
						polyshape->GetVertex(i,vtx);
						glRasterPos3f(vtx.x(),  vtx.y(),  vtx.z());
						char buf[12];
						sprintf(buf," %d",i);
						BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
					}

					for (i=0;i<polyshape->GetNumPlanes();i++)
					{
						btVector3 normal;
						btPoint3 vtx;
						polyshape->GetPlane(normal,vtx,i);
						btScalar d = vtx.dot(normal);

						glRasterPos3f(normal.x()*d,  normal.y()*d, normal.z()*d);
						char buf[12];
						sprintf(buf," plane %d",i);
						BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
						
					}
				}

				
			}
		}

		if (shape->GetShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
		{
			btTriangleMeshShape* concaveMesh = (btTriangleMeshShape*) shape;
			//btVector3 aabbMax(1e30f,1e30f,1e30f);
			//btVector3 aabbMax(100,100,100);//1e30f,1e30f,1e30f);

			//todo pass camera, for some culling
			btVector3 aabbMax(1e30f,1e30f,1e30f);
			btVector3 aabbMin(-1e30f,-1e30f,-1e30f);

			GlDrawcallback drawCallback;

			concaveMesh->ProcessAllTriangles(&drawCallback,aabbMin,aabbMax);


		}

		if (shape->GetShapeType() == CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE)
		{
			btConvexTriangleMeshShape* convexMesh = (btConvexTriangleMeshShape*) shape;
			
			//todo: pass camera for some culling			
			btVector3 aabbMax(1e30f,1e30f,1e30f);
			btVector3 aabbMin(-1e30f,-1e30f,-1e30f);
			TriangleGlDrawcallback drawCallback;
			convexMesh->GetStridingMesh()->InternalProcessAllTriangles(&drawCallback,aabbMin,aabbMax);

		}
		
		/*glDisable(GL_DEPTH_BUFFER_BIT);
		if (debugMode==btIDebugDraw::DBG_DrawText)
		{
			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),shape->GetName());
		}

		if (debugMode==btIDebugDraw::DBG_DrawFeaturesText)
		{
			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),shape->GetExtraDebugInfo());
		}
		glEnable(GL_DEPTH_BUFFER_BIT);
		*/

	//	glPopMatrix();
	}
    glPopMatrix();
	
}
