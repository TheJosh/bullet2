/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2010 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#define TEST_SERIALIZATION 1
//#undef DESERIALIZE_SOFT_BODIES

#ifdef BT_INTERNAL_UPDATE_SERIALIZATION_STRUCTURES
#define CREATE_NEW_BULLETFILE 1
#endif //BT_INTERNAL_UPDATE_SERIALIZATION_STRUCTURES

///create 125 (5x5x5) dynamic object
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5

//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024)

///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.
#define START_POS_X -5
#define START_POS_Y -5
#define START_POS_Z -3

#include "SerializeDemo.h"
#include "GlutStuff.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#ifdef TEST_SERIALIZATION
#include "LinearMath/btSerializer.h"
#include "btBulletFile.h"
#include "btBulletWorldImporter.h"
#endif //TEST_SERIALIZATION

#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include <stdio.h> //printf debugging


#include "GLDebugDrawer.h"
GLDebugDrawer	gDebugDrawer;

#ifdef DESERIALIZE_SOFT_BODIES
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"

#ifdef USE_AMD_OPENCL

#include "btOclCommon.h"
#include "btOclUtils.h"
#include <LinearMath/btScalar.h>

#include "BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/btSoftBodySolver_OpenCL.h"
#include "BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/btSoftBodySolver_OpenCLSIMDAware.h"
#include "BulletMultiThreaded/GpuSoftBodySolvers/CPU/btSoftBodySolver_CPU.h"

cl_context			g_cxMainContext;
cl_device_id		g_cdDevice;
cl_command_queue	g_cqCommandQue;
btSoftBodySolver* sbsolver = 0;

void initCL( void* glCtx, void* glDC )
{
	int ciErrNum = 0;

#if defined(CL_PLATFORM_MINI_CL)
	cl_device_type deviceType = CL_DEVICE_TYPE_CPU;
#elif defined(CL_PLATFORM_AMD)
	cl_device_type deviceType = CL_DEVICE_TYPE_GPU;
#elif defined(CL_PLATFORM_NVIDIA)
	cl_device_type deviceType = CL_DEVICE_TYPE_GPU;
#else
	cl_device_type deviceType = CL_DEVICE_TYPE_CPU;
#endif

    //g_cxMainContext = btOclCommon::createContextFromType(CL_DEVICE_TYPE_ALL, &ciErrNum);
	//g_cxMainContext = btOclCommon::createContextFromType(CL_DEVICE_TYPE_GPU, &ciErrNum);
	//g_cxMainContext = btOclCommon::createContextFromType(CL_DEVICE_TYPE_CPU, &ciErrNum);
	//try CL_DEVICE_TYPE_DEBUG for sequential, non-threaded execution, when using MiniCL on CPU, it gives a full callstack at the crash in the kernel
//#ifdef USE_MINICL
//	g_cxMainContext = btOclCommon::createContextFromType(CL_DEVICE_TYPE_DEBUG, &ciErrNum);
//#else
	g_cxMainContext = btOclCommon::createContextFromType(deviceType, &ciErrNum, (intptr_t)glCtx, (intptr_t)glDC);
//#endif
	
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	g_cdDevice = btOclGetMaxFlopsDev(g_cxMainContext);
	
	btOclPrintDevInfo(g_cdDevice);

	// create a command-queue
	g_cqCommandQue = clCreateCommandQueue(g_cxMainContext, g_cdDevice, 0, &ciErrNum);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
}

#endif //#ifdef USE_AMD_OPENCL
#endif


void SerializeDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();
	
	///step the simulation
	if (m_dynamicsWorld)
	{
		
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		if (sbsolver)
			sbsolver->copyBackToSoftBodies();
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}
	

	renderme(); 

	glFlush();

	swapBuffers();

}



void SerializeDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	
	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}


void	SerializeDemo::setupEmptyDynamicsWorld()
{
	initCL(0,0);

	///collision configuration contains default setup for memory, collision setup
	//m_collisionConfiguration = new btDefaultCollisionConfiguration();
#ifdef DESERIALIZE_SOFT_BODIES
	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
#else
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
#endif //DESERIALIZE_SOFT_BODIES

	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();
	

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

#ifdef DESERIALIZE_SOFT_BODIES
	static bool toggleSolver = 0;
	toggleSolver = 1 - toggleSolver;
	if (toggleSolver)
	{
		sbsolver = new btOpenCLSoftBodySolverSIMDAware( g_cqCommandQue, g_cxMainContext);
		//sbsolver = new btOpenCLSoftBodySolver( g_cqCommandQue, g_cxMainContext);
		//sbsolver =  new btCPUSoftBodySolver( );
	}
	//sbsolver = new btCPUSoftBodySolver();
	
	btSoftRigidDynamicsWorld* world = new btSoftRigidDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration,sbsolver);
	m_dynamicsWorld = world;

	world->setDebugDrawer(&gDebugDrawer);

	//world->setDrawFlags(world->getDrawFlags()^fDrawFlags::Clusters);
#else
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
#endif //DESERIALIZE_SOFT_BODIES

	//btGImpactCollisionAlgorithm::registerAlgorithm((btCollisionDispatcher*)m_dynamicsWorld->getDispatcher());

	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

}


#ifdef DESERIALIZE_SOFT_BODIES
#include "BulletSoftBody/btSoftBodyData.h"
class MySoftBulletWorldImporter : public btBulletWorldImporter
{

	btSoftRigidDynamicsWorld* m_softRigidWorld;

	btHashMap<btHashPtr,btSoftBody::Material*>	m_materialMap;

	btHashMap<btHashPtr,btSoftBody*>	m_clusterBodyMap;
	btHashMap<btHashPtr,btSoftBody*>	m_softBodyMap;
	


public:

	MySoftBulletWorldImporter(btSoftRigidDynamicsWorld* world)
		:btBulletWorldImporter(world),
		m_softRigidWorld(world)
	{

	}

	virtual ~MySoftBulletWorldImporter()
	{
		
	}

	virtual bool convertAllObjects(  bParse::btBulletFile* bulletFile2)
	{
		bool result = btBulletWorldImporter::convertAllObjects(bulletFile2);
		int i;
		//now the soft bodies
		for (i=0;i<bulletFile2->m_softBodies.size();i++)
		{
			if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
			{
				btAssert(0); //not yet
				//btSoftBodyFloatData* softBodyData = (btSoftBodyFloatData*)bulletFile2->m_softBodies[i];
			} else
			{
				btSoftBodyFloatData* softBodyData = (btSoftBodyFloatData*)bulletFile2->m_softBodies[i];
				int i;
				int numNodes = softBodyData->m_numNodes;
				
		
				btSoftBody*		psb=new btSoftBody(&m_softRigidWorld->getWorldInfo());
				m_softBodyMap.insert(softBodyData,psb);

				//materials
				for (i=0;i<softBodyData->m_numMaterials;i++)
				{
					SoftBodyMaterialData* matData = softBodyData->m_materials[i];
					btSoftBody::Material** matPtr = m_materialMap.find(matData);
					btSoftBody::Material* mat = 0;
					if (matPtr&& *matPtr)
					{
						mat = *matPtr;
					} else
					{
						mat = psb->appendMaterial();
						mat->m_flags = matData->m_flags;
						mat->m_kAST = matData->m_angularStiffness;
						mat->m_kLST = matData->m_linearStiffness;
						mat->m_kVST = matData->m_volumeStiffness;
						m_materialMap.insert(matData,mat);
					}
				}




				for (i=0;i<numNodes;i++)
				{
					SoftBodyNodeData& nodeData = softBodyData->m_nodes[i];
					btVector3 position;
					position.deSerializeFloat(nodeData.m_position);
					btScalar mass = nodeData.m_inverseMass? 1./nodeData.m_inverseMass : 0.f;
					psb->appendNode(position,mass);
					btSoftBody::Node* node = &psb->m_nodes[psb->m_nodes.size()-1];
					node->m_area = nodeData.m_area;
					node->m_battach = nodeData.m_attach;
					node->m_f.deSerializeFloat(nodeData.m_accumulatedForce);
					node->m_im = nodeData.m_inverseMass;

					btSoftBody::Material** matPtr = m_materialMap.find(nodeData.m_material);
					if (matPtr && *matPtr)
					{
						node->m_material = *matPtr;
					} else
					{
						printf("no mat?\n");
					}
					
					node->m_n.deSerializeFloat(nodeData.m_normal);
					node->m_q = node->m_x;
					node->m_v.deSerializeFloat(nodeData.m_velocity);
					
				}

				for (i=0;i<softBodyData->m_numLinks;i++)
				{
					SoftBodyLinkData& linkData = softBodyData->m_links[i];
					btSoftBody::Material** matPtr = m_materialMap.find(linkData.m_material);
					if (matPtr && *matPtr)
					{
						psb->appendLink(linkData.m_nodeIndices[0],linkData.m_nodeIndices[1],*matPtr);
					} else
					{
						psb->appendLink(linkData.m_nodeIndices[0],linkData.m_nodeIndices[1]);
					}
					btSoftBody::Link* link = &psb->m_links[psb->m_links.size()-1];
					link->m_bbending = linkData.m_bbending;
					link->m_rl = linkData.m_restLength;
				}

				for (i=0;i<softBodyData->m_numFaces;i++)
				{
					SoftBodyFaceData& faceData = softBodyData->m_faces[i];
					btSoftBody::Material** matPtr = m_materialMap.find(faceData.m_material);
					if (matPtr && *matPtr)
					{
						psb->appendFace(faceData.m_nodeIndices[0],faceData.m_nodeIndices[1],faceData.m_nodeIndices[2],*matPtr);
					} else
					{
						psb->appendFace(faceData.m_nodeIndices[0],faceData.m_nodeIndices[1],faceData.m_nodeIndices[2]);
					}
					btSoftBody::Face* face = &psb->m_faces[psb->m_faces.size()-1];
					face->m_normal.deSerializeFloat(faceData.m_normal);
					face->m_ra = faceData.m_restArea;
				}

			

				//anchors
				for (i=0;i<softBodyData->m_numAnchors;i++)
				{
					btCollisionObject** colAptr = m_bodyMap.find(softBodyData->m_anchors[i].m_rigidBody);
					if (colAptr && *colAptr)
					{
						btRigidBody* body = btRigidBody::upcast(*colAptr);
						if (body)
						{
							bool disableCollision = false;
							btVector3 localPivot;
							localPivot.deSerializeFloat(softBodyData->m_anchors[i].m_localFrame);
							psb->appendAnchor(softBodyData->m_anchors[i].m_nodeIndex,body,localPivot, disableCollision);
						}
					}
				}

				if (softBodyData->m_pose)
				{
					psb->m_pose.m_aqq.deSerializeFloat(  softBodyData->m_pose->m_aqq);
					psb->m_pose.m_bframe = softBodyData->m_pose->m_bframe;
					psb->m_pose.m_bvolume = softBodyData->m_pose->m_bvolume;
					psb->m_pose.m_com.deSerializeFloat(softBodyData->m_pose->m_com);
					
					psb->m_pose.m_pos.resize(softBodyData->m_pose->m_numPositions);
					for (i=0;i<softBodyData->m_pose->m_numPositions;i++)
					{
						psb->m_pose.m_pos[i].deSerializeFloat(softBodyData->m_pose->m_positions[i]);
					}
					psb->m_pose.m_rot.deSerializeFloat(softBodyData->m_pose->m_rot);
					psb->m_pose.m_scl.deSerializeFloat(softBodyData->m_pose->m_scale);
					psb->m_pose.m_wgh.resize(softBodyData->m_pose->m_numWeigts);
					for (i=0;i<softBodyData->m_pose->m_numWeigts;i++)
					{
						psb->m_pose.m_wgh[i] = softBodyData->m_pose->m_weights[i];
					}
					psb->m_pose.m_volume = softBodyData->m_pose->m_restVolume;
				}

#if 1
				psb->m_cfg.piterations=softBodyData->m_config.m_positionIterations;
				psb->m_cfg.diterations=softBodyData->m_config.m_driftIterations;
				psb->m_cfg.citerations=softBodyData->m_config.m_clusterIterations;
				psb->m_cfg.viterations=softBodyData->m_config.m_velocityIterations;

				//psb->setTotalMass(0.1);
				psb->m_cfg.aeromodel = (btSoftBody::eAeroModel::_)softBodyData->m_config.m_aeroModel;
				psb->m_cfg.kLF = softBodyData->m_config.m_lift;
				psb->m_cfg.kDG = softBodyData->m_config.m_drag;
				psb->m_cfg.kMT = softBodyData->m_config.m_poseMatch;
				psb->m_cfg.collisions = softBodyData->m_config.m_collisionFlags;
				psb->m_cfg.kDF = softBodyData->m_config.m_dynamicFriction;
				psb->m_cfg.kDP = softBodyData->m_config.m_damping;
				psb->m_cfg.kPR = softBodyData->m_config.m_pressure;
				psb->m_cfg.kVC = softBodyData->m_config.m_volume;
				psb->m_cfg.kAHR = softBodyData->m_config.m_anchorHardness;
				psb->m_cfg.kKHR = softBodyData->m_config.m_kineticContactHardness;
				psb->m_cfg.kSHR = softBodyData->m_config.m_softContactHardness;
				psb->m_cfg.kSRHR_CL = softBodyData->m_config.m_softRigidClusterHardness;
				psb->m_cfg.kSKHR_CL = softBodyData->m_config.m_softKineticClusterHardness;
				psb->m_cfg.kSSHR_CL = softBodyData->m_config.m_softSoftClusterHardness;
#endif

//				pm->m_kLST				=	1;

#if 1
				//clusters
				if (softBodyData->m_numClusters)
				{
					m_clusterBodyMap.insert(softBodyData->m_clusters,psb);
					int j;
					psb->m_clusters.resize(softBodyData->m_numClusters);
					for (i=0;i<softBodyData->m_numClusters;i++)
					{
						psb->m_clusters[i] = new(btAlignedAlloc(sizeof(btSoftBody::Cluster),16)) btSoftBody::Cluster();
						psb->m_clusters[i]->m_adamping = softBodyData->m_clusters[i].m_adamping;
						psb->m_clusters[i]->m_av.deSerializeFloat(softBodyData->m_clusters[i].m_av);
						psb->m_clusters[i]->m_clusterIndex = softBodyData->m_clusters[i].m_clusterIndex;
						psb->m_clusters[i]->m_collide = softBodyData->m_clusters[i].m_collide;
						psb->m_clusters[i]->m_com.deSerializeFloat(softBodyData->m_clusters[i].m_com);
						psb->m_clusters[i]->m_containsAnchor = softBodyData->m_clusters[i].m_containsAnchor;
						psb->m_clusters[i]->m_dimpulses[0].deSerializeFloat(softBodyData->m_clusters[i].m_dimpulses[0]);
						psb->m_clusters[i]->m_dimpulses[1].deSerializeFloat(softBodyData->m_clusters[i].m_dimpulses[1]);

						psb->m_clusters[i]->m_framerefs.resize(softBodyData->m_clusters[i].m_numFrameRefs);
						for (j=0;j<softBodyData->m_clusters[i].m_numFrameRefs;j++)
						{
							psb->m_clusters[i]->m_framerefs[j].deSerializeFloat(softBodyData->m_clusters[i].m_framerefs[j]);
						}
						psb->m_clusters[i]->m_nodes.resize(softBodyData->m_clusters[i].m_numNodes);
						for (j=0;j<softBodyData->m_clusters[i].m_numNodes;j++)
						{
							int nodeIndex = softBodyData->m_clusters[i].m_nodeIndices[j];
							psb->m_clusters[i]->m_nodes[j] = &psb->m_nodes[nodeIndex];
						}

						psb->m_clusters[i]->m_masses.resize(softBodyData->m_clusters[i].m_numMasses);
						for (j=0;j<softBodyData->m_clusters[i].m_numMasses;j++)
						{
							psb->m_clusters[i]->m_masses[j] = softBodyData->m_clusters[i].m_masses[j];
						}
						psb->m_clusters[i]->m_framexform.deSerializeFloat(softBodyData->m_clusters[i].m_framexform);
						psb->m_clusters[i]->m_idmass = softBodyData->m_clusters[i].m_idmass;
						psb->m_clusters[i]->m_imass = softBodyData->m_clusters[i].m_imass;
						psb->m_clusters[i]->m_invwi.deSerializeFloat(softBodyData->m_clusters[i].m_invwi);
						psb->m_clusters[i]->m_ldamping = softBodyData->m_clusters[i].m_ldamping;
						psb->m_clusters[i]->m_locii.deSerializeFloat(softBodyData->m_clusters[i].m_locii);
						psb->m_clusters[i]->m_lv.deSerializeFloat(softBodyData->m_clusters[i].m_lv);
						psb->m_clusters[i]->m_matching = softBodyData->m_clusters[i].m_matching;
						psb->m_clusters[i]->m_maxSelfCollisionImpulse = 0;//softBodyData->m_clusters[i].m_maxSelfCollisionImpulse;
						psb->m_clusters[i]->m_ndamping = softBodyData->m_clusters[i].m_ndamping;
						psb->m_clusters[i]->m_ndimpulses = softBodyData->m_clusters[i].m_ndimpulses;
						psb->m_clusters[i]->m_nvimpulses = softBodyData->m_clusters[i].m_nvimpulses;
						psb->m_clusters[i]->m_selfCollisionImpulseFactor = softBodyData->m_clusters[i].m_selfCollisionImpulseFactor;
						psb->m_clusters[i]->m_vimpulses[0].deSerializeFloat(softBodyData->m_clusters[i].m_vimpulses[0]);
						psb->m_clusters[i]->m_vimpulses[1].deSerializeFloat(softBodyData->m_clusters[i].m_vimpulses[1]);
						
					}
					//psb->initializeClusters();
					//psb->updateClusters();

				}
#else

				psb->m_cfg.piterations	=	2;
				psb->m_cfg.collisions	=	btSoftBody::fCollision::CL_SS+	btSoftBody::fCollision::CL_RS;
				//psb->setTotalMass(50,true);
				//psb->generateClusters(64);
				//psb->m_cfg.kDF=1;
				psb->generateClusters(8);


#endif //



				psb->updateConstants();
				m_softRigidWorld->getWorldInfo().m_dispatcher = m_softRigidWorld->getDispatcher();
				
				m_softRigidWorld->addSoftBody(psb);


			}
		}


		//now the soft body joints
		for (i=0;i<bulletFile2->m_softBodies.size();i++)
		{
			if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
			{
				btAssert(0); //not yet
				//btSoftBodyFloatData* softBodyData = (btSoftBodyFloatData*)bulletFile2->m_softBodies[i];
			} else
			{
				btSoftBodyFloatData* softBodyData = (btSoftBodyFloatData*)bulletFile2->m_softBodies[i];
				btSoftBody** sbp = m_softBodyMap.find(softBodyData);
				if (sbp && *sbp)
				{
					btSoftBody* sb = *sbp;
					int i;
					for (int i=0;i<softBodyData->m_numJoints;i++)
					{
						btSoftBodyJointData* sbjoint = &softBodyData->m_joints[i];


						btSoftBody::Body bdyB;

						btSoftBody* sbB = 0;
						btTransform transA;
						transA.setIdentity();
						transA = sb->m_clusters[0]->m_framexform;

						btCollisionObject** colBptr = m_bodyMap.find(sbjoint->m_bodyB);
						if (colBptr && *colBptr)
						{
							btRigidBody* rbB = btRigidBody::upcast(*colBptr);
							if (rbB)
							{
								bdyB = rbB;
							} else
							{
								bdyB = *colBptr;
							}
						}


						btSoftBody** bodyBptr = m_clusterBodyMap.find(sbjoint->m_bodyB);
						if (bodyBptr && *bodyBptr )
						{
							sbB = *bodyBptr;
							bdyB = sbB->m_clusters[0];
						}


						if (sbjoint->m_jointType==btSoftBody::Joint::eType::Linear)
						{
							btSoftBody::LJoint::Specs specs;
							specs.cfm = sbjoint->m_cfm;
							specs.erp = sbjoint->m_erp;
							specs.split = sbjoint->m_split;
							btVector3 relA;
							relA.deSerializeFloat(sbjoint->m_refs[0]);
							specs.position = transA*relA;
							sb->appendLinearJoint(specs,sb->m_clusters[0],bdyB);
						}

						if (sbjoint->m_jointType==btSoftBody::Joint::eType::Angular)
						{
							btSoftBody::AJoint::Specs specs;
							specs.cfm = sbjoint->m_cfm;
							specs.erp = sbjoint->m_erp;
							specs.split = sbjoint->m_split;
							btVector3 relA;
							relA.deSerializeFloat(sbjoint->m_refs[0]);
							specs.axis = transA.getBasis()*relA;
							sb->appendAngularJoint(specs,sb->m_clusters[0],bdyB);
						}
					}
				}

			}
		}

		return result;

	}
};
#endif //DESERIALIZE_SOFT_BODIES

btBulletWorldImporter* fileLoader=0;


void	SerializeDemo::initPhysics()
{
	setTexturing(true);
	setShadows(true);

	setCameraDistance(btScalar(SCALING*50.));

	setupEmptyDynamicsWorld();
	
#ifdef DESERIALIZE_SOFT_BODIES
	btBulletWorldImporter* fileLoader = new MySoftBulletWorldImporter((btSoftRigidDynamicsWorld*)m_dynamicsWorld);
#else
	btBulletWorldImporter* fileLoader = new btBulletWorldImporter(m_dynamicsWorld);
#endif //DESERIALIZE_SOFT_BODIES
//	fileLoader->setVerboseMode(true);

	if (!fileLoader->loadFile("testFile.bullet"))
	{
		btAssert(0);
		exit(0);
	}


}
	

void	SerializeDemo::clientResetScene()
{
	exitPhysics();

	initPhysics();
}

void	SerializeDemo::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	m_collisionShapes.clear();

	delete m_dynamicsWorld;
	
#ifdef DESERIALIZE_SOFT_BODIES
	delete sbsolver;
	sbsolver = 0;
#endif

	delete m_solver;
	
	delete m_broadphase;
	
	delete m_dispatcher;

	delete m_collisionConfiguration;

	
}




