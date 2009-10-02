/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006, 2007 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "LinearMath/btMinMax.h"

//----------------------------------------------------------------------------------------

#define USE_FRICTION 1
#define FRICTION_BOX_GROUND_FACT 0.01f
#define FRICTION_BOX_BOX_FACT 0.01f
//#define FRICTION_BOX_BOX_FACT 0.05f
#define USE_CENTERS 1

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------   C o n s t r a i n t   s o l v e r    d e m o  3D --------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------

// kernel functions


BT_GPU___global__ void clearAccumulationOfLambdaDtD(float* lambdaDtBox, int numConstraints, int numContPoints)
{
    int index = BT_GPU___mul24(BT_GPU_blockIdx.x, BT_GPU_blockDim.x) + BT_GPU_threadIdx.x;
	if(index < numConstraints)
	{
		for(int i=0; i < numContPoints; i++)
			lambdaDtBox[numContPoints * index + i] = 0;
	}
} // clearAccumulationOfLambdaDtD()

//----------------------------------------------------------------------------------------

BT_GPU___device__ float computeImpulse3D(float4 rVel,
								 float positionConstraint,
								 float4 cNormal,
								 float dt)
{
	const float collisionConstant	=	0.1f;
	const float baumgarteConstant	=	0.1f;
	const float penetrationError	=	0.02f;

	float lambdaDt=0;

	if(positionConstraint >= 0)
		return lambdaDt;

	positionConstraint = btMin(0.0f,positionConstraint+penetrationError);
	
	lambdaDt	=	-(BT_GPU_dot(cNormal,rVel)*(collisionConstant));
	lambdaDt	-=	(baumgarteConstant/dt*positionConstraint);

	return lambdaDt;
} // computeImpulse3D()

//----------------------------------------------------------------------------------------

#if 0
#define VLIM 1000.f
void BT_GPU___device__ chk_vect(float4* v)
{
	if(v->x < -VLIM) v->x = 0.f;
	if(v->x >  VLIM) v->x = 0.f;
	if(v->y < -VLIM) v->y = 0.f;
	if(v->y >  VLIM) v->y = 0.f;
	if(v->z < -VLIM) v->z = 0.f;
	if(v->z >  VLIM) v->z = 0.f;
} // chk_vect()
#endif

//----------------------------------------------------------------------------------------

BT_GPU___global__ void collisionWithWallBox3DD(float4 *trans,
								   float4 *vel,
								   float4* angVel,
								   btCudaPartProps pProp,
								   btCudaBoxProps gProp,
								   int nParticles,
								   float dt)
{
    int idx = BT_GPU___mul24(BT_GPU_blockIdx.x, BT_GPU_blockDim.x) + BT_GPU_threadIdx.x;
	float4 aPos;
	float positionConstraint;
	float4 impulse;
	
	if(idx < nParticles)
	{
		aPos = trans[idx * 4 + 3];
		for(int iVtx=0; iVtx < 8; iVtx++)
		{
			float4 dx = trans[idx * 4 + 0];
			float4 dy = trans[idx * 4 + 1];
			float4 dz = trans[idx * 4 + 2];
			float4 rerVertex = ((iVtx & 1) ? dx : dx * (-1.f));
			
			rerVertex += ((iVtx & 2) ? dy : dy * (-1.f));
			rerVertex += ((iVtx & 4) ? dz : dz * (-1.f));
			float4 vPos = aPos + rerVertex;
			float4 aVel	= vel[idx];
			float4 aAngVel	= angVel[idx];
			float4 vVel	=aVel + BT_GPU_cross(aAngVel, rerVertex);
			float restitution=0.5;
			{
				positionConstraint  = vPos.y - gProp.minY;
				impulse				= BT_GPU_make_float4(0.0f, 0.f, 0.f, 0.f);
				if(positionConstraint < 0)
				{
					float4 groundNormal;
					groundNormal = BT_GPU_make_float4(0.0f,1.0f,0.0f, 0.f);
					impulse	= groundNormal * restitution * computeImpulse3D(vVel, positionConstraint, groundNormal, dt);
#if USE_FRICTION	// only with ground for now
					float4 lat_vel = vVel - groundNormal * BT_GPU_dot(groundNormal,vVel);
					float lat_vel_len = BT_GPU_dot(lat_vel, lat_vel);
					if (lat_vel_len > 0)
					{
						lat_vel_len = sqrtf(lat_vel_len);
						lat_vel *= 1.f/lat_vel_len;	
						impulse	-= lat_vel * BT_GPU_dot(lat_vel, vVel) * FRICTION_BOX_GROUND_FACT;
					}
#endif //USE_FRICTION
					vel[idx]	+=	impulse;
					angVel[idx]	+=	BT_GPU_cross(rerVertex,impulse);
				}
			}
			{
				positionConstraint	= vPos.x - gProp.minX;
				impulse				= BT_GPU_make_float4(0.0f, 0.f, 0.f, 0.f);
				if(positionConstraint < 0)
				{
					float4 normal = BT_GPU_make_float4(1.0f,0.0f,0.0f, 0.f);
					impulse	= normal * restitution * computeImpulse3D(vVel,positionConstraint,normal,dt);
					vel[idx]	+=	impulse;
					angVel[idx]	+=	BT_GPU_cross(rerVertex,impulse);
				}
			}
			{
				positionConstraint	= gProp.maxX - vPos.x;
				impulse				= BT_GPU_make_float4(0.0f, 0.f, 0.f, 0.f);
				if(positionConstraint < 0)
				{
					float4 normal = BT_GPU_make_float4(-1.0f,0.0f,0.0f, 0.f);
					impulse	= normal * restitution * computeImpulse3D(vVel,positionConstraint,normal,dt);
					vel[idx]	+=	impulse;
					angVel[idx]	+=	BT_GPU_cross(rerVertex,impulse);
				}
			}
			{
				positionConstraint	= vPos.z - gProp.minZ;
				impulse				= BT_GPU_make_float4(0.0f, 0.f, 0.f, 0.f);
				if(positionConstraint < 0)
				{
					float4 normal = BT_GPU_make_float4(0.0f,0.0f,1.0f, 0.f);
					impulse	= normal * restitution * computeImpulse3D(vVel,positionConstraint,normal,dt);
					vel[idx]	+=	impulse;
					angVel[idx]	+=	BT_GPU_cross(rerVertex,impulse);
				}
			}
			{
				positionConstraint	= gProp.maxZ - vPos.z;
				impulse				= BT_GPU_make_float4(0.0f, 0.f, 0.f, 0.f);
				if(positionConstraint < 0)
				{
					float4 normal = BT_GPU_make_float4(0.0f,0.0f,-1.0f, 0.f);
					impulse	= normal * restitution * computeImpulse3D(vVel,positionConstraint,normal,dt);
					vel[idx]	+=	impulse;
					angVel[idx]	+=	BT_GPU_cross(rerVertex,impulse);
				}
			}
		}
	}
} // collisionWithWallBox3DD()

//----------------------------------------------------------------------------------------

BT_GPU___global__ void collisionBatchResolutionBox3DD(int2 *constraints,
										 int *batch,
										 int nConstraints,
										 float4 *trans,
										 float4 *vel,
										 float4 *angularVel,
										 float *lambdaDtBox,
										 float *iPositionConstraint,
										 float4 *normal,
										 float4 *contact,
										 btCudaPartProps pProp,
										 int iBatch,
										 float dt)
{
	float4 relVel;
	float4 impulse;
	float lambdaDt;
	float positionConstraint;
    int k_idx = BT_GPU___mul24(BT_GPU_blockIdx.x, BT_GPU_blockDim.x) + BT_GPU_threadIdx.x;
	if(k_idx < nConstraints)
	{
		int idx = batch[k_idx];
		int aId=constraints[idx].x;
		int bId=constraints[idx].y;
		float4 aPos = trans[aId * 4 + 3];
		float4 bPos = trans[bId * 4 + 3];
		float4 aVel = vel[aId];
		float4 bVel = vel[bId];
		float4 aAngVel = angularVel[aId];
		float4 bAngVel = angularVel[bId];
		for(int iVtx = 0; iVtx < 4; iVtx++)
		{
			float4 contactPoint	= contact[idx * 4 + iVtx] - aPos;
			positionConstraint = iPositionConstraint[idx * 4 + iVtx];
			if(positionConstraint > 0)
			{
				float4 contactNormal = normal[idx * 4 + iVtx];
				relVel = (aVel + BT_GPU_cross(aAngVel, contactPoint))
				 -(bVel + BT_GPU_cross(bAngVel, contactPoint+aPos-bPos));

				lambdaDt=	computeImpulse3D(relVel, -positionConstraint, contactNormal, dt);
				{
					float rLambdaDt=lambdaDtBox[idx * 4 + iVtx];
					float pLambdaDt=rLambdaDt;
					rLambdaDt=btMax(pLambdaDt+lambdaDt,0.0f);
					lambdaDt=rLambdaDt-pLambdaDt;
					lambdaDtBox[idx * 4 + iVtx]=rLambdaDt;
				}
				impulse = contactNormal*lambdaDt*0.5;
#if USE_FRICTION
				float4 lat_vel = relVel - contactNormal * BT_GPU_dot(contactNormal, relVel);
				float lat_vel_len = BT_GPU_dot(lat_vel, lat_vel);
				if (lat_vel_len > 0)
				{
					lat_vel_len = sqrtf(lat_vel_len);
					lat_vel *= 1.f/lat_vel_len;
					impulse	-= lat_vel * BT_GPU_dot(lat_vel , relVel) * FRICTION_BOX_BOX_FACT;
				}
#endif //USE_FRICTION
				aVel+=	impulse;
				bVel-=	impulse;
				aAngVel += BT_GPU_cross(contactPoint, impulse);
				bAngVel -= BT_GPU_cross(contactPoint+aPos-bPos, impulse);
			}
		}
		vel[aId]= aVel;
		vel[bId]= bVel;
		angularVel[aId]= aAngVel;
		angularVel[bId]= bAngVel;
	}
} // collisionBatchResolutionBox3DD()

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------


extern "C"
{

// global functions

//----------------------------------------------------------------------------------------

void BT_GPU_PREF(clearAccumulationOfLambdaDt(float* lambdaDtBox, int numConstraints, int numContPoints))
{
	if(!numConstraints) 
	{
		return;
	}
    int numThreads, numBlocks;
    BT_GPU_PREF(computeGridSize)(numConstraints, 256, numBlocks, numThreads);
    // execute the kernel
    BT_GPU_EXECKERNEL(numBlocks, numThreads, clearAccumulationOfLambdaDtD, (lambdaDtBox, numConstraints, numContPoints));
    // check if kernel invocation generated an error
    BT_GPU_CHECK_ERROR("clearAccumulationOfLambdaDtD kernel execution failed");
    
} // clearAccumulationOfLambdaDt()

//----------------------------------------------------------------------------------------

void BT_GPU_PREF(collisionWithWallBox3D(void* trans,void* vel,void* angVel,btCudaPartProps pProp,	btCudaBoxProps gProp,int numObjs,float dt))
{
	if(!numObjs) 
	{
		return;
	}
	float4* pTrans = (float4*)trans;
	float4* pVel = (float4*)vel;
	float4* pAngVel = (float4*)angVel;
    int numThreads, numBlocks;
    BT_GPU_PREF(computeGridSize)(numObjs, 256, numBlocks, numThreads);
    // execute the kernel
    BT_GPU_EXECKERNEL(numBlocks, numThreads, collisionWithWallBox3DD, (pTrans,pVel,pAngVel,pProp,gProp,numObjs,dt));
    // check if kernel invocation generated an error
    BT_GPU_CHECK_ERROR("collisionWithWallBox3DD kernel execution failed");
} // collisionWithWallBox3D()

//----------------------------------------------------------------------------------------

void BT_GPU_PREF(collisionBatchResolutionBox3D(void* constraints,int *batch,int numConstraints,void *trans,void *vel,
											void *angularVel,float *lambdaDtBox,float *positionConstraint,void* normal,void* contact,
											btCudaPartProps pProp,int iBatch,float dt))
{
	if(!numConstraints) 
	{
		return;
	}
	int2* pConstr = (int2*)constraints;
	float4* pTrans = (float4*)trans;
	float4* pVel = (float4*)vel;
	float4* pAngVel = (float4*)angularVel;
	float4* pNorm = (float4*)normal;
	float4* pContact = (float4*)contact;
    int numThreads, numBlocks;
    BT_GPU_PREF(computeGridSize)(numConstraints, 128, numBlocks, numThreads);
    // execute the kernel
    BT_GPU_EXECKERNEL(numBlocks, numThreads, collisionBatchResolutionBox3DD, (pConstr,batch,numConstraints,pTrans,pVel,pAngVel,lambdaDtBox,positionConstraint,pNorm,pContact,pProp,iBatch,dt));
    // check if kernel invocation generated an error
    BT_GPU_CHECK_ERROR("collisionBatchResolutionBox3DD kernel execution failed");
} // collisionBatchResolutionBox3D()

//----------------------------------------------------------------------------------------

} // extern "C"

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------   M o t i o n   i n t e g r a t o r   d e m o   -----------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------

// kernel functions

BT_GPU___global__ void integrVelD(float4* pForceTorqueDamp, float4* pInvInertiaMass, float4* pVel, float4* pAngVel, float timeStep, unsigned int numBodies)
{
    int index = BT_GPU___mul24(BT_GPU_blockIdx.x, BT_GPU_blockDim.x) + BT_GPU_threadIdx.x;
    if(index >= (int)numBodies)
	{
		return;
	}
	// unpack input data
	float4 force =  pForceTorqueDamp[index * 2];
	float lin_damp = force.w;
	force.w = 0.f;
	float4 torque =  pForceTorqueDamp[index * 2 + 1];
	float ang_damp = torque.w;
	torque.w = 0.f;
	float4 linVel =  pVel[index];
	float4 angVel =  pAngVel[index];
	float4 in_mass_0 = pInvInertiaMass[index * 3];
	float mass = in_mass_0.w;
	in_mass_0.w = 0.f;
	float4 in_mass_1 = pInvInertiaMass[index * 3 + 1];
	in_mass_1.w = 0.f;
	float4 in_mass_2 = pInvInertiaMass[index * 3 + 2];
	in_mass_2.w = 0.f;
	// integrate linear velocity
	float4 outLinVel, outAngVel;
	outLinVel = linVel + force * mass * timeStep;
	outLinVel.w = 0.f;
	// integrate angular velocity
	outAngVel.x = BT_GPU_dot(in_mass_0, torque);
	outAngVel.y = BT_GPU_dot(in_mass_1, torque);
	outAngVel.z = BT_GPU_dot(in_mass_2, torque);
	outAngVel.w = 0.f;
	outAngVel += angVel;
	/// clamp angular velocity. collision calculations will fail on higher angular velocities	
	#if(!defined(M_PI))
	#define M_PI 3.1415926f
	#endif
	#define BT_CUDA_MAX_SQ_ANGVEL (M_PI*M_PI)
	float sq_angvel = BT_GPU_dot(outAngVel, outAngVel);
	sq_angvel *= timeStep * timeStep;
	float fact;
	if(sq_angvel > BT_CUDA_MAX_SQ_ANGVEL)
	{
		fact = sqrtf(BT_CUDA_MAX_SQ_ANGVEL/sq_angvel) / timeStep;
		outAngVel *= fact;
	}
	// now apply damping
	fact = powf(1.0f - lin_damp, timeStep);
	outLinVel *= fact;
	fact = powf(1.0f - ang_damp, timeStep);
	outAngVel *= fact;
	// pack results
	pVel[index] = outLinVel;
	pAngVel[index] = outAngVel;
} // integrVelD()

#define BT_GPU__ANGULAR_MOTION_THRESHOLD (0.25f * M_PI)

//----------------------------------------------------------------------------------------

BT_GPU___device__ float4 getRotation(float4* trans)
{
	float trace = trans[0].x + trans[1].y + trans[2].z;
	float temp[4];
	if(trace > 0.0f)
	{
		float s = sqrtf(trace + 1.0f);
		temp[3] = s * 0.5f;
		s = 0.5f / s;
		temp[0] = (trans[1].z - trans[2].y) * s;
		temp[1] = (trans[2].x - trans[0].z) * s;
		temp[2] = (trans[0].y - trans[1].x) * s;
	}
	else
	{
			typedef float btMatrRow[4];
			btMatrRow* m_el = (btMatrRow*)trans;
			int i = m_el[0][0] < m_el[1][1] ? 
				(m_el[1][1] < m_el[2][2] ? 2 : 1) :
				(m_el[0][0] < m_el[2][2] ? 2 : 0); 
			int j = (i + 1) % 3;  
			int k = (i + 2) % 3;
			float s = sqrtf(m_el[i][i] - m_el[j][j] - m_el[k][k] + 1.0f);
			temp[i] = s * 0.5f;
			s = 0.5f / s;
			temp[3] = (m_el[j][k] - m_el[k][j]) * s;
			temp[j] = (m_el[i][j] + m_el[j][i]) * s;
			temp[k] = (m_el[i][k] + m_el[k][i]) * s;
	}
	float4 q = BT_GPU_make_float44(temp[0],temp[1],temp[2],temp[3]);
	return q;
} // getRotation()

//----------------------------------------------------------------------------------------

BT_GPU___device__ float4 quatMult(float4& q1, float4& q2)
{
	return BT_GPU_make_float44(q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
		q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z,
		q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x,
		q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z); 
} // quatMult()

//----------------------------------------------------------------------------------------

BT_GPU___device__ void quatNorm(float4& q)
{
	float len = sqrtf(BT_GPU_dot4(q, q));
	q *= 1.f / len;
} // quatNorm()

//----------------------------------------------------------------------------------------

BT_GPU___device__ void setRotation(float4& q, float4* trans) 
{
	float d = BT_GPU_dot4(q, q);
	float s = 2.0f / d;
	float xs = q.x * s,   ys = q.y * s,   zs = q.z * s;
	float wx = q.w * xs,  wy = q.w * ys,  wz = q.w * zs;
	float xx = q.x * xs,  xy = q.x * ys,  xz = q.x * zs;
	float yy = q.y * ys,  yz = q.y * zs,  zz = q.z * zs;
    trans[0].x = 1.0f - (yy + zz);
	trans[1].x = xy - wz;
	trans[2].x = xz + wy;
	trans[0].y = xy + wz;
	trans[1].y = 1.0f - (xx + zz);
	trans[2].y = yz - wx;
	trans[0].z = xz - wy;
	trans[1].z = yz + wx;
	trans[2].z = 1.0f - (xx + yy);
	trans[0].w = trans[1].w = trans[2].w = 0.0f;
} // setRotation()

//----------------------------------------------------------------------------------------

BT_GPU___global__ void integrTransD(float4* pTrans, float4* pVel, float4* pAngVel, float timeStep, unsigned int numBodies)
{
    int index = BT_GPU___mul24(BT_GPU_blockIdx.x, BT_GPU_blockDim.x) + BT_GPU_threadIdx.x;
    if(index >= (int)numBodies)
	{
		return;
	}
	float4 pos = pTrans[index * 4 + 3];
	float4 linvel = pVel[index];
	pos += linvel * timeStep;

	float4 axis;
	float4 angvel = pAngVel[index];
	float fAngle = sqrtf(BT_GPU_dot(angvel, angvel));
	//limit the angular motion
	if(fAngle*timeStep > BT_GPU__ANGULAR_MOTION_THRESHOLD)
	{
		fAngle = BT_GPU__ANGULAR_MOTION_THRESHOLD / timeStep;
	}
	if(fAngle < 0.001f)
	{
		// use Taylor's expansions of sync function
		axis = angvel * (0.5f*timeStep-(timeStep*timeStep*timeStep)*0.020833333333f * fAngle * fAngle);
	}
	else
	{
		// sync(fAngle) = sin(c*fAngle)/t
		axis = angvel * ( sinf(0.5f * fAngle * timeStep) / fAngle);
	}
	float4 dorn = axis;
	dorn.w = cosf(fAngle * timeStep * 0.5f);
	float4 orn0 = getRotation(pTrans + index * 4);
	float4 predictedOrn = quatMult(dorn, orn0);
	quatNorm(predictedOrn);
	setRotation(predictedOrn, pTrans + index * 4);
	pTrans[index * 4 + 3] = pos;
} // integrTransD()


//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------

// global functions

extern "C"
{

//----------------------------------------------------------------------------------------

void BT_GPU_PREF(integrVel(float* pForceTorqueDamp, float* pInvInertiaMass, void* pVel, void* pAngVel, float timeStep, unsigned int numBodies))
{
    int numThreads, numBlocks;
    BT_GPU_PREF(computeGridSize)(numBodies, 256, numBlocks, numThreads);
    BT_GPU_EXECKERNEL(numBlocks, numThreads, integrVelD, ((float4*)pForceTorqueDamp, (float4*)pInvInertiaMass, (float4*)pVel, (float4*)pAngVel, timeStep, numBodies));
    BT_GPU_CHECK_ERROR("Kernel execution failed: btCuda_integrVelD");
} // integrVel()

//----------------------------------------------------------------------------------------

void BT_GPU_PREF(integrTrans(void* trans, void* vel, void* angVel, float timeStep, int numBodies))
{
    int numThreads, numBlocks;
    BT_GPU_PREF(computeGridSize)(numBodies, 256, numBlocks, numThreads);
    BT_GPU_EXECKERNEL(numBlocks, numThreads, integrTransD, ((float4*)trans, (float4*)vel, (float4*)angVel, timeStep, numBodies));
    BT_GPU_CHECK_ERROR("Kernel execution failed: btCuda_integrTransD");
} // integrTrans()

//----------------------------------------------------------------------------------------

} // extern "C"

//------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
