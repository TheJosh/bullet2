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

#if defined(GUID_ARG)
	extern int gMiniCLNumOutstandingTasks;
#else
	#define GUID_ARG 
#endif


#define USE_FRICTION 1
#define FRICTION_BOX_GROUND_FACT 0.01f
#define FRICTION_BOX_BOX_FACT 0.01f
//#define FRICTION_BOX_BOX_FACT 0.05f
#define USE_CENTERS 1

__kernel void kClearAccumImpulse(	int numConstraints, 
									__global float* lambdaDtBox,
									int numContPoints GUID_ARG)
{
    int index = get_global_id(0);
    if(index >= numConstraints)
	{
		return;
	}
	for(int i=0; i < numContPoints; i++)
		lambdaDtBox[numContPoints * index + i] = 0.f;
}


float computeImpulse3D(	float4 rVel,
						float positionConstraint,
						float4 cNormal,
						float dt)
{
	const float collisionConstant	=	0.1f;
	const float baumgarteConstant	=	0.1f;
	const float penetrationError	=	0.02f;

	float lambdaDt=0;

	if(positionConstraint >= 0.f)
	{
		return lambdaDt;
	}

	positionConstraint = fmin(0.0f, positionConstraint + penetrationError);
	cNormal.w = 0.f;
	lambdaDt =	-dot(cNormal,rVel) * collisionConstant;
	lambdaDt -=	(baumgarteConstant / dt * positionConstraint);
	return lambdaDt;
}


__kernel void kCollisionWithWallBox(	int numObjects,
									__global float4 *trans,
									__global float4 *vel,
									__global float4* angVel,
									__global float4* gProp,
									float dt GUID_ARG)
{
    int idx = get_global_id(0);
	float4 aPos;
	float positionConstraint;
	float4 impulse;
	
	if(idx < numObjects)
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
			float4 aAngVel = angVel[idx];
			float4 vVel	= aVel + cross(aAngVel, rerVertex);
			float restitution = 0.5f;
			{
				positionConstraint  = vPos.y - gProp[0].y;
				impulse	= (float4)(0.0f);
				if(positionConstraint < 0)
				{
					float4 groundNormal = (float4)(0.f, 1.f, 0.f, 0.f);
					impulse	= groundNormal * restitution * computeImpulse3D(vVel, positionConstraint, groundNormal, dt);
#if USE_FRICTION	// only with ground for now
					float4 lat_vel = vVel - groundNormal * dot(groundNormal, vVel);
					float lat_vel_len = dot(lat_vel, lat_vel);
					if (lat_vel_len > 0)
					{
						lat_vel_len = native_sqrt(lat_vel_len);
						lat_vel *= 1.f/lat_vel_len;	
						impulse	-= lat_vel * dot(lat_vel, vVel) * FRICTION_BOX_GROUND_FACT;
					}
#endif //USE_FRICTION
					vel[idx]	+=	impulse;
					angVel[idx]	+=	cross(rerVertex,impulse);
				}
			}
			{
				positionConstraint	= vPos.x - gProp[0].x;
				impulse	= (float4)(0.0f);
				if(positionConstraint < 0)
				{
					float4 normal = (float4)(1.0f, 0.0f, 0.0f, 0.0f);
					impulse	= normal * restitution * computeImpulse3D(vVel,positionConstraint,normal,dt);
					vel[idx]	+=	impulse;
					angVel[idx]	+=	cross(rerVertex, impulse);
				}
			}
			{
				positionConstraint	= gProp[1].x - vPos.x;
				impulse	= (float4)(0.0f);
				if(positionConstraint < 0)
				{
					float4 normal = (float4)(-1.0f, 0.0f, 0.0f, 0.0f);
					impulse	= normal * restitution * computeImpulse3D(vVel,positionConstraint,normal,dt);
					vel[idx]	+=	impulse;
					angVel[idx]	+=	cross(rerVertex, impulse);
				}
			}
			{
				positionConstraint	= vPos.z - gProp[0].z;
				impulse	= (float4)(0.0f);
				if(positionConstraint < 0)
				{
					float4 normal = (float4)(0.0f, 0.0f, 1.0f, 0.0f);
					impulse	= normal * restitution * computeImpulse3D(vVel,positionConstraint,normal,dt);
					vel[idx]	+=	impulse;
					angVel[idx]	+=	cross(rerVertex, impulse);
				}
			}
			{
				positionConstraint	= gProp[1].z - vPos.z;
				impulse	= (float4)(0.0f);
				if(positionConstraint < 0)
				{
					float4 normal = (float4)(0.0f, 0.0f, -1.0f, 0.0f);
					impulse	= normal * restitution * computeImpulse3D(vVel,positionConstraint,normal,dt);
					vel[idx]	+=	impulse;
					angVel[idx]	+=	cross(rerVertex, impulse);
				}
			}
		}
	}
}


__kernel void kSolveConstraint(	int nConstraints,
								__global int2* constraints,
								__global int* batch,
								__global float4* trans,
								__global float4* vel,
								__global float4* angularVel,
								__global float* lambdaDtBox,
								__global float* iPositionConstraint,
								__global float4 *normal,
								__global float4* contact,
								int iBatch,
								int batchIdOffset,
								float dt GUID_ARG)
{
	float4 relVel;
	float4 impulse;
	float lambdaDt;
	float positionConstraint;
    int k_idx = get_global_id(0);
	if(k_idx < nConstraints)
	{
		int idx = batch[batchIdOffset + k_idx];
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
				relVel = (aVel + cross(aAngVel, contactPoint))
				 -(bVel + cross(bAngVel, contactPoint + aPos - bPos));

				lambdaDt=	computeImpulse3D(relVel, -positionConstraint, contactNormal, dt);
				{
					float rLambdaDt=lambdaDtBox[idx * 4 + iVtx];
					float pLambdaDt=rLambdaDt;
					rLambdaDt = fmax(pLambdaDt+lambdaDt, 0.0f);
					lambdaDt=rLambdaDt-pLambdaDt;
					lambdaDtBox[idx * 4 + iVtx]=rLambdaDt;
				}
				impulse = contactNormal * lambdaDt * 0.5f;
#if USE_FRICTION
				float4 lat_vel = relVel - contactNormal * dot(contactNormal, relVel);
				float lat_vel_len = dot(lat_vel, lat_vel);
				if (lat_vel_len > 0)
				{
					lat_vel_len = native_sqrt(lat_vel_len);
					lat_vel *= 1.f/lat_vel_len;
					impulse	-= lat_vel * dot(lat_vel , relVel) * FRICTION_BOX_BOX_FACT;
				}
#endif //USE_FRICTION
				aVel+=	impulse;
				bVel-=	impulse;
				aAngVel += cross(contactPoint, impulse);
				bAngVel -= cross(contactPoint + aPos - bPos, impulse);
			}
		}
		vel[aId] = aVel;
		vel[bId] = bVel;
		angularVel[aId] = aAngVel;
		angularVel[bId] = bAngVel;
	}
}


// kernel functions

__kernel void kIntegrateVelocities(	int numObjects,
									__global float4* pForceTorqueDamp, 
									__global float4* pInvInertiaMass, 
									__global float4* pVel, 
									__global float4* pAngVel, 
									float timeStep GUID_ARG )
{
    int index = get_global_id(0);
    if(index >= numObjects)
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
	in_mass_0.w = 0;
	float4 in_mass_1 = pInvInertiaMass[index * 3 + 1];
	in_mass_1.w = 0;
	float4 in_mass_2 = pInvInertiaMass[index * 3 + 2];
	in_mass_2.w = 0;
	// integrate linear velocity
	float4 outLinVel, outAngVel;
	outLinVel = linVel + force * mass * timeStep;
	// integrate angular velocity
	outAngVel.x = dot(in_mass_0, torque);
	outAngVel.y = dot(in_mass_1, torque);
	outAngVel.z = dot(in_mass_2, torque);
	outAngVel.w = 0.f;
	outAngVel += angVel;
	/// clamp angular velocity. collision calculations will fail on higher angular velocities	
	#if(!defined(M_PI))
	#define M_PI 3.1415926f
	#endif
	#define BT_CUDA_MAX_SQ_ANGVEL (M_PI*M_PI)
	float sq_angvel = dot(outAngVel, outAngVel);
	sq_angvel *= timeStep * timeStep;
	float fact;
	if(sq_angvel > BT_CUDA_MAX_SQ_ANGVEL)
	{
		fact = native_sqrt(BT_CUDA_MAX_SQ_ANGVEL/sq_angvel) / timeStep;
		outAngVel *= fact;
	}
	// now apply damping
	fact = native_powr(1.0f - lin_damp, timeStep);
	outLinVel *= fact;
	fact = native_powr(1.0f - ang_damp, timeStep);
	outAngVel *= fact;
	// pack results
	pVel[index] = outLinVel;
	pAngVel[index] = outAngVel;
} // integrVelD()

#define BT_GPU_ANGULAR_MOTION_THRESHOLD (0.25f * M_PI)


float4 getRotation(__global float4* trans)
{
	float trace = trans[0].x + trans[1].y + trans[2].z;
	float temp[4];
	if(trace > 0.0f)
	{
		float s = native_sqrt(trace + 1.0f);
		temp[3] = s * 0.5f;
		s = 0.5f / s;
		temp[0] = (trans[1].z - trans[2].y) * s;
		temp[1] = (trans[2].x - trans[0].z) * s;
		temp[2] = (trans[0].y - trans[1].x) * s;
	}
	else
	{
		typedef float btMatrRow[4];
		__global btMatrRow* m_el = (__global btMatrRow*)trans;
		int i = m_el[0][0] < m_el[1][1] ? 
			(m_el[1][1] < m_el[2][2] ? 2 : 1) :
			(m_el[0][0] < m_el[2][2] ? 2 : 0); 
		int j = (i + 1) % 3;  
		int k = (i + 2) % 3;
		float s = native_sqrt(m_el[i][i] - m_el[j][j] - m_el[k][k] + 1.0f);
		temp[i] = s * 0.5f;
		s = 0.5f / s;
		temp[3] = (m_el[j][k] - m_el[k][j]) * s;
		temp[j] = (m_el[i][j] + m_el[j][i]) * s;
		temp[k] = (m_el[i][k] + m_el[k][i]) * s;
	}
	float4 q;
	q.x = temp[0];
	q.y = temp[1];
	q.z = temp[2];
	q.w = temp[3];
	return q;
}

float4 quatMult(float4 q1, float4 q2)
{
	float4 q;
	q.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
	q.y = q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z;
	q.z = q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x;
	q.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z; 
	return q;
}

float4 quatNorm(float4 q)
{
	float len = native_sqrt(dot(q, q));
	if(len > 0.f)
	{
		q *= 1.f / len;
	}
	else
	{
		q.x = q.y = q.z = 0.f;
		q.w = 1.f;
	}
	return q;
}

void setRotation(float4 q, __global float4* trans) 
{
	float d = dot(q, q);
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
}

__kernel void kIntegrateTransforms(	int numObjects,
									__global float4* pTrans,
									__global float4* pLinVel, 
									__global float4* pAngVel, 
									__global float4* pInvInertiaMass, 
									float timeStep GUID_ARG)
{
    int index = get_global_id(0);
    if(index >= numObjects)
    {
		return;
	}
	float4 mass0 =	pInvInertiaMass[index * 3 + 0];
    if(mass0.w > 0.f)
	{
		float4 pos = pTrans[index * 4 + 3];
		float4 linVel = pLinVel[index];
		pos += linVel * timeStep;
		float4 axis;
		float4 angvel = pAngVel[index];
		float fAngle = native_sqrt(dot(angvel, angvel));
		//limit the angular motion
		if(fAngle*timeStep > BT_GPU_ANGULAR_MOTION_THRESHOLD)
		{
			fAngle = BT_GPU_ANGULAR_MOTION_THRESHOLD / timeStep;
		}
		if(fAngle < 0.001f)
		{
			// use Taylor's expansions of sync function
			axis = angvel * (0.5f*timeStep-(timeStep*timeStep*timeStep)*0.020833333333f * fAngle * fAngle);
		}
		else
		{
			// sync(fAngle) = sin(c*fAngle)/t
			axis = angvel * ( native_sin(0.5f * fAngle * timeStep) / fAngle);
		}
		float4 dorn = axis;
		dorn.w = native_cos(fAngle * timeStep * 0.5f);
		float4 orn0 = getRotation(pTrans + index * 4);
		float4 predictedOrn = quatMult(dorn, orn0);
		predictedOrn = quatNorm(predictedOrn);
		setRotation(predictedOrn, pTrans + index * 4);
		pTrans[index * 4 + 3] = pos;
	}
}


