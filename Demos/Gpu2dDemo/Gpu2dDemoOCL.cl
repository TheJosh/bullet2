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
#define FRICTION_BOX_GROUND_FACT 0.05f
#define FRICTION_BOX_BOX_FACT 0.05f
#define USE_CENTERS 1

//----------   C o n s t r a i n t   s o l v e r    d e m o   ----------------------------

#define MAX_VTX_PER_OBJ 8


__kernel void kClearAccumImpulse(	int numConstraints, 
									__global float* lambdaDtBox, 
									int numContPoints GUID_ARG)
{
    int index = get_global_id(0);
	if(index < numConstraints)
	{
		for(int i=0; i < numContPoints; i++)
			lambdaDtBox[numContPoints * index + i] = 0;
	}
}

#define SPHERE_FACT 1.0f

void testSphSph(float4 aPos, float4 bPos, float radA, float radB, __global float4* pOut)
{
	float4 del = bPos - aPos;
	del.w = 0.f;
	float dist = dot(del, del);
	dist = native_sqrt(dist);
	float maxD = radA + radB;
	
	if(dist > maxD)
	{
		return;
	}
	float penetration = (radA + radB - dist) * SPHERE_FACT;
	float4 normal;
	if(dist > 0.f) 
	{
		float fact = -1.0f/dist;
		normal = del * fact; 
	}
	else
	{
		normal = (float4)0.f; normal.x = 1.f;
	}
	float4 tmp = (normal * radA);
	float4 contact = aPos - tmp;
	
	// now add point
	int numPoints = 0;
	for(int i = 0; i < MAX_VTX_PER_OBJ; i++)
	{
		if(pOut[i*2].w >= 0.f)
		{
			numPoints++;
		}
	}
	if(numPoints < MAX_VTX_PER_OBJ)
	{
		contact.w = penetration;
		pOut[numPoints * 2] = contact;
		pOut[numPoints * 2 + 1] = normal;
	}
} // testSphSph()

#define OBJ_DIAMETER 2.0f

__kernel void kComputeConstraints(int numConstraints,
									__global int2 *constraints,
								   __global float4 *pos,
								   __global float *rotation,
								   __global char* shapes,
								   __global int2* shapeIds,
								   __global float4 *contact GUID_ARG)
{
    int idx = get_global_id(0);
	int aId,bId;
	float4 aPos,bPos;
	float aRot,bRot;
	float sideLength2	=	OBJ_DIAMETER*0.5f/sqrt(2.0f);

	if(idx < numConstraints)
	{
		aId=constraints[idx].x;
		bId=constraints[idx].y;
		
		aPos=pos[aId];
		bPos=pos[bId];
		aRot= rotation[aId];
		bRot= rotation[bId];
		float cosA = native_cos(aRot);
		float sinA = native_sin(aRot);
		float cosB = native_cos(bRot);
		float sinB = native_sin(bRot);
		__global float4* shapeA = (__global float4*)(shapes + shapeIds[aId].x);
		int numSphA = shapeIds[aId].y;
		__global float4* shapeB = (__global float4*)(shapes + shapeIds[bId].x);
		int numSphB = shapeIds[bId].y;
		int i, j;
		float4 ai = (float4)0.f; ai.x =  cosA; ai.y = sinA;
		float4 aj = (float4)0.f; aj.x = -sinA; aj.y = cosA;
		float4 bi = (float4)0.f; bi.x =  cosB; bi.y = sinB;
		float4 bj = (float4)0.f; bj.x = -sinB; bj.y = cosB;
		__global float4* pOut = contact + idx * MAX_VTX_PER_OBJ * 2;
		for(i = 0; i < MAX_VTX_PER_OBJ; i++)
		{
			pOut[i * 2].w = -1.f;
			pOut[i * 2 + 1].w = 0.f;
		}
		for(i = 0; i < numSphA; i++)
		{	
			float4 va = aPos;
			float4 tmp = ai * shapeA[i].x; 
			float4 tmp2 = aj * shapeA[i].y;
			
			va += tmp;
			va += tmp2;
			
			float radA = shapeA[i].w;
			for(j = 0; j < numSphB; j++)
			{
				float4 vb = bPos;
				float4 tmp =bi * shapeB[j].x;
				float4 tmp2 = bj * shapeB[j].y;
				vb += tmp;
				vb += tmp2;
				float radB = shapeB[j].w;
				testSphSph(va, vb, radA, radB, pOut);
			}
		}
	}
}


float computeImpulse1(float4 rVel,
					 float positionConstraint,
					 float4 cNormal,
						 float dt)
{
//	const float collisionConstant	=	0.1f;
//	const float baumgarteConstant	=	0.5f;
//	const float penetrationError	=	0.02f;
	const float collisionConstant	=	-0.1f;
	const float baumgarteConstant	=	0.3f;
	const float penetrationError	=	0.02f;

	float lambdaDt=0;

	if(positionConstraint > 0)
		return lambdaDt;
		
	cNormal.w = 0.f;
	
	positionConstraint = (positionConstraint+penetrationError) < 0.f ? (positionConstraint+penetrationError) : 0.0f;
	
	lambdaDt	=	-dot(cNormal,rVel)*(1+collisionConstant);
	lambdaDt	-=	(baumgarteConstant/dt*positionConstraint);

	return lambdaDt;
}


__kernel void kCollisionWithWallBox( int nParticles,
									__global float4 *pos,
								   __global float4 *vel,
								   __global float *rotation,
								   __global float *angVel,
								   __global char* shapes,
								   __global int2* shapeIds,
								   __global float* invMass,
								   __global float4* gProp,
								   float dt GUID_ARG)
{
    int idx = get_global_id(0);
	float4 aPos;
	float aRot;
	float positionConstraint;
	float4 impulse;
	

	if((idx > 0) && (idx < nParticles))
	{
		float inv_mass = invMass[idx];
		if(inv_mass <= 0.f)
		{
			return;
		}
		aPos=pos[idx];
		aRot=rotation[idx];
		__global float4* shape = (__global float4*)(shapes + shapeIds[idx].x);
		int numSph = shapeIds[idx].y;
		float cosA = native_cos(aRot);
		float sinA = native_sin(aRot);
		float4 ai =	(float4)0.f; ai.x =  cosA; ai.y = sinA;
		float4 aj =	(float4)0.f; aj.x = -sinA; aj.y = cosA;

		for(int iVtx=0;iVtx < numSph; iVtx++){
			float4 aVel = vel[idx]; aVel.w = 0.f;
			float aAngVel = angVel[idx];
			float4 rerVertex = ai * shape[iVtx].x;
			float4 tmp = aj * shape[iVtx].y;
			rerVertex += tmp;
			float4 vPos = aPos + rerVertex;
			float rad = shape[iVtx].w;
			float4 tmpAvel = (float4)0.f; tmpAvel.z = aAngVel;
			float4 vVel	=aVel+cross(tmpAvel, rerVertex);
//			float restitution=1.0;
			float restitution=0.3f;
			{
				positionConstraint	=vPos.y - rad - gProp[0].y;
				impulse				=(float4)0.f;
				if(positionConstraint < 0)
				{
					float4 groundNormal;
					groundNormal = (float4)0.f; groundNormal.y = 1.0f;
					impulse	=groundNormal * restitution * computeImpulse1(vVel, positionConstraint, groundNormal, dt);
#if USE_FRICTION	// only with ground for now
					float4 lat_vel = vVel - groundNormal * dot(groundNormal,vVel);
					float lat_vel_len = dot(lat_vel, lat_vel);
					if (lat_vel_len > 0)
					{
						lat_vel_len = native_sqrt(lat_vel_len);
						lat_vel *= 1.f/lat_vel_len;	
						float4 tmp = lat_vel * dot(lat_vel, vVel) * FRICTION_BOX_GROUND_FACT;
						impulse	-= tmp;
					}
#endif //USE_FRICTION
					vel[idx]	+=	impulse;
					float tmp2 = cross(rerVertex,impulse).z;
					angVel[idx]	+=	tmp2;
				}
			}

			{
				positionConstraint	=vPos.x - rad - gProp[0].x;
				impulse				=(float4)0.f;
				if(positionConstraint < 0)
				{
					float4 norm = (float4)0.f; norm.x = 1.f;
					impulse	= norm * restitution * computeImpulse1(vVel,positionConstraint, norm, dt);
					vel[idx]	+=	impulse;
					angVel[idx]	+=	cross(rerVertex,impulse).z;
				}
			}

			{
				positionConstraint	= gProp[1].x - vPos.x - rad;
				impulse				=(float4)0.f;
				if(positionConstraint < 0)
				{
					float4 norm = (float4)0.f; norm.x = -1.f;
					impulse	= norm * restitution * computeImpulse1(vVel,positionConstraint, norm, dt);
					vel[idx]	+=	impulse;
					angVel[idx]	+=	cross(rerVertex,impulse).z;
				}
			}
		}
	}
}

void collisionResolutionBox(	int constrId,
												__global int2* constraints,
												__global float4 *pos,
												__global float4 *vel,
												__global float *rotation,
												__global float *angularVel,
												__global float *lambdaDtBox,
												__global float4* contact,
												__global float* invMass,
												float dt)
{
#if 1
	float4 relVel;
	float4 impulse;
	float lambdaDt;
	float positionConstraint;
	int aId=constraints[constrId].x;
	int bId=constraints[constrId].y;
	float4 aPos=pos[aId];
	float4 bPos=pos[bId];
	float4 aVel=vel[aId];
	float4 bVel=vel[bId];
	float aAngVel=angularVel[aId];
	float bAngVel=angularVel[bId];
	__global float4* pCont = contact + constrId * MAX_VTX_PER_OBJ * 2;
	//	test Vertices in A to Box B
	for(int iVtx=0;iVtx<MAX_VTX_PER_OBJ;iVtx++){
		float4 contactPoint	= pCont[iVtx * 2];
		contactPoint = contactPoint - aPos;
		positionConstraint = pCont[iVtx * 2].w;
		if(positionConstraint >= 0)
		{
			float4 contactNormal = pCont[iVtx * 2 + 1];
			float4 tmpAvel = (float4)0.f; tmpAvel.z = aAngVel;
			float4 tmpBvel = (float4)0.f; tmpBvel.z = bAngVel;
			relVel=(aVel+cross(tmpAvel, contactPoint))
				-(bVel+cross(tmpBvel, contactPoint+aPos-bPos));
			lambdaDt=	computeImpulse1(relVel,-positionConstraint,
				contactNormal,dt);
			{
				float rLambdaDt=lambdaDtBox[(MAX_VTX_PER_OBJ)*(constrId)+iVtx];
				float pLambdaDt=rLambdaDt;
				rLambdaDt=(pLambdaDt+lambdaDt) > 0.0f ? (pLambdaDt+lambdaDt) : 0.0f;
				lambdaDt=rLambdaDt-pLambdaDt;
				lambdaDtBox[(MAX_VTX_PER_OBJ)*(constrId)+iVtx]=rLambdaDt;
			}
			impulse=	contactNormal*lambdaDt*0.5f;
#if USE_FRICTION
			if(pCont[iVtx * 2 + 1].w <= 0)
			{
				float4 lat_vel = relVel - contactNormal * dot(contactNormal, relVel);
				float lat_vel_len = dot(lat_vel, lat_vel);
				if (lat_vel_len > 0)
				{
					lat_vel_len = native_sqrt(lat_vel_len);
					lat_vel *= 1.f/lat_vel_len;
					float4 tmp = lat_vel * dot(lat_vel , relVel) * FRICTION_BOX_BOX_FACT;
					impulse	-= tmp;
				}
			}
#endif //USE_FRICTION
			if(aId && (invMass[aId] > 0.f))
			{
				aVel+=	impulse;
				aAngVel+=	cross(contactPoint, impulse).z;
			}
			if(bId && (invMass[bId] > 0.f))
			{
				bVel-=	impulse;
				bAngVel-=	cross(contactPoint+aPos-bPos,	impulse).z;
			}
		}
	}
	vel[aId]=aVel;
	vel[bId]=bVel;
	angularVel[aId]=aAngVel;
	angularVel[bId]=bAngVel;
#endif
}

__kernel void kSolveConstraints(		 int nConstraints,
										__global int2 *constraints,
										__global int *batch,
										 __global float4 *pos,
										 __global float4 *vel,
										 __global float *rotation,
										 __global float *angularVel,
										 __global float *lambdaDtBox,
										 __global float4* contact,
										 __global float* invMass,
										 int iBatch,
										 int batchOffs,
										 float dt GUID_ARG)
{
    int k_idx = get_global_id(0);
	if(k_idx < nConstraints)
	{
		int idx = batch[batchOffs + k_idx];
		collisionResolutionBox(	idx, constraints, pos, vel, rotation, angularVel, lambdaDtBox,
								contact, invMass, dt);
	}
}


