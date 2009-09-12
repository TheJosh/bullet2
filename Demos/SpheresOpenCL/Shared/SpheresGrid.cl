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

#if !defined(GUID_ARG)
	#define GUID_ARG 
#endif

__kernel void kPredictUnconstrainedMotion(	int numObjects,
											__global float4* pLinVel, 
											__global float4* pAngVel, 
											__global float4* pParams, 
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
		float4 linVel = pLinVel[index];
		float4 gravity = pParams[0];
		linVel += gravity * timeStep;
		pLinVel[index] = linVel;
	}
}

int4 getGridPos(float4 worldPos, __global float4* pParams)
{
    int4 gridPos;
    gridPos.x = (int)floor((worldPos.x - pParams[1].x) / pParams[2].x);
    gridPos.y = (int)floor((worldPos.y - pParams[1].y) / pParams[2].y);
    gridPos.z = (int)floor((worldPos.z - pParams[1].z) / pParams[2].z);
    return gridPos;
}

unsigned int getPosHash(int4 gridPos, __global float4* pParams)
{
	int4 pGridDim = *((__global int4*)(pParams + 3));
	if(gridPos.x < 0) gridPos.x = 0;
	if(gridPos.x >= pGridDim.x) gridPos.x = pGridDim.x - 1;
	if(gridPos.y < 0) gridPos.y = 0;
	if(gridPos.y >= pGridDim.y) gridPos.y = pGridDim.y - 1;
	if(gridPos.z < 0) gridPos.z = 0;
	if(gridPos.z >= pGridDim.z) gridPos.z = pGridDim.z - 1;
	unsigned int hash = gridPos.z * pGridDim.y * pGridDim.x + gridPos.y * pGridDim.x + gridPos.x;
	return hash;
} 


__kernel void kSetSpheres(	int numSpheres, 
							__global float4* pPos, 
							__global float4* pTrans,
							__global float4* pShapeBuf,
							__global int* pBodyIds,
							__global int2* pPosHash,
							__global float4* pParams GUID_ARG)
{
    int index = get_global_id(0);
    if(index >= numSpheres)
    {
		return;
    }
    int objId = pBodyIds[index];

	float4 ai =	pTrans[objId * 4 + 0];
	float4 aj =	pTrans[objId * 4 + 1];
	float4 ak =	pTrans[objId * 4 + 2];
	float4 pos = pTrans[objId * 4 + 3];
	float4 shape = pShapeBuf[index];
	pos += ai * shape.x;
	pos += aj * shape.y;
	pos += ak * shape.z;
	pos.w = 1.0f;
	pPos[index] = pos;
	int4 gridPos = getGridPos(pos, pParams);
	unsigned int hash = getPosHash(gridPos, pParams);
	pPosHash[index].x = hash;
	pPosHash[index].y = index;
}

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


#define BT_GPU_ANGULAR_MOTION_THRESHOLD (0.25f * 3.1415926f)

__kernel void kIntegrateTransforms(	int numObjects,
									__global float4* pLinVel, 
									__global float4* pAngVel, 
									__global float4* pParams, 
									__global float4* pTrans,
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



void findPairsInCell(	int4 gridPos,
						int index,
						float4 posA,
						__global float4* pPos, 
						__global int2*  pHash,
						__global int*   pCellStart,
						__global float4* pShapeBuff, 
						__global int* pBodyIds,
						__global int*   pPairBuff,
						__global int2*	pPairBuffStartCurr,
						__global float4* pParams)
{
	int4 pGridDim = *((__global int4*)(pParams + 3));
    if (	(gridPos.x < 0) || (gridPos.x > pGridDim.x - 1)
		||	(gridPos.y < 0) || (gridPos.y > pGridDim.y - 1)
		||  (gridPos.z < 0) || (gridPos.z > pGridDim.z - 1)) 
    {
		return;
	}
    int gridHash = getPosHash(gridPos, pParams);
    // get start of bucket for this cell
    int bucketStart = pCellStart[gridHash];
    if (bucketStart == -1)
	{
        return;   // cell empty
	}
	// iterate over spheres in this cell
    int2 sortedData = pHash[index];
	int unsorted_indx = sortedData.y;
	int bodyIdA = pBodyIds[unsorted_indx];
	int2 start_curr = pPairBuffStartCurr[unsorted_indx];
	int start = start_curr.x;
	int curr = start_curr.y;
//	int2 start_curr_next = pPairBuffStartCurr[unsorted_indx+1];
	int bucketEnd = bucketStart + 8;
	for(int index2 = bucketStart; index2 < bucketEnd; index2++) 
	{
        int2 cellData = pHash[index2];
        if (cellData.x != gridHash)
        {
			break;   // no longer in same bucket
		}
		int unsorted_indx2 = cellData.y;
		int bodyIdB = pBodyIds[unsorted_indx2];
        if((bodyIdB != bodyIdA) && (unsorted_indx2 > unsorted_indx)) // check not colliding with self
        {   
			float4 posB = pPos[unsorted_indx2];
			posB.w = pShapeBuff[unsorted_indx2].w;
			float4 del = posB - posA;
			float dist2 = del.x * del.x + del.y * del.y + del.z * del.z;
			float rad2 = posB.w + posA.w;
			rad2 = rad2 * rad2;
			if((dist2 < rad2) && (curr < 12))
			{
				pPairBuff[start+curr] = unsorted_indx2;
				curr++;
			}
		}
	}
	pPairBuffStartCurr[unsorted_indx].y = curr;
    return;
}




__kernel void kBroadphaseCD(int numSpheres,
							__global float4* pPos, 
							__global float4* pShapeBuf,
							__global int* pBodyIds,
							__global int2* pHash,
							__global int* pCellStart,
							__global int* pPairBuff,
							__global int2* pPairBuffStartCurr,
							__global float4* pParams GUID_ARG)
{
    int index = get_global_id(0);
    if(index >= numSpheres)
	{
		return;
	}
    int2 sortedData = pHash[index];
	int unsorted_indx = sortedData.y;
	// clear pair buffer 
	pPairBuffStartCurr[unsorted_indx].y = 0;
	//
	// DEBUG : COULD BE REMOVED LATER
	//
	int buf_start_indx = pPairBuffStartCurr[unsorted_indx].x;
	int buf_sz = pPairBuffStartCurr[unsorted_indx+1].x - buf_start_indx;
	for(int i = 0; i < buf_sz; i++)
	{
		pPairBuff[buf_start_indx + i] = -1;
	}
    // get address in grid
	float4 pos = pPos[unsorted_indx];
    int4 gridPosA = getGridPos(pos, pParams);

	int4 pGridDim = *((__global int4*)(pParams + 3));
    if (	(gridPosA.x < 0) || (gridPosA.x > pGridDim.x - 1)
		||	(gridPosA.y < 0) || (gridPosA.y > pGridDim.y - 1)
		||  (gridPosA.z < 0) || (gridPosA.z > pGridDim.z - 1)) 
    {
		return;
	}
    
    pos.w = pShapeBuf[unsorted_indx].w;
    // examine only neighbouring cells
    int4 gridPosB; 
    for(int z=-1; z<=1; z++) 
    {
		gridPosB.z = gridPosA.z + z;
        for(int y=-1; y<=1; y++) 
        {
			gridPosB.y = gridPosA.y + y;
            for(int x=-1; x<=1; x++) 
            {
				gridPosB.x = gridPosA.x + x;
                findPairsInCell(gridPosB, index, pos, pPos, pHash, pCellStart, pShapeBuf, pBodyIds, pPairBuff, pPairBuffStartCurr, pParams);
            }
        }
    }
}


/*
struct btPairId
{
	int m_objA;	//x
	int m_objB;	//y
	int m_sphA;	//z
	int m_sphB;	//w
	int m_batch;//x
	int m_pair;	//y
	int m_pad[2];
};
struct btSpheresContPair
{
	btVector3 m_contact; // + penetration in w
	btVector3 m_normal;  // + impulse accumulator in w
};
*/

__kernel void kSetupContacts(	int numPairs,
								__global int4* pPairIds, 
								__global float4* pPos,
								__global float4* pShapeBuf,
								__global float4* pContacts,
								__global float4* pParams GUID_ARG)
{
    int index = get_global_id(0);
    if(index >= numPairs)
    {
		return;
    }
	int sphIdA = pPairIds[index * 2].z;
	int sphIdB = pPairIds[index * 2].w;
	float4 posA = pPos[sphIdA];
	float4 posB = pPos[sphIdB];
	float radA = pShapeBuf[sphIdA].w;
	float radB = pShapeBuf[sphIdB].w;
	float4 del = posB - posA;
	float dist = dot(del, del);
	dist = native_sqrt(dist);
	float maxD = radA + radB;
	if(dist > maxD)
	{ // should never happen
		return;
	}
	float penetration = maxD - dist;
	float4 normal;
	if(dist > 0.f) 
	{
		float fact = -1.0f / dist;
		normal = del * fact; 
	}
	else
	{	
		normal.x = 1.f; normal.y = normal.z = 0.f;
	}
	float4 tmp = normal * radA;
	float4 contact = posA - tmp;
	contact.w = penetration;
	normal.w = 0.f;
	pContacts[index * 2 + 0] = contact;
	pContacts[index * 2 + 1] = normal;
}


float computeImpulse(float4 relVel, float penetration, float4 normal, float timeStep)
{
	float collisionConstant	=	0.1f;
	float baumgarteConstant	=	0.1f;
	float penetrationError	=	0.02f;

	float lambdaDt = 0.f;

	if(penetration >= 0.f)
	{
		return lambdaDt;
	}

	penetration = min(0.0f, penetration + penetrationError);
	lambdaDt	= - dot(normal,relVel) * collisionConstant;
	lambdaDt	-=	(baumgarteConstant/timeStep * penetration);
	return lambdaDt;
}

__kernel void kSolveConstraints(int numPairs,
								__global float4* pPair,
								int batchNum,
								__global float4* pTrans,
								__global int4* pPairIds, 
								__global float4* pLinVel,
								__global float4* pAngVel,
								__global float4* pInvInertiaMass,
								__global float4* pParams,
								float timeStep GUID_ARG)
{
    int index = get_global_id(0);
    if(index >= numPairs)
    {
		return;
    }
    int batchId = pPairIds[index * 2 + 1].x;
    if(batchId != batchNum)
	{
		return;
	}
	int objIdA = pPairIds[index * 2].x;
	int objIdB = pPairIds[index * 2].y;
	float4 posA = pTrans[objIdA * 4 + 3];
	float4 posB = pTrans[objIdB * 4 + 3];
	float4 linVelA = pLinVel[objIdA];
	float4 linVelB = pLinVel[objIdB];
	float4 angVelA = pAngVel[objIdA];
	float4 angVelB = pAngVel[objIdB];
	float4 contPointA = pPair[index * 2 + 0] - posA;
	float4 contPointB = pPair[index * 2 + 0] - posB;
	float penetration = pPair[index * 2 + 0].w;
	if(penetration > 0.f)
	{
		float4 contNormal = pPair[index * 2 + 1];
		float4 velA = linVelA + cross(angVelA,contPointA);
		float4 velB = linVelB + cross(angVelB,contPointB);
		float4 relVel = velA - velB;
		float lambdaDt = computeImpulse(relVel, -penetration, contNormal, timeStep);
		float rLambdaDt = contNormal.w;
		float pLambdaDt = rLambdaDt;
		rLambdaDt = max(rLambdaDt + lambdaDt, 0.f);
		lambdaDt = rLambdaDt - pLambdaDt;
		pPair[index * 2 + 1].w = rLambdaDt;
		float4 impulse = contNormal * lambdaDt * 0.5f;
		float invMassA = pInvInertiaMass[objIdA * 3 + 0].w;
		float invMassB = pInvInertiaMass[objIdB * 3 + 0].w;
		if(invMassA > 0.f)
		{
			linVelA += impulse;
			angVelA += cross(contPointA,impulse);
//			linVelA.z = linVelA.w = 0.f;
//			angVelA.x = angVelA.y = angVelA.w = 0.f;
			linVelA.w = 0.f;
			angVelA.w = 0.f;
			pLinVel[objIdA] = linVelA;
			pAngVel[objIdA] = angVelA;
		}
		if(invMassB > 0.f)
		{
			linVelB -= impulse;
			angVelB -= cross(contPointB,impulse);
//			linVelB.z = linVelB.w = 0.f;
//			angVelB.x = angVelB.y = angVelB.w = 0.f;
			linVelB.w = 0.f;
			angVelB.w = 0.f;
			pLinVel[objIdB] = linVelB;
			pAngVel[objIdB] = angVelB;
		}
	}
}

__kernel void kInitObjUsageTab(	int numObjects,
								__global int* pObjUsed, 
								__global float4* pInvInertiaMass,
								__global float4* pParams GUID_ARG)
{
    int index = get_global_id(0);
    if(index >= numObjects)
    {
		return;
    }
#if 1
	// allow share static objects in one batch
	float invMass = pInvInertiaMass[index * 3 + 0].w;
	if(invMass > 0.f)
	{
		pObjUsed[index] = -1;
	}
	else
	{
		pObjUsed[index] = -2;
	}
#else
	// do not share static objects in one batch
		pObjUsed[index] = -1;
#endif	
}

__kernel void kSetupBatches(int numPairs,
							__global int4* pPairIds, 
							__global int* pObjUsed, 
							__global float4* pParams GUID_ARG)
{
    int index = get_global_id(0);
    if(index >= numPairs)
    {
		return;
    }
    int currPair = index;
	int objIdA = pPairIds[currPair * 2].x;
	int objIdB = pPairIds[currPair * 2].y;
	int batchId = pPairIds[currPair * 2 + 1].x;
	int localWorkSz = get_local_size(0);
	int localIdx = get_local_id(0);
	for(int i = 0; i < localWorkSz; i++)
	{
		if((i == localIdx) // so work item with lower local ID has priority to write
		&&(batchId < 0)
		&&(pObjUsed[objIdA] < 0)
		&&(pObjUsed[objIdB] < 0))
		{
			if(pObjUsed[objIdA] == -1) 
			{
				pObjUsed[objIdA] = index;
			}
			if(pObjUsed[objIdB] == -1) 
			{
				pObjUsed[objIdB] = index;
			}
		}
		barrier(CLK_GLOBAL_MEM_FENCE);
	}
}

__kernel void kCheckBatches(int numPairs,
							__global int4* pPairIds, 
							__global int* pObjUsed, 
							__global float4* pParams,
							int numBatches,
							int batchNum GUID_ARG)
{
    int index = get_global_id(0);
    if(index >= numPairs)
    {
		return;
    }
    int currPair = index;
	int objIdA = pPairIds[currPair * 2].x;
	int objIdB = pPairIds[currPair * 2].y;
	int batchId = pPairIds[currPair * 2 + 1].x;
    if(batchId < 0)
    {
		int objA_OK = 0;
		if(pObjUsed[objIdA] == -2) 
		{
			objA_OK = 1;
		}
		else
		{
			if(pObjUsed[objIdA] == currPair) 
			{
				objA_OK = 1;
			}
			else
			{
				objA_OK = 0;
			}
		}
		int objB_OK = 0;
		if(pObjUsed[objIdB] == -2) 
		{
			objB_OK = 1;
		}
		else
		{
			if(pObjUsed[objIdB] == currPair) 
			{
				objB_OK = 1;
			}
			else
			{
				objB_OK = 0;
			}
		}
		if((objA_OK && objB_OK) || (batchNum == (numBatches - 1)))
		{
			pPairIds[currPair * 2 + 1].x = batchNum;
		}
    }
}

void arrow(__global int2* a, __global int2* b, unsigned int dir)
{
	if((a[0].x > b[0].x) == dir)
	{
		int2 tmp = *a;
		*a = *b;
		*b = tmp;
	}
}

__kernel void kBitonicSortHash(	__global int2* pHash,
								unsigned int numBatches,
								unsigned int dir GUID_ARG)
{
	unsigned int globSize = numBatches * get_local_size(0);

    for(unsigned int size = 2; size <= 2 * globSize; size *= 2)
    {
        for(unsigned int stride = size / 2; stride > 0; stride >>= 1)
        {
			for(unsigned int batch = 0; batch < numBatches; batch++)
			{
				unsigned int globId = get_local_id(0) + batch * get_local_size(0);
				unsigned int   pos = 2 * globId - (globId & (stride - 1));
				unsigned int    dd = dir ^ ((pos & size) != 0);
				arrow(&pHash[pos], &pHash[pos + stride], dd);
			}
            barrier(CLK_LOCAL_MEM_FENCE | CLK_GLOBAL_MEM_FENCE);
        }
    }
    barrier(CLK_LOCAL_MEM_FENCE | CLK_GLOBAL_MEM_FENCE);
}



/*
 * Copyright 1993-2009 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property and 
 * proprietary rights in and to this software and related documentation. 
 * Any use, reproduction, disclosure, or distribution of this software 
 * and related documentation without an express license agreement from
 * NVIDIA Corporation is strictly prohibited.
 *
 * Please refer to the applicable NVIDIA end user license agreement (EULA) 
 * associated with this source code for terms and conditions that govern 
 * your use of this NVIDIA software.
 * 
 */

#define LOCAL_SIZE_LIMIT 1024U

inline void ComparatorPrivate(int2* keyA, int2* keyB, uint dir)
{
    if((keyA[0].x > keyB[0].x) == dir)
    {
		int2 tmp = *keyA;
		*keyA = *keyB;
		*keyB = tmp;
    }
}

inline void ComparatorLocal(__local int2* keyA, __local int2* keyB, uint dir)
{
    if((keyA[0].x > keyB[0].x) == dir)
    {
		int2 tmp = *keyA;
		*keyA = *keyB;
		*keyB = tmp;
    }
}

////////////////////////////////////////////////////////////////////////////////
// Monolithic bitonic sort kernel for short arrays fitting into local memory
////////////////////////////////////////////////////////////////////////////////
__kernel void bitonicSortLocal(__global int2* pKey, uint arrayLength, uint dir GUID_ARG)
{
    __local int2 l_key[LOCAL_SIZE_LIMIT];

    //Offset to the beginning of subbatch and load data
    pKey += get_group_id(0) * LOCAL_SIZE_LIMIT + get_local_id(0);
    l_key[get_local_id(0) +                      0] = pKey[                     0];
    l_key[get_local_id(0) + (LOCAL_SIZE_LIMIT / 2)] = pKey[(LOCAL_SIZE_LIMIT / 2)];

    for(uint size = 2; size < arrayLength; size <<= 1)
    {
        //Bitonic merge
        uint ddd = dir ^ ( (get_local_id(0) & (size / 2)) != 0 );
        for(uint stride = size / 2; stride > 0; stride >>= 1)
        {
            barrier(CLK_LOCAL_MEM_FENCE);
            uint pos = 2 * get_local_id(0) - (get_local_id(0) & (stride - 1));
            ComparatorLocal(&l_key[pos +      0], &l_key[pos + stride], ddd);
        }
    }

    //ddd == dir for the last bitonic merge step
    {
        for(uint stride = arrayLength / 2; stride > 0; stride >>= 1)
        {
            barrier(CLK_LOCAL_MEM_FENCE);
            uint pos = 2 * get_local_id(0) - (get_local_id(0) & (stride - 1));
            ComparatorLocal(&l_key[pos + 0], &l_key[pos + stride], dir);
        }
    }

    barrier(CLK_LOCAL_MEM_FENCE);
    pKey[                     0] = l_key[get_local_id(0) +                      0];
    pKey[(LOCAL_SIZE_LIMIT / 2)] = l_key[get_local_id(0) + (LOCAL_SIZE_LIMIT / 2)];
}

////////////////////////////////////////////////////////////////////////////////
// Bitonic sort kernel for large arrays (not fitting into local memory)
////////////////////////////////////////////////////////////////////////////////
//Bottom-level bitonic sort
//Almost the same as bitonicSortLocal with the only exception
//of even / odd subarrays (of LOCAL_SIZE_LIMIT points) being
//sorted in opposite directions
__kernel void bitonicSortLocal1(__global int2* pKey GUID_ARG)
{
    __local int2 l_key[LOCAL_SIZE_LIMIT];

    //Offset to the beginning of subarray and load data
    pKey += get_group_id(0) * LOCAL_SIZE_LIMIT + get_local_id(0);
    l_key[get_local_id(0) +                      0] = pKey[                     0];
    l_key[get_local_id(0) + (LOCAL_SIZE_LIMIT / 2)] = pKey[(LOCAL_SIZE_LIMIT / 2)];

    uint comparatorI = get_global_id(0) & ((LOCAL_SIZE_LIMIT / 2) - 1);

    for(uint size = 2; size < LOCAL_SIZE_LIMIT; size <<= 1)
    {
        //Bitonic merge
        uint ddd = (comparatorI & (size / 2)) != 0;
        for(uint stride = size / 2; stride > 0; stride >>= 1)
        {
            barrier(CLK_LOCAL_MEM_FENCE);
            uint pos = 2 * get_local_id(0) - (get_local_id(0) & (stride - 1));
            ComparatorLocal(&l_key[pos + 0], &l_key[pos + stride], ddd);
        }
    }

    //Odd / even arrays of LOCAL_SIZE_LIMIT elements
    //sorted in opposite directions
    {
        uint ddd = (get_group_id(0) & 1);
        for(uint stride = LOCAL_SIZE_LIMIT / 2; stride > 0; stride >>= 1)
        {
            barrier(CLK_LOCAL_MEM_FENCE);
            uint pos = 2 * get_local_id(0) - (get_local_id(0) & (stride - 1));
            ComparatorLocal(&l_key[pos + 0], &l_key[pos + stride], ddd);
        }
    }

    barrier(CLK_LOCAL_MEM_FENCE);
    pKey[                     0] = l_key[get_local_id(0) +                      0];
    pKey[(LOCAL_SIZE_LIMIT / 2)] = l_key[get_local_id(0) + (LOCAL_SIZE_LIMIT / 2)];
}

//Bitonic merge iteration for 'stride' >= LOCAL_SIZE_LIMIT
__kernel void bitonicMergeGlobal(__global int2* pKey, uint arrayLength, uint size, uint stride, uint dir GUID_ARG)
{
    uint global_comparatorI = get_global_id(0);
    uint        comparatorI = global_comparatorI & (arrayLength / 2 - 1);

    //Bitonic merge
    uint ddd = dir ^ ( (comparatorI & (size / 2)) != 0 );
    uint pos = 2 * global_comparatorI - (global_comparatorI & (stride - 1));

    int2 keyA = pKey[pos +      0];
    int2 keyB = pKey[pos + stride];

    ComparatorPrivate(&keyA, &keyB, ddd);

    pKey[pos +      0] = keyA;
    pKey[pos + stride] = keyB;
}

//Combined bitonic merge steps for
//'size' > LOCAL_SIZE_LIMIT and 'stride' = [1 .. LOCAL_SIZE_LIMIT / 2]
__kernel void bitonicMergeLocal(__global int2* pKey, uint arrayLength, uint stride, uint size, uint dir GUID_ARG)
{
    __local int2 l_key[LOCAL_SIZE_LIMIT];

    pKey += get_group_id(0) * LOCAL_SIZE_LIMIT + get_local_id(0);
    l_key[get_local_id(0) +                      0] = pKey[                     0];
    l_key[get_local_id(0) + (LOCAL_SIZE_LIMIT / 2)] = pKey[(LOCAL_SIZE_LIMIT / 2)];

    //Bitonic merge
    uint comparatorI = get_global_id(0) & ((arrayLength / 2) - 1);
    uint         ddd = dir ^ ( (comparatorI & (size / 2)) != 0 );
    for(; stride > 0; stride >>= 1)
    {
        barrier(CLK_LOCAL_MEM_FENCE);
        uint pos = 2 * get_local_id(0) - (get_local_id(0) & (stride - 1));
        ComparatorLocal(&l_key[pos + 0], &l_key[pos + stride], ddd);
    }

    barrier(CLK_LOCAL_MEM_FENCE);
    pKey[                     0] = l_key[get_local_id(0) +                      0];
    pKey[(LOCAL_SIZE_LIMIT / 2)] = l_key[get_local_id(0) + (LOCAL_SIZE_LIMIT / 2)];
}

