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

void testSphSph(float4 aPos, float4 bPos, float radA, float radB, __global float4* pOut GUID_ARG)
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
//	float penetration = (dist - radA - radB) * SPHERE_FACT;
	float4 normal;
	if(dist > 0.f) 
	{
		float fact = -1.0f/dist;
//		float fact = 1.0f/dist;
		normal = del * fact; 
	}
	else
	{
		normal = (float4)(1.f, 0.f, 0.f, 0.f);
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
								   __global float4 *contact)
{
    int idx = get_global_id(0);
	int aId,bId;
	float4 aPos,bPos;
//	float positionConstraint;
	float aRot,bRot;
//	float sideLength2	=	pProp.m_diameter*0.5f/sqrt(2.0f);
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
		float4* shapeA = (float4*)(shapes + shapeIds[aId].x);
		int numSphA = shapeIds[aId].y;
		float4* shapeB = (float4*)(shapes + shapeIds[bId].x);
		int numSphB = shapeIds[bId].y;
		int i, j;
		float4 ai = (float4)(cosA, sinA, 0.f, 0.f);
		float4 aj = (float4)(-sinA, cosA, 0.f, 0.f);
		float4 bi = (float4)(cosB, sinB, 0.f, 0.f);
		float4 bj = (float4)(-sinB, cosB, 0.f, 0.f);
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

//	positionConstraint = btMin(0.0f,positionConstraint+penetrationError);
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
								   float dt)
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
		float4* shape = (float4*)(shapes + shapeIds[idx].x);
		int numSph = shapeIds[idx].y;
		float cosA = native_cos(aRot);
		float sinA = native_sin(aRot);
		float4 ai =	(float4)(cosA, sinA, 0.f, 0.f);
		float4 aj =	(float4)(-sinA, cosA, 0.f, 0.f);

		for(int iVtx=0;iVtx < numSph; iVtx++){
			float4 aVel = (float4)(vel[idx].x, vel[idx].y, vel[idx].z, 0.f);
			float aAngVel = angVel[idx];
			float4 rerVertex = ai * shape[iVtx].x;
			float4 tmp = aj * shape[iVtx].y;
			rerVertex += tmp;
			float4 vPos = aPos + rerVertex;
			float rad = shape[iVtx].w;
			float4 vVel	=aVel+cross((float4)(0.0f,0.0f,aAngVel, 0.f),rerVertex);
//			float restitution=1.0;
			float restitution=0.3f;
			{
				positionConstraint	=vPos.y - rad - gProp[0].y;
				impulse				=(float4)(0.0f, 0.f, 0.f, 0.f);

				if(positionConstraint < 0)
				{
					float4 groundNormal;
					groundNormal = (float4)(0.0f,1.0f,0.0f, 0.f);
					impulse	=groundNormal*
						restitution * computeImpulse1(vVel,positionConstraint,
						groundNormal,
						dt);
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
				impulse				=(float4)(0.0f, 0.f, 0.f, 0.f);

				if(positionConstraint < 0){
					impulse	=(float4)(1.0f,0.0f,0.0f, 0.f)* restitution * 
						computeImpulse1(vVel,positionConstraint,
						(float4)(1.0f,0.0f,0.0f, 0.f),
						dt);

					vel[idx]	+=	impulse;
					angVel[idx]	+=	cross(rerVertex,impulse).z;
				}
			}

			{
				positionConstraint	= gProp[1].x - vPos.x - rad;
				impulse				=(float4)(0.0f, 0.f, 0.f, 0.f);

				if(positionConstraint < 0){
					impulse	=(float4)(-1.0f,0.0f,0.0f, 0.f)* restitution * 
						computeImpulse1(vVel,positionConstraint,
						(float4)(-1.0f,0.0f,0.0f, 0.f),
						dt);

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
			relVel=(aVel+cross((float4)(0.0f,0.0f,aAngVel, 0.f), contactPoint))
				-(bVel+cross((float4)(0.0f,0.0f,bAngVel, 0.f),
				contactPoint+aPos-bPos));

			lambdaDt=	computeImpulse1(relVel,-positionConstraint,
				contactNormal,dt);

			{
				float rLambdaDt=lambdaDtBox[(MAX_VTX_PER_OBJ)*(2*constrId)+iVtx];
				float pLambdaDt=rLambdaDt;
//				rLambdaDt=btMax(pLambdaDt+lambdaDt,0.0f);
				rLambdaDt=(pLambdaDt+lambdaDt) > 0.0f ? (pLambdaDt+lambdaDt) : 0.0f;
				lambdaDt=rLambdaDt-pLambdaDt;
				lambdaDtBox[(MAX_VTX_PER_OBJ)*(2*constrId)+iVtx]=rLambdaDt;
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
										 float dt)
{
    int k_idx = get_global_id(0);
	if(k_idx < nConstraints)
	{
		int idx = batch[batchOffs + k_idx];
		collisionResolutionBox(	idx, constraints, pos, vel, rotation, angularVel, lambdaDtBox,
								contact, invMass, dt);
	}
}






int4 getGridPos(float4 worldPos, __global float4* pParams)
{
    int4 gridPos;
    gridPos.x = (int)floor((worldPos.x - pParams[0].x) / pParams[1].x);
    gridPos.y = (int)floor((worldPos.y - pParams[0].y) / pParams[1].y);
    gridPos.z = (int)floor((worldPos.z - pParams[0].z) / pParams[1].z);
    return gridPos;
}

int getPosHash(int4 gridPos, __global float4* pParams)
{
	int4 pGridDim = *((__global int4*)(pParams + 2));
	if(gridPos.x < 0) gridPos.x = 0;
	if(gridPos.x >= pGridDim.x) gridPos.x = pGridDim.x - 1;
	if(gridPos.y < 0) gridPos.y = 0;
	if(gridPos.y >= pGridDim.y) gridPos.y = pGridDim.y - 1;
	if(gridPos.z < 0) gridPos.z = 0;
	if(gridPos.z >= pGridDim.z) gridPos.z = pGridDim.z - 1;
	int hash = gridPos.z * pGridDim.y * pGridDim.x + gridPos.y * pGridDim.x + gridPos.x;
	return hash;
} 


// calculate grid hash value for each body using its AABB
__kernel void kCalcHashAABB(int numObjects, __global float4* pAABB, __global int2* pHash, __global float4* pParams GUID_ARG)
{
    int index = get_global_id(0);
    if(index >= numObjects)
	{
		return;
	}
	float4 bbMin = pAABB[index*2];
	float4 bbMax = pAABB[index*2 + 1];
	float4 pos;
	pos.x = (bbMin.x + bbMax.x) * 0.5f;
	pos.y = (bbMin.y + bbMax.y) * 0.5f;
	pos.z = (bbMin.z + bbMax.z) * 0.5f;
	pos.w = 0.f;
    // get address in grid
    int4 gridPos = getGridPos(pos, pParams);
    int gridHash = getPosHash(gridPos, pParams);
    // store grid hash and body index
    int2 hashVal;
    hashVal.x = gridHash;
    hashVal.y = index;
    pHash[index] = hashVal;
}

__kernel void kClearCellStart(	int numCells, 
								__global int* pCellStart GUID_ARG)
{
    int index = get_global_id(0);
    if(index >= numCells)
	{
		return;
	}
	pCellStart[index] = -1;
}



__kernel void kFindCellStart(int numObjects, __global int2* pHash, __global int* cellStart)
{
	__local int2 sharedHash[513];
    int index = get_global_id(0);
	int2 sortedData;
    if(index < numObjects)
	{
		sortedData = pHash[index];
		// Load hash data into shared memory so that we can look 
		// at neighboring body's hash value without loading
		// two hash values per thread
		sharedHash[get_local_id(0) + 1] = sortedData.x;
		if((index > 0) && (get_local_id(0) == 0))
		{
			// first thread in block must load neighbor body hash
			sharedHash[0] = pHash[index-1];
		}
	}
    barrier(CLK_LOCAL_MEM_FENCE);
    if(index < numObjects)
	{
		if((index == 0) || (sortedData.x != sharedHash[get_local_id(0)].x))
		{
			cellStart[sortedData.x] = index;
		}
	}
}


int testAABBOverlap(float4 min0, float4 max0, float4 min1, float4 max1)
{
	return	(min0.x <= max1.x)&& (min1.x <= max0.x) && 
			(min0.y <= max1.y)&& (min1.y <= max0.y) && 
			(min0.z <= max1.z)&& (min1.z <= max0.z); 
}
 
#define BT_3DGRID_PAIR_FOUND_FLG (0x40000000)
#define BT_3DGRID_PAIR_NEW_FLG   (0x20000000)
#define BT_3DGRID_PAIR_ANY_FLG   (BT_3DGRID_PAIR_FOUND_FLG | BT_3DGRID_PAIR_NEW_FLG)

void findPairsInCell(	int numObjects,
						int4	gridPos,
						int    index,
						__global int2*  pHash,
						__global int*   pCellStart,
						__global float4* pAABB, 
						__global int*   pPairBuff,
						__global int2*	pPairBuffStartCurr,
						__global float4* pParams GUID_ARG)
{
	int4 pGridDim = *((__global int4*)(pParams + 2));


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
	// iterate over bodies in this cell
    int2 sortedData = pHash[index];
	int unsorted_indx = sortedData.y;
    float4 min0 = pAABB[unsorted_indx*2 + 0]; 
	float4 max0 = pAABB[unsorted_indx*2 + 1];
	int handleIndex =  as_int(min0.w);
	int2 start_curr = pPairBuffStartCurr[handleIndex];
	int start = start_curr.x;
	int curr = start_curr.y;
	int2 start_curr_next = pPairBuffStartCurr[handleIndex+1];
	int curr_max = start_curr_next.x - start - 1;
	int bucketEnd = bucketStart + 8;
	bucketEnd = (bucketEnd > numObjects) ? numObjects : bucketEnd;
	for(uint index2 = bucketStart; index2 < bucketEnd; index2++) 
	{
        int2 cellData = pHash[index2];
        if (cellData.x != gridHash)
        {
			break;   // no longer in same bucket
		}
		int unsorted_indx2 = cellData.y;
        if (unsorted_indx2 < unsorted_indx) // check not colliding with self
        {   
			float4 min1 = pAABB[unsorted_indx2*2 + 0];
			float4 max1 = pAABB[unsorted_indx2*2 + 1];
			if(testAABBOverlap(min0, max0, min1, max1))
			{
				int handleIndex2 = as_int(min1.w);
				int k;
				for(k = 0; k < curr; k++)
				{
					int old_pair = pPairBuff[start+k] & (~BT_3DGRID_PAIR_ANY_FLG);
					if(old_pair == handleIndex2)
					{
						pPairBuff[start+k] |= BT_3DGRID_PAIR_FOUND_FLG;
						break;
					}
				}
				if(k == curr)
				{
					if(curr >= curr_max) 
					{ // not a good solution, but let's avoid crash
						break;
					}
					pPairBuff[start+curr] = handleIndex2 | BT_3DGRID_PAIR_NEW_FLG;
					curr++;
				}
			}
		}
	}
	int2 newStartCurr;
	newStartCurr.x = start;
	newStartCurr.y = curr;
	pPairBuffStartCurr[handleIndex] = newStartCurr;
    return;
}



__kernel void kFindOverlappingPairs(	int numObjects,
										__global float4* pAABB, 
										__global int2* pHash, 
										__global int* pCellStart, 
										__global int* pPairBuff, 
										__global int2* pPairBuffStartCurr, 
										__global float4* pParams GUID_ARG)

{
    int index = get_global_id(0);
    if(index >= numObjects)
	{
		return;
	}
    int2 sortedData = pHash[index];
	int unsorted_indx = sortedData.y;
	float4 bbMin = pAABB[unsorted_indx*2 + 0];
	float4 bbMax = pAABB[unsorted_indx*2 + 1];
	float4 pos;
	pos.x = (bbMin.x + bbMax.x) * 0.5f;
	pos.y = (bbMin.y + bbMax.y) * 0.5f;
	pos.z = (bbMin.z + bbMax.z) * 0.5f;
    // get address in grid
    int4 gridPosA = getGridPos(pos, pParams);
    int4 gridPosB; 
    // examine only neighbouring cells
    for(int z=-1; z<=1; z++) 
    {
		gridPosB.z = gridPosA.z + z;
        for(int y=-1; y<=1; y++) 
        {
			gridPosB.y = gridPosA.y + y;
            for(int x=-1; x<=1; x++) 
            {
				gridPosB.x = gridPosA.x + x;
                findPairsInCell(numObjects, gridPosB, index, pHash, pCellStart, pAABB, pPairBuff, pPairBuffStartCurr, pParams);
            }
        }
    }
}

//----------------------------------------------------------------------------------------

__kernel void kFindPairsLarge(	int numObjects, 
								__global float4* pAABB, 
								__global int2* pHash, 
								__global int* pCellStart, 
								__global int* pPairBuff, 
								__global int2* pPairBuffStartCurr, 
								uint numLarge GUID_ARG)
{
    int index = get_global_id(0);
    if(index >= numObjects)
	{
		return;
	}
    int2 sortedData = pHash[index];
	int unsorted_indx = sortedData.y;
	float4 min0 = pAABB[unsorted_indx*2 + 0];
	float4 max0 = pAABB[unsorted_indx*2 + 1];
	int handleIndex =  as_int(min0.w);
	int2 start_curr = pPairBuffStartCurr[handleIndex];
	int start = start_curr.x;
	int curr = start_curr.y;
	int2 start_curr_next = pPairBuffStartCurr[handleIndex+1];
	int curr_max = start_curr_next.x - start - 1;
    for(int i = 0; i < numLarge; i++)
    {
		int indx2 = numObjects + i;
		float4 min1 = pAABB[indx2*2 + 0];
		float4 max1 = pAABB[indx2*2 + 1];
		if(testAABBOverlap(min0, max0, min1, max1))
		{
			int k;
			int handleIndex2 =  as_int(min1.w);
			for(k = 0; k < curr; k++)
			{
				int old_pair = pPairBuff[start+k] & (~BT_3DGRID_PAIR_ANY_FLG);
				if(old_pair == handleIndex2)
				{
					pPairBuff[start+k] |= BT_3DGRID_PAIR_FOUND_FLG;
					break;
				}
			}
			if(k == curr)
			{
				pPairBuff[start+curr] = handleIndex2 | BT_3DGRID_PAIR_NEW_FLG;
				if(curr >= curr_max) 
				{ // not a good solution, but let's avoid crash
					break;
				}
				curr++;
			}
		}
    }
	int2 newStartCurr;
	newStartCurr.x = start;
	newStartCurr.y = curr;
	pPairBuffStartCurr[handleIndex] = newStartCurr;
    return;
}


__kernel void kComputePairCacheChanges(	int numObjects,
										__global int* pPairBuff, 
										__global int2* pPairBuffStartCurr, 
										__global int* pPairScan, 
										__global float4* pAABB GUID_ARG)
{
    int index = get_global_id(0);
    if(index >= numObjects)
	{
		return;
	}
	float4 bbMin = pAABB[index * 2];
	int handleIndex = as_int(bbMin.w);
	int2 start_curr = pPairBuffStartCurr[handleIndex];
	int start = start_curr.x;
	int curr = start_curr.y;
	__global int *pInp = pPairBuff + start;
	int num_changes = 0;
	for(int k = 0; k < curr; k++, pInp++)
	{
		if(!((*pInp) & BT_3DGRID_PAIR_FOUND_FLG))
		{
			num_changes++;
		}
	}
	pPairScan[index+1] = num_changes;
} 

//----------------------------------------------------------------------------------------

__kernel void kSqueezeOverlappingPairBuff(	int numObjects,
											__global int* pPairBuff, 
											__global int2* pPairBuffStartCurr, 
											__global int* pPairScan,
											__global int* pPairOut, 
											__global float4* pAABB GUID_ARG)
{
    int index = get_global_id(0);
    if(index >= numObjects)
	{
		return;
	}
	float4 bbMin = pAABB[index * 2];
	int handleIndex = as_int(bbMin.w);
	int2 start_curr = pPairBuffStartCurr[handleIndex];
	int start = start_curr.x;
	int curr = start_curr.y;
	__global int* pInp = pPairBuff + start;
	__global int* pOut = pPairOut + pPairScan[index];
	__global int* pOut2 = pInp;
	int num = 0; 
	for(int k = 0; k < curr; k++, pInp++)
	{
		if(!((*pInp) & BT_3DGRID_PAIR_FOUND_FLG))
		{
			*pOut = *pInp;
			pOut++;
		}
		if((*pInp) & BT_3DGRID_PAIR_ANY_FLG)
		{
			*pOut2 = (*pInp) & (~BT_3DGRID_PAIR_ANY_FLG);
			pOut2++;
			num++;
		}
	}
	int2 newStartCurr;
	newStartCurr.x = start;
	newStartCurr.y = num;
	pPairBuffStartCurr[handleIndex] = newStartCurr;
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

//#define LOCAL_SIZE_LIMIT 1024U
#define LOCAL_SIZE_MAX 1024U

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
__kernel void kBitonicSortCellIdLocal(__global int2* pKey, uint arrayLength, uint dir GUID_ARG)
{
    __local int2 l_key[LOCAL_SIZE_MAX];
    int localSizeLimit = get_local_size(0) * 2;

    //Offset to the beginning of subbatch and load data
    pKey += get_group_id(0) * localSizeLimit + get_local_id(0);
    l_key[get_local_id(0) +                    0] = pKey[                   0];
    l_key[get_local_id(0) + (localSizeLimit / 2)] = pKey[(localSizeLimit / 2)];

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
    pKey[                   0] = l_key[get_local_id(0) +                    0];
    pKey[(localSizeLimit / 2)] = l_key[get_local_id(0) + (localSizeLimit / 2)];
}

////////////////////////////////////////////////////////////////////////////////
// Bitonic sort kernel for large arrays (not fitting into local memory)
////////////////////////////////////////////////////////////////////////////////
//Bottom-level bitonic sort
//Almost the same as bitonicSortLocal with the only exception
//of even / odd subarrays (of LOCAL_SIZE_LIMIT points) being
//sorted in opposite directions
__kernel void kBitonicSortCellIdLocal1(__global int2* pKey GUID_ARG)
{
    __local int2 l_key[LOCAL_SIZE_MAX];
    uint localSizeLimit = get_local_size(0) * 2;

    //Offset to the beginning of subarray and load data
    pKey += get_group_id(0) * localSizeLimit + get_local_id(0);
    l_key[get_local_id(0) +                    0] = pKey[                   0];
    l_key[get_local_id(0) + (localSizeLimit / 2)] = pKey[(localSizeLimit / 2)];

    uint comparatorI = get_global_id(0) & ((localSizeLimit / 2) - 1);

    for(uint size = 2; size < localSizeLimit; size <<= 1)
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

    //Odd / even arrays of localSizeLimit elements
    //sorted in opposite directions
    {
        uint ddd = (get_group_id(0) & 1);
        for(uint stride = localSizeLimit / 2; stride > 0; stride >>= 1)
        {
            barrier(CLK_LOCAL_MEM_FENCE);
            uint pos = 2 * get_local_id(0) - (get_local_id(0) & (stride - 1));
            ComparatorLocal(&l_key[pos + 0], &l_key[pos + stride], ddd);
        }
    }

    barrier(CLK_LOCAL_MEM_FENCE);
    pKey[                   0] = l_key[get_local_id(0) +                    0];
    pKey[(localSizeLimit / 2)] = l_key[get_local_id(0) + (localSizeLimit / 2)];
}

//Bitonic merge iteration for 'stride' >= LOCAL_SIZE_LIMIT
__kernel void kBitonicSortCellIdMergeGlobal(__global int2* pKey, uint arrayLength, uint size, uint stride, uint dir GUID_ARG)
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
__kernel void kBitonicSortCellIdMergeLocal(__global int2* pKey, uint arrayLength, uint stride, uint size, uint dir GUID_ARG)
{
    __local int2 l_key[LOCAL_SIZE_MAX];
    int localSizeLimit = get_local_size(0) * 2;

    pKey += get_group_id(0) * localSizeLimit + get_local_id(0);
    l_key[get_local_id(0) +                    0] = pKey[                   0];
    l_key[get_local_id(0) + (localSizeLimit / 2)] = pKey[(localSizeLimit / 2)];

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
    pKey[                   0] = l_key[get_local_id(0) +                    0];
    pKey[(localSizeLimit / 2)] = l_key[get_local_id(0) + (localSizeLimit / 2)];
}
