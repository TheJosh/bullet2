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

int waste_time(int num)
{
	int i;
	int res = 0;
	for(i = 0; i < 10000; i++)
	{
		res += num;
	}
	for(i = 0; i < 9999; i++)
	{
		res -= num;
	}
	return res;
}

#if 1
__kernel void kIntegrateMotion(	int numObjects,
								__global float4* pPosInp, 
								__global float4* pLinVelInp, 
								__global float4* pPosOut, 
								__global float4* pLinVelOut, 
								__global float4* pParams, 
								float timeStep GUID_ARG)
{
    int index = get_global_id(0);
//    index = waste_time(index);
    if(index >= numObjects)
    {
		return;
    }
	float4 pos = pPosInp[index];
	float4 linVel = pLinVelInp[index];
	float4 gravity = pParams[0];
	linVel += gravity * timeStep;
	pos += linVel * timeStep;
	pPosOut[index] = pos;
	pLinVelOut[index] = linVel;
}
#else

#define LIM_VAL 1.f

__kernel void kIntegrateMotion(	__read_only image2d_t pPosInp, 
								__read_only image2d_t pLinVelInp, 
								__write_only image2d_t pPosOut, 
								__write_only image2d_t pLinVelOut, 
								int numObjects,
								__global float4* pParams, 
								float timeStep GUID_ARG)
{
    unsigned int index = get_global_id(0);
    if(index >= numObjects)
    {
		return;
    }
	int2 coord = (int2)(index, 0);
	float4 pos = read_imagef(pPosInp, CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_NONE | CLK_FILTER_NEAREST, coord );
	float4 linVel = read_imagef(pLinVelInp, CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_NONE | CLK_FILTER_NEAREST, coord );
	float4 gravity = pParams[0];
//	linVel += gravity * timeStep;
//	linVel.x = linVel.y = linVel.z = linVel.w = 0.f;
	linVel.w = 0.f;
//	pos += linVel * timeStep;
//	pos = linVel * timeStep;
//	pos.x = (float)(index * 4);
//	pos.y = (float)(index * 4);
//	pos.z = timeStep;
//	pos.w = 0.f;
//    barrier(CLK_LOCAL_MEM_FENCE | CLK_GLOBAL_MEM_FENCE);
	float4 newPos = pos;
	write_imagef(pPosOut, coord, newPos);
//	write_imagef(pLinVelOut, coord, linVel);
}

#endif