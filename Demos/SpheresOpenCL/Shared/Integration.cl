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


__kernel void kIntegrateMotion(	__global float4* pPos, 
								__global float4* pLinVel, 
								int numObjects,
								__global float4* pParams, 
								float timeStep GUID_ARG)
{
    unsigned int index = get_global_id(0);
//    index = waste_time(index);
    if(index >= numObjects)
    {
		return;
    }
	float4 pos = pPos[index];
	float4 linVel = pLinVel[index];
	float4 gravity = pParams[0];
	linVel += gravity * timeStep;
	pos += linVel * timeStep;
	pPos[index] = pos;
	pLinVel[index] = linVel;
}

