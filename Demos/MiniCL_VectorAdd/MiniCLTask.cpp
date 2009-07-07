/*
Bullet Continuous Collision Detection and Physics Library, Copyright (c) 2007 Erwin Coumans

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

*/


#include "MiniCLTask.h"
#include "BulletMultiThreaded/PlatformDefinitions.h"
#include "BulletMultiThreaded/SpuFakeDma.h"
#include "LinearMath/btMinMax.h"
#include "BulletMultiThreaded/MiniCLTask/MiniCLTask.h"

#ifdef __SPU__
#include <spu_printf.h>
#else
#include <stdio.h>
#define spu_printf printf
#endif

#define __kernel
#define __global
#define get_global_id(a) __guid_arg

struct float4
{
	float x,y,z,w;
	float4 operator*(const float4& other)
	{
		float4 tmp;
		tmp.x = x*other.x;
		tmp.y = y*other.y;
		tmp.z = z*other.z;
		tmp.w = w*other.w;
		return tmp;
	}

	float4 operator*(const float& other)
	{
		float4 tmp;
		tmp.x = x*other;
		tmp.y = y*other;
		tmp.z = z*other;
		tmp.w = w*other;
		return tmp;
	}

	

	float4& operator+=(const float4& other)
	{
		x += other.x;
		y += other.y;
		z += other.z;
		w += other.w;
		return *this;
	}

	float4& operator *=(float scalar)
	{
		x *= scalar;
		y *= scalar;
		z *= scalar;
		w *= scalar;
		return (*this);
	}
	
};

float4 operator-(const float4& a,const float4& b)
{
	float4 tmp;
	tmp.x = a.x - b.x;
	tmp.y = a.y - b.y;
	tmp.z = a.z - b.z;
	tmp.w = a.w - b.w;
	return tmp;
}

float dot(const float4&a ,const float4& b)
{
	float4 tmp;
	tmp.x = a.x*b.x;
	tmp.y = a.y*b.y;
	tmp.z = a.z*b.z;
	tmp.w = a.w*b.w;
	return tmp.x+tmp.y+tmp.z+tmp.w;
}

struct int2
{
	int x,y;
};

struct uint2
{
	unsigned int x,y;
};

//typedef int2 uint2;

typedef unsigned int uint;

struct int4
{
	int x,y,z,w;
};

#define native_sqrt sqrtf
#define native_sin sinf
#define native_cos cosf

struct MiniCLTask_LocalStoreMemory
{
	
};

#define GUID_ARG ,int __guid_arg

#include "VectorAddKernels.cl"



//-- MAIN METHOD
void processMiniCLTask(void* userPtr, void* lsMemory)
{
	//	BT_PROFILE("processSampleTask");

	MiniCLTask_LocalStoreMemory* localMemory = (MiniCLTask_LocalStoreMemory*)lsMemory;

	MiniCLTaskDesc* taskDescPtr = (MiniCLTaskDesc*)userPtr;
	MiniCLTaskDesc& taskDesc = *taskDescPtr;

	printf("Compute Unit[%d] executed kernel %d work items [%d..%d)\n",taskDesc.m_taskId,taskDesc.m_kernelProgramId,taskDesc.m_firstWorkUnit,taskDesc.m_lastWorkUnit);
	
	
	switch (taskDesc.m_kernelProgramId)
	{
	case CMD_MINICL_ADDVECTOR:
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				VectorAdd(*(const float8**)&taskDesc.m_argData[0][0],*(const float8**)&taskDesc.m_argData[1][0],*(float8**)&taskDesc.m_argData[2][0],i);
			}
			break;
		}

	default:
		{
			printf("error in processMiniCLTask: unknown command id: %d\n",taskDesc.m_kernelProgramId);

		}
	};

}


#if defined(__CELLOS_LV2__) || defined (LIBSPE2)

ATTRIBUTE_ALIGNED16(MiniCLTask_LocalStoreMemory	gLocalStoreMemory);

void* createMiniCLLocalStoreMemory()
{
	return &gLocalStoreMemory;
}
#else
void* createMiniCLLocalStoreMemory()
{
	return new MiniCLTask_LocalStoreMemory;
};

#endif
