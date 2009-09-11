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
#include "MiniCLTask.h"

#ifdef __SPU__
#include <spu_printf.h>
#else
#include <stdio.h>
#define spu_printf printf
#endif

#define __kernel
#define __global
#define __local
#define get_global_id(a)	__guid_arg
#define get_local_id(a)		__guid_arg
#define get_local_size(a)	(4) // TODO : get from scheduler
#define get_group_id(a)		(0) // TODO : get from scheduler

#define CLK_LOCAL_MEM_FENCE		0x01
#define CLK_GLOBAL_MEM_FENCE	0x02

void barrier(unsigned int a)
{
	// TODO : implement
}

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

	float4& operator-=(const float4& other)
	{
		x -= other.x;
		y -= other.y;
		z -= other.z;
		w -= other.w;
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

float4 operator+(const float4& a,const float4& b)
{
	float4 tmp;
	tmp.x = a.x + b.x;
	tmp.y = a.y + b.y;
	tmp.z = a.z + b.z;
	tmp.w = a.w + b.w;
	return tmp;
}

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

float4 cross(const float4&a ,const float4& b)
{
	float4 tmp;
	tmp.x =  a.y*b.z - a.z*b.y;
	tmp.y = -a.x*b.z + a.z*b.x;
	tmp.z =  a.x*b.y - a.y*b.x;
	tmp.w = 0.f;
	return tmp;
}

float max(float a, float b) 
{
	return (a >= b) ? a : b;
}

float min(float a, float b) 
{
	return (a <= b) ? a : b;
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

#include "../Shared/SpheresGrid.cl"
#include "../Shared/Integration.cl"



//-- MAIN METHOD
void processMiniCLTask(void* userPtr, void* lsMemory)
{
	//	BT_PROFILE("processSampleTask");

	MiniCLTask_LocalStoreMemory* localMemory = (MiniCLTask_LocalStoreMemory*)lsMemory;

	MiniCLTaskDesc* taskDescPtr = (MiniCLTaskDesc*)userPtr;
	MiniCLTaskDesc& taskDesc = *taskDescPtr;

//	printf("Compute Unit[%d] executed kernel %d work items [%d..%d)\n",taskDesc.m_taskId,taskDesc.m_kernelProgramId,taskDesc.m_firstWorkUnit,taskDesc.m_lastWorkUnit);
	
	
	switch (taskDesc.m_kernelProgramId)
	{
/*
	case CMD_MINICL_ADDVECTOR:
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				VectorAdd(*(const float8**)&taskDesc.m_argData[0][0],*(const float8**)&taskDesc.m_argData[1][0],*(float8**)&taskDesc.m_argData[2][0],i);
			}
			break;
		}
*/
/*
	"kPredictUnconstrainedMotion",
	"kSetSpheres",
	"kIntegrateTransforms",
	"kBroadphaseCD"
*/
	case CMD_MINICL_PREDICT_MOTION :
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kPredictUnconstrainedMotion(*(float4**)&taskDesc.m_argData[0][0],
											*(float4**)&taskDesc.m_argData[1][0],
											*(float4**)&taskDesc.m_argData[2][0],
											*(float4**)&taskDesc.m_argData[3][0],
											*(int*)    &taskDesc.m_argData[4][0],
											*(float*)  &taskDesc.m_argData[5][0],
											i);
			}
			break;
		}
	case CMD_MINICL_SET_SPHERES :
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kSetSpheres(*(float4**)&taskDesc.m_argData[0][0],
							*(float4**)&taskDesc.m_argData[1][0],
							*(float4**)&taskDesc.m_argData[2][0],
							*(int**)   &taskDesc.m_argData[3][0],
							*(int2**)  &taskDesc.m_argData[4][0],
							*(float4**)&taskDesc.m_argData[5][0],
							*(uint*)   &taskDesc.m_argData[6][0],
							i);
			}
			break;
		}
	case CMD_MINICL_INTEGRATE_TRANSFORMS :
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kIntegrateTransforms(	*(float4**)&taskDesc.m_argData[0][0],
										*(float4**)&taskDesc.m_argData[1][0],
										*(float4**)&taskDesc.m_argData[2][0],
										*(float4**)&taskDesc.m_argData[3][0],
										*(float4**)&taskDesc.m_argData[4][0],
										*(int*)    &taskDesc.m_argData[5][0],
										*(float*)  &taskDesc.m_argData[6][0],
										i);
			}
			break;
		}
	case CMD_MINICL_BITONIC_SORT_HASH : 
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kBitonicSortHash(	*(int2**)		&taskDesc.m_argData[0][0],
									*(unsigned int*)&taskDesc.m_argData[1][0],
									*(unsigned int*)&taskDesc.m_argData[2][0],
									i);
			}
			break;
		}
	case CMD_MINICL_BROADPHASE_CD :
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kBroadphaseCD(	*(float4**)&taskDesc.m_argData[0][0],
								*(float4**)&taskDesc.m_argData[1][0],
								*(int**)   &taskDesc.m_argData[2][0],
								*(int2**)  &taskDesc.m_argData[3][0],
								*(int**)   &taskDesc.m_argData[4][0],
								*(int**)   &taskDesc.m_argData[5][0],
								*(int2**)  &taskDesc.m_argData[6][0],
								*(int*)    &taskDesc.m_argData[7][0],
								*(float4**)&taskDesc.m_argData[8][0],
								i);

			}
			break;
		}
	case CMD_MINICL_INIT_OBJ_USAGE_TAB : 
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kInitObjUsageTab(	*(int**)	&taskDesc.m_argData[0][0],
									*(float4**)	&taskDesc.m_argData[1][0],
									*(float4**)	&taskDesc.m_argData[2][0],
									*(int*)		&taskDesc.m_argData[3][0],
									i);

			}
			break;
		}
	case CMD_MINICL_SETUP_BATCHES :
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kSetupBatches(	*(int4**)	&taskDesc.m_argData[0][0],
								*(int**)	&taskDesc.m_argData[1][0],
								*(float4**)	&taskDesc.m_argData[2][0],
								i);

			}
			break;
		}
	case CMD_MINICL_CHECK_BATCHES :
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kCheckBatches(	*(int4**)	&taskDesc.m_argData[0][0],
								*(int**)	&taskDesc.m_argData[1][0],
								*(float4**)	&taskDesc.m_argData[2][0],
								*(int*)		&taskDesc.m_argData[3][0],
								*(int*)		&taskDesc.m_argData[4][0],
								i);

			}
			break;
		}
	case CMD_MINICL_SETUP_CONTACTS :
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kSetupContacts(	*(int4**)  &taskDesc.m_argData[0][0],
								*(float4**)&taskDesc.m_argData[1][0],
								*(float4**)&taskDesc.m_argData[2][0],
								*(float4**)&taskDesc.m_argData[3][0],
								*(float4**)&taskDesc.m_argData[4][0],
								i);

			}
			break;
		}
	case CMD_MINICL_SOLVE_CONSTRAINTS :
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kSolveConstraints(	*(float4**)	&taskDesc.m_argData[0][0],
									*(int*)		&taskDesc.m_argData[1][0],
									*(float4**)	&taskDesc.m_argData[2][0],
									*(int4**)	&taskDesc.m_argData[3][0],
									*(float4**)	&taskDesc.m_argData[4][0],
									*(float4**)	&taskDesc.m_argData[5][0],
									*(float4**)	&taskDesc.m_argData[6][0],
									*(float4**)	&taskDesc.m_argData[7][0],
									*(float*)	&taskDesc.m_argData[8][0],
									i);

			}
			break;
		}
	case CMD_MINICL_INTEGRATE_MOTION :
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kIntegrateMotion(	*(float4**)&taskDesc.m_argData[0][0],
									*(float4**)&taskDesc.m_argData[1][0],
									*(float4**)&taskDesc.m_argData[2][0],
									*(float4**)&taskDesc.m_argData[3][0],
									*(int*)    &taskDesc.m_argData[4][0],
									*(float4**)&taskDesc.m_argData[5][0],
									*(float*)  &taskDesc.m_argData[6][0],
									i);
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
