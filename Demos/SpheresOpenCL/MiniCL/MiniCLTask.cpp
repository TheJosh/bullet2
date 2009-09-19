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

int gMiniCLNumOutstandingTasks = 0;

#define __kernel
#define __global
#define __local
#define get_global_id(a)	__guid_arg
#define get_local_id(a)		((__guid_arg) % gMiniCLNumOutstandingTasks)
#define get_local_size(a)	(gMiniCLNumOutstandingTasks)
#define get_group_id(a)		((__guid_arg) / gMiniCLNumOutstandingTasks)

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

struct uint4
{
	unsigned int x,y,z,w;
	uint4() {}
	uint4(uint val) { x = y = z = w = val; }
	uint4& operator+=(const uint4& other)
	{
		x += other.x;
		y += other.y;
		z += other.z;
		w += other.w;
		return *this;
	}
};
uint4 operator+(const uint4& a,const uint4& b)
{
	uint4 tmp;
	tmp.x = a.x + b.x;
	tmp.y = a.y + b.y;
	tmp.z = a.z + b.z;
	tmp.w = a.w + b.w;
	return tmp;
}
uint4 operator-(const uint4& a,const uint4& b)
{
	uint4 tmp;
	tmp.x = a.x - b.x;
	tmp.y = a.y - b.y;
	tmp.z = a.z - b.z;
	tmp.w = a.w - b.w;
	return tmp;
}

#define native_sqrt sqrtf
#define native_sin sinf
#define native_cos cosf

struct MiniCLTask_LocalStoreMemory
{
	
};

#define GUID_ARG ,int __guid_arg
#define GUID_ARG_VAL ,__guid_arg

//#include <CL/cl_platform.h> //for CL_PLATFORM_MINI_CL definition
// this one gives error for redefinition 
// CL/cl_platform.h(56) : error C2371: 'uint64_t' : redefinition; different basic types
// src\bulletmultithreaded\PlatformDefinitions.h(22) :
// so use hack for now
#define CL_PLATFORM_MINI_CL 0x12345
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
	case CMD_MINICL_APPLY_GRAVITY :
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kApplyGravity(	*(int*)    &taskDesc.m_argData[0][0],
								*(float4**)&taskDesc.m_argData[1][0],
								*(float4**)&taskDesc.m_argData[2][0],
								*(float4**)&taskDesc.m_argData[3][0],
								*(float4**)&taskDesc.m_argData[4][0],
								*(float*)  &taskDesc.m_argData[5][0],
								i);
			}
			break;
		}
	case CMD_MINICL_COMPUTE_CELL_ID :
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kComputeCellId(	*(int*)    &taskDesc.m_argData[0][0],
								*(float4**)&taskDesc.m_argData[1][0],
								*(float4**)&taskDesc.m_argData[2][0],
								*(float4**)&taskDesc.m_argData[3][0],
								*(int**)   &taskDesc.m_argData[4][0],
								*(int2**)  &taskDesc.m_argData[5][0],
								*(float4**)&taskDesc.m_argData[6][0],
								i);
			}
			break;
		}
	case CMD_MINICL_CLEAR_CELL_START :
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kClearCellStart(*(int*)    &taskDesc.m_argData[0][0],
								*(int**)   &taskDesc.m_argData[1][0],
								i);
			}
			break;
		}
	case CMD_MINICL_FIND_CELL_START :
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kFindCellStart(	*(int*)    &taskDesc.m_argData[0][0],
								*(int2**)  &taskDesc.m_argData[1][0],
								*(int**)   &taskDesc.m_argData[2][0],
								i);
			}
			break;
		}
	case CMD_MINICL_BITONIC_SORT_CELL_ID_ALL_GLOB : 
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

	case CMD_MINICL_BITONIC_SORT_CELL_ID_LOCAL : 
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kBitonicSortCellIdLocal(*(int2**)		&taskDesc.m_argData[0][0],
										*(unsigned int*)&taskDesc.m_argData[1][0],
										*(unsigned int*)&taskDesc.m_argData[2][0],
										i);
			}
			break;
		}
	case CMD_MINICL_BITONIC_SORT_CELL_ID_LOCAL_1 : 
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kBitonicSortCellIdLocal1(	*(int2**)		&taskDesc.m_argData[0][0],
											i);
			}
			break;
		}
	case CMD_MINICL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL : 
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kBitonicSortCellIdMergeGlobal(	*(int2**)		&taskDesc.m_argData[0][0],
												*(unsigned int*)&taskDesc.m_argData[1][0],
												*(unsigned int*)&taskDesc.m_argData[2][0],
												*(unsigned int*)&taskDesc.m_argData[3][0],
												*(unsigned int*)&taskDesc.m_argData[4][0],
												i);
			}
			break;
		}
	case CMD_MINICL_BITONIC_SORT_CELL_ID_MERGE_LOCAL : 
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kBitonicSortCellIdMergeLocal(	*(int2**)		&taskDesc.m_argData[0][0],
												*(unsigned int*)&taskDesc.m_argData[1][0],
												*(unsigned int*)&taskDesc.m_argData[2][0],
												*(unsigned int*)&taskDesc.m_argData[3][0],
												*(unsigned int*)&taskDesc.m_argData[4][0],
												i);
			}
			break;
		}
	case CMD_MINICL_FIND_PAIRS :
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kFindPairs(	*(int*)		&taskDesc.m_argData[0][0],
							*(float4**)	&taskDesc.m_argData[1][0],
							*(float4**)	&taskDesc.m_argData[2][0],
							*(int**)	&taskDesc.m_argData[3][0],
							*(int2**)	&taskDesc.m_argData[4][0],
							*(int**)	&taskDesc.m_argData[5][0],
							*(int**)	&taskDesc.m_argData[6][0],
							*(int**)	&taskDesc.m_argData[7][0],
							*(int**)	&taskDesc.m_argData[8][0],
							*(float4**)	&taskDesc.m_argData[9][0],
							i);

			}
			break;
		}
#if 1
	case CMD_MINICL_SCAN_PAIRS_EXCLUSIVE_LOCAL_1 : 
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kScanPairsExclusiveLocal1(	*(uint4**)			&taskDesc.m_argData[0][0],
											*(uint4**)			&taskDesc.m_argData[1][0],
											*(unsigned int**)	&taskDesc.m_argData[2][0],
											*(int*)				&taskDesc.m_argData[3][0],
											i);
			}
			break;
		}
	case CMD_MINICL_SCAN_PAIRS_EXCLUSIVE_LOCAL_2 : 
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kScanPairsExclusiveLocal2(	*(unsigned int**)	&taskDesc.m_argData[0][0],
											*(unsigned int**)	&taskDesc.m_argData[1][0],
											*(unsigned int**)	&taskDesc.m_argData[2][0],
											*(unsigned int**)	&taskDesc.m_argData[3][0],
											*(unsigned int*)	&taskDesc.m_argData[4][0],
											*(unsigned int*)	&taskDesc.m_argData[5][0],
											i);
			}
			break;
		}
	case CMD_MINICL_SCAN_PAIRS_UNIFORM_UPDATE : 
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kScanPairsUniformUpdate(*(uint4**)			&taskDesc.m_argData[0][0],
										*(unsigned int**)	&taskDesc.m_argData[2][0],
										i);
			}
			break;
		}
#endif

	case CMD_MINICL_COMPACT_PAIRS : 
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kCompactPairs(	*(int*)		&taskDesc.m_argData[0][0],
								*(int**)	&taskDesc.m_argData[1][0],
								*(int**)	&taskDesc.m_argData[2][0],
								*(int**)	&taskDesc.m_argData[3][0],
								*(int**)	&taskDesc.m_argData[4][0],
								*(int**)	&taskDesc.m_argData[5][0],
								*(int4**)	&taskDesc.m_argData[6][0],
								i);
			}
			break;
		}
	case CMD_MINICL_INIT_BATCHES : 
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kInitBatches(	*(int*)		&taskDesc.m_argData[0][0],
								*(int**)	&taskDesc.m_argData[1][0],
								*(float4**)	&taskDesc.m_argData[2][0],
								*(float4**)	&taskDesc.m_argData[3][0],
								i);

			}
			break;
		}
	case CMD_MINICL_COMPUTE_BATCHES :
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kComputeBatches(*(int*)		&taskDesc.m_argData[0][0],	
								*(int4**)	&taskDesc.m_argData[1][0],
								*(int**)	&taskDesc.m_argData[2][0],
								*(float4**)	&taskDesc.m_argData[3][0],
								i);

			}
			break;
		}
	case CMD_MINICL_CHECK_BATCHES :
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kCheckBatches(	*(int*)		&taskDesc.m_argData[0][0],	
								*(int4**)	&taskDesc.m_argData[1][0],
								*(int**)	&taskDesc.m_argData[2][0],
								*(float4**)	&taskDesc.m_argData[3][0],
								*(int*)		&taskDesc.m_argData[4][0],
								*(int*)		&taskDesc.m_argData[5][0],
								i);

			}
			break;
		}
	case CMD_MINICL_COMPUTE_CONTACTS :
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kComputeContacts(	*(int*)		&taskDesc.m_argData[0][0],	
									*(int4**)	&taskDesc.m_argData[1][0],
									*(float4**)	&taskDesc.m_argData[2][0],
									*(float4**)	&taskDesc.m_argData[3][0],
									*(float4**)	&taskDesc.m_argData[4][0],
									*(float4**)	&taskDesc.m_argData[5][0],
									i);

			}
			break;
		}
	case CMD_MINICL_SOLVE_CONSTRAINTS :
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kSolveConstraints(	*(int*)		&taskDesc.m_argData[0][0],	
									*(float4**)	&taskDesc.m_argData[1][0],
									*(int*)		&taskDesc.m_argData[2][0],
									*(float4**)	&taskDesc.m_argData[3][0],
									*(int4**)	&taskDesc.m_argData[4][0],
									*(float4**)	&taskDesc.m_argData[5][0],
									*(float4**)	&taskDesc.m_argData[6][0],
									*(float4**)	&taskDesc.m_argData[7][0],
									*(float4**)	&taskDesc.m_argData[8][0],
									*(float*)	&taskDesc.m_argData[9][0],
									i);

			}
			break;
		}
	case CMD_MINICL_INTEGRATE_TRANSFORMS :
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kIntegrateTransforms(	*(int*)    &taskDesc.m_argData[0][0],
										*(float4**)&taskDesc.m_argData[1][0],
										*(float4**)&taskDesc.m_argData[2][0],
										*(float4**)&taskDesc.m_argData[3][0],
										*(float4**)&taskDesc.m_argData[4][0],
										*(float4**)&taskDesc.m_argData[5][0],
										*(float*)  &taskDesc.m_argData[6][0],
										i);
			}
			break;
		}
	case CMD_MINICL_INTEGRATE_MOTION :
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				kIntegrateMotion(	*(int*)    &taskDesc.m_argData[0][0],
									*(float4**)&taskDesc.m_argData[1][0],
									*(float4**)&taskDesc.m_argData[2][0],
									*(float4**)&taskDesc.m_argData[3][0],
									*(float4**)&taskDesc.m_argData[4][0],
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
