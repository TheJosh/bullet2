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

#ifndef MINICL__TASK_H
#define MINICL__TASK_H

#include "BulletMultiThreaded/PlatformDefinitions.h"
#include "LinearMath/btScalar.h"

#include "LinearMath/btAlignedAllocator.h"


enum
{
	CMD_MINICL_1= 1,
	CMD_MINICL_APPLY_GRAVITY,
	CMD_MINICL_COMPUTE_CELL_ID,
	CMD_MINICL_CLEAR_CELL_START,
	CMD_MINICL_FIND_CELL_START,
	CMD_MINICL_BITONIC_SORT_CELL_ID_ALL_GLOB,
	CMD_MINICL_BITONIC_SORT_CELL_ID_LOCAL,
	CMD_MINICL_BITONIC_SORT_CELL_ID_LOCAL_1,
	CMD_MINICL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL,
	CMD_MINICL_BITONIC_SORT_CELL_ID_MERGE_LOCAL,
	CMD_MINICL_FIND_PAIRS,
	CMD_MINICL_SCAN_PAIRS_EXCLUSIVE_LOCAL_1,
	CMD_MINICL_SCAN_PAIRS_EXCLUSIVE_LOCAL_2,
	CMD_MINICL_SCAN_PAIRS_UNIFORM_UPDATE,
	CMD_MINICL_COMPACT_PAIRS,
	CMD_MINICL_INIT_BATCHES,
	CMD_MINICL_COMPUTE_BATCHES,
	CMD_MINICL_CHECK_BATCHES,
	CMD_MINICL_COMPUTE_CONTACTS,
	CMD_MINICL_SOLVE_CONSTRAINTS,
	CMD_MINICL_INTEGRATE_TRANSFORMS,
	CMD_MINICL_INTEGRATE_MOTION,
	CMD_MINICL_TOTAL_COMMANDS  // this should be the last
};



struct float8
{
	float s0;
	float s1;
	float s2;
	float s3;
	float s4;
	float s5;
	float s6;
	float s7;

	float8(float scalar)
	{
		s0=s1=s2=s3=s4=s5=s6=s7=scalar;
	}
};

#define MINICL_MAX_ARGLENGTH 128
#define MINI_CL_MAX_ARG 16

ATTRIBUTE_ALIGNED16(struct) MiniCLTaskDesc
{
	BT_DECLARE_ALIGNED_ALLOCATOR();

	MiniCLTaskDesc()
	{
		for (int i=0;i<MINI_CL_MAX_ARG;i++)
		{
			m_argSizes[i]=0;
		}
	}

	uint32_t	m_taskId;

	uint32_t	m_kernelProgramId;
	uint32_t	m_firstWorkUnit;
	uint32_t	m_lastWorkUnit;

	char		m_argData[MINI_CL_MAX_ARG][MINICL_MAX_ARGLENGTH];
	int			m_argSizes[MINI_CL_MAX_ARG];
};

extern int gMiniCLNumOutstandingTasks;


void	processMiniCLTask(void* userPtr, void* lsMemory);
void*	createMiniCLLocalStoreMemory();


#endif //MINICL__TASK_H

