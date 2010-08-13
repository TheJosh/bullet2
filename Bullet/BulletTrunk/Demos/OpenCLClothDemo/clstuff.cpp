


#include "clstuff.hpp"
#include "gl_win.hpp"


#include "btOclCommon.h"
#include "btOclUtils.h"
#include "LinearMath/btScalar.h"

cl_context			g_cxMainContext;
cl_device_id		g_cdDevice;
cl_command_queue	g_cqCommandQue;

void initCL(void)
{
	int ciErrNum = 0;
    //g_cxMainContext = btOclCommon::createContextFromType(CL_DEVICE_TYPE_GPU, &ciErrNum);
	//g_cxMainContext = btOclCommon::createContextFromType(CL_DEVICE_TYPE_ALL, &ciErrNum);
	
	//#ifdef USE_MINICL try CL_DEVICE_TYPE_DEBUG for sequential, non-threaded execution
	//it gives a full callstack at the crash in the kernel
	g_cxMainContext = btOclCommon::createContextFromType(CL_DEVICE_TYPE_ALL, &ciErrNum);
	
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	g_cdDevice = btOclGetMaxFlopsDev(g_cxMainContext);
	
	btOclPrintDevInfo(g_cdDevice);

	// create a command-queue
	g_cqCommandQue = clCreateCommandQueue(g_cxMainContext, g_cdDevice, 0, &ciErrNum);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
}