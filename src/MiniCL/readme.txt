This verson of MiniCL has the following restrictions:

	- Some functions (like barrier()) are not implemented

	- Number of kernel arguments should be less or equal to 16

	- Size of all kernel argiments should be equal to sizeof(void*)

Program should encapsulate all OpenCL programs into a CPP file like:

//---------------- start of file MyOpenCLWrap.cpp ------------------------

// it is supposed that file Your_CL_Program.cl
// contains N kernel functions kernelFunction1() ...  kernelFunctionN()

#include <CL/cl_MiniCL_Defs.h>

extern "C"
{
	#include "Your_CL_Program.cl"
}

MINICL_REGISTER(kernelFunction1)
MINICL_REGISTER(kernelFunction2)
// ...
MINICL_REGISTER(kernelFunctionN)

//---------------- end of file MyOpenCLWrap.cpp ------------------------
