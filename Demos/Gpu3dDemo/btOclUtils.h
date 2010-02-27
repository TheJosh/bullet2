
#ifndef BT_OCL_UTILS_H
#define BT_OCL_UTILS_H

#ifdef __APPLE__
#include <OpenCL/cl.h>

#else
#ifdef USE_MINICL
#include <MiniCL/cl.h>
#else
#include <CL/cl.h>
#endif
#endif

//#define oclCHECKERROR(a, b) btAssert((a) == (b))
#define oclCHECKERROR(a, b) if((a)!=(b)) { printf("OCL Error : %d\n", (a)); btAssert((a) == (b)); }



cl_device_id btOclGetDev(cl_context cxMainContext, unsigned int nr);
cl_device_id btOclGetMaxFlopsDev(cl_context cxMainContext);
char* btOclLoadProgSource(const char* cFilename, const char* cPreamble, size_t* szFinalLength);
cl_device_id btOclGetFirstDev(cl_context cxMainContext);
#endif //BT_OCL_UTILS_H
