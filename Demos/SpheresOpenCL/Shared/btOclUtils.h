#include <CL/cl.h>


#define oclCHECKERROR(a, b) btAssert((a) == (b))



cl_device_id btOclGetDev(cl_context cxMainContext, unsigned int nr);
cl_device_id btOclGetMaxFlopsDev(cl_context cxMainContext);
char* btOclLoadProgSource(const char* cFilename, const char* cPreamble, size_t* szFinalLength);
cl_device_id btOclGetFirstDev(cl_context cxMainContext);
