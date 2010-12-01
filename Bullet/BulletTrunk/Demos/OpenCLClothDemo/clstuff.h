#ifndef __CLSTUFF_HDR__
#define __CLSTUFF_HDR__

#include <cstdint>


// OpenCL initialization.
// Takes an optional GL context which, if passed, will create an interop-enabled CL context.
void initCL( intptr_t glContext = 0 );

#endif //__CLSTUFF_HDR__