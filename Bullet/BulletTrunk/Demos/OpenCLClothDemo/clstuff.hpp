#ifndef __CLSTUFF_HDR__
#define __CLSTUFF_HDR__

#include <GL/glew.h>

#ifndef _WIN32
#include <GL/glx.h>
#endif //!_WIN32

#if defined(__APPLE__) || defined(__MACOSX)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <CL/cl.hpp>

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <iterator>
#include <math.h>
#include <cmath>
#include <cstring>
#include <vector>

inline void 
checkErr(cl_int err, const char * name)
{
    if (err != CL_SUCCESS) {
        std::cerr << "ERROR: " <<  name << " (" << err << ")" << std::endl;
        exit(EXIT_FAILURE);
    }
}

void initCL(void);

#endif //__CLSTUFF_HDR__