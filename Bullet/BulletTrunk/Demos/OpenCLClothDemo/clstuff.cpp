
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

#include "clstuff.hpp"
#include "gl_win.hpp"

#ifndef _WIN32
#include <GL/glx.h>
#endif //!_WIN32

#include "btBulletDynamicsCommon.h"
#include "btOpenCLSupport.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

static cl::Context context;
static std::vector<cl::Device> devices;

cl::CommandQueue g_queue;

void initCL(void)
{
    cl_int err;

	std::vector<cl::Platform> platforms;
	err = cl::Platform::get(&platforms);
	checkErr(platforms.size() != 0 ? CL_SUCCESS : -1, "Platform::get()");

	std::string platformVendor;
	platforms[0].getInfo(CL_PLATFORM_VENDOR, &platformVendor);
	std::cout << "Platform is by: " << platformVendor << "\n";

		intptr_t properties[] = {
			CL_CONTEXT_PLATFORM, (intptr_t)platforms[0](),
            0, 0
        };
	context = cl::Context(CL_DEVICE_TYPE_GPU, properties, NULL, NULL, &err);
	//context = cl::Context(CL_DEVICE_TYPE_CPU, properties, NULL, NULL, &err);

	checkErr(err, "Conext::Context()");

    devices = context.getInfo<CL_CONTEXT_DEVICES>();
    checkErr(devices.size() > 0 ? CL_SUCCESS : -1, "devices.size() > 0");

	g_queue = cl::CommandQueue(context, devices[0], 0, &err);
    checkErr(err, "CommandQueue::CommandQueue()");
}