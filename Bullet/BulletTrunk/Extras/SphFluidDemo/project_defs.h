
#ifndef __PROJECT_DEFS_H__
#define __PROJECT_DEFS_H__

#define FORCE_HOST_SORT // RadixSort is failing

#define OPENCL_DEVICE_TYPE CL_DEVICE_TYPE_GPU // CL_DEVICE_TYPE_GPU or CL_DEVICE_TYPE_CPU
//#define OPENCL_DEVICE_TYPE CL_DEVICE_TYPE_CPU // CL_DEVICE_TYPE_GPU or CL_DEVICE_TYPE_CPU

//#define TEST_PIPELINE // enable this to run correctness tests for every frame of the simulation
//#define CAPTURE_PIPELINE // enable this to capture input and output buffers to each kernel

// demo control
#define FRAME_TO_REMOVE_BOX  1000 

#ifdef TEST_PIPELINE
#undef CAPTURE_PIPELINE // only one of these may be defined
#endif

#endif//__PROJECT_DEFS_H__
