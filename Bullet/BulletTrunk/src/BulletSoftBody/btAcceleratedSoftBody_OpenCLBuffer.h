#ifndef BT_ACCELERATED_SOFT_BODY_OPENCL_BUFFER_H
#define BT_ACCELERATED_SOFT_BODY_OPENCL_BUFFER_H

#include "BulletSoftBody/btAcceleratedSoftBody_Settings.h"

#ifdef BULLET_USE_OPENCL

// OpenCL support
#include <CL/cl.hpp>

#ifndef SAFE_RELEASE
#define SAFE_RELEASE(p)      { if(p) { (p)->Release(); (p)=NULL; } }
#endif

template <typename ElementType> class btOpenCLBuffer
{
protected:
	cl::CommandQueue m_queue;
	btAlignedObjectArray< ElementType > * m_CPUBuffer;
	cl::Buffer m_buffer;

	int  m_gpuSize;
	bool m_onGPU;

	bool m_readOnlyOnGPU;

	bool m_allocated;
	// TODO: Remove this once C++ bindings are fixed
	cl::Context context;

	bool createBuffer( cl::Buffer *preexistingBuffer = 0)
	{
		cl_int err;
		 

		if( preexistingBuffer )
		{
			m_buffer = *preexistingBuffer;
		} 
		else {
			m_buffer = cl::Buffer(
					context, 
					m_readOnlyOnGPU ? CL_MEM_READ_ONLY : CL_MEM_READ_WRITE, 
					m_CPUBuffer->size() * sizeof(ElementType), 
					0, 
					&err);
			if( err != CL_SUCCESS ) btAssert( "Buffer::Buffer(m_buffer)");
		}

		m_gpuSize = m_CPUBuffer->size();
		return true;
	}

public:
	btOpenCLBuffer( 
		cl::CommandQueue queue,
		btAlignedObjectArray< ElementType > *CPUBuffer, 
		bool readOnly) :
		m_queue(queue),
		m_CPUBuffer(CPUBuffer),
		m_gpuSize(0),
		m_onGPU(false),
		m_readOnlyOnGPU(readOnly),
		m_allocated(false)
	{
		context = m_queue.getInfo<CL_QUEUE_CONTEXT>();
	}

	~btOpenCLBuffer()
	{
	}

	cl::Buffer getBuffer()
	{
		return m_buffer;
	}

	bool moveToGPU()
	{
		cl_int err;

		if( (m_CPUBuffer->size() != m_gpuSize) )
		{
			m_onGPU = false;
		}

		if( !m_onGPU && m_CPUBuffer->size() > 0 )
		{
			if (!m_allocated || (m_CPUBuffer->size() != m_gpuSize)) {
				if (!createBuffer()) {
					return false;
				}
				m_allocated = true;
			}
			
			err = m_queue.enqueueWriteBuffer(
				m_buffer,
				CL_FALSE,
				0,
				m_CPUBuffer->size() * sizeof(ElementType), 
				&((*m_CPUBuffer)[0]));
			if( err != CL_SUCCESS ) btAssert( "CommandQueue::enqueueWriteBuffer(m_buffer)" );

			m_onGPU = true;
		}

		return true;
	}

	bool moveFromGPU()
	{
		cl_int err;

		if (m_CPUBuffer->size() > 0) {
			if (m_onGPU && !m_readOnlyOnGPU) {
				err = m_queue.enqueueReadBuffer(
					m_buffer,
					CL_TRUE,
					0,
					m_CPUBuffer->size() * sizeof(ElementType), 
					&((*m_CPUBuffer)[0]));

				if( err != CL_SUCCESS ) btAssert( "CommandQueue::enqueueReadBuffer(m_buffer)" );

				m_onGPU = false;
			}
		}

		return true;
	}

	bool copyFromGPU()
	{
		cl_int err;

		if (m_CPUBuffer->size() > 0) {
			if (m_onGPU && !m_readOnlyOnGPU) {
				err = m_queue.enqueueReadBuffer(
					m_buffer,
					CL_TRUE,
					0,
					m_CPUBuffer->size() * sizeof(ElementType), 
					&((*m_CPUBuffer)[0]));

				if( err != CL_SUCCESS ) btAssert( "CommandQueue::enqueueReadBuffer(m_buffer)");

			}
		}

		return true;
	}

	virtual void changedOnCPU()
	{
		m_onGPU = false;
	}
}; // class btOpenCLBuffer


#endif // #ifdef BULLET_USE_DX11

#endif // #ifndef BT_ACCELERATED_SOFT_BODY_OPENCL_BUFFER_H