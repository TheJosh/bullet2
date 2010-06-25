#ifndef BT_ACCELERATED_SOFT_BODY_DX_VERTEX_BUFFERS_H
#define BT_ACCELERATED_SOFT_BODY_DX_VERTEX_BUFFERS_H 

#include "BulletSoftBody/btAcceleratedSoftBody_Settings.h"

#ifdef BULLET_USE_DX11

#include "BulletSoftBody/btAcceleratedSoftBody_VertexBuffers.h"
#include <utility>

#include <windows.h>
#include <crtdbg.h>
#include <d3d11.h>
#include <d3dx11.h>
#include <d3dcompiler.h>

class btDX11VertexBufferDescriptor : public btVertexBufferDescriptor
{
protected:
	/** Context of the DX11 device on which the vertex buffer is stored. */
	ID3D11DeviceContext* m_context;
	/** DX11 vertex buffer */
	ID3D11Buffer* m_vertexBuffer;
	/** UAV for DX11 buffer */
	ID3D11UnorderedAccessView*  m_vertexBufferUAV;


public:
	/**
	 * buffer is a pointer to the DX11 buffer to place the vertex data in.
	 * UAV is a pointer to the UAV representation of the buffer laid out in floats.
	 * vertexOffset is the offset in floats to the first vertex.
	 * vertexStride is the stride in floats between vertices.
	 */
	btDX11VertexBufferDescriptor( ID3D11DeviceContext* context, ID3D11Buffer* buffer, ID3D11UnorderedAccessView *UAV, int vertexOffset, int vertexStride )
	{
		m_context = context;
		m_vertexBuffer = buffer;
		m_vertexBufferUAV = UAV;
		m_vertexOffset = vertexOffset;
		m_vertexStride = vertexStride;
		m_hasVertexPositions = true;
	}

	/**
	 * buffer is a pointer to the DX11 buffer to place the vertex data in.
	 * UAV is a pointer to the UAV representation of the buffer laid out in floats.
	 * vertexOffset is the offset in floats to the first vertex.
	 * vertexStride is the stride in floats between vertices.
	 * normalOffset is the offset in floats to the first normal.
	 * normalStride is the stride in floats between normals.
	 */
	btDX11VertexBufferDescriptor( ID3D11DeviceContext* context, ID3D11Buffer* buffer, ID3D11UnorderedAccessView *UAV, int vertexOffset, int vertexStride, int normalOffset, int normalStride )
	{
		m_context = context;
		m_vertexBuffer = buffer;
		m_vertexBufferUAV = UAV;
		m_vertexOffset = vertexOffset;
		m_vertexStride = vertexStride;
		m_hasVertexPositions = true;
		
		m_normalOffset = normalOffset;
		m_normalStride = normalStride;
		m_hasNormals = true;
	}

	virtual ~btDX11VertexBufferDescriptor()
	{

	}

	/**
	 * Return the type of the vertex buffer descriptor.
	 */
	virtual BufferTypes getBufferType() const
	{
		return DX11_BUFFER;
	}

	virtual ID3D11DeviceContext* getContext() const
	{
		return m_context;
	}

	virtual ID3D11Buffer* getbtDX11Buffer() const
	{
		return m_vertexBuffer;
	}

	virtual ID3D11UnorderedAccessView* getDX11UAV() const
	{
		return m_vertexBufferUAV;
	}		
};

#endif // #ifdef BULLET_USE_DX11

#endif // #ifndef BT_ACCELERATED_SOFT_BODY_DX_VERTEX_BUFFERS_H