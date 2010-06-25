
class cap 
{
	public:

	ID3D11Buffer* g_pIndexBuffer;
	ID3D11Buffer* pVB[1];
	UINT Strides[1];
	UINT Offsets[1];

	double x_offset, y_offset, z_offset;


	ID3D11Texture2D *texture2D;
	ID3D11ShaderResourceView *texture2D_view;

	void create_texture(void)
	{
		D3DX11_IMAGE_LOAD_INFO loadInfo;
		ZeroMemory(&loadInfo, sizeof(D3DX11_IMAGE_LOAD_INFO) );
		loadInfo.BindFlags = D3D11_BIND_SHADER_RESOURCE;
		loadInfo.Format = DXGI_FORMAT_BC1_UNORM;

		HRESULT hr = D3DX11CreateShaderResourceViewFromFile(g_pd3dDevice, L"texture.bmp", &loadInfo, NULL, &texture2D_view, NULL);
		hr = hr;
	}

	void draw(void)
	{
		ID3D11DeviceContext* pd3dImmediateContext = DXUTGetD3D11DeviceContext();

		D3DXMATRIX mWorldViewProjection;
		D3DXVECTOR3 vLightDir;
		D3DXMATRIX mWorld;
		D3DXMATRIX mView;
		D3DXMATRIX mProj;

		// Get the projection & view matrix from the camera class
		mProj = *g_Camera.GetProjMatrix();
		mView = *g_Camera.GetViewMatrix();

		// Get the light direction
		vLightDir = g_LightControl.GetLightDirection();

		// Per frame cb update
		D3D11_MAPPED_SUBRESOURCE MappedResource;

		HRESULT hr;

		V( pd3dImmediateContext->Map( g_pcbPSPerFrame, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource ) );
		CB_PS_PER_FRAME* pPerFrame = ( CB_PS_PER_FRAME* )MappedResource.pData;
		float fAmbient = 0.1f;
		pPerFrame->m_vLightDirAmbient = D3DXVECTOR4( vLightDir.x, vLightDir.y, vLightDir.z, fAmbient );
		pd3dImmediateContext->Unmap( g_pcbPSPerFrame, 0 );

		pd3dImmediateContext->PSSetConstantBuffers( g_iCBPSPerFrameBind, 1, &g_pcbPSPerFrame );


	    ///////////////////////////////////////Modify below//////////////////////////////////////////////////////

		//Get the mesh
		//IA setup
		pd3dImmediateContext->IASetInputLayout( g_pVertexLayout11 );

		//This is where we pass the vertex buffer to DX
		pd3dImmediateContext->IASetVertexBuffers( 0, 1, pVB, Strides, Offsets );

		//This is where we pass the index buffer to DX
		pd3dImmediateContext->IASetIndexBuffer( g_pIndexBuffer, DXGI_FORMAT_R32_UINT, 0 );

		/////////////////////////////////////////////////////////////////////////////////////////////////////////

		// Set the shaders
		pd3dImmediateContext->VSSetShader( g_pVertexShader, NULL, 0 );
		pd3dImmediateContext->PSSetShader( g_pPixelShader, NULL, 0 );
		pd3dImmediateContext->GSSetShader( g_pGeometryShader, NULL, 0);
	    
		// Set the per object constant data		
		D3DXVECTOR3 vCenter( 0.25767413f, -28.503521f, 111.00689f);
		D3DXMatrixTranslation( &g_mCenterMesh, -vCenter.x+x_offset, -vCenter.y+y_offset, -vCenter.z+z_offset );

		D3DXMATRIXA16 m;
		D3DXMATRIXA16 m2;
		D3DXMATRIXA16 m3;
		D3DXMATRIXA16 m4;
		
		D3DXMatrixTranslation(&m2, 0, 0, -500);
		D3DXMatrixTranslation(&m3, 0, 0, 500);
	    
		D3DXMatrixRotationY( &m, D3DX_PI/4.0);
		D3DXMatrixRotationZ( &m4, D3DX_PI/2.0);

		m = m4*m3*m*m2;
	   

		mWorld = m * g_mCenterMesh * *g_Camera.GetWorldMatrix();
		mProj = *g_Camera.GetProjMatrix();
		mView = *g_Camera.GetViewMatrix();

		mWorldViewProjection = mWorld * mView * mProj;
	     

		// VS Per object
		V( pd3dImmediateContext->Map( g_pcbVSPerObject, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource ) );
		CB_VS_PER_OBJECT* pVSPerObject = ( CB_VS_PER_OBJECT* )MappedResource.pData;
		D3DXMatrixTranspose( &pVSPerObject->m_WorldViewProj, &mWorldViewProjection );
		D3DXMatrixTranspose( &pVSPerObject->m_World, &mWorld );
		pd3dImmediateContext->Unmap( g_pcbVSPerObject, 0 );

		pd3dImmediateContext->VSSetConstantBuffers( g_iCBVSPerObjectBind, 1, &g_pcbVSPerObject );

		// PS Per object
		V( pd3dImmediateContext->Map( g_pcbPSPerObject, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource ) );
		CB_PS_PER_OBJECT* pPSPerObject = ( CB_PS_PER_OBJECT* )MappedResource.pData;
		pPSPerObject->m_vObjectColor = D3DXVECTOR4( 1, 1, 1, 1 );
		pd3dImmediateContext->Unmap( g_pcbPSPerObject, 0 );

		pd3dImmediateContext->PSSetConstantBuffers( g_iCBPSPerObjectBind, 1, &g_pcbPSPerObject );

		//Render
		SDKMESH_SUBSET* pSubset = NULL;
		D3D11_PRIMITIVE_TOPOLOGY PrimType;

		pd3dImmediateContext->PSSetSamplers( 0, 1, &g_pSamLinear );

		{
			// Get the subset
			pSubset = g_Mesh11.GetSubset( 0, 0 );
			pd3dImmediateContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
			pd3dImmediateContext->PSSetShaderResources(0,1,&texture2D_view);

			pd3dImmediateContext->DrawIndexed( ( UINT )pSubset->IndexCount, 0, ( UINT )pSubset->VertexStart );
		}
	}


	void create_buffers(int width, int height)
	{

		D3D11_BUFFER_DESC bufferDesc;
		bufferDesc.Usage = D3D11_USAGE_DEFAULT;
		bufferDesc.ByteWidth = sizeof(vertex_struct)*width*height;
		bufferDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
		bufferDesc.CPUAccessFlags = 0;
		bufferDesc.MiscFlags = 0;


		vertex_struct *vertices = new vertex_struct[width*height];
		for(int y = 0; y < height; y++)
		{
			for(int x = 0; x < width; x++)
			{	
				float X =  (x/((float)(width-1)))*3.14159;
				float Y =  (y/((float)(height-1)))*3.14159;
				float z_coord = 100*cos(X)*sin(Y);
				float y_coord = 100*sin(X)*sin(Y);
				float x_coord = 100*cos(Y);
				vertices[y*width+x].Pos = D3DXVECTOR3(x_coord, y_coord, z_coord); 
				vertices[y*width+x].Normal = D3DXVECTOR3(1,0,0);
				vertices[y*width+x].Texcoord = D3DXVECTOR2(x/( (float)(width-1)), y/((float)(height-1)));
			}
		}

		D3D11_SUBRESOURCE_DATA InitData;
		InitData.pSysMem = vertices;
		InitData.SysMemPitch = 0;
		InitData.SysMemSlicePitch = 0;

		HRESULT hr = g_pd3dDevice->CreateBuffer(&bufferDesc, &InitData, &pVB[0]);
	    
		
		
		//What is this vertex stride thing all about?
		Strides[0] = ( UINT )g_Mesh11.GetVertexStride( 0, 0 );
		Offsets[0] = 0;

		unsigned int* indices = new unsigned int[width*3*2+2 + height*width*3*2];

		for(int y = 0; y < height-1; y++)
		{
			for(int x = 0; x < width-1; x++)
			{
				indices[x*3*2 + y*width*3*2] = x + y*width;
				indices[x*3*2+1 + y*width*3*2] = x+1 + y*width;
				indices[x*3*2+2 + y*width*3*2] = x+width + y*width;

				indices[x*3*2 + 3 + y*width*3*2] = x + 1 +  y*width;
				indices[x*3*2 + 4 + y*width*3*2] = x+(width+1) + y*width;
				indices[x*3*2 + 5 + y*width*3*2] = x+width + y*width;

			}
		}

		bufferDesc.ByteWidth = sizeof(unsigned int)*(width*3*2+2 + height*width*3*2);
		bufferDesc.BindFlags = D3D11_BIND_INDEX_BUFFER;

		InitData.pSysMem = indices;

		hr = g_pd3dDevice->CreateBuffer(&bufferDesc, &InitData, &g_pIndexBuffer);
		hr = hr;
	}
};