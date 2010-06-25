#include "dx10_render.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
// Global Variables 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 

//-------------------Simulator----------------------------//
static btSphFluid * fluid_;
static btFluidWorld * world_;
static unsigned int maxParticles_;

//-------------------Tweakables----------------------------//
float depthThreshold = 0.04f;
float g_timeStep = 0.00005f;
float g_particleRad = 1.0f;
int iterations = 10;
int frameCount=0;
bool	g_bFirst = true;
bool	g_bFirstCreate = true;
bool	bHelp = true;							
bool takeSnapshot = false;
bool bPause = true;


//-------------------Geometry----------------------------//
// Sphere
static int n_vertices;
static int n_faces;
static int n_edges;
static float *vertices = NULL;
static unsigned int *faces = NULL; 
static int edge_walk; 
static int *start = NULL; 
static int *end = NULL; 
static int *midpoint = NULL; 
VPos* sphVertices;
unsigned int *sphIndices;

// Full screen quads
VPos worldSpaceQuadVertices[4] = 
{	D3DXVECTOR3(-1.0f, -1.0f, 0.5f ), D3DXVECTOR3( 0.0f, 0.0f, 1.0f ), D3DXVECTOR2( 0.0f, 1.0f ), D3DXVECTOR3(-1.0f, -1.0f, 0.5f ), 
	D3DXVECTOR3( 1.0f, -1.0f, 0.5f ), D3DXVECTOR3( 0.0f, 0.0f, 1.0f ), D3DXVECTOR2( 1.0f, 1.0f ), D3DXVECTOR3(-1.0f, -1.0f, 0.5f ), 
	D3DXVECTOR3( 1.0f,  1.0f, 0.5f ), D3DXVECTOR3( 0.0f, 0.0f, 1.0f ), D3DXVECTOR2( 1.0f, 0.0f ), D3DXVECTOR3(-1.0f, -1.0f, 0.5f ), 
	D3DXVECTOR3(-1.0f,  1.0f, 0.5f ), D3DXVECTOR3( 0.0f, 0.0f, 1.0f ), D3DXVECTOR2( 0.0f, 0.0f ), D3DXVECTOR3(-1.0f, -1.0f, 0.5f )
};
VPos fullScreenQuadVertices[4] = 
{	D3DXVECTOR3(-1.0f, -1.0f, 0.5f ), D3DXVECTOR3( 0.0f, 0.0f, 1.0f ), D3DXVECTOR2( 0.0f, 1.0f ), D3DXVECTOR3(-1.0f, -1.0f, 0.5f ),  
	D3DXVECTOR3( 1.0f, -1.0f, 0.5f ), D3DXVECTOR3( 0.0f, 0.0f, 1.0f ), D3DXVECTOR2( 1.0f, 1.0f ), D3DXVECTOR3(-1.0f, -1.0f, 0.5f ), 
	D3DXVECTOR3( 1.0f,  1.0f, 0.5f ), D3DXVECTOR3( 0.0f, 0.0f, 1.0f ), D3DXVECTOR2( 1.0f, 0.0f ), D3DXVECTOR3(-1.0f, -1.0f, 0.5f ), 
	D3DXVECTOR3(-1.0f,  1.0f, 0.5f ), D3DXVECTOR3( 0.0f, 0.0f, 1.0f ), D3DXVECTOR2( 0.0f, 0.0f ), D3DXVECTOR3(-1.0f, -1.0f, 0.5f )
};
unsigned int quadIndices[6] = {0,2,1,0,3,2};

// Cube Map
VPos cubeVertices[8] = 
{	D3DXVECTOR3(-1.0f, -1.0f, -1.0f ), D3DXVECTOR3( -1.0f, -1.0f, 1.0f ), D3DXVECTOR2( 0.0f, 1.0f ), D3DXVECTOR3(-1.0f, -1.0f, 0.5f ),  // blf
	D3DXVECTOR3( 1.0f, -1.0f, -1.0f ), D3DXVECTOR3( 1.0f, -1.0f, 1.0f ), D3DXVECTOR2( 1.0f, 1.0f ), D3DXVECTOR3(-1.0f, -1.0f, 0.5f ),  // brf
	D3DXVECTOR3( 1.0f,  1.0f, -1.0f ), D3DXVECTOR3( 1.0f, 1.0f, 1.0f ), D3DXVECTOR2( 1.0f, 0.0f ), D3DXVECTOR3(-1.0f, -1.0f, 0.5f ),  // trf
	D3DXVECTOR3(-1.0f,  1.0f, -1.0f ), D3DXVECTOR3( -1.0f, 1.0f, 1.0f ), D3DXVECTOR2( 0.0f, 0.0f ), D3DXVECTOR3(-1.0f, -1.0f, 0.5f ),  // tlf
	D3DXVECTOR3(-1.0f, -1.0f,  1.0f ), D3DXVECTOR3( -1.0f, -1.0f, -1.0f ), D3DXVECTOR2( 0.0f, 1.0f ), D3DXVECTOR3(-1.0f, -1.0f, 0.5f ),  // blb
	D3DXVECTOR3( 1.0f, -1.0f,  1.0f ), D3DXVECTOR3( 1.0f, -1.0f, -1.0f ), D3DXVECTOR2( 1.0f, 1.0f ), D3DXVECTOR3(-1.0f, -1.0f, 0.5f ),  // brb
	D3DXVECTOR3( 1.0f,  1.0f,  1.0f ), D3DXVECTOR3( 1.0f, 1.0f, -1.0f ), D3DXVECTOR2( 1.0f, 0.0f ), D3DXVECTOR3(-1.0f, -1.0f, 0.5f ),  // trb
	D3DXVECTOR3(-1.0f,  1.0f,  1.0f ), D3DXVECTOR3( -1.0f, 1.0f, -1.0f ), D3DXVECTOR2( 0.0f, 0.0f ), D3DXVECTOR3(-1.0f, -1.0f, 0.5f )   // tlb
};
unsigned int cubeIndices[36] = 
{
	0,2,1,0,3,2,		// front 
	1,6,5,1,2,6,		// right
	5,7,4,5,6,7,		// back
	4,3,0,4,7,3,		// left
	3,6,2,3,7,6,		// top
	4,1,5,4,0,1			// bottom
};

// World space frustum corners
D3DXVECTOR3 frustumCornersWS[4];

// Point Sprites
float* spriteVertices;

// Single Sphere 
float* spherePos;

// Glass Box
VPosSimple boxVertices[16];
unsigned int boxIndices[84];
// Glass Box Simple
VPosSimple simpleBoxVertices[20];
unsigned int simpleBoxIndices[30];
// Floor
VPosSimple xzPlaneVertices[4];
unsigned int xzPlaneIndices[6];
// Logo
VPosSimple xyPlaneVertices[8];
unsigned int xyPlaneIndices[12];

//------------------------Rendering----------------------//
bool depthInZero = true;
int nMips = 1;
float fov[2];
float focalLengths[2];
float viewport[2];
D3DXVECTOR3 fluid_blf, fluid_trb;
float cameraParams[4];
float eyePt[3];



//--------------------D3D10 Resources---------------------------------------------------------------------------------//
class MyCModelViewerCamera : public CModelViewerCamera
{
public:
	float m_fFOV;
	float m_fAspect;
	float GetFOV(){ return m_fFOV; }
	float GetAspectRatio(){ return m_fAspect; }

	VOID SetProjParams( FLOAT fFOV, FLOAT fAspect, FLOAT fNearPlane,
                                 FLOAT fFarPlane )
{
    // Set attributes for the projection matrix
    m_fFOV = fFOV;
    m_fAspect = fAspect;
    m_fNearPlane = fNearPlane;
    m_fFarPlane = fFarPlane;

    D3DXMatrixPerspectiveFovLH( &m_mProj, fFOV, fAspect, fNearPlane, fFarPlane );
}
};
MyCModelViewerCamera          g_Camera;					// A model viewing camera
CDXUTDialogResourceManager  g_DialogResourceManager;	// manager for shared resources of dialogs
CD3DSettingsDlg             g_D3DSettingsDlg;			// Device settings dialog
CDXUTTextHelper*            g_pTxtHelper = NULL;
CDXUTDialog                 g_HUD;						// dialog for standard controls
CDXUTDialog                 g_SampleUI;					// dialog for sample specific controls

ID3DX10Font*                        g_pFont10 = NULL;
ID3DX10Sprite*                      g_pSprite10 = NULL;
ID3D10InputLayout*					g_pVertexLayout = NULL;
ID3D10InputLayout*					g_pVertexLayoutSprites = NULL;
ID3D10InputLayout*					g_pVertexLayoutGlassBox = NULL;
ID3D10Effect*                       g_pEffect10 = NULL;
ID3D10EffectTechnique*				g_pRenderScene = NULL;
ID3D10EffectTechnique*				g_pRenderDepthThicknessSprites = NULL;
ID3D10EffectTechnique*				g_pSmoothDepth = NULL;
ID3D10EffectTechnique*				g_pShowTextureDebug = NULL;
ID3D10EffectTechnique*				g_pComposite = NULL;
ID3D10EffectTechnique*				g_pRenderGlassBox = NULL;
ID3D10EffectMatrixVariable*			g_pWorldVariable = NULL;
ID3D10EffectMatrixVariable*			g_pViewVariable = NULL;
ID3D10EffectMatrixVariable*			g_pInvViewVariable = NULL;
ID3D10EffectMatrixVariable*			g_pProjectionVariable = NULL;
ID3D10EffectMatrixVariable*			g_pCameraProjectionVariable = NULL;
ID3D10EffectMatrixVariable*			g_wvpVariable = NULL;               
ID3D10EffectVectorVariable*			g_LightDiffuseVariable = NULL;
ID3D10EffectVectorVariable*			g_MaterialAmbientColorVariable = NULL;      // Material's ambient color
ID3D10EffectVectorVariable*			g_MaterialDiffuseColorVariable = NULL;      // Material's diffuse color
ID3D10EffectVectorVariable*			g_LightDirVariable = NULL;                  // Light's direction in world space
ID3D10EffectVectorVariable*			g_fovVariable = NULL;
ID3D10EffectVectorVariable*			g_viewportVariable = NULL;
ID3D10EffectScalarVariable*			g_timeStepVariable = NULL;
ID3D10EffectSamplerVariable*		g_samplerLinearVariable = NULL;
ID3D10EffectShaderResourceVariable*	g_depthTexVariable = NULL;
ID3D10Buffer*						g_spriteGeometry1  = NULL;
ID3D10Buffer*						g_spriteGeometry2  = NULL;
ID3D10Buffer*						g_surfaceNormals = NULL;
ID3D10Buffer*						g_sphereGeometry = NULL;
ID3D10Buffer*						g_sphereIndices = NULL;
ID3D10Buffer*						g_WSQuadGeometry = NULL;
ID3D10Buffer*						g_FSQuadGeometry = NULL;
ID3D10Buffer*						g_quadIndices = NULL;
ID3D10Buffer*						g_spriteGeometry  = NULL;
ID3D10Buffer*						g_singleSphereGeometry  = NULL;
ID3D10Buffer*						g_cubeMapGeometry  = NULL;
ID3D10Buffer*						g_cubeIndices  = NULL;
ID3D10Buffer*						g_sceneIndices = NULL;
ID3D10Buffer*						g_sceneGeometry = NULL;
ID3D10Texture2D*					g_pDepthTex[2] = {NULL,NULL};
ID3D10Texture2D*					g_pDebugTex = NULL;
ID3D10Texture2D*					g_pDepthStencil = NULL;
ID3D10Texture2D*					g_pBackgroundTex = NULL;
ID3D10Texture2D*					g_pThicknessTex = NULL;
ID3D10Resource*						g_pEnvMap = NULL;
ID3D10RenderTargetView*				pRTs[6];
ID3D10RenderTargetView*				g_pRTViewDepth[2];
ID3D10RenderTargetView*				g_pRTViewThickness = NULL;
ID3D10RenderTargetView*				g_pRTViewDebug = NULL;
ID3D10RenderTargetView*				g_pRTViewBackground = NULL;
ID3D10ShaderResourceView*			g_pTextureDepthView[2];
ID3D10ShaderResourceView*			g_pTextureCubeMapView = NULL;
ID3D10ShaderResourceView*			g_pTextureThicknessView = NULL;
ID3D10ShaderResourceView*			g_pTextureDebugView = NULL;
ID3D10ShaderResourceView*			g_pTextureBackgroundView = NULL;
ID3D10ShaderResourceView*			g_pDSView = NULL;
ID3D10ShaderResourceView*			g_pFloorTexView = NULL;
ID3D10DepthStencilView*				g_pDepthStencilView = NULL;
D3DXVECTOR3							g_LightDir(10,10,10);
D3DXVECTOR4							g_LightDiffuse(1.0f,1.0f,1.0f,1.0f);
D3DXVECTOR4							g_MaterialAmbient(0.1f,0.1f,0.1f,0.1f);
D3DXVECTOR4							g_MaterialDiffuse(0.8f,0.8f,0.8f,1.0f);
ID3DX10Mesh*						g_SphereMesh;
D3DXMATRIX							g_mWorld, g_mView, g_mProj, g_mWorldViewProjection, g_mInvView;

////////////////////////////////////////////////////////////////////////////////////////
// --- Begin DXUT CALLBACKS ------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////

//--------------------------------------------------------------------------------------
// Handles the GUI events
//--------------------------------------------------------------------------------------
void CALLBACK OnGUIEvent( UINT nEvent, int nControlID, CDXUTControl* pControl, void* pUserContext )
{
    switch( nControlID )
    {
        case IDC_TOGGLEFULLSCREEN:
            DXUTToggleFullScreen(); break;
        case IDC_TOGGLEREF:
            DXUTToggleREF(); break;
        case IDC_TOGGLEWARP:
            DXUTToggleWARP(); break;
        case IDC_CHANGEDEVICE:
            g_D3DSettingsDlg.SetActive( !g_D3DSettingsDlg.IsActive() ); break;
    }
}

//--------------------------------------------------------------------------------------
// Reject any D3D10 devices that aren't acceptable by returning false
//--------------------------------------------------------------------------------------
bool CALLBACK IsD3D10DeviceAcceptable( UINT Adapter, UINT Output, D3D10_DRIVER_TYPE DeviceType,
                                       DXGI_FORMAT BackBufferFormat, bool bWindowed, void* pUserContext )
{
    return true;
}


//--------------------------------------------------------------------------------------
// Called right before creating a D3D9 or D3D10 device, allowing the app to modify the device settings as needed
//--------------------------------------------------------------------------------------
bool CALLBACK ModifyDeviceSettings( DXUTDeviceSettings* pDeviceSettings, void* pUserContext )
{
    return true;
}

//--------------------------------------------------------------------------------------
// Handle updates to the scene.  This is called regardless of which D3D API is used
//--------------------------------------------------------------------------------------
void CALLBACK OnFrameMove( double fTime, float fElapsedTime, void* pUserContext )
{
    // Update the camera's position based on user input 
    g_Camera.FrameMove( fElapsedTime );
}

//--------------------------------------------------------------------------------------
// Create any D3D10 resources that aren't dependant on the back buffer
//--------------------------------------------------------------------------------------
HRESULT CALLBACK OnD3D10CreateDevice( ID3D10Device* pd3dDevice, const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc,
                                      void* pUserContext )
{
    HRESULT hr;

	V_RETURN( g_DialogResourceManager.OnD3D10CreateDevice( pd3dDevice ) );
    V_RETURN( g_D3DSettingsDlg.OnD3D10CreateDevice( pd3dDevice ) );
    V_RETURN( D3DX10CreateFont( pd3dDevice, 15, 0, FW_BOLD, 1, FALSE, DEFAULT_CHARSET,
                                OUT_DEFAULT_PRECIS, DEFAULT_QUALITY, DEFAULT_PITCH | FF_DONTCARE,
                                L"Century Gothic", &g_pFont10 ) );
    V_RETURN( D3DX10CreateSprite( pd3dDevice, 512, &g_pSprite10 ) );
    g_pTxtHelper = new CDXUTTextHelper( NULL, NULL, g_pFont10, g_pSprite10, 15 );

    // Read the D3DX effect file
    WCHAR str[MAX_PATH];
    V_RETURN( DXUTFindDXSDKMediaFileCch( str, MAX_PATH, L"sph.fx" ) );
    DWORD dwShaderFlags = D3D10_SHADER_ENABLE_STRICTNESS;
#if defined( DEBUG ) || defined( _DEBUG )
    // Set the D3D10_SHADER_DEBUG flag to embed debug information in the shaders.
    // Setting this flag improves the shader debugging experience, but still allows 
    // the shaders to be optimized and to run exactly the way they will run in 
    // the release configuration of this program.
    dwShaderFlags |= D3D10_SHADER_DEBUG;
#endif
    V_RETURN( D3DX10CreateEffectFromFile( str, NULL, NULL, "fx_4_0", dwShaderFlags, 0, pd3dDevice, NULL,
                                              NULL, &g_pEffect10, NULL, NULL ) );
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Obtain handles to effect parameters
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 1. Techniques
	g_pRenderScene = g_pEffect10->GetTechniqueByName("RenderScene");
	g_pRenderDepthThicknessSprites = g_pEffect10->GetTechniqueByName("RenderDepthThicknessSprites");
	g_pSmoothDepth = g_pEffect10->GetTechniqueByName("SmoothDepth");
	g_pShowTextureDebug = g_pEffect10->GetTechniqueByName("ShowTextureDebug");
	g_pComposite = g_pEffect10->GetTechniqueByName("CompositingPass");
	g_pRenderGlassBox = g_pEffect10->GetTechniqueByName("RenderGlassBox");
    
	// 2. Variables
    g_pWorldVariable				= g_pEffect10->GetVariableByName( "g_mWorld" )->AsMatrix();
	g_wvpVariable					= g_pEffect10->GetVariableByName( "g_mWorldViewProjection" )->AsMatrix();
	g_pCameraProjectionVariable		= g_pEffect10->GetVariableByName( "g_mCameraProjection" )->AsMatrix();
	g_pViewVariable					= g_pEffect10->GetVariableByName( "g_mView" )->AsMatrix();
	g_pProjectionVariable			= g_pEffect10->GetVariableByName( "g_mProj" )->AsMatrix();
	g_pInvViewVariable				= g_pEffect10->GetVariableByName( "g_mInvView" )->AsMatrix();
	g_LightDiffuseVariable = g_pEffect10->GetVariableByName( "g_LightDiffuse" )->AsVector();
	g_LightDirVariable = g_pEffect10->GetVariableByName( "g_vLightDir" )->AsVector();
	g_MaterialAmbientColorVariable = g_pEffect10->GetVariableByName( "g_MaterialAmbientColor" )->AsVector();
	g_MaterialDiffuseColorVariable = g_pEffect10->GetVariableByName( "g_MaterialDiffuseColor" )->AsVector();
	g_fovVariable = g_pEffect10->GetVariableByName( "g_cameraFOV" )->AsVector();
	g_viewportVariable = g_pEffect10->GetVariableByName( "g_viewport" )->AsVector();
	g_timeStepVariable = g_pEffect10->GetVariableByName( "g_timeStep" )->AsScalar();
	g_depthTexVariable = g_pEffect10->GetVariableByName( "g_txDepth" )->AsShaderResource();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Create Input Layouts
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 1. Layout for the object data - spheres
    const D3D10_INPUT_ELEMENT_DESC layout[] =
    {
        { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0,  0, D3D10_INPUT_PER_VERTEX_DATA, 0 },
        { "NORMAL",   0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D10_INPUT_PER_VERTEX_DATA, 0 },
        { "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT,    0, 24, D3D10_INPUT_PER_VERTEX_DATA, 0 },
        { "TEXCOORD", 1, DXGI_FORMAT_R32G32B32_FLOAT,    0, 32, D3D10_INPUT_PER_VERTEX_DATA, 0 }
    };
    D3D10_PASS_DESC PassDesc;
    g_pRenderScene->GetPassByIndex( 0 )->GetDesc( &PassDesc );
    hr = pd3dDevice->CreateInputLayout( layout, sizeof( layout ) / sizeof( layout[0] ),
                                             PassDesc.pIAInputSignature,
                                             PassDesc.IAInputSignatureSize, &g_pVertexLayout );
    if( FAILED( hr ) )
        return hr;


	// 2. Layout for the object data - point sprites
    const D3D10_INPUT_ELEMENT_DESC layout_sprites[] =
    {
        { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0,  0, D3D10_INPUT_PER_VERTEX_DATA, 0 },
    };
    g_pRenderDepthThicknessSprites->GetPassByIndex( 0 )->GetDesc( &PassDesc );
    hr = pd3dDevice->CreateInputLayout( layout_sprites, sizeof( layout_sprites ) / sizeof( layout_sprites[0] ),
                                             PassDesc.pIAInputSignature,
                                             PassDesc.IAInputSignatureSize, &g_pVertexLayoutSprites );
	if(FAILED( hr ) )
		return hr;

	// 3. Layout for the object data - glass box
    const D3D10_INPUT_ELEMENT_DESC layout_box[] =
    {
        { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0,  0, D3D10_INPUT_PER_VERTEX_DATA, 0 },
        { "NORMAL",   0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D10_INPUT_PER_VERTEX_DATA, 0 },
        { "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT,    0, 24, D3D10_INPUT_PER_VERTEX_DATA, 0 },
    };
    g_pRenderGlassBox->GetPassByIndex( 0 )->GetDesc( &PassDesc );
    hr = pd3dDevice->CreateInputLayout( layout_box, sizeof( layout_box ) / sizeof( layout_box[0] ),
                                             PassDesc.pIAInputSignature,
                                             PassDesc.IAInputSignatureSize, &g_pVertexLayoutGlassBox );
	if(FAILED( hr ) )
		return hr;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Create Vertex/Index Buffers
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Generate sphere geometry
	tessellateSphere();

	// 1. Create vertex/index buffer - sphere
	D3D10_BUFFER_DESC vbDesc;
	vbDesc.Usage = D3D10_USAGE_DEFAULT;
	vbDesc.BindFlags = D3D10_BIND_VERTEX_BUFFER;
	vbDesc.ByteWidth = sizeof(VPos)*n_vertices;
	vbDesc.CPUAccessFlags = 0;
	vbDesc.MiscFlags = 0;	
	D3D10_SUBRESOURCE_DATA InitData;
	InitData.pSysMem = sphVertices;
	InitData.SysMemPitch = 0;
	InitData.SysMemSlicePitch = 0;
	V_RETURN( pd3dDevice->CreateBuffer( &vbDesc, &InitData, &g_sphereGeometry) );

	D3D10_BUFFER_DESC ibDesc;
	ibDesc.Usage = D3D10_USAGE_DEFAULT;
	ibDesc.BindFlags = D3D10_BIND_INDEX_BUFFER;
	ibDesc.ByteWidth = sizeof(int)*n_faces*3;
	ibDesc.CPUAccessFlags = 0;
	ibDesc.MiscFlags = 0;
	D3D10_SUBRESOURCE_DATA InitData1;
	InitData1.pSysMem = sphIndices;
	InitData1.SysMemPitch = 0;
	InitData1.SysMemSlicePitch = 0;
	V_RETURN( pd3dDevice->CreateBuffer( &ibDesc, &InitData1, &g_sphereIndices) );


	// 2. Create vertex buffer - sprites
	int j = 0;
	spriteVertices = new float[ maxParticles_ * 4];
	for( unsigned int i = 0; i < maxParticles_; i++ )
	{
		float * ptr = (float*)(spriteVertices+j);
		ptr[0] = 0.0;
		ptr[1] = 0.0;
		ptr[2] = 0.0;
		ptr[3] = 0.0;
		j += 4;
	}
	vbDesc.ByteWidth = sizeof(float)*maxParticles_*4;

//#if defined(USE_DX10_INTEROP)
//	vbDesc.Usage          = D3D10_USAGE_DEFAULT;
//	vbDesc.CPUAccessFlags =  0;
////	InitData.pSysMem = 0;
//#else
	vbDesc.Usage = D3D10_USAGE_DYNAMIC;
	vbDesc.CPUAccessFlags =  D3D10_CPU_ACCESS_WRITE;
//#endif
	vbDesc.BindFlags = D3D10_BIND_VERTEX_BUFFER | D3D10_BIND_SHADER_RESOURCE;
	vbDesc.MiscFlags = D3D10_RESOURCE_MISC_SHARED;
	InitData.pSysMem = spriteVertices;
	InitData.SysMemPitch = 0;
	InitData.SysMemSlicePitch = 0;
	V_RETURN( pd3dDevice->CreateBuffer( &vbDesc, &InitData, &g_spriteGeometry ) );

	// 3. Create vertex buffer - a single sphere
	spherePos = new float[4];
	spherePos[0] = 0.0f;
	spherePos[1] = 0.0f;
	spherePos[2] = 0.0f; 
	spherePos[3] = 1.0f; 

	vbDesc.ByteWidth = sizeof(float)*4;
	vbDesc.Usage = D3D10_USAGE_DYNAMIC;
	vbDesc.CPUAccessFlags =  D3D10_CPU_ACCESS_WRITE;
	InitData.pSysMem = spherePos;
	InitData.SysMemPitch = 0;
	InitData.SysMemSlicePitch = 0;
	V_RETURN( pd3dDevice->CreateBuffer( &vbDesc, &InitData, &g_singleSphereGeometry) );


	// 4. Create vertex/index buffer - full screen quad
	vbDesc.Usage = D3D10_USAGE_DEFAULT;
	vbDesc.BindFlags = D3D10_BIND_VERTEX_BUFFER;
	vbDesc.ByteWidth = sizeof(VPos)*4;
	vbDesc.CPUAccessFlags = 0;
	vbDesc.MiscFlags = 0;
	InitData.pSysMem = fullScreenQuadVertices;
	InitData.SysMemPitch = 0;
	InitData.SysMemSlicePitch = 0;
	V_RETURN( pd3dDevice->CreateBuffer( &vbDesc, &InitData, &g_FSQuadGeometry ) );

	ibDesc.ByteWidth = sizeof(int)*6;
	InitData1.pSysMem = quadIndices;
	InitData1.SysMemPitch = 0;
	InitData1.SysMemSlicePitch = 0;
	V_RETURN( pd3dDevice->CreateBuffer( &ibDesc, &InitData1, &g_quadIndices) );
	

	// 5. Create vertex/index buffer - cube map
	createCubeMapBuffer(pd3dDevice);
	V_RETURN( D3DX10CreateShaderResourceViewFromFile( pd3dDevice, L"floor3.jpg", NULL, NULL, &g_pFloorTexView, NULL ) );

	cameraParams[0] = g_Camera.GetFOV();
	cameraParams[1] = g_Camera.GetAspectRatio();
	cameraParams[2] = g_Camera.GetNearClip();
	cameraParams[3] = g_Camera.GetFarClip();

	return S_OK;
}


//--------------------------------------------------------------------------------------
// Create any D3D10 resources that depend on the back buffer
//--------------------------------------------------------------------------------------
HRESULT CALLBACK OnD3D10ResizedSwapChain( ID3D10Device* pd3dDevice, IDXGISwapChain* pSwapChain,
                                          const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc, void* pUserContext )
{
    HRESULT hr;

	V_RETURN( g_DialogResourceManager.OnD3D10ResizedSwapChain( pd3dDevice, pBackBufferSurfaceDesc ) );
    V_RETURN( g_D3DSettingsDlg.OnD3D10ResizedSwapChain( pd3dDevice, pBackBufferSurfaceDesc ) );

    // Setup the camera's projection parameters
    float fAspectRatio = pBackBufferSurfaceDesc->Width / ( FLOAT )pBackBufferSurfaceDesc->Height;
    g_Camera.SetProjParams( D3DX_PI / 4, fAspectRatio, 2.0f, 2000.0f );
    g_Camera.SetWindow( pBackBufferSurfaceDesc->Width, pBackBufferSurfaceDesc->Height );
    g_Camera.SetButtonMasks( MOUSE_LEFT_BUTTON, MOUSE_WHEEL, MOUSE_MIDDLE_BUTTON );

    g_HUD.SetLocation( pBackBufferSurfaceDesc->Width - 170, 0 );
    g_HUD.SetSize( 170, 170 );
    g_SampleUI.SetLocation( pBackBufferSurfaceDesc->Width - 170, pBackBufferSurfaceDesc->Height - 300 );
    g_SampleUI.SetSize( 170, 300 );

    // Update things that are dependend on the window size
    V_RETURN(InitBackBufferDependentData(pd3dDevice, pBackBufferSurfaceDesc));

	// Compute the frustum corners and create a vertex buffer for a full screen quad
	ComputeFrustumCornersWS();

	worldSpaceQuadVertices[0].FrustumCorner = frustumCornersWS[0]; 
	worldSpaceQuadVertices[1].FrustumCorner = frustumCornersWS[1];
	worldSpaceQuadVertices[2].FrustumCorner = frustumCornersWS[2];
	worldSpaceQuadVertices[3].FrustumCorner = frustumCornersWS[3];

	SAFE_RELEASE( g_WSQuadGeometry );
	D3D10_BUFFER_DESC vbDesc;
	vbDesc.Usage = D3D10_USAGE_DEFAULT;
	vbDesc.BindFlags = D3D10_BIND_VERTEX_BUFFER;
	vbDesc.ByteWidth = sizeof(VPos)*4;
	vbDesc.CPUAccessFlags = 0;
	vbDesc.MiscFlags = 0;
	
	D3D10_SUBRESOURCE_DATA InitData;
	InitData.pSysMem = worldSpaceQuadVertices;
	InitData.SysMemPitch = 0;
	InitData.SysMemSlicePitch = 0;

	V_RETURN( pd3dDevice->CreateBuffer( &vbDesc, &InitData, &g_WSQuadGeometry) );

	g_bFirst = true;

	return hr;
}

//--------------------------------------------------------------------------------------
// Collision geometry for demo scene
//--------------------------------------------------------------------------------------

// the boundaries of the domain
#define XMIN 0.0f
#define XMAX 100.0f
#define YMIN 0.0f
#define YMAX 40.0f 
#define ZMIN 0.0f
#define ZMAX 40.0f


void clearCollisionGeometry( btFluidWorld * world ){
	btfTriangleList triangles;
	btTriangle tri;
	// bottom
	const float x_min = -10000.f;
	const float x_max =  10000.f;
	const float z_min = -10000.f;
	const float z_max =  10000.f;
	const float y_min = -1.f;
	tri.m_vertex0 = btVector3( x_min, y_min, z_min );
	tri.m_vertex1 = btVector3( x_max, y_min, z_max );
	tri.m_vertex2 = btVector3( x_max, y_min, z_min );
	triangles.push_back( tri );
	tri.m_vertex0 = btVector3( x_min, y_min, z_min );
	tri.m_vertex1 = btVector3( x_max, y_min, z_max );
	tri.m_vertex2 = btVector3( x_min, y_min, z_max );
	triangles.push_back( tri );
	world->specifyCollisionTriangleList( triangles );
}


void loadCollisionGeometry( btFluidWorld * world ){
	btfTriangleList triangles;
	btTriangle tri;
	// front 
	tri.m_vertex0 = btVector3( XMIN, YMIN, ZMIN );
	tri.m_vertex1 = btVector3( XMAX, YMIN, ZMIN );
	tri.m_vertex2 = btVector3( XMAX, YMAX, ZMIN );
	triangles.push_back( tri );
	tri.m_vertex0 = btVector3( XMIN, YMIN, ZMIN );
	tri.m_vertex1 = btVector3( XMIN, YMAX, ZMIN );
	tri.m_vertex2 = btVector3( XMAX, YMAX, ZMIN );
	triangles.push_back( tri );
	// back 
	tri.m_vertex0 = btVector3( XMIN, YMIN, ZMAX );
	tri.m_vertex1 = btVector3( XMAX, YMIN, ZMAX );
	tri.m_vertex2 = btVector3( XMAX, YMAX, ZMAX );
	triangles.push_back( tri );
	tri.m_vertex0 = btVector3( XMIN, YMIN, ZMAX );
	tri.m_vertex1 = btVector3( XMIN, YMAX, ZMAX );
	tri.m_vertex2 = btVector3( XMAX, YMAX, ZMAX );
	triangles.push_back( tri );
	// top
	tri.m_vertex0 = btVector3( XMIN, YMAX, ZMIN );
	tri.m_vertex1 = btVector3( XMAX, YMAX, ZMAX );
	tri.m_vertex2 = btVector3( XMAX, YMAX, ZMIN );
	triangles.push_back( tri );
	tri.m_vertex0 = btVector3( XMIN, YMAX, ZMIN );
	tri.m_vertex1 = btVector3( XMAX, YMAX, ZMAX );
	tri.m_vertex2 = btVector3( XMIN, YMAX, ZMAX );
	triangles.push_back( tri );
	// bottom
	tri.m_vertex0 = btVector3( XMIN, YMIN, ZMIN );
	tri.m_vertex1 = btVector3( XMAX, YMIN, ZMAX );
	tri.m_vertex2 = btVector3( XMAX, YMIN, ZMIN );
	triangles.push_back( tri );
	tri.m_vertex0 = btVector3( XMIN, YMIN, ZMIN );
	tri.m_vertex1 = btVector3( XMAX, YMIN, ZMAX );
	tri.m_vertex2 = btVector3( XMIN, YMIN, ZMAX );
	triangles.push_back( tri );
	// left
	tri.m_vertex0 = btVector3( XMIN, YMIN, ZMIN );
	tri.m_vertex1 = btVector3( XMIN, YMAX, ZMAX );
	tri.m_vertex2 = btVector3( XMIN, YMIN, ZMAX );
	triangles.push_back( tri );
	tri.m_vertex0 = btVector3( XMIN, YMIN, ZMIN );
	tri.m_vertex1 = btVector3( XMIN, YMAX, ZMAX );
	tri.m_vertex2 = btVector3( XMIN, YMAX, ZMIN );
	triangles.push_back( tri );
	// right
	tri.m_vertex0 = btVector3( XMAX, YMIN, ZMIN );
	tri.m_vertex1 = btVector3( XMAX, YMAX, ZMAX );
	tri.m_vertex2 = btVector3( XMAX, YMIN, ZMAX );
	triangles.push_back( tri );
	tri.m_vertex0 = btVector3( XMAX, YMIN, ZMIN );
	tri.m_vertex1 = btVector3( XMAX, YMAX, ZMAX );
	tri.m_vertex2 = btVector3( XMAX, YMAX, ZMIN );
	triangles.push_back( tri );
	world->specifyCollisionTriangleList( triangles );
}

//--------------------------------------------------------------------------------------
// OnD3D10FrameRender: Render the scene using the D3D10 device
//--------------------------------------------------------------------------------------
void CALLBACK OnD3D10FrameRender( ID3D10Device* pd3dDevice, double fTime, float fElapsedTime, void* pUserContext )
{

	if(g_bFirst)
	{
		// Time step the simulation
		AdvanceSimulation();

		// Set effect parameters
		SetEffectParameters();

		float ClearColor[4] = {0.0f, 0.0f, 0.0f, 1.0f};
		pd3dDevice->ClearDepthStencilView( DXUTGetD3D10DepthStencilView(), D3D10_CLEAR_DEPTH, 1.0, 0 );
		pd3dDevice->ClearRenderTargetView( DXUTGetD3D10RenderTargetView(), ClearColor );

		if(g_HUD.GetRadioButton( RENDER_FLUID+4 )->GetChecked() || 
			g_HUD.GetRadioButton( RENDER_SINGLE_SPHERE+4 )->GetChecked())
		{
			RenderDepthThicknessToBuffer(pd3dDevice);
			SmoothDepth(pd3dDevice);
			pd3dDevice->ClearDepthStencilView( DXUTGetD3D10DepthStencilView(), D3D10_CLEAR_DEPTH, 1.0, 0 );
			pd3dDevice->ClearRenderTargetView( pRTs[RTV_BACKGROUND], ClearColor );
			RenderBackground(pd3dDevice, RTV_BACKGROUND);
			Composite(pd3dDevice);
			RenderBackground(pd3dDevice, RTV_DEFAULT);
			if(frameCount<= FRAME_TO_REMOVE_BOX )
				RenderGlassBox(pd3dDevice, RTV_DEFAULT);
			if(takeSnapshot)
			{
				//AnalyzeD3D10Texture(g_pThicknessTex, pd3dDevice);
				AnalyzeD3D10Texture(g_pDebugTex, pd3dDevice);
				//if(depthInZero)
				//	AnalyzeD3D10Texture(g_pDepthTex[0], pd3dDevice);
				//else
				//	AnalyzeD3D10Texture(g_pDepthTex[1], pd3dDevice);
				//AnalyzeD3D10Texture(g_pDepthStencil, pd3dDevice);
				takeSnapshot = false;
			}

			if( frameCount == 0 ){
				loadCollisionGeometry( world_ );
			}
			else if( frameCount == FRAME_TO_REMOVE_BOX ){
				clearCollisionGeometry( world_ );
			}

			//g_bFirst = false;
		}
		else if (g_HUD.GetRadioButton( RENDER_PARTICLES_POINTS+4 )->GetChecked())
		{
			//ID3D10RenderTargetView *pRTs[1] = {g_pRTViewDebug};
			ID3D10RenderTargetView *pRTs[1] = {DXUTGetD3D10RenderTargetView()};
			pd3dDevice->ClearDepthStencilView( g_pDepthStencilView, D3D10_CLEAR_DEPTH, 1.0, 0 );
			//pd3dDevice->ClearRenderTargetView( g_pRTViewDepth[0], ClearColor);
			pd3dDevice->OMSetRenderTargets(1, &pRTs[0], g_pDepthStencilView);
			RenderParticlesWithPoints(pd3dDevice);
		}
	}

	//if(g_HUD.GetRadioButton( RENDER_PARTICLES_POINTS+4 )->GetChecked())
	//	ShowTextureDebug(pd3dDevice, g_pTextureDebugView);
	//else
	//{
	//	//if(depthInZero)
	//	//	ShowTextureDebug(pd3dDevice, g_pTextureDepthView[0]);
	//	//else
	//	//	ShowTextureDebug(pd3dDevice, g_pTextureDepthView[1]);
	//ShowTextureDebug(pd3dDevice, g_pTextureBackgroundView);
	//	ShowTextureDebug(pd3dDevice, g_pTextureDebugView);
	//	//ShowTextureDebug(pd3dDevice, g_pTextureThicknessView);
	//}

	//TestDepthCalculations(pd3dDevice);

	if(bHelp)
	{
		/////////////////////////////////////////////////
		// Render screen text and perf data
		/////////////////////////////////////////////////
		DXUT_BeginPerfEvent( DXUT_PERFEVENTCOLOR, L"HUD / Stats" );
		g_HUD.OnRender( fElapsedTime );
		g_SampleUI.OnRender( fElapsedTime );
		RenderText();
		DXUT_EndPerfEvent();

		static DWORD dwTimefirst = GetTickCount();
		if ( GetTickCount() - dwTimefirst > 5000 )
		{    
			OutputDebugString( DXUTGetFrameStats( DXUTIsVsyncEnabled() ) );
			OutputDebugString( L"\n" );
			dwTimefirst = GetTickCount();
		}
	}
}


//--------------------------------------------------------------------------------------
// OnD3D10ReleasingSwapChain: Release D3D10 resources created in OnD3D10ResizedSwapChain 
//--------------------------------------------------------------------------------------
void CALLBACK OnD3D10ReleasingSwapChain( void* pUserContext )
{
}


//--------------------------------------------------------------------------------------
// OnD3D10DestroyDevice: Release D3D10 resources created in OnD3D10CreateDevice 
//--------------------------------------------------------------------------------------
void CALLBACK OnD3D10DestroyDevice( void* pUserContext )
{
    g_DialogResourceManager.OnD3D10DestroyDevice();
    g_D3DSettingsDlg.OnD3D10DestroyDevice();
    DXUTGetGlobalResourceCache().OnDestroyDevice();

    SAFE_RELEASE( g_pVertexLayout );
    SAFE_RELEASE( g_pVertexLayoutSprites );
    SAFE_RELEASE( g_pVertexLayoutGlassBox );
	SAFE_DELETE( g_pTxtHelper );
    SAFE_RELEASE( g_pFont10 );
    SAFE_RELEASE( g_pSprite10 );
	SAFE_RELEASE( g_pEffect10 );
	SAFE_RELEASE( g_sphereGeometry );
	SAFE_RELEASE( g_spriteGeometry );
	SAFE_RELEASE( g_singleSphereGeometry );
	SAFE_RELEASE( g_sphereIndices );
	SAFE_RELEASE( g_WSQuadGeometry );
	SAFE_RELEASE( g_FSQuadGeometry );
	SAFE_RELEASE( g_sceneGeometry );
	SAFE_RELEASE( g_quadIndices );
	SAFE_RELEASE( g_sceneIndices );
	SAFE_RELEASE( g_pRTViewDepth[0] );
	SAFE_RELEASE( g_pRTViewDepth[1] );
	SAFE_RELEASE( g_pRTViewThickness );
	SAFE_RELEASE( g_pRTViewDebug );
	SAFE_RELEASE( g_pRTViewBackground );
	SAFE_RELEASE( g_pDebugTex );
	SAFE_RELEASE( g_pDepthTex[0] );
	SAFE_RELEASE( g_pDepthTex[1] );
	SAFE_RELEASE( g_pThicknessTex );
	SAFE_RELEASE( g_pBackgroundTex );
	SAFE_RELEASE( g_pDepthStencil );
	SAFE_RELEASE( g_pTextureDepthView[0] );
	SAFE_RELEASE( g_pTextureDepthView[1] );
	SAFE_RELEASE( g_pTextureThicknessView );
	SAFE_RELEASE( g_pTextureBackgroundView );
	SAFE_RELEASE( g_pDSView );
	SAFE_RELEASE( g_pTextureDebugView );
	SAFE_RELEASE( g_pDepthStencilView );
	SAFE_RELEASE( g_pTextureCubeMapView );
	SAFE_RELEASE( g_pFloorTexView );

	// Release CL resources
	// sph->terminate();
}


//--------------------------------------------------------------------------------------
// MsgProc: Handle messages to the application
//--------------------------------------------------------------------------------------
LRESULT CALLBACK MsgProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam,
                          bool* pbNoFurtherProcessing, void* pUserContext )
{
    // Pass messages to dialog resource manager calls so GUI state is updated correctly
    *pbNoFurtherProcessing = g_DialogResourceManager.MsgProc( hWnd, uMsg, wParam, lParam );
    if( *pbNoFurtherProcessing )
        return 0;

    // Pass messages to settings dialog if its active
    if( g_D3DSettingsDlg.IsActive() )
    {
        g_D3DSettingsDlg.MsgProc( hWnd, uMsg, wParam, lParam );
        return 0;
    }

    // Give the dialogs a chance to handle the message first
    *pbNoFurtherProcessing = g_HUD.MsgProc( hWnd, uMsg, wParam, lParam );
    if( *pbNoFurtherProcessing )
        return 0;
    *pbNoFurtherProcessing = g_SampleUI.MsgProc( hWnd, uMsg, wParam, lParam );
    if( *pbNoFurtherProcessing )
        return 0;

    // Pass all remaining windows messages to camera so it can respond to user input
    g_Camera.HandleMessages( hWnd, uMsg, wParam, lParam );
	
	return 0;
}


//--------------------------------------------------------------------------------------
// OnKeyboard: Handle key presses
//--------------------------------------------------------------------------------------
void CALLBACK OnKeyboard( UINT nChar, bool bKeyDown, bool bAltDown, void* pUserContext )
{
	if( bKeyDown )
	{
		switch(nChar)
		{
			case 'p': case 'P': takeSnapshot = true; break;
			case 'h': case 'H':	bHelp = !bHelp; break;
			case ' ':		
				bPause = !bPause;	break;
			default:
				break;
			}
		}
}


//--------------------------------------------------------------------------------------
// OnMouse: Handle mouse button presses
//--------------------------------------------------------------------------------------
void CALLBACK OnMouse( bool bLeftButtonDown, bool bRightButtonDown, bool bMiddleButtonDown,
                       bool bSideButton1Down, bool bSideButton2Down, int nMouseWheelDelta,
                       int xPos, int yPos, void* pUserContext )
{
}


//--------------------------------------------------------------------------------------
// OnDeviceRemoved: Call if device was removed.  Return true to find a new device, false to quit
//--------------------------------------------------------------------------------------
bool CALLBACK OnDeviceRemoved( void* pUserContext )
{
    return true;
}
////////////////////////////////////////////////////////////////////////////////////////
// --- End DXUT CALLBACKS --------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////
// --- Begin Rendering functions -------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////
//--------------------------------------------------------------------------------------
// SetEffectParameters: Set all the common effect parameters 
//--------------------------------------------------------------------------------------
void SetEffectParameters()
{
	HRESULT hr;

	eyePt[0]= g_Camera.GetEyePt()->x;
	eyePt[1]=g_Camera.GetEyePt()->y;
	eyePt[2]=g_Camera.GetEyePt()->z;

	g_mWorld = *g_Camera.GetWorldMatrix();
	g_mView = *g_Camera.GetViewMatrix();
	g_mProj = *g_Camera.GetProjMatrix();
	g_mWorldViewProjection = g_mWorld * g_mView * g_mProj;
	D3DXMatrixInverse(&g_mInvView, NULL, &(g_mWorld*g_mView));
	V(g_pWorldVariable->SetMatrix((float*)&g_mWorld));
	V(g_pViewVariable->SetMatrix((float*)&g_mView));
	V(g_pProjectionVariable->SetMatrix((float*)&g_mProj));
	V(g_wvpVariable->SetMatrix((float*)&g_mWorldViewProjection));
	V(g_pInvViewVariable->SetMatrix((float*)&g_mInvView));

	V(g_LightDirVariable->SetFloatVector((float*)&g_LightDir)); 
	V(g_LightDiffuseVariable->SetFloatVector((float*)&g_LightDiffuse));
	V(g_MaterialAmbientColorVariable->SetFloatVector((float*)&g_MaterialAmbient));
	V(g_MaterialDiffuseColorVariable->SetFloatVector((float*)&g_MaterialDiffuse));
	V(g_pEffect10->GetVariableByName( "g_CameraParams" )->AsVector()->SetFloatVector(cameraParams));
	V(g_pEffect10->GetVariableByName( "g_vEyePt" )->AsVector()->SetFloatVector(eyePt));
	V(g_pEffect10->GetVariableByName("g_gamma")->AsScalar()->SetFloat((float)g_HUD.GetSlider(GAMMA+4)->GetValue()));
	V(g_viewportVariable->SetFloatVector(viewport));
	V(g_pEffect10->GetVariableByName("g_fParticleRad")->AsScalar()->SetFloat(float(g_HUD.GetSlider(PARTICLE_RAD+4)->GetValue())/10));
	V(g_pEffect10->GetVariableByName("g_FrustumCorners")->AsVector()->SetFloatVectorArray((float*)frustumCornersWS,0,12));
	V(g_pEffect10->GetVariableByName("g_refractiveIndex")->AsScalar()->SetFloat(float(g_HUD.GetSlider(RI+4)->GetValue()/100.0)));
	V(g_pEffect10->GetVariableByName("g_threshold")->AsScalar()->SetFloat(float(g_HUD.GetSlider(DEPTH_THRESHOLD+4)->GetValue())));
}
//--------------------------------------------------------------------------------------
// ComputeFrustumCornersWS: Compute corners of the frustum in world space
//--------------------------------------------------------------------------------------
void ComputeFrustumCornersWS()
{
	//D3DXVECTOR3* eyePt = *g_Camera.GetEyePt();
	float hFar = 2 * tan(g_Camera.GetFOV()/2) * g_Camera.GetFarClip();
	float wFar = hFar * g_Camera.GetAspectRatio(); 
	float zFar = g_Camera.GetFarClip() + g_Camera.GetEyePt()->z - depthThreshold;

	frustumCornersWS[0].x=-wFar/2;
	frustumCornersWS[0].y=-hFar/2;
	frustumCornersWS[0].z=zFar;

	frustumCornersWS[1].x=wFar/2;
	frustumCornersWS[1].y=-hFar/2;
	frustumCornersWS[1].z=zFar;

	frustumCornersWS[2].x=wFar/2;
	frustumCornersWS[2].y=hFar/2;
	frustumCornersWS[2].z=zFar;

	frustumCornersWS[3].x=-wFar/2;
	frustumCornersWS[3].y=hFar/2;
	frustumCornersWS[3].z=zFar;
}

//--------------------------------------------------------------------------------------
// RenderParticlesWithSprites: Render particles with spheres as sprites
//--------------------------------------------------------------------------------------
void RenderParticlesWithSprites(ID3D10Device* pd3dDevice)
{
	/////////////////////////////////////////////////
	// Set vertex buffer and draw sprites
	/////////////////////////////////////////////////
	pd3dDevice->IASetInputLayout( g_pVertexLayoutSprites );
	unsigned int stride[1] = {sizeof(float)*4};
	unsigned int offset[1] = {0};
	pd3dDevice->IASetVertexBuffers(0,1,&g_spriteGeometry,stride,offset);
	pd3dDevice->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_POINTLIST);

	// Render objects here...
	pd3dDevice->Draw(world_->particleCount(),0);
	//pd3dDevice->Draw(2,0);
}

//--------------------------------------------------------------------------------------
// RenderSingleSphere: Render a single sphere
//--------------------------------------------------------------------------------------
void RenderSingleSphere(ID3D10Device* pd3dDevice, bool useSprites)
{
	if(useSprites)
	{
		/////////////////////////////////////////////////
		// Set vertex buffer and draw sphere
		/////////////////////////////////////////////////
		pd3dDevice->IASetInputLayout( g_pVertexLayoutSprites );
		unsigned int stride[1] = {sizeof(float)*4};
		unsigned int offset[1] = {0};
		pd3dDevice->IASetVertexBuffers(0,1,&g_singleSphereGeometry,stride,offset);
		pd3dDevice->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_POINTLIST);

		g_pRenderDepthThicknessSprites->GetPassByIndex(0)->Apply(0);

		// Render objects here...
		pd3dDevice->Draw(1,0);
	}
	else
	{
		/////////////////////////////////////////////////
		// Set render resources
		/////////////////////////////////////////////////
		pd3dDevice->IASetInputLayout( g_pVertexLayout );
		unsigned int stride[1] = {sizeof(VPos)};
		unsigned int offset[1] = {0};
		pd3dDevice->IASetVertexBuffers(0,1,&g_sphereGeometry,stride,offset);
		pd3dDevice->IASetIndexBuffer(g_sphereIndices, DXGI_FORMAT_R32_UINT, 0);
		pd3dDevice->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

		g_pRenderScene->GetPassByIndex(0)->Apply(0);

		// Render objects here...
		pd3dDevice->DrawIndexed(n_faces*3, 0, 0);
	}
}

//--------------------------------------------------------------------------------------
// RenderParticlesWithPoints: Render particles with points 
//--------------------------------------------------------------------------------------
void RenderParticlesWithPoints(ID3D10Device* pd3dDevice)
{
	HRESULT hr;

	/////////////////////////////////////////////////
	// Set effect parameters
	/////////////////////////////////////////////////
	D3DXMATRIX mWorld = *g_Camera.GetWorldMatrix();
	D3DXMATRIX mView = *g_Camera.GetViewMatrix();
	D3DXMATRIX mProj = *g_Camera.GetProjMatrix();
	D3DXMATRIX mWorldViewProjection, mInvView, mInvProj, mRot;
	mWorldViewProjection = mWorld * mView * mProj;
	D3DXMatrixInverse(&mInvView, NULL, &(mWorld*mView));

	V(g_wvpVariable->SetMatrix((float*)&mWorldViewProjection));
	V(g_pWorldVariable->SetMatrix((float*)&mWorld));
	V(g_pInvViewVariable->SetMatrix((float*)&mInvView));
	V(g_pViewVariable->SetMatrix((float*)&mView));

	/////////////////////////////////////////////////
	// Set vertex buffer and draw sprites
	/////////////////////////////////////////////////
	pd3dDevice->IASetInputLayout( g_pVertexLayoutSprites );
	unsigned int stride[1] = {sizeof(float)*4};
	unsigned int offset[1] = {0};
	pd3dDevice->IASetVertexBuffers(0,1,&g_spriteGeometry,stride,offset);
	pd3dDevice->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_POINTLIST);

	g_pEffect10->GetTechniqueByName("RenderPoints")->GetPassByIndex(0)->Apply(0);

	// Render objects here...
	pd3dDevice->Draw(world_->particleCount(),0);
}

//--------------------------------------------------------------------------------------
// Create Vertex Buffer Object: Create a combined VBO for floor and box
//--------------------------------------------------------------------------------------
HRESULT CreateVBOBoxPlane()
{
	ID3D10Device* pd3dDevice = DXUTGetD3D10Device1();
	HRESULT hr;

	SAFE_RELEASE( g_sceneGeometry );
	SAFE_RELEASE( g_sceneIndices );

	// 1. Create geometry - simple glass box
	//float blf[3] = {-2.0f,-2.0f,-2.0f};
	//float trb[3] = { 2.0f, 2.0f, 2.0f};
	float blf[3] = {fluid_blf.x,fluid_blf.y,fluid_blf.z};
	float trb[3] = {fluid_trb.x,fluid_trb.y,fluid_trb.z};
	generateSimpleBoxVertices(blf, trb);

	// 2. Create geometry - floor
	float lf[2] = {-300.0f, -300.0f};
	float rb[2] = {300.0f, 300.0f};
	generateXZPlaneVertices(lf, rb, fluid_blf.y+0.1f);

	// 3. Create geometry - logo
	lf[0] =  1.0f - (0.5f / g_Camera.GetAspectRatio()); 
	lf[1] =  0.0f - (0.7f);
	rb[1] = -0.2f - (0.7f);
	rb[0] =  lf[0] + (rb[1]-lf[1]) / g_Camera.GetAspectRatio();
	generateXYPlaneVertices(lf, rb, 0.1f);

	VPosSimple *combinedVertices = new VPosSimple[32];
	for(int i = 0; i < 20; i ++)
		combinedVertices[i] = simpleBoxVertices[i];
	for(int i = 20; i < 24; i ++)
		combinedVertices[i] = xzPlaneVertices[i-20];
	for(int i = 24; i < 32; i ++)
		combinedVertices[i] = xyPlaneVertices[i-24];
	unsigned int *combinedIndices = new unsigned int[48];
	for(int i = 0; i < 30; i ++)
		combinedIndices[i] = simpleBoxIndices[i];
	for(int i = 30; i < 36; i ++)
		combinedIndices[i] = xzPlaneIndices[i-30];
	for(int i = 36; i < 48; i ++)
		combinedIndices[i] = xyPlaneIndices[i-36];

	D3D10_BUFFER_DESC vbDesc;
	D3D10_SUBRESOURCE_DATA InitData;
	vbDesc.Usage = D3D10_USAGE_DEFAULT;
	vbDesc.BindFlags = D3D10_BIND_VERTEX_BUFFER;
	vbDesc.ByteWidth = sizeof(simpleBoxVertices) + sizeof(xzPlaneVertices) + sizeof(xyPlaneVertices);
	vbDesc.CPUAccessFlags = 0;
	vbDesc.MiscFlags = 0;
	InitData.pSysMem = combinedVertices;
	InitData.SysMemPitch = 0;
	InitData.SysMemSlicePitch = 0;
	V_RETURN( pd3dDevice->CreateBuffer( &vbDesc, &InitData, &g_sceneGeometry ) );

	D3D10_BUFFER_DESC ibDesc;
	ibDesc.Usage = D3D10_USAGE_DEFAULT;
	ibDesc.BindFlags = D3D10_BIND_INDEX_BUFFER;
	ibDesc.ByteWidth = sizeof(simpleBoxIndices) + sizeof(xzPlaneIndices) + sizeof(xyPlaneIndices);
	ibDesc.CPUAccessFlags = 0;
	ibDesc.MiscFlags = 0;
	InitData.pSysMem = combinedIndices;
	InitData.SysMemPitch = 0;
	InitData.SysMemSlicePitch = 0;
	V_RETURN( pd3dDevice->CreateBuffer( &ibDesc, &InitData, &g_sceneIndices) );

	delete combinedVertices;
	delete combinedIndices;

	return S_OK;
}

//--------------------------------------------------------------------------------------
// createCubeMapBuffer: Create a buffer for the cube map
//--------------------------------------------------------------------------------------
HRESULT createCubeMapBuffer( ID3D10Device* pd3dDevice)
{
	//HRESULT hr;
	//Vector3DF gridMin, gridMax, gridCenter, gridDims;
	//float greaterDim;
	//
	//gridMin = psys.GetGridMin();
	//gridMax = psys.GetGridMax();
	//gridDims = gridCenter = Vector3DF(0,0,0);
	//gridCenter += gridMin;
	//gridCenter += gridMax;
	//gridCenter *= 0.5;

	//gridDims += gridMax;
	//gridDims -= gridMin;
	//gridDims *= 0.5;

	//greaterDim = max(gridDims.x,max(gridDims.y, gridDims.z))*10;

	//// scale and translate cube to fit around fluid 
	//for(int i = 0; i < 8; i++)
	//{
	//	cubeVertices[i].Pos.x *= greaterDim;
	//	cubeVertices[i].Pos.y *= greaterDim;
	//	cubeVertices[i].Pos.z *= greaterDim;

	//	cubeVertices[i].Pos.x += gridCenter.x;
	//	cubeVertices[i].Pos.y += gridCenter.y;
	//	cubeVertices[i].Pos.z += gridCenter.z;
	//}

	//// create the vertex and index buffers for the cube map
	//SAFE_RELEASE(g_cubeMapGeometry);
	//D3D10_BUFFER_DESC vbDesc;
	//vbDesc.Usage = D3D10_USAGE_DEFAULT;
	//vbDesc.BindFlags = D3D10_BIND_VERTEX_BUFFER;
	//vbDesc.ByteWidth = sizeof(VPos)*8;
	//vbDesc.CPUAccessFlags = 0;
	//vbDesc.MiscFlags = 0;
	//
	//D3D10_SUBRESOURCE_DATA InitData;
	//InitData.pSysMem = cubeVertices;
	//InitData.SysMemPitch = 0;
	//InitData.SysMemSlicePitch = 0;

	//// Create vertex buffer to hold vertices of a sphere 
	//V_RETURN( pd3dDevice->CreateBuffer( &vbDesc, &InitData, &g_cubeMapGeometry) );

	//D3D10_BUFFER_DESC ibDesc;
	//ibDesc.Usage = D3D10_USAGE_DEFAULT;
	//ibDesc.BindFlags = D3D10_BIND_INDEX_BUFFER;
	//ibDesc.ByteWidth = sizeof(unsigned int)*36;
	//ibDesc.CPUAccessFlags = 0;
	//ibDesc.MiscFlags = 0;

	//InitData.pSysMem = cubeIndices;
	//InitData.SysMemPitch = 0;
	//InitData.SysMemSlicePitch = 0;

	//V_RETURN( pd3dDevice->CreateBuffer( &ibDesc, &InitData, &g_cubeIndices) );

	return S_OK;
}

//--------------------------------------------------------------------------------------
// AnalyzeD3D10Texture: Examine the texture using imdebug
//--------------------------------------------------------------------------------------
int AnalyzeD3D10Texture(ID3D10Texture2D* tex, ID3D10Device* pd3dDevice)
{
	D3D10_MAPPED_TEXTURE2D mappedTexture;
	ID3D10Texture2D* stgTex;
	D3D10_TEXTURE2D_DESC texDesc;
	tex->GetDesc(&texDesc);

	texDesc.CPUAccessFlags = D3D10_CPU_ACCESS_READ;
	texDesc.BindFlags = 0;
	texDesc.Usage = D3D10_USAGE_STAGING;

	if( FAILED( pd3dDevice->CreateTexture2D(&texDesc, NULL, &stgTex) ) )    {
        return E_FAIL;
    }

	pd3dDevice->CopyResource(stgTex, tex);

	if(stgTex->Map(0,D3D10_MAP_READ,0,&mappedTexture) == S_OK)
	{
		float* pTexels = (float*)mappedTexture.pData;
		switch(texDesc.Format)
		{
		case DXGI_FORMAT_R32_FLOAT: case DXGI_FORMAT_R32_TYPELESS: 
//			imdebug("r b=32f w=%d h=%d %p",texDesc.Width, texDesc.Height, pTexels);
			break;
		case DXGI_FORMAT_R32G32B32A32_FLOAT:
//			imdebug("rgba b=32f w=%d h=%d %p",texDesc.Width, texDesc.Height, pTexels);
			break;
		default:
			break;
		}
	}

	SAFE_RELEASE(stgTex);

	return 1;
}

//--------------------------------------------------------------------------------------
// Composite: The final rendering pass, composite the fluid image
//--------------------------------------------------------------------------------------
void Composite( ID3D10Device* pd3dDevice )
{
	int source;

	/////////////////////////////////////////////////
	// Set render resources
	///////////////////////////////////////// ////////
	pd3dDevice->IASetInputLayout( g_pVertexLayout );
	unsigned int stride[1] = {sizeof(VPos)};
	unsigned int offset[1] = {0};
	pd3dDevice->IASetVertexBuffers(0,1,&g_WSQuadGeometry,stride,offset);
	pd3dDevice->IASetIndexBuffer(g_quadIndices, DXGI_FORMAT_R32_UINT, 0);
	pd3dDevice->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

	/////////////////////////////////////////////////
	// Select render target
	/////////////////////////////////////////////////
	if(depthInZero)		{
		source = 0;
	}
	else		{
		source = 1;
	}

	/////////////////////////////////////////////////
	// Set render target and the depth stencil view
	/////////////////////////////////////////////////
	pd3dDevice->OMSetRenderTargets( 1, &pRTs[RTV_DEFAULT], DXUTGetD3D10DepthStencilView() );

	/////////////////////////////////////////////////
	// Set scene variables
	/////////////////////////////////////////////////
	g_pEffect10->GetVariableByName("g_txDepth")->AsShaderResource()->SetResource(g_pTextureDepthView[source]);
	g_pEffect10->GetVariableByName("g_txThickness")->AsShaderResource()->SetResource(g_pTextureThicknessView);
	g_pEffect10->GetVariableByName("g_txBackground")->AsShaderResource()->SetResource(g_pTextureBackgroundView);

	// Render objects here...
	g_pComposite->GetPassByIndex(0)->Apply(0);
	pd3dDevice->DrawIndexed(6, 0, 0);
}

//--------------------------------------------------------------------------------------
// AdvanceSimulation: Advance the simulation by a single time step
//--------------------------------------------------------------------------------------

#define LAST_FRAME 1600

void AdvanceSimulation()
{
	//////////////////////////////////////
	// Advance particles
	//////////////////////////////////////
	//if ( !bPause ) psys.Run ();
	if ( !bPause ){
		
		if(frameCount < LAST_FRAME )
			frameCount++; 
		else 
		{
			frameCount = 0;
			//initializeSimulation( sph );//to do reinstate this
		}

		//sph->run();
		//step( sph );
		world_->step();
	}

#if defined(USE_CL_INTEROP)
	if ( !psys.GetToggle(USE_GPU) ) {
#endif
		/////////////////////////////////////////////////
		// Update vertex buffer - @Saif: this is slowwww!!!
		/////////////////////////////////////////////////
		//int j = 0;
		//buffer = psys.GetBuffer(0);
		//dat = buffer->data;	
		//numPoints = psys.NumPoints();
//		printf("numPoint = %d\n", numPoints);
#if 0
		for (int n = 0; n < numPoints; n++) {
			p = (Point*) dat;
			dat += buffer->stride;
			spriteVertices[j] = p->pos.x; j++;
			spriteVertices[j] = p->pos.y; j++;
			spriteVertices[j] = p->pos.z; j++;
		}
#endif

#if 0
		hr = g_spriteGeometry->Map(D3D10_MAP_WRITE_DISCARD,0,(void**)&spritePtr);
		//	memcpy(spritePtr, spriteVertices, sizeof(float)*psys.NumPoints()*3);
		//  memcpy(spritePtr, buffer->data, (sizeof(Fluid)) * psys.NumPoints());

		cl_int err = sph->queue.enqueueReadBuffer( 
			sph->position, 
			CL_TRUE, 
			0, 
			fluid_->particleCount() * 4 * sizeof( float ), 
			(void*)spritePtr );		 
		g_spriteGeometry->Unmap();
#endif

#if 1
		HRESULT hr;
		float* spritePtr;
		hr = g_spriteGeometry->Map(D3D10_MAP_WRITE_DISCARD,0,(void**)&spritePtr);
		memcpy(spritePtr, fluid_->m_descriptor->m_particlesWriteData->m_position, 
			sizeof(float)*4*fluid_->m_descriptor->particleCount() );
		g_spriteGeometry->Unmap();
#endif

#if defined(USE_CL_INTEROP)
	}
#endif
}

//--------------------------------------------------------------------------------------
// SmoothDepth: Smoothing pass, smooth the depth buffer using Gaussian or Curvature Flow
//--------------------------------------------------------------------------------------
void SmoothDepth( ID3D10Device* pd3dDevice )
{
	int source, dest;
	ID3D10RenderTargetView *pRTs[1];


	/////////////////////////////////////////////////
	// Set render resources
	///////////////////////////////////////// ////////
	iterations = g_HUD.GetSlider( ITERATIONS+4 )->GetValue();
	//iterations = 1;
	pd3dDevice->IASetInputLayout( g_pVertexLayout );
	unsigned int stride[1] = {sizeof(VPos)};
	unsigned int offset[1] = {0};
	pd3dDevice->IASetVertexBuffers(0,1,&g_FSQuadGeometry,stride,offset);
	pd3dDevice->IASetIndexBuffer(g_quadIndices, DXGI_FORMAT_R32_UINT, 0);
	pd3dDevice->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

	for(int i = 0; i < iterations; i++)
	{
		/////////////////////////////////////////////////
		// Select render target
		/////////////////////////////////////////////////
		if(depthInZero)
		{
			source = 0;
			dest = 1;
		}
		else
		{
			source = 1;
			dest = 0;
		}
		pRTs[0] = g_pRTViewDepth[dest];

		/////////////////////////////////////////////////
		// Clear render target and the depth stencil 
		/////////////////////////////////////////////////
		pd3dDevice->OMSetRenderTargets( 1, pRTs, DXUTGetD3D10DepthStencilView() );

		/////////////////////////////////////////////////
		// Set scene variables
		/////////////////////////////////////////////////
		g_pEffect10->GetVariableByName("g_txDepth")->AsShaderResource()->SetResource(g_pTextureDepthView[source]);

		// Render objects here...
		g_pSmoothDepth->GetPassByIndex(0)->Apply(0);
		pd3dDevice->DrawIndexed(6, 0, 0);
		depthInZero = !depthInZero;
	}
}

//--------------------------------------------------------------------------------------
// TestDepthCalculations: Test the depth calculated by pixel shader
//--------------------------------------------------------------------------------------
void TestDepthCalculations(ID3D10Device* pd3dDevice)
{
	HRESULT hr;

	float ClearColor[4] = {1.0f, 1.0f, 1.0f, 1.0f};
	ID3D10RenderTargetView *pRTs[1] = {g_pRTViewDebug};
	pd3dDevice->ClearDepthStencilView( g_pDepthStencilView, D3D10_CLEAR_DEPTH, 1.0, 0 );
	pd3dDevice->ClearRenderTargetView( g_pRTViewDebug, ClearColor );
	pd3dDevice->OMSetRenderTargets(1, &pRTs[0], g_pDepthStencilView);
	
	/////////////////////////////////////////////////////////////
	// Set effect parameters
	/////////////////////////////////////////////////////////////
	D3DXMATRIX mWorld = *g_Camera.GetWorldMatrix();
	D3DXMATRIX mView = *g_Camera.GetViewMatrix();
	D3DXMATRIX mProj = *g_Camera.GetProjMatrix();
	D3DXMATRIX mWorldViewProjection, mInvView, mInvProj, mRot;

	float cameraParams[4];
	cameraParams[0] = g_Camera.GetFOV();
	cameraParams[1] = g_Camera.GetAspectRatio();
	cameraParams[2] = g_Camera.GetNearClip();
	cameraParams[3] = g_Camera.GetFarClip();
	float eyePt[3];
	eyePt[0]= g_Camera.GetEyePt()->x;
	eyePt[1]=g_Camera.GetEyePt()->y;
	eyePt[2]=g_Camera.GetEyePt()->z;

	D3DXMatrixRotationX(&mRot, float(-PI/2));
	mWorld = mRot * mWorld;
	D3DXMatrixInverse(&mInvView, NULL, &(mWorld*mView));
	D3DXMatrixInverse(&mInvProj, NULL, &(mProj));
	mWorldViewProjection = mWorld * mView * mProj;
	V(g_pWorldVariable->SetMatrix((float*)&mWorld));
	V(g_wvpVariable->SetMatrix((float*)&mWorldViewProjection));
	V(g_pInvViewVariable->SetMatrix((float*)&mInvView));
	V(g_pViewVariable->SetMatrix((float*)&mView));
	V(g_pProjectionVariable->SetMatrix((float*)&mProj));
	V(g_pEffect10->GetVariableByName( "g_CameraParams" )->AsVector()->SetFloatVector(cameraParams));
	V(g_pEffect10->GetVariableByName( "g_vEyePt" )->AsVector()->SetFloatVector(eyePt));
	V(g_pEffect10->GetVariableByName("g_txDepth")->AsShaderResource()->SetResource(g_pTextureDepthView[0]));
	g_pEffect10->GetVariableByName("g_gamma")->AsScalar()->SetFloat((float)g_HUD.GetSlider(GAMMA+4)->GetValue());
	g_viewportVariable->SetFloatVector(viewport);
	g_LightDirVariable->SetFloatVector((float*)&g_LightDir); 
	g_LightDiffuseVariable->SetFloatVector((float*)&g_LightDiffuse);
	g_MaterialAmbientColorVariable->SetFloatVector((float*)&g_MaterialAmbient);
	g_MaterialDiffuseColorVariable->SetFloatVector((float*)&g_MaterialDiffuse);

	// Render geometry sphere
	RenderSingleSphere(pd3dDevice);
	if(takeSnapshot)
	{
		AnalyzeD3D10Texture(g_pDepthStencil, pd3dDevice);
	}

	// Render sprite sphere
	pd3dDevice->ClearDepthStencilView( g_pDepthStencilView, D3D10_CLEAR_DEPTH, 1.0, 0 );
	pd3dDevice->ClearRenderTargetView( g_pRTViewDebug, ClearColor );
	RenderSingleSphere(pd3dDevice, true);
	if(takeSnapshot)
	{
		AnalyzeD3D10Texture(g_pDepthStencil, pd3dDevice);
		takeSnapshot = false;
	}

	ShowTextureDebug(pd3dDevice, g_pTextureDebugView);
}


//--------------------------------------------------------------------------------------
// ShowTextureDebug: Debug function to display the contents of a texture to screen
//--------------------------------------------------------------------------------------
void ShowTextureDebug( ID3D10Device* pd3dDevice, ID3D10ShaderResourceView* resView )
{
	HRESULT hr;

	/////////////////////////////////////////////////
	// Clear render target and the depth stencil 
	/////////////////////////////////////////////////
    float ClearColor[4] = {1.0f, 1.0f, 1.0f, 1.0f};
	ID3D10RenderTargetView *pRTs[] = {DXUTGetD3D10RenderTargetView()};
	pd3dDevice->OMSetRenderTargets( 1, pRTs, DXUTGetD3D10DepthStencilView() );
	pd3dDevice->ClearRenderTargetView( DXUTGetD3D10RenderTargetView(), ClearColor );
    pd3dDevice->ClearDepthStencilView( DXUTGetD3D10DepthStencilView(), D3D10_CLEAR_DEPTH, 1.0, 0 );

	/////////////////////////////////////////////////
	// Set render resources
	/////////////////////////////////////////////////
	pd3dDevice->IASetInputLayout( g_pVertexLayout );
	unsigned int stride[1] = {sizeof(VPos)};
	unsigned int offset[1] = {0};
	pd3dDevice->IASetVertexBuffers(0,1,&g_FSQuadGeometry,stride,offset);
	pd3dDevice->IASetIndexBuffer(g_quadIndices, DXGI_FORMAT_R32_UINT, 0);
	pd3dDevice->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

	/////////////////////////////////////////////////
	// Set scene variables
	/////////////////////////////////////////////////
	g_LightDirVariable->SetFloatVector((float*)&g_LightDir);
	g_LightDiffuseVariable->SetFloatVector((float*)&g_LightDiffuse);
	g_MaterialAmbientColorVariable->SetFloatVector((float*)&g_MaterialAmbient);
	g_MaterialDiffuseColorVariable->SetFloatVector((float*)&g_MaterialDiffuse);
	g_pEffect10->GetVariableByName("g_txDisplay")->AsShaderResource()->SetResource(resView);

	/////////////////////////////////////////////////
	// Get the projection & view matrix from the camera class
	/////////////////////////////////////////////////
	D3DXMATRIX mWorld, mWorldViewProjection;
	D3DXMatrixIdentity(&mWorld);
	D3DXMatrixIdentity(&mWorldViewProjection);
	V(g_pWorldVariable->SetMatrix((float*)&mWorld));
	V(g_wvpVariable->SetMatrix((float*)&mWorldViewProjection));

	// Render objects here...
	g_pShowTextureDebug->GetPassByIndex(0)->Apply(0);
	pd3dDevice->DrawIndexed(6, 0, 0);
}

//--------------------------------------------------------------------------------------
// RenderFullScreenQuad: Render a full screen quad
//--------------------------------------------------------------------------------------
void RenderFullScreenQuad( ID3D10Device* pd3dDevice )
{
	HRESULT hr;

	/////////////////////////////////////////////////
	// Set render resources
	///////////////////////////////////////// ////////
	pd3dDevice->IASetInputLayout( g_pVertexLayout );
	unsigned int stride[1] = {sizeof(VPos)};
	unsigned int offset[1] = {0};
	pd3dDevice->IASetVertexBuffers(0,1,&g_FSQuadGeometry,stride,offset);
	pd3dDevice->IASetIndexBuffer(g_quadIndices, DXGI_FORMAT_R32_UINT, 0);
	pd3dDevice->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

	/////////////////////////////////////////////////
	// Get the projection & view matrix from the camera class
	/////////////////////////////////////////////////
	D3DXMATRIX mWorld, mWorldViewProjection;
	D3DXMatrixIdentity(&mWorld);
	D3DXMatrixIdentity(&mWorldViewProjection); 
	V(g_pWorldVariable->SetMatrix((float*)&mWorld));
	V(g_wvpVariable->SetMatrix((float*)&mWorldViewProjection));

	// Render objects here...
	g_pRenderScene->GetPassByIndex(0)->Apply(0);
	pd3dDevice->DrawIndexed(6, 0, 0);
}

//--------------------------------------------------------------------------------------
// RenderCubeMap: Show the cube map
//--------------------------------------------------------------------------------------
void RenderCubeMap( ID3D10Device* pd3dDevice, ID3D10RenderTargetView* pRTV )
{
	float ClearColor[4] = {0.0f, 0.0f, 0.0f, 0.0f};

	/////////////////////////////////////////////////
	// Set render resources
	///////////////////////////////////////// ////////
	pd3dDevice->IASetInputLayout( g_pVertexLayout );
	unsigned int stride[1] = {sizeof(VPos)};
	unsigned int offset[1] = {0};
	pd3dDevice->IASetVertexBuffers(0,1,&g_cubeMapGeometry,stride,offset);
	pd3dDevice->IASetIndexBuffer(g_cubeIndices, DXGI_FORMAT_R32_UINT, 0);
	pd3dDevice->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

	/////////////////////////////////////////////////
	// Get the projection & view matrix from the camera class
	/////////////////////////////////////////////////
	g_pEffect10->GetVariableByName("g_txEnvMap")->AsShaderResource()->SetResource(g_pTextureCubeMapView);

	/////////////////////////////////////////////////
    // Set render target
	/////////////////////////////////////////////////
	pd3dDevice->OMSetRenderTargets(1, &pRTV, DXUTGetD3D10DepthStencilView());

	// Render objects here...
	g_pEffect10->GetTechniqueByName("ShowCubeMap")->GetPassByIndex(0)->Apply(0);
	pd3dDevice->DrawIndexed(36, 0, 0);
}  

//--------------------------------------------------------------------------------------
// RenderGlassBox: Render a glass box
//--------------------------------------------------------------------------------------
void RenderBackground(ID3D10Device* pd3dDevice, unsigned int renderTarget)
{
		/////////////////////////////////////////////////
		// Set vertex buffer and draw glass box
		/////////////////////////////////////////////////
		pd3dDevice->IASetInputLayout( g_pVertexLayoutGlassBox );
		unsigned int stride[1] = {sizeof(VPosSimple)};
		unsigned int offset[1] = {0};
		pd3dDevice->IASetVertexBuffers(0,1,&g_sceneGeometry,stride,offset);
		pd3dDevice->IASetIndexBuffer(g_sceneIndices, DXGI_FORMAT_R32_UINT, 0);
		pd3dDevice->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

		/////////////////////////////////////////////////
		// Set render target and the depth stencil view
		/////////////////////////////////////////////////
		pd3dDevice->OMSetRenderTargets( 1, &pRTs[renderTarget], DXUTGetD3D10DepthStencilView() );

		g_pEffect10->GetVariableByName("g_txFloor")->AsShaderResource()->SetResource(g_pFloorTexView);
		g_pEffect10->GetTechniqueByName("RenderFloor")->GetPassByIndex(0)->Apply(0);

		// Render objects here...
		//pd3dDevice->DrawIndexed(30,0,0);
		pd3dDevice->DrawIndexed(6,30,20);
}

//--------------------------------------------------------------------------------------
// RenderGlassBox: Render a glass box
//--------------------------------------------------------------------------------------
void RenderGlassBox(ID3D10Device* pd3dDevice, unsigned int renderTarget)
{
	// Outer box: bottom
	simpleBoxIndices[0] = 3; 	simpleBoxIndices[1] = 1; 	simpleBoxIndices[2] = 0; 
	simpleBoxIndices[3] = 2; 	simpleBoxIndices[4] = 1; 	simpleBoxIndices[5] = 3; 
	// Outer box: left
	simpleBoxIndices[6] = 6; 	simpleBoxIndices[7] = 4; 	simpleBoxIndices[8] = 5; 
	simpleBoxIndices[9] = 7; 	simpleBoxIndices[10] = 4; simpleBoxIndices[11] = 6; 
	// Outer box: right
	simpleBoxIndices[12] = 11; simpleBoxIndices[13] = 9; simpleBoxIndices[14] = 8; 
	simpleBoxIndices[15] = 10; simpleBoxIndices[16] = 9; simpleBoxIndices[17] = 11; 
	// Outer box: front
	simpleBoxIndices[18] = 14; simpleBoxIndices[19] = 12; simpleBoxIndices[20] = 13; 
	simpleBoxIndices[21] = 15;	simpleBoxIndices[22] = 12; simpleBoxIndices[23] = 14; 
	// Outer box: back
	simpleBoxIndices[24] = 19; simpleBoxIndices[25] = 17; simpleBoxIndices[26] = 16; 
	simpleBoxIndices[27] = 18;	simpleBoxIndices[28] = 17; simpleBoxIndices[29] = 19; 

	
		/////////////////////////////////////////////////
		// Set render target and the depth stencil view
		/////////////////////////////////////////////////
		pd3dDevice->OMSetRenderTargets( 1, &pRTs[renderTarget], DXUTGetD3D10DepthStencilView() );

		g_pEffect10->GetVariableByName("g_transparency")->AsScalar()->SetFloat(float(g_HUD.GetSlider(TRANSPARENCY+4)->GetValue()/100.0));
		g_pEffect10->GetTechniqueByName("RenderGlassBox")->GetPassByIndex(0)->Apply(0);

		// Render objects here...
		pd3dDevice->DrawIndexed(6,18,0);	// front
		pd3dDevice->DrawIndexed(6,12,0);	//right
		pd3dDevice->DrawIndexed(6,6,0);		// left
		pd3dDevice->DrawIndexed(6,0,0);		// bottom
		pd3dDevice->DrawIndexed(6,24,0);	// back
}
//--------------------------------------------------------------------------------------
// RenderSceneToTexture: Render the background to a texture
//--------------------------------------------------------------------------------------
void RenderSceneToTexture( ID3D10Device* pd3dDevice, ID3D10RenderTargetView* pRTV )
{
	HRESULT hr;

	/////////////////////////////////////////////////
	// Clear color
	/////////////////////////////////////////////////
    float ClearColor[4] = {0.0f, 0.0f, 0.0f, 0.0f};

	/////////////////////////////////////////////////
	// Set render resources
	///////////////////////////////////////// ////////
	pd3dDevice->IASetInputLayout( g_pVertexLayout );
	unsigned int stride[1] = {sizeof(VPos)};
	unsigned int offset[1] = {0};
	pd3dDevice->IASetVertexBuffers(0,1,&g_FSQuadGeometry,stride,offset);
	pd3dDevice->IASetIndexBuffer(g_quadIndices, DXGI_FORMAT_R32_UINT, 0);
	pd3dDevice->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

	/////////////////////////////////////////////////
	// Get the projection & view matrix from the camera class
	/////////////////////////////////////////////////
	D3DXMATRIX mWorld;
	D3DXMATRIX scale, trans;
	D3DXMatrixIdentity(&mWorld);
	D3DXMatrixScaling(&scale, 100,100,100);
	D3DXMatrixTranslation(&trans, 0,0,1);
	D3DXMATRIX mWorldViewProjection;
	D3DXMatrixIdentity(&mWorldViewProjection);
	mWorldViewProjection = mWorld;

	/////////////////////////////////////////////////////////////
	// Set effect parameters
	/////////////////////////////////////////////////////////////
	V(g_LightDirVariable->SetFloatVector((float*)&g_LightDir));
	V(g_LightDiffuseVariable->SetFloatVector((float*)&g_LightDiffuse));
	V(g_MaterialAmbientColorVariable->SetFloatVector((float*)&g_MaterialAmbient));
	V(g_MaterialDiffuseColorVariable->SetFloatVector((float*)&g_MaterialDiffuse));
	V(g_pWorldVariable->SetMatrix((float*)&mWorld));
	V(g_wvpVariable->SetMatrix((float*)&mWorldViewProjection));

	/////////////////////////////////////////////////
    // Set render target
	/////////////////////////////////////////////////
	pd3dDevice->OMSetRenderTargets(1, &pRTV, DXUTGetD3D10DepthStencilView());
	pd3dDevice->ClearDepthStencilView( DXUTGetD3D10DepthStencilView(), D3D10_CLEAR_DEPTH | D3D10_CLEAR_STENCIL, 1.0, 0 );
    pd3dDevice->ClearRenderTargetView( pRTV, ClearColor);

	// Render objects here...
	g_pEffect10->GetTechniqueByName("RenderCheckerboard")->GetPassByIndex(0)->Apply(0);
	pd3dDevice->DrawIndexed(6, 0, 0);

}

//--------------------------------------------------------------------------------------
// RenderDepthThicknessToBuffer: Render depth and thickness to offscreen targets
//--------------------------------------------------------------------------------------
void RenderDepthThicknessToBuffer( ID3D10Device* pd3dDevice )
{
	HRESULT hr;

	/////////////////////////////////////////////////
	// Clear color
	/////////////////////////////////////////////////
    float ClearColor[4] = {-100000.0f, 0.0f, 0.0f, 0.0f};
    
	depthInZero = true;



	/////////////////////////////////////////////////////////////
	// Set effect parameters
	/////////////////////////////////////////////////////////////
	//D3DXMATRIX mWorld = *g_Camera.GetWorldMatrix();
	//D3DXMATRIX mView = *g_Camera.GetViewMatrix();
	//D3DXMATRIX mProj = *g_Camera.GetProjMatrix();
	//D3DXMATRIX mWorldViewProjection, mInvView, mInvProj, mRot;

	//D3DXMatrixInverse(&mInvView, NULL, &(mWorld*mView));
	//D3DXMatrixInverse(&mInvProj, NULL, &(mProj));
	//mWorldViewProjection = mWorld * mView * mProj;
	//V(g_pWorldVariable->SetMatrix((float*)&mWorld));
	//V(g_wvpVariable->SetMatrix((float*)&mWorldViewProjection));
	//V(g_pInvViewVariable->SetMatrix((float*)&mInvView));
	//V(g_pViewVariable->SetMatrix((float*)&mView));
	//V(g_pProjectionVariable->SetMatrix((float*)&mProj));
	V(g_pEffect10->GetVariableByName("g_txDepth")->AsShaderResource()->SetResource(g_pTextureDepthView[0]));

	// Pass 1: Depth

	/////////////////////////////////////////////////
    // Set depth texture as render target
	/////////////////////////////////////////////////
	pd3dDevice->OMSetRenderTargets(1, &pRTs[RTV_DEPTH0], g_pDepthStencilView);
	pd3dDevice->ClearDepthStencilView( g_pDepthStencilView, D3D10_CLEAR_DEPTH, 1.0, 0 );
    pd3dDevice->ClearRenderTargetView( g_pRTViewDepth[0], ClearColor);

	g_pRenderDepthThicknessSprites->GetPassByIndex(0)->Apply(0);
	if( g_HUD.GetRadioButton( RENDER_FLUID+4)->GetChecked() )
		RenderParticlesWithSprites(pd3dDevice);
	else if (g_HUD.GetRadioButton( RENDER_SINGLE_SPHERE+4)->GetChecked() )
		RenderSingleSphere(pd3dDevice, true);


	// Pass 2: Thickness

	/////////////////////////////////////////////////
    // Set thickness texture as render target
	/////////////////////////////////////////////////
	ClearColor[0] = 0.0f;
	pd3dDevice->OMSetRenderTargets(1, &pRTs[RTV_THICKNESS], DXUTGetD3D10DepthStencilView());
    pd3dDevice->ClearDepthStencilView( DXUTGetD3D10DepthStencilView(), D3D10_CLEAR_DEPTH, 1.0, 0 );
	pd3dDevice->ClearRenderTargetView( g_pRTViewThickness, ClearColor);

	g_pRenderDepthThicknessSprites->GetPassByIndex(1)->Apply(0);
	if( g_HUD.GetRadioButton( RENDER_FLUID+4)->GetChecked() )
		RenderParticlesWithSprites(pd3dDevice);
	else if (g_HUD.GetRadioButton( RENDER_SINGLE_SPHERE+4)->GetChecked() )
		RenderSingleSphere(pd3dDevice, true);

	if(takeSnapshot)
		AnalyzeD3D10Texture(g_pThicknessTex, pd3dDevice);
}

//--------------------------------------------------------------------------------------
// RenderText: Render the help and statistics text
//--------------------------------------------------------------------------------------
void RenderText()
{
    g_pTxtHelper->Begin();
    g_pTxtHelper->SetInsertionPos( 5, 5 );
    g_pTxtHelper->SetForegroundColor( D3DXCOLOR( 0.0f, 0.0f, 0.0f, 1.0f ) );
    g_pTxtHelper->DrawTextLine( DXUTGetFrameStats( DXUTIsVsyncEnabled() ) );
    g_pTxtHelper->DrawTextLine( DXUTGetDeviceStats() );
    g_pTxtHelper->End();

	g_pTxtHelper->Begin();
	g_pTxtHelper->SetInsertionPos( 5, 50 );
	g_pTxtHelper->SetForegroundColor( D3DXCOLOR( 0.0f, 0.0f, 0.0f, 1.0f ) );
	g_pTxtHelper->DrawFormattedTextLine ( L"Press H for Help" );						
	g_pTxtHelper->End();

	if ( bHelp ) {	

		g_pTxtHelper->Begin();
		g_pTxtHelper->SetInsertionPos( 5, 80 );
		g_pTxtHelper->SetForegroundColor( D3DXCOLOR( 1.0f, 0.0f, 0.0f, 1.0f ) );

		g_pTxtHelper->DrawFormattedTextLine ( L"KEYBOARD" );						
		g_pTxtHelper->DrawFormattedTextLine ( L"Press SPACE bar to start and stop the simulation" );						
		g_pTxtHelper->DrawFormattedTextLine ( L"Press ESC to exit" );						
		g_pTxtHelper->End();
	}
}
////////////////////////////////////////////////////////////////////////////////////////
// --- End Rendering functions ---------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////
// --- Begin Init functions ------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////
//--------------------------------------------------------------------------------------
// InitApp: Initialize the app 
//--------------------------------------------------------------------------------------
void InitApp()
{
    g_D3DSettingsDlg.Init( &g_DialogResourceManager );
    g_HUD.Init( &g_DialogResourceManager );
    g_SampleUI.Init( &g_DialogResourceManager );

    g_HUD.SetCallback( OnGUIEvent ); int iY = 10;
    g_HUD.AddButton( IDC_TOGGLEFULLSCREEN, L"Toggle full screen", 35, iY, 125, 22 );
    g_HUD.AddButton( IDC_CHANGEDEVICE, L"Change device (F2)", 35, iY += 24, 125, 22, VK_F2 );
    g_HUD.AddButton( IDC_TOGGLEREF, L"Toggle REF (F3)", 35, iY += 24, 125, 22, VK_F3 );
    g_HUD.AddButton( IDC_TOGGLEWARP, L"Toggle WARP (F4)", 35, iY += 24, 125, 22, VK_F4 );

	g_HUD.AddStatic( RENDER_SINGLE_SPHERE_TEXT+4, L"Single Sphere", 55, iY += 40, 80, 10, false, 0 ); 
	g_HUD.AddRadioButton( RENDER_SINGLE_SPHERE+4, 0, L"Single Sphere", 35, iY, 10, 10, false, 0, false, 0 );
	g_HUD.AddStatic( RENDER_PARTICLES_POINTS_TEXT+4, L"Points", 45, iY += 24, 50, 10, false, 0 );
	g_HUD.AddRadioButton( RENDER_PARTICLES_POINTS+4, 0, L"Points", 35, iY, 10, 10, false, 0, false, 0 );
	g_HUD.AddStatic( RENDER_FLUID_TEXT+4, L"Fluid", 45, iY += 24, 50, 10, false, 0 );
	g_HUD.AddRadioButton( RENDER_FLUID+4, 0, L"Fluid", 35, iY, 10, 10, true, 0, true, 0 );
	g_HUD.AddStatic( RENDER_NORMALS_TEXT+4, L"Normals", 45, iY += 24, 50, 10, false, 0 );
	g_HUD.AddRadioButton( RENDER_NORMALS+4, 0, L"Normals", 35, iY, 10, 10, false, 0, false, 0 );

	g_HUD.AddStatic( GAMMATEXT+4, L"Gamma", 0, iY+=36, 50, 22, false, 0);
	g_HUD.AddSlider( GAMMA+4, 52, iY, 100, 22, 0, 10000, 5000, false, 0);
	g_HUD.AddStatic( RITEXT+4, L"Ref. Index", 0, iY+=36, 50, 22, false, 0);
	g_HUD.AddSlider( RI+4, 52, iY, 100, 22, 0, 10000, 50, false, 0);
	g_HUD.AddStatic( ITERATIONSTEXT+4, L"Iterations", 0, iY+=36, 50, 22, false, 0);
	g_HUD.AddSlider( ITERATIONS+4, 52, iY, 100, 22, 0, 100, 13, false, 0);
	g_HUD.AddStatic( DEPTH_THRESHOLD_TEXT+4, L"Threshold", 0, iY+=36, 50, 22, false, 0);
	g_HUD.AddSlider( DEPTH_THRESHOLD+4, 58, iY, 90, 22, 0, 100000, 100, false, 0);
	g_HUD.AddStatic( PARTICLE_RAD_TEXT+4, L"P. Radius", 0, iY+=36, 50, 22, false, 0);
	g_HUD.AddSlider( PARTICLE_RAD+4, 58, iY, 90, 22, 1, 10, 7, false, 0);
	g_HUD.AddStatic( TRANSPARENCY_TEXT+4, L"Opacity", 0, iY+=36, 50, 22, false, 0);
	g_HUD.AddSlider( TRANSPARENCY+4, 58, iY, 90, 22, 1, 100, 70, false, 0);
}

//--------------------------------------------------------------------------------------
// goDX10: Start DirectX 10 rendering
//--------------------------------------------------------------------------------------
void preInitDX10( unsigned int maxParticles )
{
	maxParticles_ = maxParticles;

	// DXUT will create and use the best device (either D3D9 or D3D10) 
    // that is available on the system depending on which D3D callbacks are set below

    // Set general DXUT callbacks
    DXUTSetCallbackFrameMove( OnFrameMove );
    DXUTSetCallbackKeyboard( OnKeyboard );
    DXUTSetCallbackMouse( OnMouse );
    DXUTSetCallbackMsgProc( MsgProc );
    DXUTSetCallbackDeviceChanging( ModifyDeviceSettings );
    DXUTSetCallbackDeviceRemoved( OnDeviceRemoved );

    // Set the D3D10 DXUT callbacks. Remove these sets if the app doesn't need to support D3D10
    DXUTSetCallbackD3D10DeviceAcceptable( IsD3D10DeviceAcceptable );
    DXUTSetCallbackD3D10DeviceCreated( OnD3D10CreateDevice );
    DXUTSetCallbackD3D10SwapChainResized( OnD3D10ResizedSwapChain );
    DXUTSetCallbackD3D10FrameRender( OnD3D10FrameRender );
    DXUTSetCallbackD3D10SwapChainReleasing( OnD3D10ReleasingSwapChain );
    DXUTSetCallbackD3D10DeviceDestroyed( OnD3D10DestroyDevice );

    // Perform any application-level initialization here

	InitApp();
	
	DXUTInit( true, true, NULL ); // Parse the command line, show msgboxes on error, no extra command line params	

    DXUTSetCursorSettings( true, true ); // Show the cursor and clip it when in full screen
    DXUTCreateWindow( L"SPH" );
    DXUTCreateDevice( true, 1024, 980 );
}

void goDX10( 
			 btSphFluid * fluid,
			 btFluidWorld * world
			 )
{
	fluid_ = fluid;
	world_ = world;

	// Enable run-time memory check for debug builds.
#if defined(DEBUG) | defined(_DEBUG)
    _CrtSetDbgFlag( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
#endif

	//////////////////////////////////////
	// Center fluid
	//////////////////////////////////////

	g_Camera.SetModelCenter(
		D3DXVECTOR3(
		( XMIN + XMAX ) / 2, 
		( YMIN + YMAX ) / 2, 
		( ZMIN + ZMAX ) / 2
		));
	fluid_blf.x = XMIN;
	fluid_blf.y = YMIN;
	fluid_blf.z = ZMIN;
	fluid_trb.x = XMAX;
	fluid_trb.y = YMAX;
	fluid_trb.z = ZMAX;


	//////////////////////////////////////
	// Setup the camera's view parameters
	//////////////////////////////////////
    D3DXVECTOR3 vecEye( -80.0f, 60.0f, 0.0f );
	vecEye.z = fluid_blf.z - (fluid_trb.z-fluid_blf.z)*3.0f;
    D3DXVECTOR3 vecAt ( 0.0f, 0.0f, -0.0f );
    g_Camera.SetViewParams( &vecEye, &vecAt );

	CreateVBOBoxPlane();

	DXUTMainLoop(); // Enter into the DXUT render loop
}

//--------------------------------------------------------------------------------------
// InitBackBufferDependentData: Initialize window-size dependent stuff
//--------------------------------------------------------------------------------------
HRESULT InitBackBufferDependentData(ID3D10Device* pd3dDevice, const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc)
{
    //////////////////////////////////////////////////////
    // Render targets for depth and thickness
    //////////////////////////////////////////////////////
    ID3D10Texture2D           *pRTTexture2D;
    DXUTGetDXGISwapChain()->GetBuffer(0, __uuidof(*pRTTexture2D), reinterpret_cast<void**> ( &pRTTexture2D ) );
    D3D10_TEXTURE2D_DESC rtTexDesc;
    pRTTexture2D->GetDesc( &rtTexDesc ); // take some data from the basic RT
    SAFE_RELEASE(pRTTexture2D);

	// Create textures
    rtTexDesc.Format = DXGI_FORMAT_R32_FLOAT;
    rtTexDesc.MipLevels = 1;
    rtTexDesc.ArraySize = 1;
    rtTexDesc.BindFlags = D3D10_BIND_SHADER_RESOURCE | D3D10_BIND_RENDER_TARGET;
	rtTexDesc.Usage = D3D10_USAGE_DEFAULT;
    rtTexDesc.CPUAccessFlags = 0;
    rtTexDesc.MiscFlags = 0;
    SAFE_RELEASE(g_pDepthTex[0]);
    SAFE_RELEASE(g_pDepthTex[1]);
    SAFE_RELEASE(g_pThicknessTex);
	SAFE_RELEASE(g_pDebugTex);
	SAFE_RELEASE(g_pBackgroundTex);

	if( FAILED( pd3dDevice->CreateTexture2D(&rtTexDesc, NULL, &g_pDepthTex[0]) ) )    {
        return E_FAIL;
    }
	if( FAILED( pd3dDevice->CreateTexture2D(&rtTexDesc, NULL, &g_pDepthTex[1]) ) )    {
        return E_FAIL;
    }
	if( FAILED( pd3dDevice->CreateTexture2D(&rtTexDesc, NULL, &g_pThicknessTex) ) )    {
        return E_FAIL;
    }
	rtTexDesc.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;
	if( FAILED( pd3dDevice->CreateTexture2D(&rtTexDesc, NULL, &g_pDebugTex) ) )    {
        return E_FAIL;
    }
	if( FAILED( pd3dDevice->CreateTexture2D(&rtTexDesc, NULL, &g_pBackgroundTex) ) )    {
        return E_FAIL;
    }

	// Create render target views
    SAFE_RELEASE(g_pRTViewDepth[0]);
    SAFE_RELEASE(g_pRTViewDepth[1]);
    SAFE_RELEASE(g_pRTViewThickness);
    SAFE_RELEASE(g_pRTViewDebug);
    SAFE_RELEASE(g_pRTViewBackground);
    if( FAILED( pd3dDevice->CreateRenderTargetView(g_pDepthTex[0], NULL, &g_pRTViewDepth[0]) ) )    {
        return E_FAIL;
    }
    if( FAILED( pd3dDevice->CreateRenderTargetView(g_pDepthTex[1], NULL, &g_pRTViewDepth[1]) ) )    {
        return E_FAIL;
    }
    if( FAILED( pd3dDevice->CreateRenderTargetView(g_pThicknessTex, NULL, &g_pRTViewThickness) ) )    {
        return E_FAIL;
    }
	if( FAILED( pd3dDevice->CreateRenderTargetView(g_pDebugTex, NULL, &g_pRTViewDebug) ) )    {
        return E_FAIL;
    }
	if( FAILED( pd3dDevice->CreateRenderTargetView(g_pBackgroundTex, NULL, &g_pRTViewBackground) ) )    {
        return E_FAIL;
    }

    // Create the shader resource views
    D3D10_SHADER_RESOURCE_VIEW_DESC viewDesc;
    viewDesc.Format = DXGI_FORMAT_R32_FLOAT;
    viewDesc.ViewDimension = D3D10_SRV_DIMENSION_TEXTURE2D;
    viewDesc.Texture2D.MostDetailedMip = 0;
    viewDesc.Texture2D.MipLevels = 1;
    SAFE_RELEASE(g_pTextureDepthView[0]);
    SAFE_RELEASE(g_pTextureDepthView[1]);
    SAFE_RELEASE(g_pTextureThicknessView);
    SAFE_RELEASE(g_pTextureDebugView);
    SAFE_RELEASE(g_pTextureBackgroundView);
    SAFE_RELEASE(g_pDSView);
    if ( FAILED( pd3dDevice->CreateShaderResourceView( g_pDepthTex[0], &viewDesc, &g_pTextureDepthView[0]) ) )    {
        return E_FAIL;
    }
    if ( FAILED( pd3dDevice->CreateShaderResourceView( g_pDepthTex[1], &viewDesc, &g_pTextureDepthView[1]) ) )    {
        return E_FAIL;
    }
    if ( FAILED( pd3dDevice->CreateShaderResourceView( g_pThicknessTex, &viewDesc, &g_pTextureThicknessView) ) )    {
        return E_FAIL;
    }
	viewDesc.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;
    if ( FAILED( pd3dDevice->CreateShaderResourceView( g_pDebugTex, &viewDesc, &g_pTextureDebugView) ) )    {
        return E_FAIL;
    }
    if ( FAILED( pd3dDevice->CreateShaderResourceView( g_pBackgroundTex, &viewDesc, &g_pTextureBackgroundView) ) )    {
        return E_FAIL;
    }

    // Create depth stencil texture
    D3D10_TEXTURE2D_DESC descDepth;
	descDepth.Width = rtTexDesc.Width;
	descDepth.Height = rtTexDesc.Height;
    descDepth.MipLevels = 1;
    descDepth.ArraySize = 1;
    descDepth.Format = DXGI_FORMAT_R32_TYPELESS;
    descDepth.SampleDesc.Count = 1;
    descDepth.SampleDesc.Quality = 0;
    descDepth.Usage = D3D10_USAGE_DEFAULT;
    descDepth.BindFlags = D3D10_BIND_DEPTH_STENCIL | D3D10_BIND_SHADER_RESOURCE;
    descDepth.CPUAccessFlags = 0;
    descDepth.MiscFlags = 0;
    HRESULT hr = pd3dDevice->CreateTexture2D( &descDepth, NULL, &g_pDepthStencil );
    if( FAILED( hr ) )
        return hr;

    // Create the depth stencil view
    D3D10_DEPTH_STENCIL_VIEW_DESC descDSV;
	descDSV.Format =   DXGI_FORMAT_D32_FLOAT;
    descDSV.ViewDimension = D3D10_DSV_DIMENSION_TEXTURE2D;
    descDSV.Texture2D.MipSlice = 0;
	hr = pd3dDevice->CreateDepthStencilView( g_pDepthStencil, &descDSV, &g_pDepthStencilView );
    if( FAILED( hr ) )
        return hr;

    // Create the depth stencil shader resource
	D3D10_SHADER_RESOURCE_VIEW_DESC srDesc;
	srDesc.Format = DXGI_FORMAT_R32_FLOAT;
	srDesc.ViewDimension = D3D10_SRV_DIMENSION_TEXTURE2D;
	srDesc.Texture2D.MostDetailedMip = 0;
	srDesc.Texture2D.MipLevels = 1;
	hr = pd3dDevice->CreateShaderResourceView( g_pDepthStencil, &srDesc, &g_pDSView );
    if( FAILED( hr ) )
        return hr;

	// Grab viewport dimensions
	viewport[0] = float(pBackBufferSurfaceDesc->Width);
	viewport[1] = float(pBackBufferSurfaceDesc->Height);

	// Create an array of render target pointers
	pRTs[RTV_DEFAULT] =		DXUTGetD3D10RenderTargetView(); 
	pRTs[RTV_DEPTH0] =		g_pRTViewDepth[0]; 
	pRTs[RTV_DEPTH1] = 		g_pRTViewDepth[1];
	pRTs[RTV_THICKNESS] =	g_pRTViewThickness;
	pRTs[RTV_DEBUG] =		g_pRTViewDebug;
	pRTs[RTV_BACKGROUND] =	g_pRTViewBackground;
	
	// Create geometry
	if(!g_bFirstCreate)
	{
		CreateVBOBoxPlane();
	}
	g_bFirstCreate = false;
	return S_OK;
}

////////////////////////////////////////////////////////////////////////////////////////
// --- End Init functions --------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////
// --- Begin Sphere tessellation functions ---------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////
static void 
init_tetrahedron (void) 
{ 
  float sqrt3 = float(1 / sqrt(3.0));
  float tetrahedron_vertices[] = {sqrt3, sqrt3, sqrt3,
				  -sqrt3, -sqrt3, sqrt3,
				  -sqrt3, sqrt3, -sqrt3,
				  sqrt3, -sqrt3, -sqrt3}; 
  int tetrahedron_faces[] = {0, 2, 1, 0, 1, 3, 2, 3, 1, 3, 2, 0};

  n_vertices = 4; 
  n_faces = 4; 
  n_edges = 6; 
  vertices = (float*)malloc(3*n_vertices*sizeof(float)); 
  faces = (unsigned int*)malloc(3*n_faces*sizeof(unsigned int)); 
  memcpy ((void*)vertices, (void*)tetrahedron_vertices, 3*n_vertices*sizeof(float)); 
  memcpy ((void*)faces, (void*)tetrahedron_faces, 3*n_faces*sizeof(int)); 
} 


static int 
search_midpoint (int index_start, int index_end) 
{ 
  int i;
  for (i=0; i<edge_walk; i++) 
    if ((start[i] == index_start && end[i] == index_end) || 
	(start[i] == index_end && end[i] == index_start)) 
      {
	int res = midpoint[i];

	/* update the arrays */
	start[i]    = start[edge_walk-1];
	end[i]      = end[edge_walk-1];
	midpoint[i] = midpoint[edge_walk-1];
	edge_walk--;
	
	return res; 
      }

  /* vertex not in the list, so we add it */
  start[edge_walk] = index_start;
  end[edge_walk] = index_end; 
  midpoint[edge_walk] = n_vertices; 
  
  /* create new vertex */ 
  vertices[3*n_vertices]   = float((vertices[3*index_start] + vertices[3*index_end]) / 2.0);
  vertices[3*n_vertices+1] = float((vertices[3*index_start+1] + vertices[3*index_end+1]) / 2.0);
  vertices[3*n_vertices+2] = float((vertices[3*index_start+2] + vertices[3*index_end+2]) / 2.0);
  
  /* normalize the new vertex */ 
   float length = sqrt (vertices[3*n_vertices] * vertices[3*n_vertices] +
		       vertices[3*n_vertices+1] * vertices[3*n_vertices+1] +
		       vertices[3*n_vertices+2] * vertices[3*n_vertices+2]);
  length = 1/length;
  vertices[3*n_vertices] *= length;
  vertices[3*n_vertices+1] *= length;
  vertices[3*n_vertices+2] *= length;
  
  n_vertices++;
  edge_walk++;
  return midpoint[edge_walk-1];
} 

static void 
subdivide (void) 
{ 
  int n_vertices_new = n_vertices+2*n_edges; 
  int n_faces_new = 4*n_faces; 
  int i; 

  edge_walk = 0; 
  n_edges = 2*n_vertices + 3*n_faces; 
  start = (int*)malloc(n_edges*sizeof (int)); 
  end = (int*)malloc(n_edges*sizeof (int)); 
  midpoint = (int*)malloc(n_edges*sizeof (int)); 

  int *faces_old = (int*)malloc (3*n_faces*sizeof(int)); 
  faces_old = (int*)memcpy((void*)faces_old, (void*)faces, 3*n_faces*sizeof(int)); 
  vertices = (float*)realloc ((void*)vertices, 3*n_vertices_new*sizeof(float)); 
  faces = (unsigned int*)realloc ((void*)faces, 3*n_faces_new*sizeof(unsigned int)); 
  n_faces_new = 0; 

  for (i=0; i<n_faces; i++) 
    { 
      int a = faces_old[3*i]; 
      int b = faces_old[3*i+1]; 
      int c = faces_old[3*i+2]; 

      int ab_midpoint = search_midpoint (b, a); 
      int bc_midpoint = search_midpoint (c, b); 
      int ca_midpoint = search_midpoint (a, c); 

      faces[3*n_faces_new] = a; 
      faces[3*n_faces_new+1] = ab_midpoint; 
      faces[3*n_faces_new+2] = ca_midpoint; 
      n_faces_new++; 
      faces[3*n_faces_new] = ca_midpoint; 
      faces[3*n_faces_new+1] = ab_midpoint; 
      faces[3*n_faces_new+2] = bc_midpoint; 
      n_faces_new++; 
      faces[3*n_faces_new] = ca_midpoint; 
      faces[3*n_faces_new+1] = bc_midpoint; 
      faces[3*n_faces_new+2] = c; 
      n_faces_new++; 
      faces[3*n_faces_new] = ab_midpoint; 
      faces[3*n_faces_new+1] = b; 
      faces[3*n_faces_new+2] = bc_midpoint; 
      n_faces_new++; 
    } 
  n_faces = n_faces_new; 
  free (start); 
  free (end); 
  free (midpoint); 
  free (faces_old); 
} 


void tessellateSphere()
{
	int n_subdivisions = 4, j = 0;
	float x,y,z,d;
	init_tetrahedron();
	for (int i=0; i<n_subdivisions; i++) 
	subdivide (); 

	sphVertices = new VPos[n_vertices];
	for(int i = 0; i < n_vertices; i++)
	{
	  x = sphVertices[i].Pos.x = vertices[j++];
	  y = sphVertices[i].Pos.y = vertices[j++];
	  z = sphVertices[i].Pos.z = vertices[j++];
	  d = sqrt(x*x+y*y+z*z);
	  sphVertices[i].Norm.x = x/d;
	  sphVertices[i].Norm.y = y/d;
	  sphVertices[i].Norm.z = z/d;
	  sphVertices[i].Tex.x = float(asin( x/d ) / PI + 0.5); 
	  sphVertices[i].Tex.y = float(asin( y/d ) / PI + 0.5); 
	}

	sphIndices = new unsigned int[3*n_faces];
	for(int i = 0; i < 3*n_faces; i++)
	{
		sphIndices[i] = faces[i]; 
	}

	output_sphere("sphere.txt");
}


static void 
output_sphere (char *filename) 
{ 
  int i; 
  FILE *ptr = fopen (filename, "w"); 
  if (ptr == NULL) 
    printf ("Unable to open \"%s\" :(\n", filename); 
  fprintf (ptr, "OFF\n%d %d %d\n", n_vertices, n_faces, n_edges); 
  for (i=0; i<n_vertices; i++) 
    fprintf (ptr, "%f %f %f\n", vertices[3*i], vertices[3*i+1], vertices[3*i+2]); 
  for (i=0; i<n_faces; i++)
    fprintf (ptr, "%d %d %d\n", faces[3*i], faces[3*i+1], faces[3*i+2]); 
}

void 
generateXYPlaneVertices(float bl[2], float tr[2], float depth, unsigned int indexOffset)
{
	// xzPlane: Vertices
	xyPlaneVertices[0].Pos = D3DXVECTOR3(bl[0], bl[1], depth);	//0
	xyPlaneVertices[0].Norm= D3DXVECTOR3(0, 1, 0);	
	xyPlaneVertices[0].Tex = D3DXVECTOR2(1, 0);	
	xyPlaneVertices[1].Pos = D3DXVECTOR3(tr[0], bl[1], depth);	//1
	xyPlaneVertices[1].Norm= D3DXVECTOR3(0, 1, 0);	
	xyPlaneVertices[1].Tex = D3DXVECTOR2(0, 0);
	xyPlaneVertices[2].Pos = D3DXVECTOR3(tr[0], tr[1], depth);	//2
	xyPlaneVertices[2].Norm= D3DXVECTOR3(0, 1, 0);	
	xyPlaneVertices[2].Tex = D3DXVECTOR2(0, 1);
	xyPlaneVertices[3].Pos = D3DXVECTOR3(bl[0], tr[1], depth);	//3
	xyPlaneVertices[3].Norm= D3DXVECTOR3(0, 1, 0);		
	xyPlaneVertices[3].Tex = D3DXVECTOR2(1, 1);
	// xzPlane: Vertices
	xyPlaneVertices[4].Pos = D3DXVECTOR3(bl[0]+0.22f / g_Camera.GetAspectRatio(), bl[1], depth);	//0
	xyPlaneVertices[4].Norm= D3DXVECTOR3(0, 1, 0);	
	xyPlaneVertices[4].Tex = D3DXVECTOR2(1, 0);	
	xyPlaneVertices[5].Pos = D3DXVECTOR3(tr[0]+0.22f / g_Camera.GetAspectRatio(), bl[1], depth);	//1
	xyPlaneVertices[5].Norm= D3DXVECTOR3(0, 1, 0);	
	xyPlaneVertices[5].Tex = D3DXVECTOR2(0, 0);
	xyPlaneVertices[6].Pos = D3DXVECTOR3(tr[0]+0.22f / g_Camera.GetAspectRatio(), tr[1], depth);	//2
	xyPlaneVertices[6].Norm= D3DXVECTOR3(0, 1, 0);	
	xyPlaneVertices[6].Tex = D3DXVECTOR2(0, 1);
	xyPlaneVertices[7].Pos = D3DXVECTOR3(bl[0]+0.22f / g_Camera.GetAspectRatio(), tr[1], depth);	//3
	xyPlaneVertices[7].Norm= D3DXVECTOR3(0, 1, 0);		
	xyPlaneVertices[7].Tex = D3DXVECTOR2(1, 1);
	// xyPlane: Indices
	xyPlaneIndices[0] = 0; 	xyPlaneIndices[1] = 2; 	xyPlaneIndices[2] = 1; 
	xyPlaneIndices[3] = 0; 	xyPlaneIndices[4] = 3; 	xyPlaneIndices[5] = 2; 
	xyPlaneIndices[6] = 4; 	xyPlaneIndices[7] = 6; 	xyPlaneIndices[8] = 5; 
	xyPlaneIndices[9] = 4; 	xyPlaneIndices[10] = 7; xyPlaneIndices[11] = 6; 
}

void 
generateXZPlaneVertices(float lf[2], float rb[2], float height, unsigned int indexOffset)
{
	// xzPlane: Vertices
	xzPlaneVertices[0].Pos = D3DXVECTOR3(lf[0], height, lf[1]);	//0
	xzPlaneVertices[0].Norm= D3DXVECTOR3(0, 1, 0);	
	xzPlaneVertices[0].Tex = D3DXVECTOR2(0, 0);	
	xzPlaneVertices[1].Pos = D3DXVECTOR3(rb[0], height, lf[1]);	//1
	xzPlaneVertices[1].Norm= D3DXVECTOR3(0, 1, 0);	
	xzPlaneVertices[1].Tex = D3DXVECTOR2(15, 0);
	xzPlaneVertices[2].Pos = D3DXVECTOR3(rb[0], height, rb[1]);	//2
	xzPlaneVertices[2].Norm= D3DXVECTOR3(0, 1, 0);	
	xzPlaneVertices[2].Tex = D3DXVECTOR2(15, 15);
	xzPlaneVertices[3].Pos = D3DXVECTOR3(lf[0], height, rb[1]);	//3
	xzPlaneVertices[3].Norm= D3DXVECTOR3(0, 1, 0);		
	xzPlaneVertices[3].Tex = D3DXVECTOR2(0, 15);
	// xzPlane: Indices
	xzPlaneIndices[0] = 0; 	xzPlaneIndices[1] = 2; 	xzPlaneIndices[2] = 1; 
	xzPlaneIndices[3] = 0; 	xzPlaneIndices[4] = 3; 	xzPlaneIndices[5] = 2; 
}

void 
generateSimpleBoxVertices(float blf[3], float trb[3], unsigned int indexOffset)
{
    //simpleBoxVertices[0].Pos = D3DXVECTOR3( blf[0], trb[1], blf[2] ); 
    //simpleBoxVertices[1].Pos = D3DXVECTOR3( trb[0], trb[1], blf[2] ); 
    //simpleBoxVertices[2].Pos = D3DXVECTOR3( trb[0], trb[1], trb[2] ); 
    //simpleBoxVertices[3].Pos = D3DXVECTOR3( blf[0], trb[1], trb[2] ); 

	// bottom
	simpleBoxVertices[0].Pos = D3DXVECTOR3( blf[0], blf[1], blf[2] ); 
    simpleBoxVertices[1].Pos = D3DXVECTOR3( trb[0], blf[1], blf[2] ); 
    simpleBoxVertices[2].Pos = D3DXVECTOR3( trb[0], blf[1], trb[2] ); 
    simpleBoxVertices[3].Pos = D3DXVECTOR3( blf[0], blf[1], trb[2] ); 
	simpleBoxVertices[0].Norm = D3DXVECTOR3( 0, -1, 0 ); 
    simpleBoxVertices[1].Norm = D3DXVECTOR3( 0, -1, 0 ); 
    simpleBoxVertices[2].Norm = D3DXVECTOR3( 0, -1, 0 ); 
    simpleBoxVertices[3].Norm = D3DXVECTOR3( 0, -1, 0 ); 

	// left
    simpleBoxVertices[4].Pos = D3DXVECTOR3( blf[0], blf[1], trb[2] ); 
    simpleBoxVertices[5].Pos = D3DXVECTOR3( blf[0], blf[1], blf[2] ); 
    simpleBoxVertices[6].Pos = D3DXVECTOR3( blf[0], trb[1], blf[2] );
    simpleBoxVertices[7].Pos = D3DXVECTOR3( blf[0], trb[1], trb[2] );
    simpleBoxVertices[4].Norm = D3DXVECTOR3(1, 0, 0 ); 
    simpleBoxVertices[5].Norm = D3DXVECTOR3(1, 0, 0 ); 
    simpleBoxVertices[6].Norm = D3DXVECTOR3(1, 0, 0 );
    simpleBoxVertices[7].Norm = D3DXVECTOR3(1, 0, 0 );

	// right
	simpleBoxVertices[ 8].Pos = D3DXVECTOR3( trb[0], blf[1], trb[2] );
    simpleBoxVertices[ 9].Pos = D3DXVECTOR3( trb[0], blf[1], blf[2] );
    simpleBoxVertices[10].Pos = D3DXVECTOR3( trb[0], trb[1], blf[2] );
    simpleBoxVertices[11].Pos = D3DXVECTOR3( trb[0], trb[1], trb[2] );
    simpleBoxVertices[ 8].Norm = D3DXVECTOR3(-1, 0, 0 ); 
    simpleBoxVertices[ 9].Norm = D3DXVECTOR3(-1, 0, 0 ); 
    simpleBoxVertices[10].Norm = D3DXVECTOR3(-1, 0, 0 );
    simpleBoxVertices[11].Norm = D3DXVECTOR3(-1, 0, 0 );

	// front
    simpleBoxVertices[12].Pos = D3DXVECTOR3( blf[0], blf[1], blf[2] );
    simpleBoxVertices[13].Pos = D3DXVECTOR3( trb[0], blf[1], blf[2] );
    simpleBoxVertices[14].Pos = D3DXVECTOR3( trb[0], trb[1], blf[2] );
    simpleBoxVertices[15].Pos = D3DXVECTOR3( blf[0], trb[1], blf[2] );
    simpleBoxVertices[12].Norm = D3DXVECTOR3( 0, 0, 1 );
    simpleBoxVertices[13].Norm = D3DXVECTOR3( 0, 0, 1 );
    simpleBoxVertices[14].Norm = D3DXVECTOR3( 0, 0, 1 );
    simpleBoxVertices[15].Norm = D3DXVECTOR3( 0, 0, 1 );

	// back
    simpleBoxVertices[16].Pos = D3DXVECTOR3( blf[0], blf[1], trb[2] );
    simpleBoxVertices[17].Pos = D3DXVECTOR3( trb[0], blf[1], trb[2] );
    simpleBoxVertices[18].Pos = D3DXVECTOR3( trb[0], trb[1], trb[2] );
    simpleBoxVertices[19].Pos = D3DXVECTOR3( blf[0], trb[1], trb[2] );
    simpleBoxVertices[16].Norm = D3DXVECTOR3( 0, 0, -1 );
    simpleBoxVertices[17].Norm = D3DXVECTOR3( 0, 0, -1 );
    simpleBoxVertices[18].Norm = D3DXVECTOR3( 0, 0, -1 );
    simpleBoxVertices[19].Norm = D3DXVECTOR3( 0, 0, -1 );

	//// Outer box: Front
	//simpleBoxVertices[0].Pos = D3DXVECTOR3(blf[0], blf[1], blf[2]);	//0
	//simpleBoxVertices[0].Norm= D3DXVECTOR3(0, 0,  1);	
	//simpleBoxVertices[1].Pos = D3DXVECTOR3(trb[0], blf[1], blf[2]);	//1
	//simpleBoxVertices[1].Norm= D3DXVECTOR3( 0, 0,  1);	
	//simpleBoxVertices[2].Pos = D3DXVECTOR3(trb[0], trb[1], blf[2]);	//2
	//simpleBoxVertices[2].Norm= D3DXVECTOR3( 0,  0,  1);	
	//simpleBoxVertices[3].Pos = D3DXVECTOR3(blf[0], trb[1], blf[2]);	//3
	//simpleBoxVertices[3].Norm= D3DXVECTOR3(0,  0,  1);	
	//// Outer box: Back
	//simpleBoxVertices[4].Pos = D3DXVECTOR3(blf[0], blf[1], trb[2]);	//4
	//simpleBoxVertices[4].Norm= D3DXVECTOR3(0, 0, -1);	
	//simpleBoxVertices[5].Pos = D3DXVECTOR3(trb[0], blf[1], trb[2]);	//5
	//simpleBoxVertices[5].Norm= D3DXVECTOR3( 0, 0, -1);	
	//simpleBoxVertices[6].Pos = D3DXVECTOR3(trb[0], trb[1], trb[2]);	//6
	//simpleBoxVertices[6].Norm= D3DXVECTOR3( 0,  0, -1);	
	//simpleBoxVertices[7].Pos = D3DXVECTOR3(blf[0], trb[1], trb[2]);	//7
	//simpleBoxVertices[7].Norm= D3DXVECTOR3(0,  0, -1);	

	// Outer box: bottom
	simpleBoxIndices[0] = 3; 	simpleBoxIndices[1] = 1; 	simpleBoxIndices[2] = 0; 
	simpleBoxIndices[3] = 2; 	simpleBoxIndices[4] = 1; 	simpleBoxIndices[5] = 3; 
	// Outer box: left
	simpleBoxIndices[6] = 6; 	simpleBoxIndices[7] = 4; 	simpleBoxIndices[8] = 5; 
	simpleBoxIndices[9] = 7; 	simpleBoxIndices[10] = 4; simpleBoxIndices[11] = 6; 
	// Outer box: right
	simpleBoxIndices[12] = 11; simpleBoxIndices[13] = 9; simpleBoxIndices[14] = 8; 
	simpleBoxIndices[15] = 10; simpleBoxIndices[16] = 9; simpleBoxIndices[17] = 11; 
	// Outer box: front
	simpleBoxIndices[18] = 14; simpleBoxIndices[19] = 12; simpleBoxIndices[20] = 13; 
	simpleBoxIndices[21] = 15;	simpleBoxIndices[22] = 12; simpleBoxIndices[23] = 14; 
	// Outer box: back
	simpleBoxIndices[24] = 19; simpleBoxIndices[25] = 17; simpleBoxIndices[26] = 16; 
	simpleBoxIndices[27] = 18;	simpleBoxIndices[28] = 17; simpleBoxIndices[29] = 19; 
}

void 
generateDoubleLayeredBoxVertices(float blf[3], float trb[3], float thickness, unsigned int indexOffset)
{
	// Outer box: Front
	boxVertices[0].Pos = D3DXVECTOR3(blf[0], blf[1], blf[2]);	//0
	boxVertices[0].Norm= D3DXVECTOR3(-1, -1,  1);	
	boxVertices[1].Pos = D3DXVECTOR3(trb[0], blf[1], blf[2]);	//1
	boxVertices[1].Norm= D3DXVECTOR3( 1, -1,  1);	
	boxVertices[2].Pos = D3DXVECTOR3(trb[0], trb[1], blf[2]);	//2
	boxVertices[2].Norm= D3DXVECTOR3( 1,  1,  1);	
	boxVertices[3].Pos = D3DXVECTOR3(blf[0], trb[1], blf[2]);	//3
	boxVertices[3].Norm= D3DXVECTOR3(-1,  1,  1);	
	// Outer box: Back
	boxVertices[4].Pos = D3DXVECTOR3(blf[0], blf[1], trb[2]);	//4
	boxVertices[4].Norm= D3DXVECTOR3(-1, -1, -1);	
	boxVertices[5].Pos = D3DXVECTOR3(trb[0], blf[1], trb[2]);	//5
	boxVertices[5].Norm= D3DXVECTOR3( 1, -1, -1);	
	boxVertices[6].Pos = D3DXVECTOR3(trb[0], trb[1], trb[2]);	//6
	boxVertices[6].Norm= D3DXVECTOR3( 1,  1, -1);	
	boxVertices[7].Pos = D3DXVECTOR3(blf[0], trb[1], trb[2]);	//7
	boxVertices[7].Norm= D3DXVECTOR3(-1,  1, -1);	
	// Inner box: Front
	boxVertices[8].Pos = D3DXVECTOR3(blf[0]+thickness, blf[1]+thickness, blf[2]+thickness);	//8
	boxVertices[8].Norm= D3DXVECTOR3(-1, -1,  1);	
	boxVertices[9].Pos = D3DXVECTOR3(trb[0]-thickness, blf[1]+thickness, blf[2]+thickness);	//9
	boxVertices[9].Norm= D3DXVECTOR3( 1, -1,  1);	
	boxVertices[10].Pos = D3DXVECTOR3(trb[0]-thickness, trb[1], blf[2]+thickness);	//10
	boxVertices[10].Norm= D3DXVECTOR3( 1,  1,  1);	
	boxVertices[11].Pos = D3DXVECTOR3(blf[0]+thickness, trb[1], blf[2]+thickness);	//11
	boxVertices[11].Norm= D3DXVECTOR3(-1,  1,  1);	
	// Inner box: Back
	boxVertices[12].Pos = D3DXVECTOR3(blf[0]+thickness, blf[1]+thickness, trb[2]-thickness);	//12
	boxVertices[12].Norm= D3DXVECTOR3(-1, -1, -1);	
	boxVertices[13].Pos = D3DXVECTOR3(trb[0]-thickness, blf[1]+thickness, trb[2]-thickness);	//13
	boxVertices[13].Norm= D3DXVECTOR3( 1, -1, -1);	
	boxVertices[14].Pos = D3DXVECTOR3(trb[0]-thickness, trb[1], trb[2]-thickness);	//14
	boxVertices[14].Norm= D3DXVECTOR3( 1,  1,  1);	
	boxVertices[15].Pos = D3DXVECTOR3(blf[0]+thickness, trb[1], trb[2]-thickness);	//15
	boxVertices[15].Norm= D3DXVECTOR3(-1,  1, -1);	

	// Outer box: Front
	boxIndices[0] = 0; 	boxIndices[1] = 2; 	boxIndices[2] = 1; 
	boxIndices[3] = 0; 	boxIndices[4] = 3; 	boxIndices[5] = 2; 
	// Outer box: Right
	boxIndices[6] = 1; 	boxIndices[7] = 6; 	boxIndices[8] = 5; 
	boxIndices[9] = 1; 	boxIndices[10] = 2; boxIndices[11] = 6; 
	// Outer box: Back
	boxIndices[12] = 5; boxIndices[13] = 7; boxIndices[14] = 4; 
	boxIndices[15] = 5; boxIndices[16] = 6; boxIndices[17] = 7; 
	// Outer box: Left
	boxIndices[18] = 4; boxIndices[19] = 3; boxIndices[20] = 0; 
	boxIndices[21] = 4;	boxIndices[22] = 7; boxIndices[23] = 3; 
	// Inner box: Front
	boxIndices[24] = 8;	boxIndices[25] = 10;boxIndices[26] = 9; 
	boxIndices[27] = 8;	boxIndices[28] = 11;boxIndices[29] = 10; 
	// Inner box: Right
	boxIndices[30] = 9;	boxIndices[31] = 14;boxIndices[32] = 13; 
	boxIndices[33] = 9;	boxIndices[34] = 10;boxIndices[35] = 14; 
	// Inner box: Back
	boxIndices[36] = 13;boxIndices[37] = 15;boxIndices[38] = 12; 
	boxIndices[39] = 13;boxIndices[40] = 14;boxIndices[41] = 15; 
	// Inner box: Left
	boxIndices[42] = 12;boxIndices[43] = 11; boxIndices[44] = 8; 
	boxIndices[45] = 12;boxIndices[46] = 15; boxIndices[47] = 11; 
	// Outer box: Bottom
	boxIndices[48] = 4; boxIndices[49] = 1; boxIndices[50] = 5; 
	boxIndices[51] = 4;	boxIndices[52] = 0; boxIndices[53] = 1; 
	// Inner box: Bottom
	boxIndices[54] = 12;boxIndices[55] = 9; boxIndices[56] = 13; 
	boxIndices[57] = 12;boxIndices[58] = 8; boxIndices[59] = 9; 
	// Top lid: Front
	boxIndices[60] = 3;boxIndices[61] = 11; boxIndices[62] = 2; 
	boxIndices[63] = 11;boxIndices[64] = 10; boxIndices[65] = 2; 
	// Top lid: Right
	boxIndices[66] = 2;boxIndices[67] = 10; boxIndices[68] = 6; 
	boxIndices[69] = 6;boxIndices[70] = 10; boxIndices[71] = 14; 
	// Top lid: Back
	boxIndices[72] = 6;boxIndices[73] = 14; boxIndices[74] = 7; 
	boxIndices[75] = 7;boxIndices[76] = 14; boxIndices[77] = 15; 
	// Top lid: Left
	boxIndices[78] = 7;boxIndices[79] = 15; boxIndices[80] = 3; 
	boxIndices[81] = 11;boxIndices[82] = 15; boxIndices[83] = 3; 
}
////////////////////////////////////////////////////////////////////////////////////////
// --- End Sphere tessellation functions -----------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////

#ifdef DX_PIPELINE//should the entire file be inside this?

ID3D10Buffer * getVBO( std::string )
{
	return g_spriteGeometry;
}

#endif
