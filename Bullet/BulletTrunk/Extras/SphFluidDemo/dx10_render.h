
#include "project_defs.h"

#include "BulletFluids/btSphFluid.h"
#include "BulletFluids/btFluidWorld.h"

#include "D3D10_1.h"
#include "DXUT.h"
#include "DXUT.h"
#include "DXUTgui.h"
#include "DXUTmisc.h"
#include "DXUTCamera.h"
#include "DXUTSettingsDlg.h"
#include "DXUTShapes.h"
#include "SDKmisc.h"
#include "SDKmesh.h"
//#include "imdebug.h"
//#include "imdebuggl.h"
#include <vector>

#define PI 3.14159265 

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
// UI control IDs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
#define IDC_TOGGLEFULLSCREEN    1
#define IDC_TOGGLEREF           2
#define IDC_CHANGEDEVICE        3
#define IDC_TOGGLEWARP          5

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
// Structs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
struct VPos
{
	D3DXVECTOR3 Pos;  
	D3DXVECTOR3 Norm;  
	D3DXVECTOR2 Tex;  
	D3DXVECTOR3 FrustumCorner;  
};

struct VPosSimple
{
	D3DXVECTOR3 Pos;  
	D3DXVECTOR3 Norm;  
	D3DXVECTOR2 Tex;  
};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
// Enumerations
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
enum SPH_UI_ELEMENTS 
{	
	GAMMA, GAMMATEXT, RI, RITEXT, ITERATIONS, ITERATIONSTEXT, DEPTH_THRESHOLD, DEPTH_THRESHOLD_TEXT,
	PARTICLE_RAD, PARTICLE_RAD_TEXT, 
	RENDER_SINGLE_SPHERE, RENDER_PARTICLES_POINTS, RENDER_FLUID, TRANSPARENCY, RENDER_NORMALS,
	RENDER_SINGLE_SPHERE_TEXT, RENDER_PARTICLES_POINTS_TEXT, RENDER_FLUID_TEXT, TRANSPARENCY_TEXT, RENDER_NORMALS_TEXT
};

enum SPH_RENDER_TARGET_VIEWS
{
	RTV_DEFAULT, RTV_DEPTH0, RTV_DEPTH1, RTV_DEBUG, RTV_THICKNESS, RTV_BACKGROUND
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
// Forward Declarations
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
static void logFluidSystemInfo(char* filename);
static void init_tetrahedron (void);
static void output_sphere (char *filename);
static void subdivide (void);
static int search_midpoint (int index_start, int index_end);
void tessellateSphere();
void ComputeFrustumCornersWS();
void Composite(ID3D10Device *pd3dDevice);
void ShowTextureDebug( ID3D10Device* pd3dDevice, ID3D10ShaderResourceView* resView );
void TestDepthCalculations(ID3D10Device* pd3dDevice);
void RenderDepthThicknessToBuffer(ID3D10Device *pd3dDevice);
void RenderSphere(ID3D10Device* pd3dDevice, ID3D10RenderTargetView* pRTV);
void AdvanceSimulation();
void SmoothDepth(ID3D10Device *pd3dDevice);
void SetEffectParameters();
void RenderParticlesWithGeometry(ID3D10Device* pd3dDevice);
void RenderParticlesWithSprites(ID3D10Device* pd3dDevice);
void RenderParticlesWithPoints(ID3D10Device* pd3dDevice);
void RenderSceneToTexture(ID3D10Device* pd3dDevice, ID3D10RenderTargetView* pRTV);
void RenderSingleSphere(ID3D10Device* pd3dDevice, bool useSprites=false);
void RenderBackground(ID3D10Device* pd3dDevice, unsigned int renderTarget);
void RenderGlassBox(ID3D10Device* pd3dDevice, unsigned int renderTarget);
void RenderLogo(ID3D10Device* pd3dDevice, unsigned int renderTarget);
void RenderCubeMap( ID3D10Device* pd3dDevice, ID3D10RenderTargetView* pRTV );
void RenderText();
int AnalyzeD3D10Texture(ID3D10Texture2D* tex, ID3D10Device* pd3dDevice);
HRESULT InitBackBufferDependentData(ID3D10Device* pd3dDevice, const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc);
HRESULT createCubeMapBuffer( ID3D10Device* pd3dDevice);
ID3D10Device* InitializeDX10();
HRESULT CALLBACK OnD3D10ResizedSwapChain( 
	ID3D10Device* pd3dDevice, IDXGISwapChain* pSwapChain,
	const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc, 
	void* pUserContext );
HRESULT CALLBACK OnD3D10CreateDevice( 
	ID3D10Device* pd3dDevice, 
	const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc,                                      
	void* pUserContext );
bool CALLBACK ModifyDeviceSettings( 
	DXUTDeviceSettings* pDeviceSettings, 
	void* pUserContext );
bool CALLBACK IsD3D10DeviceAcceptable( 
	UINT Adapter, UINT Output, 
	D3D10_DRIVER_TYPE DeviceType,
	DXGI_FORMAT BackBufferFormat, 
	bool bWindowed, 
	void* pUserContext );
void CALLBACK OnGUIEvent( 
	UINT nEvent, 
	int nControlID, 
	CDXUTControl* pControl, 
	void* pUserContext );
void CALLBACK OnKeyboard( 
	UINT nChar, 
	bool bKeyDown, 
	bool bAltDown, 
	void* pUserContext );
void CALLBACK OnMouse( 
	bool bLeftButtonDown, bool bRightButtonDown, bool bMiddleButtonDown,
	bool bSideButton1Down, bool bSideButton2Down, int nMouseWheelDelta,
	int xPos, int yPos, void* pUserContext );
bool CALLBACK OnDeviceRemoved( void* pUserContext );
void CALLBACK OnD3D10ReleasingSwapChain( void* pUserContext );
LRESULT CALLBACK MsgProc( 
	HWND hWnd, UINT uMsg, 
	WPARAM wParam, 
	LPARAM lParam,
	bool* pbNoFurtherProcessing, 
	void* pUserContext );
void InitApp();
//void goDX10(Sph *);
void preInitDX10( unsigned int maxParticles );
void goDX10( btSphFluid * fluid, btFluidWorld * world );
void generateBoxVertices(float blf[3], float trb[3], float thickness, unsigned int indexOffset=0);
void generateSimpleBoxVertices(float blf[3], float trb[3], unsigned int indexOffset=0);
void generateXZPlaneVertices(float lf[2], float rb[2], float height, unsigned int indexOffset=0);
void generateXYPlaneVertices(float bl[2], float tr[2], float depth, unsigned int indexOffset=0);
HRESULT CreateVBOBoxPlane();
ID3D10Buffer * getVBO( std::string );


