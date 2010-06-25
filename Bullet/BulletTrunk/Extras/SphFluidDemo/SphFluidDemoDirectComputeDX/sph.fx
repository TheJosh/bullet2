//--------------------------------------------------------------------------------------
// File: sph.fx
//
// Copyright (c) Microsoft Corporation. All rights reserved.
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
// Constant Buffers
//--------------------------------------------------------------------------------------
float4x4  	g_mWorldViewProjection;
float4x4  	g_mWorld;
float4x4  	g_mView;
float4x4  	g_mProj;
float4x4	g_mInvView;
float4  	g_MaterialAmbientColor;
float4  	g_MaterialDiffuseColor;
float3  	g_vLightDir;
float3		g_vEyePt;
float   	g_SpecExpon=48, g_Kd=0.8, g_Ks=0.6;
float		g_gamma;					// Constant that depends on the type of fluid 
float		g_threshold;					// Constant that depends on the type of fluid 
float		g_refractiveIndex;			// Constant that depends on the type of fluid 
float		g_transparency;
float4		g_LightDiffuse;
float4		g_CameraParams;				// (fov, aspect, zNear, zFar)
float3		g_FrustumCorners[4];		// Corners of the view frustum in world space
float2		g_viewport;			
Texture2D   g_txDepth;
Texture2D   g_txThickness;
Texture2D   g_txBackground;
Texture2D   g_txFloor;
Texture2D   g_txBranding;
Texture2D	g_txDisplay;

#define PI 3.14159265
SamplerState g_samLinear
{
    Filter = MIN_MAG_MIP_LINEAR;
    AddressU = Wrap;
    AddressV = Wrap;
};

SamplerState g_samCube
{
    Filter = ANISOTROPIC;
    AddressU = Clamp;
    AddressV = Clamp;
};


cbuffer cb1
{
    float g_fParticleRad;
};

cbuffer cbImmutable
{
    float3 g_positions[4] =
    {
        float3( -1, 1, 0 ),
        float3( 1, 1, 0 ),
        float3( -1, -1, 0 ),
        float3( 1, -1, 0 ),
    };
    float2 g_texcoords[4] = 
    { 
        float2(0,0), 
        float2(1,0),
        float2(0,1),
        float2(1,1),
    };
};
//--------------------------------------------------------------------------------------
// Structs for rendering geometry
//--------------------------------------------------------------------------------------
struct VS_INPUT
{
    float4 Position     : POSITION; // vertex position 
    float3 Normal       : NORMAL;   // this normal comes in per-vertex
    float2 TextureUV    : TEXCOORD0;// vertex texture coords 
    float3 FrustumCorner : TEXCOORD1;// frustum corner
};

struct VS_OUTPUT
{
    float4 Position     : SV_POSITION; // vertex position 
    float4 Diffuse      : COLOR0;      // vertex diffuse color (note that COLOR0 is clamped from 0..1)
    float2 TextureUV    : TEXCOORD0;   // vertex texture coords 
	float3 Normal		: TEXCOORD2;
};

struct VS_OUTPUT_DEPTH
{
    float4 Position     : SV_POSITION; // vertex position 
    float2 TextureUV    : TEXCOORD0;   // vertex texture coords 
	float3 ViewDirection : TEXCOORD2; // view space position for normal computation
    float  depth		: TEXCOORD1;   // depth
};
struct VS_OUTPUT_COMPOSITE
{
    float4 Position     : SV_POSITION; // vertex position 
    float2 TextureUV    : TEXCOORD0;   // vertex texture coords 
    float  depth		: TEXCOORD1;   // depth
	float4 ViewSpacePosition : TEXCOORD2; // view space position for normal computation
	float3 FrustumRay : TEXCOORD3; // view space position for normal computation
};
struct VS_SIMPLE_INPUT
{
    float4 Position     : POSITION; // vertex position 
    float3 Normal       : NORMAL;   // this normal comes in per-vertex
    float2 TextureUV    : TEXCOORD0;   // vertex texture coords 
};
struct VS_SIMPLE_OUTPUT
{
    float4 Position     : SV_POSITION; // vertex position 
	float3 WorldNormal	: TEXCOORD1;
	float3 LightVec		: TEXCOORD2;
	float3 WorldView	: TEXCOORD3;
    float2 TextureUV    : TEXCOORD4;   // vertex texture coords 
};

//--------------------------------------------------------------------------------------
// Structs for rendering point sprites
//--------------------------------------------------------------------------------------
struct VS_SPRITE_IN
{
    float4 Position     : POSITION; 
};
struct VS_SPRITE_OUT
{
    float4 Position				: SV_POSITION; 
    float4 ParticlePosition     : TEXCOORD0; 
};
struct GS_SPRITE_OUT
{
    float4 Position							: SV_POSITION;
    float2 TextureUV						: TEXCOORD0;
	float4 ViewSpacePosition				: TEXCOORD1;
	nointerpolation float4 ParticlePosition : TEXCOORD2;
};
struct PS_SPRITE_OUT
{
	float Z		: SV_Target0;
	float depth	: SV_DEPTH;
};

//--------------------------------------------------------------------------------------
// Structs for rendering points
//--------------------------------------------------------------------------------------
struct VS_POINT_OUT
{
    float4 Position	: SV_POSITION; 
};
struct GS_POINT_OUT
{
    float4 Position : SV_POSITION;
};
struct PS_DEPTH_OUT
{
  	float4 depth	: SV_Target0;
};



//--------------------------------------------------------------------------------------
// Pipeline State
//--------------------------------------------------------------------------------------
RasterizerState CullOff
{
	CullMode = NONE;
};

DepthStencilState EnableDepth
{
    DepthEnable = TRUE;
    DepthWriteMask = ALL;
    DepthFunc = LESS_EQUAL;
};
DepthStencilState DisableDepth
{
    DepthEnable = FALSE;
    DepthWriteMask = ZERO;
};
DepthStencilState RenderWithStencilState2
{
    DepthEnable = false;
    DepthWriteMask = ZERO;
    DepthFunc = LESS_EQUAL;
    
    // Setup stencil states
    StencilEnable = true;
    StencilReadMask = 0xFF;
    StencilWriteMask = 0x00;
    
    FrontFaceStencilFunc = Equal;
    FrontFaceStencilPass = Keep;
    FrontFaceStencilFail = Keep;
    
    BackFaceStencilFunc = Equal;
    BackFaceStencilPass = Keep;
    BackFaceStencilFail = Keep;
};

DepthStencilState RenderWithStencilState1
{
    DepthEnable = true;
    DepthWriteMask = ALL;
    DepthFunc = LESS_EQUAL;
    
    // Setup stencil states
    StencilEnable = true;
    StencilReadMask = 0xFF;
    StencilWriteMask = 0xFF;
    
    FrontFaceStencilFunc = ALWAYS;
    FrontFaceStencilPass = Replace;
    FrontFaceStencilFail = Replace;

    BackFaceStencilFunc = ALWAYS;
    BackFaceStencilPass = Replace;
    BackFaceStencilFail = Replace;
};
BlendState NoBlending
{
    AlphaToCoverageEnable = FALSE;
    BlendEnable[0] = FALSE;
};
BlendState AlphaBlending
{
    AlphaToCoverageEnable = FALSE;
    BlendEnable[0] = TRUE;
    SrcBlend = SRC_ALPHA;
    DestBlend = ONE;
    BlendOp = ADD;
    RenderTargetWriteMask[0] = 0x0F;
};
BlendState AdditiveBlending
{
    AlphaToCoverageEnable = FALSE;
    BlendEnable[0] = TRUE;
    SrcBlend = ONE;
    DestBlend = ONE;
    BlendOp = ADD;
    RenderTargetWriteMask[0] = 0x0F;
};


//--------------------------------------------------------------------------------------
// Vertex Shaders
//--------------------------------------------------------------------------------------
VS_POINT_OUT VS_POINTS( VS_SPRITE_IN input )
{
	VS_POINT_OUT Output;
	Output.Position = input.Position ;
	return Output;
}

VS_SIMPLE_OUTPUT VS_BOX( VS_SIMPLE_INPUT input )
{
	VS_SIMPLE_OUTPUT Output;
    float4 WorldPos;
	float3 WorldLight;

    // Transform the position from object space to homogeneous projection space
    Output.Position = mul( input.Position, g_mWorldViewProjection );

    // Transform the normal from object space to world space    
	Output.WorldNormal = mul(input.Normal, (float3x3)g_mWorld).xyz; // will not work with non-uniform scaling

	// Compute world space light vector
	WorldPos = mul(input.Position, g_mWorld);
	WorldLight = mul(g_vLightDir, (float3x3)g_mWorld);
	Output.LightVec = (g_vLightDir - WorldPos.xyz).xyz;

	// Compute view vector
	Output.WorldView = WorldPos.xyz - g_vEyePt;

	// Pass through tex coords
	Output.TextureUV = input.TextureUV;

    return Output;    
}


VS_SIMPLE_OUTPUT VS_LOGO( VS_SIMPLE_INPUT input )
{
	VS_SIMPLE_OUTPUT Output;

    // Pass through position 
    Output.Position = input.Position;

	// Pass through tex coords
	Output.TextureUV = input.TextureUV;

    return Output;    
}

VS_OUTPUT VS( VS_INPUT input )
{
    VS_OUTPUT Output;
    float3 vNormalWorldSpace;
    
    // Transform the position from object space to homogeneous projection space
    Output.Position = mul( input.Position, g_mWorldViewProjection );


    // Transform the normal from object space to world space    
    vNormalWorldSpace = normalize(mul(input.Normal, (float3x3)g_mWorld)); // normal (world space)
	Output.Normal = vNormalWorldSpace; 

    // Calc diffuse color    
    Output.Diffuse.rgb = g_MaterialDiffuseColor * g_LightDiffuse * max(0,dot(vNormalWorldSpace, g_vLightDir)) + 
                         g_MaterialAmbientColor;   
    Output.Diffuse.a = 1.0f; 
    
    // Just copy the texture coordinate through
    Output.TextureUV = input.TextureUV; 
     
    return Output;    
}

VS_OUTPUT_DEPTH VS_MINIMAL( VS_INPUT input ) 
{
    VS_OUTPUT_DEPTH Output;

    // Transform the position from object space to homogeneous projection space
    Output.Position = input.Position ;

    // Pass through depth
    Output.depth = Output.Position.z;
    
    // Just copy the texture coordinate through
    Output.TextureUV = input.TextureUV; 

	Output.ViewDirection = input.FrustumCorner.xyz;

    return Output;
}
 
VS_OUTPUT_COMPOSITE VS_COMPOSITE( VS_INPUT input )
{
    VS_OUTPUT_COMPOSITE Output;

    // Transform the position from object space to homogeneous projection space
    Output.Position = input.Position;

	// Transform the position from object space to view space 
	Output.ViewSpacePosition = input.Position;

    // Pass through depth
    Output.depth = Output.Position.z;
    
    // Just copy the texture coordinate through
    Output.TextureUV = input.TextureUV; 

	Output.FrustumRay = input.FrustumCorner.xyz;

    return Output;
}

VS_SPRITE_OUT VS_DRAW_SPRITE( VS_SPRITE_IN input )
{
	VS_SPRITE_OUT Output;
	
    // Transform the position from object space to homogeneous projection space
    Output.Position = input.Position;

	Output.ParticlePosition = mul( input.Position, g_mWorldViewProjection );

	return Output;
}

//--------------------------------------------------------------------------------------
// Pixel Shaders
//--------------------------------------------------------------------------------------
float4 PS(  VS_OUTPUT In ) : SV_TARGET
{
	//return float4(In.Normal,0);
	return In.Diffuse;
}

float4 PS_BOX(  VS_SIMPLE_OUTPUT In ) : SV_TARGET
{
    float3 DiffuseContrib;
    float3 SpecularContrib;
    float3 Ln = normalize(In.LightVec);
    float3 Nn = normalize(In.WorldNormal);
    float3 Vn = normalize(-In.WorldView);
    float3 Hn = normalize(Vn + Ln);
    float4 litV = lit(dot(Ln,Nn),dot(Hn,Nn),g_SpecExpon);
    DiffuseContrib = litV.y * g_Kd * float3(0.3,0.3,0.3);
    SpecularContrib = litV.z * g_Ks * g_LightDiffuse.xyz;
    float3 result = DiffuseContrib + SpecularContrib;
    return float4(result,g_transparency);
}

float4 PS_LOGO(  VS_SIMPLE_OUTPUT In ) : SV_TARGET
{
	float4 color = g_txBranding.Sample(g_samLinear, In.TextureUV);
	return color;
}

float4 PS_POINTS() : SV_TARGET
{
	return float4(0,0,1.0,1.0);
}

float4 PS_TEXTURE(  VS_OUTPUT In ) : SV_TARGET
{
	float4 color = g_txDisplay.Sample(g_samLinear, In.TextureUV);
	return color;
}

float4 PS_TEXTURED_FLOOR(  VS_SIMPLE_OUTPUT In ) : SV_TARGET
{
    float3 DiffuseContrib;
    //float3 SpecularContrib;
    float3 Ln = normalize(In.LightVec);
    float3 Nn = normalize(In.WorldNormal);
    float3 Vn = normalize(-In.WorldView);
    float3 Hn = normalize(Vn + Ln);
    float4 litV = lit(dot(Ln,Nn),dot(Hn,Nn),g_SpecExpon);
	float4 color = g_txFloor.Sample(g_samLinear, In.TextureUV);
    DiffuseContrib = litV.y * g_Kd * color.xyz;
    //SpecularContrib = litV.z * g_Ks * g_LightDiffuse.xyz;
    float3 result = DiffuseContrib;
    return float4(result,1);
    //return color;
}

float4 PS_CUBEMAP(  VS_OUTPUT In ) : SV_TARGET
{
	//float3 Rln = reflect(In.ViewSpacePos.xyz, normal);
	//float4 color = g_txEnvMap.Sample(g_samCube, In.Normal);
	//float4 color = float4(In.Normal,1);
	return float4(1,0,0,1);
}

float4 PS_CHECKERBOARD(  VS_OUTPUT In ) : SV_TARGET
{
	float4 Color; 
	int2 WhiteOrBlack;

	WhiteOrBlack = int2((In.TextureUV * 8))  % 8;
	if((WhiteOrBlack.x+WhiteOrBlack.y) % 2 == 0)
		Color = float4(1,1,1,1);
	else
		Color = float4(0,0,1,1);
	return Color;
}

// Pixel Shader - Gaussian smoothing
PS_DEPTH_OUT PS_SMOOTH_DEPTH_GAUSSIAN( VS_OUTPUT_DEPTH In )
{
/*
	gaussian_kernel_5x5[25] = 
	{
		1.0/273.0, 4.0/273.0, 7.0/273.0, 4.0/273.0, 1.0/273.0, 
		4.0/273.0, 16.0/273.0, 26.0/273.0, 16.0/273.0, 4.0/273.0, 
		7.0/273.0, 26.0/273.0, 41.0/273.0, 26.0/273.0, 7.0/273.0, 
		4.0/273.0, 16.0/273.0, 26.0/273.0, 16.0/273.0, 4.0/273.0, 
		1.0/273.0, 4.0/273.0, 7.0/273.0, 4.0/273.0, 1.0/273.0
	};
*/
	float2 delta = float2(1,1);
	int2 lookup;
	float texSample, result=0.0f;
	float centerSample;
	PS_DEPTH_OUT Output;

	// Lookup depth texture	
	In.TextureUV *= g_viewport;
	lookup = int2(In.TextureUV);
	centerSample = g_txDepth.Load( int3(lookup,0) ).r;
	if(centerSample < -9999)
	{
		Output.depth = centerSample;
	}
	else
	{ 
		texSample = max( g_txDepth.Load(int3(lookup.x-2,lookup.y-2,0)).r, 0);
		result += (texSample);
		texSample = max( g_txDepth.Load(int3(lookup.x-2,lookup.y-1,0)).r, 0);
		result += (4*texSample);
		texSample = max( g_txDepth.Load(int3(lookup.x-2,lookup.y,0)).r, 0);
		result += (7*texSample);
		texSample = max( g_txDepth.Load(int3(lookup.x-2,lookup.y+1,0)).r, 0);
		result += (4*texSample);
		texSample = max( g_txDepth.Load(int3(lookup.x-2,lookup.y+2,0)).r, 0);
		result += (texSample);

		texSample = max( g_txDepth.Load(int3(lookup.x-1,lookup.y-2,0)).r, 0);
		result += (4*texSample);
		texSample = max( g_txDepth.Load(int3(lookup.x-1,lookup.y-1,0)).r, 0);
		result += (16*texSample);
		texSample = max( g_txDepth.Load(int3(lookup.x-1,lookup.y,0)).r, 0);
		result += (26*texSample);
		texSample = max( g_txDepth.Load(int3(lookup.x-1,lookup.y+1,0)).r, 0);
		result += (16*texSample);
		texSample = max( g_txDepth.Load(int3(lookup.x-1,lookup.y+2,0)).r, 0);
		result += (4*texSample);

		texSample = max( g_txDepth.Load( int3(lookup.x,lookup.y-2,0)).r,  0);
		result += (7*texSample);
		texSample = max( g_txDepth.Load( int3(lookup.x,lookup.y-1,0)).r,  0);
		result += (26*texSample);
		texSample = max( g_txDepth.Load( int3(lookup.x,lookup.y,0)).r,  0);
		result += (41*texSample);
		texSample = max( g_txDepth.Load( int3(lookup.x,lookup.y+1,0)).r,  0);
		result += (26*texSample);
		texSample = max( g_txDepth.Load( int3(lookup.x,lookup.y+2,0)).r,  0);
		result += (7*texSample);

		texSample = max( g_txDepth.Load(int3(lookup.x+1,lookup.y-2,0)).r, 0);
		result += (4*texSample);
		texSample = max( g_txDepth.Load(int3(lookup.x+1,lookup.y-1,0)).r, 0);
		result += (16*texSample);
		texSample = max( g_txDepth.Load(int3(lookup.x+1,lookup.y,0)).r, 0);
		result += (26*texSample);
		texSample = max( g_txDepth.Load(int3(lookup.x+1,lookup.y+1,0)).r, 0);
		result += (16*texSample);
		texSample = max( g_txDepth.Load(int3(lookup.x+1,lookup.y+2,0)).r, 0);
		result += (4*texSample);

		texSample = max( g_txDepth.Load(int3(lookup.x+2,lookup.y-2,0)).r, 0);
		result += (texSample);
		texSample = max( g_txDepth.Load(int3(lookup.x+2,lookup.y-1,0)).r, 0);
		result += (4*texSample);
		texSample = max( g_txDepth.Load(int3(lookup.x+2,lookup.y,0)).r, 0);
		result += (7*texSample);
		texSample = max( g_txDepth.Load(int3(lookup.x+2,lookup.y+1,0)).r, 0);
		result += (5*texSample);
		texSample = max( g_txDepth.Load(int3(lookup.x+2,lookup.y+2,0)).r, 0);
		result += (texSample);

		Output.depth = result/273.0f;
	}
	return Output;
}

// Pixel Shader - smooth depth with curvature flow
PS_DEPTH_OUT PS_SMOOTH_DEPTH_CURVATURE_FLOW( VS_OUTPUT_DEPTH In )
{
	float2 delta = float2(1,1);
	float4 zRight, zLeft, zUp, zDown;
	float dzdx, dzdy, d2zdx, d2zdy, dDdx, dDdy, H, D;
	float2 C, F, E;
	float3 surfNormal;
	PS_DEPTH_OUT Output;
	int2 lookup;

	// Lookup depth texture	
	In.TextureUV *= g_viewport;
	lookup = int2(In.TextureUV);
	float4  centerSample = g_txDepth.Load( int3(lookup,0) );
	if(centerSample.x > 9999.9)
		discard;

	zRight	= g_txDepth.Load(float3(In.TextureUV.x+delta.x,In.TextureUV.y,0) );
	zLeft	= g_txDepth.Load(float3(In.TextureUV.x-delta.x,In.TextureUV.y,0) );
	zUp		= g_txDepth.Load(float3(In.TextureUV.x,In.TextureUV.y-delta.y,0) );
	zDown	= g_txDepth.Load(float3(In.TextureUV.x,In.TextureUV.y+delta.y,0) );

	if( zRight.z > 9999.9 || zLeft.z > 9999.9 || zUp.z > 9999.9 || zDown.z > 9999.9 )
		discard;


	// Compute spatial derivatives of z using finite differences
	delta /= g_viewport;
	dzdx	= (zRight.z - zLeft.z) / (2 * delta);
	dzdy	= (zDown.z - zUp.z) / (2 * delta);
	d2zdx	= (zRight.z - 2 * centerSample.z + zLeft.z) / (delta * delta);
	d2zdy	= (zUp.z - 2 * centerSample.z + zDown.z) / (delta * delta);

	// Compute focal length in x and y direction
	F.x = g_mProj[0][0];
	F.y = g_mProj[1][1];
	C = F;

	surfNormal = float3(-C.y*dzdx, -C.x*dzdy, C.x*C.y*centerSample.z);
	D = dot(surfNormal, surfNormal);
	dDdx = C.y*C.y*2*dzdx*d2zdx + C.x*C.x*C.y*C.y*2*centerSample.z*dzdx;
	dDdy = C.x*C.x*2*dzdy*d2zdy + C.x*C.x*C.y*C.y*2*centerSample.z*dzdy;


	// Compute curvature
	E.x = (dzdx * dDdx * 0.5) - d2zdx * D;
	E.y = (dzdy * dDdy * 0.5) - d2zdy * D;
	H = (C.y * E.x + C.x * E.y)/(2*pow(D,1.5));

	Output.depth = float4(normalize(surfNormal),F.x);
	return Output;
}

PS_SPRITE_OUT PS_DRAW_SPRITE_DEPTH(GS_SPRITE_OUT input )
{
	float fragPos,z;
	PS_SPRITE_OUT Output;
	fragPos = dot(input.TextureUV*2-1.0, input.TextureUV*2-1.0);
	if(fragPos > 1)
	{
		discard;
	} 
	else 
	{
		fragPos *= g_fParticleRad;
		fragPos *= g_fParticleRad;
		z = sqrt(g_fParticleRad*g_fParticleRad - fragPos);
		input.ViewSpacePosition.z -= z;
		float4 clipSpacePosition = mul(float4(input.ViewSpacePosition.xyz,1.0), g_mProj);
		Output.depth = clipSpacePosition.z/clipSpacePosition.w;
		Output.Z = input.ViewSpacePosition.z/g_CameraParams.w;
	}
	return Output;
}

// modified refraction function that returns boolean for total internal reflection
float3 refract2( float3 I, float3 N, float eta, out bool fail )
{
	float IdotN = dot(I, N);
	float k = 1 - eta*eta*(1 - IdotN*IdotN);
	fail = k < 0;
	return eta*I - (eta*IdotN + sqrt(k))*N;
}

// approximate Fresnel function
float fresnel(float NdotV, float bias, float power)
{
   return bias + (1.0-bias)*pow(1.0 - max(NdotV, 0), power);
}

float3 GetFrustumRay( float2 texCoords )
{
	float3 fRay;
	texCoords.y = 1-texCoords.y;
	fRay.x = g_FrustumCorners[0].x + (texCoords.x) * (g_FrustumCorners[1].x - g_FrustumCorners[0].x);
	fRay.y = g_FrustumCorners[1].y + (texCoords.y) * (g_FrustumCorners[2].y - g_FrustumCorners[1].y);
	fRay.z = g_FrustumCorners[1].z; 

	return fRay;
}


float4 PS_COMPOSITE( VS_OUTPUT_COMPOSITE In ) : SV_TARGET
{
	int signdX=1, signdY=1;
	int2 delta = int2(1,1);
	float3 VSNormal, VSPos, VSPosdX, VSPosdY;
	float4 zCenter, zdX, zdY;

	int2 integerTexCoords; 
	integerTexCoords = In.TextureUV * g_viewport;
	zCenter	= g_txDepth.Load( int3( integerTexCoords, 0 ) );
	if( zCenter.x < -9999 )	{
		discard;
	}
	zdX		= g_txDepth.Load( int3( integerTexCoords.x+delta.x, integerTexCoords.y, 0 ) );
	if( zdX.x < -9999 )	{
		zdX		= g_txDepth.Load( int3( integerTexCoords.x-delta.x, integerTexCoords.y, 0 ) );
		signdX = -1;
	}
	zdY		= g_txDepth.Load( int3( integerTexCoords.x, integerTexCoords.y-delta.y, 0 ) );
	if( zdY.x < -9999 )	{
		zdY		= g_txDepth.Load( int3( integerTexCoords.x, integerTexCoords.y+delta.y, 0 ) );
		signdY = -1;
	}
	
	// Compute surface normal and lighting vectors
	float2 fdelta = float2(delta)/g_viewport;
	VSPos = zCenter.x * In.FrustumRay;
	VSPosdX = zdX.x * GetFrustumRay( float2(In.TextureUV.x+signdX*fdelta.x, In.TextureUV.y) );
	VSPosdY = zdY.x * GetFrustumRay( float2(In.TextureUV.x, In.TextureUV.y-signdY*fdelta.y) );
	VSNormal = -cross(signdX*(VSPosdX-VSPos), signdY*(VSPosdY-VSPos));

	////////////////////////////////////////////////////
	// 1. Phong Lighting
	////////////////////////////////////////////////////
    float3 Ln = normalize(g_vLightDir);
    float3 Vn = normalize(-VSPos);
    float3 Nn = normalize(VSNormal);
    float3 Hn = normalize(Vn + Ln);
	float4 litV = lit(dot(Ln,Nn),dot(Hn,Nn),g_SpecExpon);
	//float3 DiffuseContrib = litV.y * g_Kd * g_LightDiffuse.xyz;
	float3 SpecularContrib = litV.z * g_Ks * g_LightDiffuse.xyz;
	//float3 result ;
	//result = DiffuseContrib + SpecularContrib + g_MaterialAmbientColor.xyz;
	//return float4(result,1.0);

	////////////////////////////////////////////////////
	// 2. Rendering Formula in Curvature Flow paper
	////////////////////////////////////////////////////
	float4 Cout;							// Final color 
	float4 Cfluid = float4(0.1,0.5,0.8,0);	// Fluid color
	float4 a = float4(1,1,1,1);				// Refracted fluid color 
	float4 Sxy = float4(1,1,1,1);			// From background texture 
	float Txy;								// Thickness of fluid

	// lookup thickness
	Sxy = g_txBackground.Sample( g_samLinear, In.TextureUV );
	Txy = g_txThickness.Load( int3 ( integerTexCoords, 0) ) * g_gamma * 0.5;
	a = lerp(Cfluid, Sxy, exp(-Txy.x));
	Cout = a*( 1 - fresnel(dot(Nn, Vn), 0.2, g_gamma ))*1.2 + float4(SpecularContrib,0.0);
	
	return Cout;
}


float PS_DRAW_SPRITE_THICKNESS(GS_SPRITE_OUT input) : SV_Target
{
	float Output;

	float fragPos = dot(input.TextureUV*2-1.0, input.TextureUV*2-1.0);
	if(fragPos > 1)
	{
		discard;
	} 
	else
	{
		//perspective divide and viewport transform
		input.ParticlePosition.xyz /= input.ParticlePosition.w;
		input.ParticlePosition.x =  (input.ParticlePosition.x+1) * g_viewport.x * 0.5;
		input.ParticlePosition.y =  (1-input.ParticlePosition.y) * g_viewport.y * 0.5;

		// distance from center of particle
		float r = length(input.Position.xy - input.ParticlePosition.xy);

		// compute projected radius - suspect method
		float h = g_fParticleRad * g_viewport.x / input.ParticlePosition.w;
		h += 50.0;

		Output = 315 / ( 64 * 3.14159265 * pow( h, 9 ) ) * pow(abs(( h * h - r * r )), 3);
	}
	return Output;
}

//--------------------------------------------------------------------------------------
// Geometry Shaders
//--------------------------------------------------------------------------------------
[maxvertexcount(4)]
void GS_DRAW_SPRITE(point VS_SPRITE_OUT input[1], inout TriangleStream<GS_SPRITE_OUT> SpriteStream)
{
    GS_SPRITE_OUT output;

    //
    // Emit two new triangles
    //
    for(int i=0; i<4; i++)
    {
        float3 position = g_positions[i]*g_fParticleRad;
        position = mul( position, (float3x3)g_mInvView ) + input[0].Position;						// world space
		float3 particlePosition = mul( float3(0,0,0), (float3x3)g_mInvView ) + input[0].Position;	// world space

		// Compute view space position
        output.Position = mul( float4(position,1), g_mWorld);										// still world space
		output.Position = mul( output.Position, g_mView );											// view space
		output.ViewSpacePosition = output.Position;	
        output.ParticlePosition = mul( float4(particlePosition,1.0), g_mWorld );
        output.ParticlePosition = mul( output.ParticlePosition, g_mView );
        output.ParticlePosition = mul( output.ParticlePosition, g_mProj );
		output.Position = mul( output.Position, g_mProj );											// proj space

		// Pass down tex coords
        output.TextureUV = g_texcoords[i];

        SpriteStream.Append(output);
    }
    SpriteStream.RestartStrip();
}


[maxvertexcount(4)]
void GS_POINT(point float4 input[1] : SV_POSITION, inout TriangleStream<GS_POINT_OUT> SpriteStream)
{
    GS_POINT_OUT output;
	
    //
    // Emit two new triangles
    //
    for(int i=0; i<4; i++)
    {
        float3 position = g_positions[i]*0.05;
        position = mul( position.xyz, (float3x3)g_mInvView ) + input[0];

        output.Position = mul( float4(position,1.0), g_mWorldViewProjection );

        SpriteStream.Append(output);
    }
    SpriteStream.RestartStrip();
}

//--------------------------------------------------------------------------------------
technique10 RenderScene
{
    pass P0
    {
		SetDepthStencilState( EnableDepth, 0 );
		SetBlendState( NoBlending, float4( 0.0f, 0.0f, 0.0f, 0.0f ), 0xFFFFFFFF );
        SetVertexShader( CompileShader( vs_4_0, VS() ) );
        SetGeometryShader( NULL );
        SetPixelShader( CompileShader( ps_4_0, PS_CUBEMAP() ) );
    }
}

technique10 RenderPoints
{
    pass P0
    {
		SetDepthStencilState( EnableDepth, 0 );
		SetBlendState( NoBlending, float4( 0.0f, 0.0f, 0.0f, 0.0f ), 0xFFFFFFFF );
        SetVertexShader( CompileShader( vs_4_0, VS_POINTS() ) );
        SetGeometryShader( CompileShader( gs_4_0, GS_POINT() ));
        SetPixelShader( CompileShader( ps_4_0, PS_POINTS() ) );
    }
}

technique10 RenderFloor
{
    pass P0
    {
		SetRasterizerState( CullOff );
		SetDepthStencilState( EnableDepth, 0 );
		SetBlendState( NoBlending, float4( 0.0f, 0.0f, 0.0f, 0.0f ), 0xFFFFFFFF );
        SetVertexShader( CompileShader( vs_4_0, VS_BOX() ) );
        SetGeometryShader( NULL );
        SetPixelShader( CompileShader( ps_4_0, PS_TEXTURED_FLOOR() ) );
    }
}
//--------------------------------------------------------------------------------------
technique10 RenderGlassBox
{
    pass P0
    {
		SetRasterizerState( CullOff );
		SetDepthStencilState( EnableDepth, 0 );
		SetBlendState( AlphaBlending, float4( 0.0f, 0.0f, 0.0f, 0.0f ), 0xFFFFFFFF );
        SetVertexShader( CompileShader( vs_4_0, VS_BOX() ) );
        SetGeometryShader( NULL );
        SetPixelShader( CompileShader( ps_4_0, PS_BOX() ) );
    }
}
//--------------------------------------------------------------------------------------
technique10 RenderLogo
{
    pass P0
    {
		SetRasterizerState( CullOff );
		SetDepthStencilState( DisableDepth, 0 );
		SetBlendState( NoBlending, float4( 0.0f, 0.0f, 0.0f, 0.0f ), 0xFFFFFFFF );
        SetVertexShader( CompileShader( vs_4_0, VS_LOGO() ) );
        SetGeometryShader( NULL );
        SetPixelShader( CompileShader( ps_4_0, PS_LOGO() ) );
    }
}

//--------------------------------------------------------------------------------------
technique10 RenderDepthThicknessSprites
{
    pass P0
    {
		SetDepthStencilState( EnableDepth, 0 );
		SetBlendState( NoBlending, float4( 0.0f, 0.0f, 0.0f, 0.0f ), 0xFFFFFFFF );
        SetVertexShader( CompileShader( vs_4_0, VS_DRAW_SPRITE() ) );
        SetGeometryShader( CompileShader( gs_4_0, GS_DRAW_SPRITE() ));
        SetPixelShader( CompileShader( ps_4_0, PS_DRAW_SPRITE_DEPTH() ) );
    }
    pass P1
    {
		SetDepthStencilState( DisableDepth, 0 );
		SetBlendState( AdditiveBlending, float4( 0.0f, 0.0f, 0.0f, 0.0f ), 0xFFFFFFFF );
        SetVertexShader( CompileShader( vs_4_0, VS_DRAW_SPRITE() ) );
        SetGeometryShader( CompileShader( gs_4_0, GS_DRAW_SPRITE() ));
        SetPixelShader( CompileShader( ps_4_0, PS_DRAW_SPRITE_THICKNESS() ) );
    }
}

//-------------------------------------------------------------------------------------- 
technique10 SmoothDepth
{
    pass P0

    {
		SetDepthStencilState( EnableDepth, 0 );
		SetBlendState( NoBlending, float4( 0.0f, 0.0f, 0.0f, 0.0f ), 0xFFFFFFFF );
        SetVertexShader( CompileShader( vs_4_0, VS_MINIMAL() ) );
        SetGeometryShader( NULL );
        //SetPixelShader( CompileShader( ps_4_0, PS_SMOOTH_DEPTH_CURVATURE_FLOW() ) );
		SetPixelShader( CompileShader( ps_4_0, PS_SMOOTH_DEPTH_GAUSSIAN() ) ); 
    }
}

//-------------------------------------------------------------------------------------- 
technique10 ShowTextureDebug
{
    pass P0
    {
		SetDepthStencilState( EnableDepth, 0 );
		SetBlendState( NoBlending, float4( 0.0f, 0.0f, 0.0f, 0.0f ), 0xFFFFFFFF );
        SetVertexShader( CompileShader( vs_4_0, VS() ) );
        SetGeometryShader( NULL );
        SetPixelShader( CompileShader( ps_4_0, PS_TEXTURE() ) );
    }
}

//-------------------------------------------------------------------------------------- 
technique10 ShowCubeMap
{
    pass P0
    {
		SetRasterizerState( CullOff );
		SetDepthStencilState( EnableDepth, 0 );
		SetBlendState( NoBlending, float4( 0.0f, 0.0f, 0.0f, 0.0f ), 0xFFFFFFFF );
        SetVertexShader( CompileShader( vs_4_0, VS() ) );
        SetGeometryShader( NULL );
        SetPixelShader( CompileShader( ps_4_0, PS_CUBEMAP() ) );
    }
}

//-------------------------------------------------------------------------------------- 
technique10 CompositingPass
{
    pass P0
    {
		SetDepthStencilState( EnableDepth, 0 );
		SetBlendState( NoBlending, float4( 0.0f, 0.0f, 0.0f, 0.0f ), 0xFFFFFFFF );
        SetVertexShader( CompileShader( vs_4_0, VS_COMPOSITE() ) );
        SetGeometryShader( NULL );
        SetPixelShader( CompileShader( ps_4_0, PS_COMPOSITE() ) );
    }
}
