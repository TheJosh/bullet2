
cbuffer AddVelocityCB : register( b0 )
{
	int startNode;
	int lastNode;
	float velocityX;
	float velocityY;
	float velocityZ;
	int padding1;
	int padding2;
	int padding3;
};


StructuredBuffer<float> g_nodesim : register( t0 );

RWStructuredBuffer<float4> g_nodesv : register( u0 );


[numthreads(128, 1, 1)]
void 
AddVelocityKernel( uint3 Gid : SV_GroupID, uint3 DTid : SV_DispatchThreadID, uint3 GTid : SV_GroupThreadID, uint GI : SV_GroupIndex )
{
	int nodeID = DTid.x + startNode;
	if( nodeID < lastNode )
	{	
		if( g_nodesim[nodeID] )
		{
			float4 nodeV = g_nodesv[nodeID];
			nodeV.x += velocityX;
			nodeV.y += velocityY;
			nodeV.z += velocityZ;
			g_nodesv[nodeID] = nodeV;
		}
	}
}