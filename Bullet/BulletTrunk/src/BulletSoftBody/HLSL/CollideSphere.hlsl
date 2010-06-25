
cbuffer CollideSphereCB  : register( b0 )
{
	int numNodes;
	float positionX;
	float positionY;
	float positionZ;

	float radius;
	int padding1;
	int padding2;
	int padding3;
};


// Node positions
RWStructuredBuffer<float4> g_nodesx : register( u0 );

[numthreads(128, 1, 1)]
void 
CollideSphereKernel( uint3 Gid : SV_GroupID, uint3 DTid : SV_DispatchThreadID, uint3 GTid : SV_GroupThreadID, uint GI : SV_GroupIndex )
{
	int nodeID = DTid.x;
	if( nodeID < numNodes )
	{	
		float3 origin = float3(positionX, positionY, positionZ);
		float3 nodeX = g_nodesx[nodeID].xyz;
		float3 distanceVector = nodeX-origin;
		float dist = length(distanceVector);
		const float collisionBoundary = 0.4f;
	
		//penetrationDepth = distanceVector-radius;
		float3 contactNormal = normalize(distanceVector);
		float3 newX = origin + contactNormal*(radius + collisionBoundary);
		if( dist < (radius + collisionBoundary) )
			g_nodesx[nodeID] = float4(newX, 0.0f);
	}
}