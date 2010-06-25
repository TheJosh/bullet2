
cbuffer CollideCylinderCB  : register( b0 )
{
	int startNode;
	int numNodes;
	float positionX;
	float positionY;

	float positionZ;
	float radius;
	int padding1;
	int padding2;
};


// Node positions
RWStructuredBuffer<float4> g_nodesx : register( u0 );

[numthreads(128, 1, 1)]
void 
CollideCylinderKernel( uint3 Gid : SV_GroupID, uint3 DTid : SV_DispatchThreadID, uint3 GTid : SV_GroupThreadID, uint GI : SV_GroupIndex )
{
	int nodeID = DTid.x;
	if( nodeID < numNodes )
	{	
		// Account for subcloth offset
		nodeID += startNode;
		
		float3 origin = float3(positionX, positionY, positionZ);
		float3 nodeX = g_nodesx[nodeID].xyz;
		float3 distanceVector = nodeX-origin;
		// Vertical cylinder so reset y
		distanceVector.y = 0;
		float dist = length(distanceVector);
		const float collisionBoundary = 0.0f;
	
		//penetrationDepth = distanceVector-radius;
		float3 contactNormal = normalize(distanceVector);
		float3 newX = origin + contactNormal*(radius + collisionBoundary);
		newX.y = nodeX.y;
		if( dist < (radius + collisionBoundary) )
			g_nodesx[nodeID] = float4(newX, 0.0f);
	}
}