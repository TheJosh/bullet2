#define float3 float4

float length3(float3 a)
{
	a.w = 0;
	return length(a);
}

float normalize3(float3 a)
{
	a.w = 0;
	return normalize(a);
}

__kernel void 
CollideCylinderKernel( 
	const int startNode,
	cosnt int numNodes,
	const float positionX,
	const float positionY,
	const float positionZ,
	const float radius,
	const int padding1,
	const int padding2,
	__global float4 * g_nodesx)
{
	int nodeID = get_global_id(0);
	if( nodeID < numNodes )
	{	
		// Account for subcloth offset
		nodeID += startNode;
		
		float3 origin = (float3)(positionX, positionY, positionZ, 0.0f);
		float3 nodeX  = g_nodesx[nodeID].xyzw; // fudge for float3 -> float4
		float3 distanceVector = nodeX-origin;
		// Vertical cylinder so reset y
		distanceVector.y = 0;
		float dist = length3(distanceVector);
		const float collisionBoundary = 0.0f;
	
		//penetrationDepth = distanceVector-radius;
		float3 contactNormal = normalize3(distanceVector);
		float3 newX = origin + contactNormal*(radius + collisionBoundary);
		newX.y = nodeX.y;
		if( dist < (radius + collisionBoundary) )
			g_nodesx[nodeID] = (float4)(newX, 0.0f);
	}
}