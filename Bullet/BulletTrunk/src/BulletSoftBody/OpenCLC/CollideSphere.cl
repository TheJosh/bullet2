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


// Node positions

__kernel void 
CollideSphereKernel( 
	int numNodes,
	float positionX,
	float positionY,
	float positionZ,
	float radius,
	__global float4 * g_nodesx)
{
	int nodeID = get_global_id(0);
	if( nodeID < numNodes )
	{	
		float3 origin = (float3)(positionX, positionY, positionZ, 0.0f);
		float3 nodeX = g_nodesx[nodeID].xyzw; // fudge for float3
		float3 distanceVector = nodeX-origin;
		float dist = length3(distanceVector);
		const float collisionBoundary = 0.4f;
	
		//penetrationDepth = distanceVector-radius;
		float3 contactNormal = normalize3(distanceVector);
		float3 newX = origin + contactNormal*(radius + collisionBoundary);
		if( dist < (radius + collisionBoundary) )
			g_nodesx[nodeID] = (float4)(newX, 0.0f);
	}
}