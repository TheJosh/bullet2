
cbuffer UpdateConstantsCB : register( b0 )
{
	int numLinks;
	int padding0;
	int padding1;
	int padding2;
};

// Node indices for each link
StructuredBuffer<int2> g_linksVertexIndices : register( t0 );
StructuredBuffer<float4> g_vertexPositions : register( t1 );
StructuredBuffer<float> g_vertexInverseMasses : register( t2 );
StructuredBuffer<float> g_linksMaterialLSC : register( t3 );

RWStructuredBuffer<float> g_linksMassLSC : register( u0 );
RWStructuredBuffer<float> g_linksRestLengthSquared : register( u1 );
RWStructuredBuffer<float> g_linksRestLengths : register( u2 );

[numthreads(128, 1, 1)]
void 
UpdateConstantsKernel( uint3 Gid : SV_GroupID, uint3 DTid : SV_DispatchThreadID, uint3 GTid : SV_GroupThreadID, uint GI : SV_GroupIndex )
{
	int linkID = DTid.x;
	if( linkID < numLinks )
	{	
		int2 nodeIndices = g_linksVertexIndices[linkID];
		int node0 = nodeIndices.x;
		int node1 = nodeIndices.y;
		float linearStiffnessCoefficient = g_linksMaterialLSC[ linkID ];
		
		float3 position0 = g_vertexPositions[node0].xyz;
		float3 position1 = g_vertexPositions[node1].xyz;
		float inverseMass0 = g_vertexInverseMasses[node0];
		float inverseMass1 = g_vertexInverseMasses[node1];

		float3 difference = position0 - position1;
		float length2 = dot(difference, difference);
		float length = sqrt(length2);
	
		g_linksRestLengths[linkID] = length;
		g_linksMassLSC[linkID] = (inverseMass0 + inverseMass1)/linearStiffnessCoefficient;
		g_linksRestLengthSquared[linkID] = length*length;		
	}
}

#if 0
[numthreads(128, 1, 1)]
void 
UpdateFaceAreasKernel( uint3 Gid : SV_GroupID, uint3 DTid : SV_DispatchThreadID, uint3 GTid : SV_GroupThreadID, uint GI : SV_GroupIndex )
{
	int faceID = DTid.x;
	if( faceID < numFaces )
	{	
		float4 node0 = g_nodesx[g_FaceNodeIndices0[faceID]];
		float4 node1 = g_nodesx[g_FaceNodeIndices1[faceID]];
		float4 node2 = g_nodesx[g_FaceNodeIndices2[faceID]];
		
		float3 vector0 = node1.xyz - node0.xyz;
		float3 vector1 = node2.xyz - node0.xyz;
		float3 crossResult = cross(vector0, vector1);
		float area = length(crossResult);
		
		g_FaceAreas[faceID] = area;
	}
}

[numthreads(128, 1, 1)]
void 
ResetAreasKernel( uint3 Gid : SV_GroupID, uint3 DTid : SV_DispatchThreadID, uint3 GTid : SV_GroupThreadID, uint GI : SV_GroupIndex )
{
	int nodeID = DTid.x;
	if( nodeID < numNodes )
	{	
		g_nodesArea[nodeID] = 0.0f;
		g_nodeAreaCounts[nodeID] = 0;
	}
}


[numthreads(128, 1, 1)]
void 
UpdateAreasKernel( uint3 Gid : SV_GroupID, uint3 DTid : SV_DispatchThreadID, uint3 GTid : SV_GroupThreadID, uint GI : SV_GroupIndex )
{
	int faceID = DTid.x + startFace;
	if( DTid.x < numFaces )
	{	
		int nodeIndex0 = g_FaceNodeIndices0[faceID];
		int nodeIndex1 = g_FaceNodeIndices1[faceID];
		int nodeIndex2 = g_FaceNodeIndices2[faceID];
		float nodeArea0 = g_nodesArea[nodeIndex0];
		float nodeArea1 = g_nodesArea[nodeIndex1];
		float nodeArea2 = g_nodesArea[nodeIndex2];
		
		float faceArea = abs(g_FaceAreas[faceID]);
		g_nodeAreaCounts[nodeIndex0] = g_nodeAreaCounts[nodeIndex0] + 1;
		g_nodeAreaCounts[nodeIndex1] = g_nodeAreaCounts[nodeIndex1] + 1;
		g_nodeAreaCounts[nodeIndex2] = g_nodeAreaCounts[nodeIndex2] + 1;
		
		
		nodeArea0 += faceArea;
		nodeArea1 += faceArea;
		nodeArea2 += faceArea;
		
		g_nodesArea[nodeIndex0] = nodeArea0;
		g_nodesArea[nodeIndex1] = nodeArea1;
		g_nodesArea[nodeIndex2] = nodeArea2;		
	}
}


[numthreads(128, 1, 1)]
void 
NormalizeAreasKernel( uint3 Gid : SV_GroupID, uint3 DTid : SV_DispatchThreadID, uint3 GTid : SV_GroupThreadID, uint GI : SV_GroupIndex )
{
	int nodeID = DTid.x;
	if( nodeID < numNodes )
	{	
		float area = g_nodesArea[nodeID];
		int count = g_nodeAreaCounts[nodeID];
		
		float normalizedArea = 0.0f;
		if( count > 0 )
			normalizedArea = area/float(count);
		
		g_nodesArea[nodeID] = normalizedArea;
	}
}
#endif