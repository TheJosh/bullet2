/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef OPTIMIZED_BVH_H
#define OPTIMIZED_BVH_H
#include "LinearMath/SimdVector3.h"
#include <vector>

class btStridingMeshInterface;

/// btOptimizedBvhNode contains both internal and leaf node information.
/// It hasn't been optimized yet for storage. Some obvious optimizations are:
/// Removal of the pointers (can already be done, they are not used for traversal)
/// and storing aabbmin/max as quantized integers.
/// 'subpart' doesn't need an integer either. It allows to re-use graphics triangle
/// meshes stored in a non-uniform way (like batches/subparts of triangle-fans
struct btOptimizedBvhNode
{

	btSimdVector3	m_aabbMin;
	btSimdVector3	m_aabbMax;

//these 2 pointers are obsolete, the stackless traversal just uses the escape index
	btOptimizedBvhNode*	m_leftChild;
	btOptimizedBvhNode*	m_rightChild;

	int	m_escapeIndex;

	//for child nodes
	int	m_subPart;
	int	m_triangleIndex;

};

class btNodeOverlapCallback
{
public:
	virtual ~btNodeOverlapCallback() {};

	virtual void ProcessNode(const btOptimizedBvhNode* node) = 0;
};

typedef std::vector<btOptimizedBvhNode>	NodeArray;


///OptimizedBvh store an AABB tree that can be quickly traversed on CPU (and SPU,GPU in future)
class btOptimizedBvh
{
	btOptimizedBvhNode*	m_rootNode1;
	
	btOptimizedBvhNode*	m_contiguousNodes;
	int					m_curNodeIndex;

	int					m_numNodes;

	NodeArray			m_leafNodes;

public:
	btOptimizedBvh() :m_rootNode1(0), m_numNodes(0) { }
	virtual ~btOptimizedBvh();
	
	void	Build(btStridingMeshInterface* triangles);

	btOptimizedBvhNode*	BuildTree	(NodeArray&	leafNodes,int startIndex,int endIndex);

	int	CalcSplittingAxis(NodeArray&	leafNodes,int startIndex,int endIndex);

	int	SortAndCalcSplittingIndex(NodeArray&	leafNodes,int startIndex,int endIndex,int splitAxis);
	
	void	WalkTree(btOptimizedBvhNode* rootNode,btNodeOverlapCallback* nodeCallback,const btSimdVector3& aabbMin,const btSimdVector3& aabbMax) const;
	
	void	WalkStacklessTree(btOptimizedBvhNode* rootNode,btNodeOverlapCallback* nodeCallback,const btSimdVector3& aabbMin,const btSimdVector3& aabbMax) const;
	

	//OptimizedBvhNode*	GetRootNode() { return m_rootNode1;}

	int					GetNumNodes() { return m_numNodes;}

	void	ReportAabbOverlappingNodex(btNodeOverlapCallback* nodeCallback,const btSimdVector3& aabbMin,const btSimdVector3& aabbMax) const;

	void	ReportSphereOverlappingNodex(btNodeOverlapCallback* nodeCallback,const btSimdVector3& aabbMin,const btSimdVector3& aabbMax) const;


};


#endif //OPTIMIZED_BVH_H

