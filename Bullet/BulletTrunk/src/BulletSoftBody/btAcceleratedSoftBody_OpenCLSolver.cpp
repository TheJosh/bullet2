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


#include "BulletSoftBody/btAcceleratedSoftBody_Settings.h"

#ifdef BULLET_USE_OPENCL


#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"
#include "BulletMultiThreaded/vectormath/scalar/cpp/vectormath_aos.h"
#include "BulletMultiThreaded/vectormath/scalar/cpp/mat_aos.h"
#include "BulletMultiThreaded/vectormath/scalar/cpp/vec_aos.h"

#include "BulletSoftBody/btAcceleratedSoftBody_OpenCLSolver.h"
#include "BulletSoftBody/btAcceleratedSoftBody_VertexBuffers.h"


#define MSTRINGIFY(A) #A
static char* PrepareLinksCLString = 
#include "OpenCLC/PrepareLinks.cl"
static char* UpdatePositionsFromVelocitiesCLString = 
#include "OpenCLC/UpdatePositionsFromVelocities.cl"
static char* SolvePositionsCLString = 
#include "OpenCLC/SolvePositions.cl"
static char* UpdateNodesCLString = 
#include "OpenCLC/UpdateNodes.cl"
static char* UpdatePositionsCLString = 
#include "OpenCLC/UpdatePositions.cl"
static char* UpdateConstantsCLString = 
#include "OpenCLC/UpdateConstants.cl"
static char* IntegrateCLString = 
#include "OpenCLC/Integrate.cl"
static char* ApplyForcesCLString = 
#include "OpenCLC/ApplyForces.cl"
static char* UpdateNormalsCLString = 
#include "OpenCLC/UpdateNormals.cl"


btSoftBodyVertexDataOpenCL::btSoftBodyVertexDataOpenCL( cl::CommandQueue queue) :
    m_queue(queue),
	m_clClothIdentifier( queue, &m_clothIdentifier, false ),
	m_clVertexPosition( queue, &m_vertexPosition, false ),
	m_clVertexPreviousPosition( queue, &m_vertexPreviousPosition, false ),
	m_clVertexVelocity( queue, &m_vertexVelocity, false ),
	m_clVertexForceAccumulator( queue, &m_vertexForceAccumulator, false ),
	m_clVertexNormal( queue, &m_vertexNormal, false ),
	m_clVertexInverseMass( queue, &m_vertexInverseMass, false ),
	m_clVertexArea( queue, &m_vertexArea, false ),
	m_clVertexTriangleCount( queue, &m_vertexTriangleCount, false )
{
}

btSoftBodyVertexDataOpenCL::~btSoftBodyVertexDataOpenCL()
{

}

bool btSoftBodyVertexDataOpenCL::onAccelerator()
{
	return m_onGPU;
}

bool btSoftBodyVertexDataOpenCL::moveToAccelerator()
{
	bool success = true;
	success = success && m_clClothIdentifier.moveToGPU();
	success = success && m_clVertexPosition.moveToGPU();
	success = success && m_clVertexPreviousPosition.moveToGPU();
	success = success && m_clVertexVelocity.moveToGPU();
	success = success && m_clVertexForceAccumulator.moveToGPU();
	success = success && m_clVertexNormal.moveToGPU();
	success = success && m_clVertexInverseMass.moveToGPU();
	success = success && m_clVertexArea.moveToGPU();
	success = success && m_clVertexTriangleCount.moveToGPU();

	if( success )
		m_onGPU = true;

	return success;
}

bool btSoftBodyVertexDataOpenCL::moveFromAccelerator()
{
	bool success = true;
	success = success && m_clClothIdentifier.moveFromGPU();
	success = success && m_clVertexPosition.moveFromGPU();
	success = success && m_clVertexPreviousPosition.moveFromGPU();
	success = success && m_clVertexVelocity.moveFromGPU();
	success = success && m_clVertexForceAccumulator.moveFromGPU();
	success = success && m_clVertexNormal.moveFromGPU();
	success = success && m_clVertexInverseMass.moveFromGPU();
	success = success && m_clVertexArea.moveFromGPU();
	success = success && m_clVertexTriangleCount.moveFromGPU();

	if( success )
		m_onGPU = true;

	return success;
}




btSoftBodyLinkDataOpenCL::btSoftBodyLinkDataOpenCL(cl::CommandQueue queue) :
    m_queue(queue),
	m_clLinks( queue, &m_links, false ),
	m_clLinkStrength( queue, &m_linkStrength, false ),
	m_clLinksMassLSC( queue, &m_linksMassLSC, false ),
	m_clLinksRestLengthSquared( queue, &m_linksRestLengthSquared, false ),
	m_clLinksCLength( queue, &m_linksCLength, false ),
	m_clLinksLengthRatio( queue, &m_linksLengthRatio, false ),
	m_clLinksRestLength( queue, &m_linksRestLength, false ),
	m_clLinksMaterialLinearStiffnessCoefficient( queue, &m_linksMaterialLinearStiffnessCoefficient, false )
{
}

btSoftBodyLinkDataOpenCL::~btSoftBodyLinkDataOpenCL()
{
}

/** Allocate enough space in all link-related arrays to fit numLinks links */
void btSoftBodyLinkDataOpenCL::createLinks( int numLinks )
{
	int previousSize = m_links.size();
	int newSize = previousSize + numLinks;

	btSoftBodyLinkData::createLinks( numLinks );

	// Resize the link addresses array as well
	m_linkAddresses.resize( newSize );
}

/** Insert the link described into the correct data structures assuming space has already been allocated by a call to createLinks */
void btSoftBodyLinkDataOpenCL::setLinkAt( 
	const LinkDescription &link, 
	int linkIndex )
{
	btSoftBodyLinkData::setLinkAt( link, linkIndex );

	// Set the link index correctly for initialisation
	m_linkAddresses[linkIndex] = linkIndex;
}

bool btSoftBodyLinkDataOpenCL::onAccelerator()
{
	return m_onGPU;
}

bool btSoftBodyLinkDataOpenCL::moveToAccelerator()
{
	bool success = true;
	success = success && m_clLinks.moveToGPU();
	success = success && m_clLinkStrength.moveToGPU();
	success = success && m_clLinksMassLSC.moveToGPU();
	success = success && m_clLinksRestLengthSquared.moveToGPU();
	success = success && m_clLinksCLength.moveToGPU();
	success = success && m_clLinksLengthRatio.moveToGPU();
	success = success && m_clLinksRestLength.moveToGPU();
	success = success && m_clLinksMaterialLinearStiffnessCoefficient.moveToGPU();

	if( success ) {
		m_onGPU = true;
	}

	return success;
}

bool btSoftBodyLinkDataOpenCL::moveFromAccelerator()
{
	bool success = true;
	success = success && m_clLinks.moveFromGPU();
	success = success && m_clLinkStrength.moveFromGPU();
	success = success && m_clLinksMassLSC.moveFromGPU();
	success = success && m_clLinksRestLengthSquared.moveFromGPU();
	success = success && m_clLinksCLength.moveFromGPU();
	success = success && m_clLinksLengthRatio.moveFromGPU();
	success = success && m_clLinksRestLength.moveFromGPU();
	success = success && m_clLinksMaterialLinearStiffnessCoefficient.moveFromGPU();

	if( success ) {
		m_onGPU = false;
	}

	return success;
}

/**
 * Generate (and later update) the batching for the entire link set.
 * This redoes a lot of work because it batches the entire set when each cloth is inserted.
 * In theory we could delay it until just before we need the cloth.
 * It's a one-off overhead, though, so that is a later optimisation.
 */
void btSoftBodyLinkDataOpenCL::generateBatches()
{
	int numLinks = getNumLinks();

	// Do the graph colouring here temporarily
	btAlignedObjectArray< int > batchValues;
	batchValues.resize( numLinks, 0 );

	// Find the maximum vertex value internally for now
	int maxVertex = 0;
	for( int linkIndex = 0; linkIndex < numLinks; ++linkIndex )
	{
		int vertex0 = getVertexPair(linkIndex).vertex0;
		int vertex1 = getVertexPair(linkIndex).vertex1;
		if( vertex0 > maxVertex )
			maxVertex = vertex0;
		if( vertex1 > maxVertex )
			maxVertex = vertex1;
	}
	int numVertices = maxVertex + 1;

	// Set of lists, one for each node, specifying which colours are connected
	// to that node.
	// No two edges into a node can share a colour.
	btAlignedObjectArray< btAlignedObjectArray< int > > vertexConnectedColourLists;
	vertexConnectedColourLists.resize(numVertices);

	// Simple algorithm that chooses the lowest batch number
	// that none of the links attached to either of the connected 
	// nodes is in
	for( int linkIndex = 0; linkIndex < numLinks; ++linkIndex )
	{				
		int linkLocation = m_linkAddresses[linkIndex];

		int vertex0 = getVertexPair(linkLocation).vertex0;
		int vertex1 = getVertexPair(linkLocation).vertex1;

		// Get the two node colour lists
		btAlignedObjectArray< int > &colourListVertex0( vertexConnectedColourLists[vertex0] );
		btAlignedObjectArray< int > &colourListVertex1( vertexConnectedColourLists[vertex1] );

		// Choose the minimum colour that is in neither list
		int colour = 0;
		while( colourListVertex0.findLinearSearch(colour) != colourListVertex0.size() || colourListVertex1.findLinearSearch(colour) != colourListVertex1.size()  )
			++colour;
		// i should now be the minimum colour in neither list
		// Add to the two lists so that future edges don't share
		// And store the colour against this edge

		colourListVertex0.push_back(colour);
		colourListVertex1.push_back(colour);
		batchValues[linkIndex] = colour;
	}

	// Check the colour counts
	btAlignedObjectArray< int > batchCounts;
	for( int i = 0; i < numLinks; ++i )
	{
		int batch = batchValues[i];
		if( batch >= batchCounts.size() )
			batchCounts.push_back(1);
		else
			++(batchCounts[batch]);
	}

	m_batchStartLengths.resize(batchCounts.size());
	if( m_batchStartLengths.size() > 0 )
	{
		m_batchStartLengths.resize(batchCounts.size());
		m_batchStartLengths[0] = std::pair< int, int >( 0, 0 );

		int sum = 0;
		for( int batchIndex = 0; batchIndex < batchCounts.size(); ++batchIndex )
		{
			m_batchStartLengths[batchIndex].first = sum;
			m_batchStartLengths[batchIndex].second = batchCounts[batchIndex];
			sum += batchCounts[batchIndex];
		}
	}

	/////////////////////////////
	// Sort data based on batches

	// Create source arrays by copying originals
	btAlignedObjectArray<LinkNodePair>									m_links_Backup(m_links);
	btAlignedObjectArray<float>											m_linkStrength_Backup(m_linkStrength);
	btAlignedObjectArray<float>											m_linksMassLSC_Backup(m_linksMassLSC);
	btAlignedObjectArray<float>											m_linksRestLengthSquared_Backup(m_linksRestLengthSquared);
	btAlignedObjectArray<Vectormath::Aos::Vector3>						m_linksCLength_Backup(m_linksCLength);
	btAlignedObjectArray<float>											m_linksLengthRatio_Backup(m_linksLengthRatio);
	btAlignedObjectArray<float>											m_linksRestLength_Backup(m_linksRestLength);
	btAlignedObjectArray<float>											m_linksMaterialLinearStiffnessCoefficient_Backup(m_linksMaterialLinearStiffnessCoefficient);


	for( int batch = 0; batch < batchCounts.size(); ++batch )
		batchCounts[batch] = 0;

	// Do sort as single pass into destination arrays	
	for( int linkIndex = 0; linkIndex < numLinks; ++linkIndex )
	{
		// To maintain locations run off the original link locations rather than the current position.
		// It's not cache efficient, but as we run this rarely that should not matter.
		// It's faster than searching the link location array for the current location and then updating it.
		// The other alternative would be to unsort before resorting, but this is equivalent to doing that.
		int linkLocation = m_linkAddresses[linkIndex];

		// Obtain batch and calculate target location for the
		// next element in that batch, incrementing the batch counter
		// afterwards
		int batch = batchValues[linkIndex];
		int newLocation = m_batchStartLengths[batch].first + batchCounts[batch];

		batchCounts[batch] = batchCounts[batch] + 1;
		m_links[newLocation] = m_links_Backup[linkLocation];
#if 1
		m_linkStrength[newLocation] = m_linkStrength_Backup[linkLocation];
		m_linksMassLSC[newLocation] = m_linksMassLSC_Backup[linkLocation];
		m_linksRestLengthSquared[newLocation] = m_linksRestLengthSquared_Backup[linkLocation];
		m_linksLengthRatio[newLocation] = m_linksLengthRatio_Backup[linkLocation];
		m_linksRestLength[newLocation] = m_linksRestLength_Backup[linkLocation];
		m_linksMaterialLinearStiffnessCoefficient[newLocation] = m_linksMaterialLinearStiffnessCoefficient_Backup[linkLocation];
#endif
		// Update the locations array to account for the moved entry
		m_linkAddresses[linkIndex] = newLocation;
	}


} // void generateBatches()





btSoftBodyTriangleDataOpenCL::btSoftBodyTriangleDataOpenCL( cl::CommandQueue queue ) : 
    m_queue( queue ),
	m_clVertexIndices( queue, &m_vertexIndices, false ),
	m_clArea( queue, &m_area, false ),
	m_clNormal( queue, &m_normal, false )
{
}

btSoftBodyTriangleDataOpenCL::~btSoftBodyTriangleDataOpenCL()
{
}

/** Allocate enough space in all link-related arrays to fit numLinks links */
void btSoftBodyTriangleDataOpenCL::createTriangles( int numTriangles )
{
	int previousSize = getNumTriangles();
	int newSize = previousSize + numTriangles;

	btSoftBodyTriangleData::createTriangles( numTriangles );

	// Resize the link addresses array as well
	m_triangleAddresses.resize( newSize );
}

/** Insert the link described into the correct data structures assuming space has already been allocated by a call to createLinks */
void btSoftBodyTriangleDataOpenCL::setTriangleAt( const btSoftBodyTriangleData::TriangleDescription &triangle, int triangleIndex )
{
	btSoftBodyTriangleData::setTriangleAt( triangle, triangleIndex );

	m_triangleAddresses[triangleIndex] = triangleIndex;
}

bool btSoftBodyTriangleDataOpenCL::onAccelerator()
{
	return m_onGPU;
}

bool btSoftBodyTriangleDataOpenCL::moveToAccelerator()
{
	bool success = true;
	success = success && m_clVertexIndices.moveToGPU();
	success = success && m_clArea.moveToGPU();
	success = success && m_clNormal.moveToGPU();

	if( success )
		m_onGPU = true;

	return success;
}

bool btSoftBodyTriangleDataOpenCL::moveFromAccelerator()
{
	bool success = true;
	success = success && m_clVertexIndices.moveFromGPU();
	success = success && m_clArea.moveFromGPU();
	success = success && m_clNormal.moveFromGPU();

	if( success )
		m_onGPU = true;

	return success;
}

/**
 * Generate (and later update) the batching for the entire triangle set.
 * This redoes a lot of work because it batches the entire set when each cloth is inserted.
 * In theory we could delay it until just before we need the cloth.
 * It's a one-off overhead, though, so that is a later optimisation.
 */
void btSoftBodyTriangleDataOpenCL::generateBatches()
{
	int numTriangles = getNumTriangles();
	if( numTriangles == 0 )
		return;

	// Do the graph colouring here temporarily
	btAlignedObjectArray< int > batchValues;
	batchValues.resize( numTriangles );

	// Find the maximum vertex value internally for now
	int maxVertex = 0;
	for( int triangleIndex = 0; triangleIndex < numTriangles; ++triangleIndex )
	{
		int vertex0 = getVertexSet(triangleIndex).vertex0;
		int vertex1 = getVertexSet(triangleIndex).vertex1;
		int vertex2 = getVertexSet(triangleIndex).vertex2;
		
		if( vertex0 > maxVertex )
			maxVertex = vertex0;
		if( vertex1 > maxVertex )
			maxVertex = vertex1;
		if( vertex2 > maxVertex )
			maxVertex = vertex2;
	}
	int numVertices = maxVertex + 1;

	// Set of lists, one for each node, specifying which colours are connected
	// to that node.
	// No two edges into a node can share a colour.
	btAlignedObjectArray< btAlignedObjectArray< int > > vertexConnectedColourLists;
	vertexConnectedColourLists.resize(numVertices);


	//std::cout << "\n";
	// Simple algorithm that chooses the lowest batch number
	// that none of the faces attached to either of the connected 
	// nodes is in
	for( int triangleIndex = 0; triangleIndex < numTriangles; ++triangleIndex )
	{
		// To maintain locations run off the original link locations rather than the current position.
		// It's not cache efficient, but as we run this rarely that should not matter.
		// It's faster than searching the link location array for the current location and then updating it.
		// The other alternative would be to unsort before resorting, but this is equivalent to doing that.
		int triangleLocation = m_triangleAddresses[triangleIndex];

		int vertex0 = getVertexSet(triangleLocation).vertex0;
		int vertex1 = getVertexSet(triangleLocation).vertex1;
		int vertex2 = getVertexSet(triangleLocation).vertex2;

		// Get the three node colour lists
		btAlignedObjectArray< int > &colourListVertex0( vertexConnectedColourLists[vertex0] );
		btAlignedObjectArray< int > &colourListVertex1( vertexConnectedColourLists[vertex1] );
		btAlignedObjectArray< int > &colourListVertex2( vertexConnectedColourLists[vertex2] );

		// Choose the minimum colour that is in none of the lists
		int colour = 0;
		while( 
			colourListVertex0.findLinearSearch(colour) != colourListVertex0.size() || 
			colourListVertex1.findLinearSearch(colour) != colourListVertex1.size() ||
			colourListVertex2.findLinearSearch(colour) != colourListVertex2.size() )
		{
			++colour;
		}
		// i should now be the minimum colour in neither list
		// Add to the three lists so that future edges don't share
		// And store the colour against this face
		colourListVertex0.push_back(colour);
		colourListVertex1.push_back(colour);
		colourListVertex2.push_back(colour);

		batchValues[triangleIndex] = colour;
	}


	// Check the colour counts
	btAlignedObjectArray< int > batchCounts;
	for( int i = 0; i < numTriangles; ++i )
	{
		int batch = batchValues[i];
		if( batch >= batchCounts.size() )
			batchCounts.push_back(1);
		else
			++(batchCounts[batch]);
	}


	m_batchStartLengths.resize(batchCounts.size());
	m_batchStartLengths[0] = std::pair< int, int >( 0, 0 );


	int sum = 0;
	for( int batchIndex = 0; batchIndex < batchCounts.size(); ++batchIndex )
	{
		m_batchStartLengths[batchIndex].first = sum;
		m_batchStartLengths[batchIndex].second = batchCounts[batchIndex];
		sum += batchCounts[batchIndex];
	}
	
	/////////////////////////////
	// Sort data based on batches
	
	// Create source arrays by copying originals
	btAlignedObjectArray<btSoftBodyTriangleData::TriangleNodeSet>							m_vertexIndices_Backup(m_vertexIndices);
	btAlignedObjectArray<float>										m_area_Backup(m_area);
	btAlignedObjectArray<Vectormath::Aos::Vector3>					m_normal_Backup(m_normal);


	for( int batch = 0; batch < batchCounts.size(); ++batch )
		batchCounts[batch] = 0;

	// Do sort as single pass into destination arrays	
	for( int triangleIndex = 0; triangleIndex < numTriangles; ++triangleIndex )
	{
		// To maintain locations run off the original link locations rather than the current position.
		// It's not cache efficient, but as we run this rarely that should not matter.
		// It's faster than searching the link location array for the current location and then updating it.
		// The other alternative would be to unsort before resorting, but this is equivalent to doing that.
		int triangleLocation = m_triangleAddresses[triangleIndex];

		// Obtain batch and calculate target location for the
		// next element in that batch, incrementing the batch counter
		// afterwards
		int batch = batchValues[triangleIndex];
		int newLocation = m_batchStartLengths[batch].first + batchCounts[batch];

		batchCounts[batch] = batchCounts[batch] + 1;
		m_vertexIndices[newLocation] = m_vertexIndices_Backup[triangleLocation];
		m_area[newLocation] = m_area_Backup[triangleLocation];
		m_normal[newLocation] = m_normal_Backup[triangleLocation];

		// Update the locations array to account for the moved entry
		m_triangleAddresses[triangleIndex] = newLocation;
	}
} // btSoftBodyTriangleDataOpenCL::generateBatches







btOpenCLSoftBodySolver::btOpenCLSoftBodySolver(const cl::CommandQueue &queue) :
	m_linkData(queue),
	m_vertexData(queue),
	m_triangleData(queue),
	m_clPerClothAcceleration(queue, &m_perClothAcceleration, true ),
	m_clPerClothWindVelocity(queue, &m_perClothWindVelocity, true ),
	m_clPerClothDampingFactor(queue, &m_perClothDampingFactor, true ),
	m_clPerClothVelocityCorrectionCoefficient(queue, &m_perClothVelocityCorrectionCoefficient, true ),
	m_clPerClothLiftFactor(queue, &m_perClothLiftFactor, true ),
	m_clPerClothDragFactor(queue, &m_perClothDragFactor, true ),
	m_clPerClothMediumDensity(queue, &m_perClothMediumDensity, true ),
	m_queue( queue )
{
	// Initial we will clearly need to update solver constants
	// For now this is global for the cloths linked with this solver - we should probably make this body specific 
	// for performance in future once we understand more clearly when constants need to be updated
	m_updateSolverConstants = true;

	m_shadersInitialized = false;
}

btOpenCLSoftBodySolver::~btOpenCLSoftBodySolver()
{
}

void btOpenCLSoftBodySolver::optimize()
{
	if( checkInitialized() )
	{
		btAssert("Initialization of OpenCL solver failed\n");
	}
	m_linkData.generateBatches();		
	m_triangleData.generateBatches();
}

int btOpenCLSoftBodySolver::ownCloth( btAcceleratedSoftBodyInterface *cloth )
{
	// Ensure that per-cloth acceleration and velocity are large enough to cope
	int clothIdentifier = m_cloths.size();

	// TODO: Check that it's not already there and ensure it stays ordered
	m_cloths.push_back( cloth );	

	if( m_perClothAcceleration.size() <= clothIdentifier )
		m_perClothAcceleration.resize( clothIdentifier + 1 );
	if( m_perClothWindVelocity.size() <= clothIdentifier )
		m_perClothWindVelocity.resize( clothIdentifier + 1 );
	if( m_perClothDampingFactor.size() <= clothIdentifier )
		m_perClothDampingFactor.resize( clothIdentifier + 1 );
	if( m_perClothVelocityCorrectionCoefficient.size() <= clothIdentifier )
		m_perClothVelocityCorrectionCoefficient.resize( clothIdentifier + 1 );
	if( m_perClothLiftFactor.size() <= clothIdentifier )
		m_perClothLiftFactor.resize( clothIdentifier + 1 );
	if( m_perClothDragFactor.size() <= clothIdentifier )
		m_perClothDragFactor.resize( clothIdentifier + 1 );
	if( m_perClothMediumDensity.size() <= clothIdentifier )
		m_perClothMediumDensity.resize( clothIdentifier + 1 );			

	return clothIdentifier;
}

void btOpenCLSoftBodySolver::removeCloth( btAcceleratedSoftBodyInterface *cloth )
{
	btAssert("Cannot remove cloths yet.");
}

btSoftBodyLinkData &btOpenCLSoftBodySolver::getLinkData()
{
	// TODO: Consider setting link data to "changed" here
	return m_linkData;
}

btSoftBodyVertexData &btOpenCLSoftBodySolver::getVertexData()
{
	// TODO: Consider setting vertex data to "changed" here
	return m_vertexData;
}

btSoftBodyTriangleData &btOpenCLSoftBodySolver::getTriangleData()
{
	// TODO: Consider setting triangle data to "changed" here
	return m_triangleData;
}

void btOpenCLSoftBodySolver::addVelocity( Vectormath::Aos::Vector3 velocity )
{
	int numVertices = m_vertexData.getNumVertices();
	for( int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex )
	{
		if( m_vertexData.getInverseMass( vertexIndex ) > 0 )
			m_vertexData.getVelocity( vertexIndex ) += velocity;
	}
}

void btOpenCLSoftBodySolver::setPerClothAcceleration( int clothIdentifier, Vectormath::Aos::Vector3 acceleration )
{
	m_perClothAcceleration[clothIdentifier] = acceleration;
	m_clPerClothAcceleration.changedOnCPU();
}

void btOpenCLSoftBodySolver::setPerClothWindVelocity( int clothIdentifier, Vectormath::Aos::Vector3 windVelocity )
{
	m_perClothWindVelocity[clothIdentifier] = windVelocity;
	m_clPerClothWindVelocity.changedOnCPU();
}

void btOpenCLSoftBodySolver::setPerClothMediumDensity( int clothIdentifier, float mediumDensity )
{
	m_perClothMediumDensity[clothIdentifier] = mediumDensity;
	m_clPerClothMediumDensity.changedOnCPU();
}

void btOpenCLSoftBodySolver::setPerClothDampingFactor( int clothIdentifier, float dampingFactor )
{
	m_perClothDampingFactor[clothIdentifier] = dampingFactor;
	m_clPerClothDampingFactor.changedOnCPU();
}

void btOpenCLSoftBodySolver::setPerClothVelocityCorrectionCoefficient( int clothIdentifier, float velocityCorrectionCoefficient )
{
	m_perClothVelocityCorrectionCoefficient[clothIdentifier] = velocityCorrectionCoefficient;
	m_clPerClothVelocityCorrectionCoefficient.changedOnCPU();
}		

void btOpenCLSoftBodySolver::setPerClothLiftFactor( int clothIdentifier, float liftFactor )
{
	m_perClothLiftFactor[clothIdentifier] = liftFactor;
	m_clPerClothLiftFactor.changedOnCPU();
}

/** Drag parameter for wind action on cloth. */
void btOpenCLSoftBodySolver::setPerClothDragFactor( int clothIdentifier, float dragFactor )
{
	m_perClothDragFactor[clothIdentifier] = dragFactor;
	m_clPerClothDragFactor.changedOnCPU();
}

bool btOpenCLSoftBodySolver::checkInitialized()
{
	return buildShaders();
}

void btOpenCLSoftBodySolver::resetNormalsAndAreas( int numVertices )
{
	resetNormalsAndAreasKernel.kernel.setArg(0, numVertices);
	resetNormalsAndAreasKernel.kernel.setArg(1, m_vertexData.m_clVertexNormal.getBuffer());
	resetNormalsAndAreasKernel.kernel.setArg(2, m_vertexData.m_clVertexArea.getBuffer());

	int	numWorkItems = workGroupSize*((numVertices + (workGroupSize-1)) / workGroupSize);
	cl_int err = m_queue.enqueueNDRangeKernel(resetNormalsAndAreasKernel.kernel, cl::NullRange, cl::NDRange(numWorkItems), cl::NDRange(workGroupSize));
	if( err != CL_SUCCESS )
	{
		btAssert( "enqueueNDRangeKernel(resetNormalsAndAreasKernel)" );
	}
}

void btOpenCLSoftBodySolver::normalizeNormalsAndAreas( int numVertices )
{
	normalizeNormalsAndAreasKernel.kernel.setArg(0, numVertices);
	normalizeNormalsAndAreasKernel.kernel.setArg(1, m_vertexData.m_clVertexTriangleCount.getBuffer());
	normalizeNormalsAndAreasKernel.kernel.setArg(2, m_vertexData.m_clVertexNormal.getBuffer());
	normalizeNormalsAndAreasKernel.kernel.setArg(3, m_vertexData.m_clVertexArea.getBuffer());

	int	numWorkItems = workGroupSize*((numVertices + (workGroupSize-1)) / workGroupSize);
	cl_int err = m_queue.enqueueNDRangeKernel(normalizeNormalsAndAreasKernel.kernel, cl::NullRange, cl::NDRange(numWorkItems), cl::NDRange(workGroupSize));
	if( err != CL_SUCCESS ) 
	{
		btAssert( "enqueueNDRangeKernel(normalizeNormalsAndAreasKernel)");
	}
}

void btOpenCLSoftBodySolver::executeUpdateSoftBodies( int firstTriangle, int numTriangles )
{
	updateSoftBodiesKernel.kernel.setArg(0, firstTriangle);
	updateSoftBodiesKernel.kernel.setArg(1, numTriangles);
	updateSoftBodiesKernel.kernel.setArg(2, m_triangleData.m_clVertexIndices.getBuffer());
	updateSoftBodiesKernel.kernel.setArg(3, m_vertexData.m_clVertexPosition.getBuffer());
	updateSoftBodiesKernel.kernel.setArg(4, m_vertexData.m_clVertexNormal.getBuffer());
	updateSoftBodiesKernel.kernel.setArg(5, m_vertexData.m_clVertexArea.getBuffer());
	updateSoftBodiesKernel.kernel.setArg(6, m_triangleData.m_clNormal.getBuffer());
	updateSoftBodiesKernel.kernel.setArg(7, m_triangleData.m_clArea.getBuffer());


	int	numWorkItems = workGroupSize*((numTriangles + (workGroupSize-1)) / workGroupSize);
	cl_int err = m_queue.enqueueNDRangeKernel(updateSoftBodiesKernel.kernel, cl::NullRange, cl::NDRange(numWorkItems), cl::NDRange(workGroupSize));
	if( err != CL_SUCCESS ) 
	{
		btAssert(  "enqueueNDRangeKernel(normalizeNormalsAndAreasKernel)");
	}
}

void btOpenCLSoftBodySolver::updateSoftBodies()
{
	using namespace Vectormath::Aos;


	int numVertices = m_vertexData.getNumVertices();
	int numTriangles = m_triangleData.getNumTriangles();

	// Ensure data is on accelerator
	m_vertexData.moveToAccelerator();
	m_triangleData.moveToAccelerator();

	resetNormalsAndAreas( numVertices );


	// Go through triangle batches so updates occur correctly
	for( int batchIndex = 0; batchIndex < m_triangleData.m_batchStartLengths.size(); ++batchIndex )
	{

		int startTriangle = m_triangleData.m_batchStartLengths[batchIndex].first;
		int numTriangles = m_triangleData.m_batchStartLengths[batchIndex].second;

		executeUpdateSoftBodies( startTriangle, numTriangles );
	}


	normalizeNormalsAndAreas( numVertices );
} // updateSoftBodies


Vectormath::Aos::Vector3 btOpenCLSoftBodySolver::ProjectOnAxis( const Vectormath::Aos::Vector3 &v, const Vectormath::Aos::Vector3 &a )
{
	return a*Vectormath::Aos::dot(v, a);
}

void btOpenCLSoftBodySolver::ApplyClampedForce( float solverdt, const Vectormath::Aos::Vector3 &force, const Vectormath::Aos::Vector3 &vertexVelocity, float inverseMass, Vectormath::Aos::Vector3 &vertexForce )
{
	float dtInverseMass = solverdt*inverseMass;
	if( Vectormath::Aos::lengthSqr(force * dtInverseMass) > Vectormath::Aos::lengthSqr(vertexVelocity) )
	{
		vertexForce -= ProjectOnAxis( vertexVelocity, normalize( force ) )/dtInverseMass;
	} else {
		vertexForce += force;
	}
}

void btOpenCLSoftBodySolver::applyForces( float solverdt )
{	
	// Ensure data is on accelerator
	m_vertexData.moveToAccelerator();
	m_clPerClothAcceleration.moveToGPU();
	m_clPerClothLiftFactor.moveToGPU();
	m_clPerClothDragFactor.moveToGPU();
	m_clPerClothMediumDensity.moveToGPU();
	m_clPerClothWindVelocity.moveToGPU();			

	cl_int err;
	err = applyForcesKernel.kernel.setArg(0, m_vertexData.getNumVertices());
	if( err != CL_SUCCESS ) 
	{
		btAssert(  "enqueueNDRangeKernel(applyForcesKernel)");
	}
	err = applyForcesKernel.kernel.setArg(1, solverdt);
	if( err != CL_SUCCESS ) 
	{
		btAssert(  "enqueueNDRangeKernel(applyForcesKernel)");
	}
	err = applyForcesKernel.kernel.setArg(2, FLT_EPSILON);
	if( err != CL_SUCCESS ) 
	{
		btAssert(  "enqueueNDRangeKernel(applyForcesKernel)");
	}
	err = applyForcesKernel.kernel.setArg(3, m_vertexData.m_clClothIdentifier.getBuffer());
	if( err != CL_SUCCESS ) 
	{
		btAssert(  "enqueueNDRangeKernel(applyForcesKernel)");
	}
	err = applyForcesKernel.kernel.setArg(4, m_vertexData.m_clVertexNormal.getBuffer());
	if( err != CL_SUCCESS ) 
	{
		btAssert(  "enqueueNDRangeKernel(applyForcesKernel)");
	}
	err = applyForcesKernel.kernel.setArg(5, m_vertexData.m_clVertexArea.getBuffer());
	if( err != CL_SUCCESS ) 
	{
		btAssert(  "enqueueNDRangeKernel(applyForcesKernel)");
	}
	err = applyForcesKernel.kernel.setArg(6, m_vertexData.m_clVertexInverseMass.getBuffer());
	if( err != CL_SUCCESS ) 
	{
		btAssert(  "enqueueNDRangeKernel(applyForcesKernel)");
	}
	err = applyForcesKernel.kernel.setArg(7, m_clPerClothLiftFactor.getBuffer());
	if( err != CL_SUCCESS ) 
	{
		btAssert(  "enqueueNDRangeKernel(applyForcesKernel)");
	}
	err = applyForcesKernel.kernel.setArg(8, m_clPerClothDragFactor.getBuffer());
	if( err != CL_SUCCESS ) 
	{
		btAssert(  "enqueueNDRangeKernel(applyForcesKernel)");
	}
	err = applyForcesKernel.kernel.setArg(9, m_clPerClothWindVelocity.getBuffer());
	if( err != CL_SUCCESS ) 
	{
		btAssert(  "enqueueNDRangeKernel(applyForcesKernel)");
	}
	err = applyForcesKernel.kernel.setArg(10, m_clPerClothAcceleration.getBuffer());
	if( err != CL_SUCCESS )
	{
		btAssert(  "enqueueNDRangeKernel(applyForcesKernel)");
	}
	err = applyForcesKernel.kernel.setArg(11, m_clPerClothMediumDensity.getBuffer());
	if( err != CL_SUCCESS ) 
	{
		btAssert(  "enqueueNDRangeKernel(applyForcesKernel)");
	}
	err = applyForcesKernel.kernel.setArg(12, m_vertexData.m_clVertexForceAccumulator.getBuffer());
	if( err != CL_SUCCESS ) 
	{
		btAssert(  "enqueueNDRangeKernel(applyForcesKernel)");
	}
	err = applyForcesKernel.kernel.setArg(13, m_vertexData.m_clVertexVelocity.getBuffer());
	if( err != CL_SUCCESS ) 
	{
		btAssert(  "enqueueNDRangeKernel(applyForcesKernel)");
	}

	int	numWorkItems = workGroupSize*((m_vertexData.getNumVertices() + (workGroupSize-1)) / workGroupSize);

	err = m_queue.enqueueNDRangeKernel(applyForcesKernel.kernel, cl::NullRange, cl::NDRange(numWorkItems), cl::NDRange(workGroupSize));
	if( err != CL_SUCCESS ) 
	{
		btAssert(  "enqueueNDRangeKernel(applyForcesKernel)");
	}
}

/**
 * Integrate motion on the solver.
 */
void btOpenCLSoftBodySolver::integrate( float solverdt )
{
	// Ensure data is on accelerator
	m_vertexData.moveToAccelerator();

	integrateKernel.kernel.setArg(0, m_vertexData.getNumVertices());
	integrateKernel.kernel.setArg(1, solverdt);
	integrateKernel.kernel.setArg(2, m_vertexData.m_clVertexInverseMass.getBuffer());
	integrateKernel.kernel.setArg(3, m_vertexData.m_clVertexPosition.getBuffer());
	integrateKernel.kernel.setArg(4, m_vertexData.m_clVertexVelocity.getBuffer());
	integrateKernel.kernel.setArg(5, m_vertexData.m_clVertexPreviousPosition.getBuffer());
	integrateKernel.kernel.setArg(6, m_vertexData.m_clVertexForceAccumulator.getBuffer());

	int	numWorkItems = workGroupSize*((m_vertexData.getNumVertices() + (workGroupSize-1)) / workGroupSize);
	cl_int err = m_queue.enqueueNDRangeKernel(integrateKernel.kernel, cl::NullRange, cl::NDRange(numWorkItems), cl::NDRange(workGroupSize));
	if( err != CL_SUCCESS )
	{
		btAssert(  "enqueueNDRangeKernel(integrateKernel)");
	}

}

float btOpenCLSoftBodySolver::computeTriangleArea( 
	const Vectormath::Aos::Point3 &vertex0,
	const Vectormath::Aos::Point3 &vertex1,
	const Vectormath::Aos::Point3 &vertex2 )
{
	Vectormath::Aos::Vector3 a = vertex1 - vertex0;
	Vectormath::Aos::Vector3 b = vertex2 - vertex0;
	Vectormath::Aos::Vector3 crossProduct = cross(a, b);
	float area = length( crossProduct );
	return area;
}

void btOpenCLSoftBodySolver::updateConstants( float timeStep )
{			
	using namespace Vectormath::Aos;

	// TODO: Fix CL code. Something is wrong here.
#if 0
	if( m_updateSolverConstants )
	{
		// Ensure data is on accelerator
		m_vertexData.moveToAccelerator();
		m_linkData.moveToAccelerator();

		cl_int err;
		err = updateConstantsKernel.kernel.setArg(0, m_linkData.getNumLinks());
		err = updateConstantsKernel.kernel.setArg(1, m_linkData.m_clLinks.getBuffer());
		err = updateConstantsKernel.kernel.setArg(2, m_vertexData.m_clVertexPosition.getBuffer());
		err = updateConstantsKernel.kernel.setArg(3, m_vertexData.m_clVertexInverseMass.getBuffer());
		err = updateConstantsKernel.kernel.setArg(4, m_linkData.m_clLinksMaterialLinearStiffnessCoefficient.getBuffer());
		err = updateConstantsKernel.kernel.setArg(5, m_linkData.m_clLinksMassLSC.getBuffer());
		err = updateConstantsKernel.kernel.setArg(6, m_linkData.m_clLinksRestLengthSquared.getBuffer());
		err = updateConstantsKernel.kernel.setArg(7, m_linkData.m_clLinksRestLength.getBuffer());

		int	numWorkItems = workGroupSize*((m_linkData.getNumLinks() + (workGroupSize-1)) / workGroupSize);
		err = m_queue.enqueueNDRangeKernel(updateConstantsKernel.kernel, cl::NullRange, cl::NDRange(numWorkItems), cl::NDRange(workGroupSize));
		if( err != CL_SUCCESS ) 
		{
			btAssert(  "enqueueNDRangeKernel(integrate)");
		}
	}
#else
	using namespace Vectormath::Aos;

	if( m_updateSolverConstants )
	{
		m_vertexData.moveFromAccelerator();
		m_linkData.moveFromAccelerator();

		m_updateSolverConstants = false;

		// Will have to redo this if we change the structure (tear, maybe) or various other possible changes

		// Initialise link constants
		const int numLinks = m_linkData.getNumLinks();
		for( int linkIndex = 0; linkIndex < numLinks; ++linkIndex )
		{
			btSoftBodyLinkData::LinkNodePair &vertices( m_linkData.getVertexPair(linkIndex) );
			m_linkData.getRestLength(linkIndex) = length((m_vertexData.getPosition( vertices.vertex0 ) - m_vertexData.getPosition( vertices.vertex1 )));
			float invMass0 = m_vertexData.getInverseMass(vertices.vertex0);
			float invMass1 = m_vertexData.getInverseMass(vertices.vertex1);
			float linearStiffness = m_linkData.getLinearStiffnessCoefficient(linkIndex);
			float massLSC = (invMass0 + invMass1)/linearStiffness;
			m_linkData.getMassLSC(linkIndex) = massLSC;
			float restLength = m_linkData.getRestLength(linkIndex);
			float restLengthSquared = restLength*restLength;
			m_linkData.getRestLengthSquared(linkIndex) = restLengthSquared;
		}

		
		m_vertexData.moveToAccelerator();
		m_linkData.moveToAccelerator();
	}

#endif
}

void btOpenCLSoftBodySolver::solveConstraints( float solverdt )
{
	using Vectormath::Aos::Vector3;
	using Vectormath::Aos::Point3;
	using Vectormath::Aos::lengthSqr;
	using Vectormath::Aos::dot;

	// Prepare links
	int numLinks = m_linkData.getNumLinks();
	int numVertices = m_vertexData.getNumVertices();

	float kst = 1.f;
	float ti = 0.f;


	m_clPerClothDampingFactor.moveToGPU();
	m_clPerClothVelocityCorrectionCoefficient.moveToGPU();


	// Ensure data is on accelerator
	m_linkData.moveToAccelerator();
	m_vertexData.moveToAccelerator();

	prepareLinks();	



	// Prepare anchors
	/*for(i=0,ni=m_anchors.size();i<ni;++i)
	{
		Anchor&			a=m_anchors[i];
		const btVector3	ra=a.m_body->getWorldTransform().getBasis()*a.m_local;
		a.m_c0	=	ImpulseMatrix(	m_sst.sdt,
			a.m_node->m_im,
			a.m_body->getInvMass(),
			a.m_body->getInvInertiaTensorWorld(),
			ra);
		a.m_c1	=	ra;
		a.m_c2	=	m_sst.sdt*a.m_node->m_im;
		a.m_body->activate();
	}*/

	// Really want to combine these into a single loop, don't we? No update in the middle?

	// TODO: Double check what kst is meant to mean - passed in as 1 in the bullet code


	// Solve drift
	for( int iteration = 0; iteration < m_numberOfPositionIterations ; ++iteration )
	{
		for( int i = 0; i < m_linkData.m_batchStartLengths.size(); ++i )
		{
			int startLink = m_linkData.m_batchStartLengths[i].first;
			int numLinks = m_linkData.m_batchStartLengths[i].second;

			solveLinksForPosition( startLink, numLinks, kst, ti );
		}
		
	} // for( int iteration = 0; iteration < m_numberOfPositionIterations ; ++iteration )


	updateVelocitiesFromPositionsWithoutVelocities( 1.f/solverdt );
}


//////////////////////////////////////
// Kernel dispatches
void btOpenCLSoftBodySolver::prepareLinks()
{
	prepareLinksKernel.kernel.setArg(0, m_linkData.getNumLinks());
	prepareLinksKernel.kernel.setArg(1, m_linkData.m_clLinks.getBuffer());
	prepareLinksKernel.kernel.setArg(2, m_linkData.m_clLinksMassLSC.getBuffer());
	prepareLinksKernel.kernel.setArg(3, m_vertexData.m_clVertexPreviousPosition.getBuffer());
	prepareLinksKernel.kernel.setArg(4, m_linkData.m_clLinksLengthRatio.getBuffer());
	prepareLinksKernel.kernel.setArg(5, m_linkData.m_clLinksCLength.getBuffer());

	int	numWorkItems = workGroupSize*((m_linkData.getNumLinks() + (workGroupSize-1)) / workGroupSize);
	cl_int err = m_queue.enqueueNDRangeKernel(prepareLinksKernel.kernel, cl::NullRange, cl::NDRange(numWorkItems), cl::NDRange(workGroupSize));
	if( err != CL_SUCCESS ) 
	{
		btAssert(  "enqueueNDRangeKernel(prepareLinksKernel)");
	}
}

void btOpenCLSoftBodySolver::updatePositionsFromVelocities( float solverdt )
{
	updatePositionsFromVelocitiesKernel.kernel.setArg(0, m_vertexData.getNumVertices());
	updatePositionsFromVelocitiesKernel.kernel.setArg(1, solverdt);
	updatePositionsFromVelocitiesKernel.kernel.setArg(2, m_vertexData.m_clVertexVelocity.getBuffer());
	updatePositionsFromVelocitiesKernel.kernel.setArg(3, m_vertexData.m_clVertexPreviousPosition.getBuffer());
	updatePositionsFromVelocitiesKernel.kernel.setArg(4, m_vertexData.m_clVertexPosition.getBuffer());

	int	numWorkItems = workGroupSize*((m_vertexData.getNumVertices() + (workGroupSize-1)) / workGroupSize);
	cl_int err = m_queue.enqueueNDRangeKernel(updatePositionsFromVelocitiesKernel.kernel, cl::NullRange, cl::NDRange(numWorkItems), cl::NDRange(workGroupSize));
	if( err != CL_SUCCESS ) 
	{
		btAssert(  "enqueueNDRangeKernel(updatePositionsFromVelocitiesKernel)");
	}
}

void btOpenCLSoftBodySolver::solveLinksForPosition( int startLink, int numLinks, float kst, float ti )
{
	solvePositionsFromLinksKernel.kernel.setArg(0, startLink);
	solvePositionsFromLinksKernel.kernel.setArg(1, numLinks);
	solvePositionsFromLinksKernel.kernel.setArg(2, kst);
	solvePositionsFromLinksKernel.kernel.setArg(3, ti);
	solvePositionsFromLinksKernel.kernel.setArg(4, m_linkData.m_clLinks.getBuffer());
	solvePositionsFromLinksKernel.kernel.setArg(5, m_linkData.m_clLinksMassLSC.getBuffer());
	solvePositionsFromLinksKernel.kernel.setArg(6, m_linkData.m_clLinksRestLengthSquared.getBuffer());
	solvePositionsFromLinksKernel.kernel.setArg(7, m_vertexData.m_clVertexInverseMass.getBuffer());
	solvePositionsFromLinksKernel.kernel.setArg(8, m_vertexData.m_clVertexPosition.getBuffer());

	int	numWorkItems = workGroupSize*((numLinks + (workGroupSize-1)) / workGroupSize);
	cl_int err = m_queue.enqueueNDRangeKernel(solvePositionsFromLinksKernel.kernel, cl::NullRange, cl::NDRange(numWorkItems), cl::NDRange(workGroupSize));
	if( err != CL_SUCCESS ) 
	{
		btAssert(  "enqueueNDRangeKernel(solvePositionsFromLinksKernel)");
	}
} // solveLinksForPosition


void btOpenCLSoftBodySolver::updateVelocitiesFromPositionsWithVelocities( float isolverdt )
{
	updateVelocitiesFromPositionsWithVelocitiesKernel.kernel.setArg(0, m_vertexData.getNumVertices());
	updateVelocitiesFromPositionsWithVelocitiesKernel.kernel.setArg(1, isolverdt);
	updateVelocitiesFromPositionsWithVelocitiesKernel.kernel.setArg(2, m_vertexData.m_clVertexPosition.getBuffer());
	updateVelocitiesFromPositionsWithVelocitiesKernel.kernel.setArg(3, m_vertexData.m_clVertexPreviousPosition.getBuffer());
	updateVelocitiesFromPositionsWithVelocitiesKernel.kernel.setArg(4, m_vertexData.m_clClothIdentifier.getBuffer());
	updateVelocitiesFromPositionsWithVelocitiesKernel.kernel.setArg(5, m_clPerClothVelocityCorrectionCoefficient.getBuffer());
	updateVelocitiesFromPositionsWithVelocitiesKernel.kernel.setArg(6, m_clPerClothDampingFactor.getBuffer());
	updateVelocitiesFromPositionsWithVelocitiesKernel.kernel.setArg(7, m_vertexData.m_clVertexVelocity.getBuffer());
	updateVelocitiesFromPositionsWithVelocitiesKernel.kernel.setArg(8, m_vertexData.m_clVertexForceAccumulator.getBuffer());

	int	numWorkItems = workGroupSize*((m_vertexData.getNumVertices() + (workGroupSize-1)) / workGroupSize);
	cl_int err = m_queue.enqueueNDRangeKernel(updateVelocitiesFromPositionsWithVelocitiesKernel.kernel, cl::NullRange, cl::NDRange(numWorkItems), cl::NDRange(workGroupSize));
	if( err != CL_SUCCESS ) 
	{
		btAssert(  "enqueueNDRangeKernel(updateVelocitiesFromPositionsWithVelocitiesKernel)");
	}

} // updateVelocitiesFromPositionsWithVelocities

void btOpenCLSoftBodySolver::updateVelocitiesFromPositionsWithoutVelocities( float isolverdt )
{
	updateVelocitiesFromPositionsWithoutVelocitiesKernel.kernel.setArg(0, m_vertexData.getNumVertices());
	updateVelocitiesFromPositionsWithoutVelocitiesKernel.kernel.setArg(1, isolverdt);
	updateVelocitiesFromPositionsWithoutVelocitiesKernel.kernel.setArg(2, m_vertexData.m_clVertexPosition.getBuffer());
	updateVelocitiesFromPositionsWithoutVelocitiesKernel.kernel.setArg(3, m_vertexData.m_clVertexPreviousPosition.getBuffer());
	updateVelocitiesFromPositionsWithoutVelocitiesKernel.kernel.setArg(4, m_vertexData.m_clClothIdentifier.getBuffer());
	updateVelocitiesFromPositionsWithoutVelocitiesKernel.kernel.setArg(5, m_clPerClothDampingFactor.getBuffer());
	updateVelocitiesFromPositionsWithoutVelocitiesKernel.kernel.setArg(6, m_vertexData.m_clVertexVelocity.getBuffer());
	updateVelocitiesFromPositionsWithoutVelocitiesKernel.kernel.setArg(7, m_vertexData.m_clVertexForceAccumulator.getBuffer());

	int	numWorkItems = workGroupSize*((m_vertexData.getNumVertices() + (workGroupSize-1)) / workGroupSize);
	cl_int err = m_queue.enqueueNDRangeKernel(updateVelocitiesFromPositionsWithoutVelocitiesKernel.kernel, cl::NullRange, cl::NDRange(numWorkItems), cl::NDRange(workGroupSize));
	if( err != CL_SUCCESS ) 
	{
		btAssert(  "enqueueNDRangeKernel(updateVelocitiesFromPositionsWithoutVelocitiesKernel)");
	}
} // updateVelocitiesFromPositionsWithoutVelocities

// End kernel dispatches
/////////////////////////////////////



void btOpenCLSoftBodySolver::outputToVertexBuffers()
{
	// If we are going to do a CPU output for any of the cloths ensure that we copy
	// the positions and normals back
	bool needCopyBack = false;
	for( int clothIndex = 0; clothIndex < m_cloths.size(); ++clothIndex )
	{
		btAcceleratedSoftBodyInterface *currentCloth = m_cloths[clothIndex];

		if( btVertexBufferDescriptor *vertexBufferTarget = currentCloth->getVertexBufferTarget() )
		{
			if( vertexBufferTarget->getBufferType() == btVertexBufferDescriptor::CPU_BUFFER )
			{
				needCopyBack = true;
			}
		}
	}
	if( needCopyBack )
	{
		m_vertexData.m_clVertexPosition.copyFromGPU();
		m_vertexData.m_clVertexNormal.copyFromGPU();
	}

	// Currently only support CPU output buffers
	// TODO: check for DX11 buffers. Take all offsets into the same DX11 buffer
	// and use them together on a single kernel call if possible by setting up a
	// per-cloth target buffer array for the copy kernel.
	for( int clothIndex = 0; clothIndex < m_cloths.size(); ++clothIndex )
	{
		btAcceleratedSoftBodyInterface *currentCloth = m_cloths[clothIndex];

		const int firstVertex = currentCloth->getFirstVertex();
		const int lastVertex = firstVertex + currentCloth->getNumVertices();
		if( btVertexBufferDescriptor *vertexBufferTarget = currentCloth->getVertexBufferTarget() )
		{
			if( vertexBufferTarget->getBufferType() == btVertexBufferDescriptor::CPU_BUFFER )
			{
				const btCPUVertexBufferDescriptor *cpuVertexBuffer = static_cast< btCPUVertexBufferDescriptor* >(vertexBufferTarget);						
				float *basePointer = cpuVertexBuffer->getBasePointer();						

				if( vertexBufferTarget->hasVertexPositions() )
				{
					const int vertexOffset = cpuVertexBuffer->getVertexOffset();
					const int vertexStride = cpuVertexBuffer->getVertexStride();
					float *vertexPointer = basePointer + vertexOffset;

					for( int vertexIndex = firstVertex; vertexIndex < lastVertex; ++vertexIndex )
					{
						Vectormath::Aos::Point3 position = m_vertexData.getPosition(vertexIndex);
						*(vertexPointer + 0) = position.getX();
						*(vertexPointer + 1) = position.getY();
						*(vertexPointer + 2) = position.getZ();
						vertexPointer += vertexStride;
					}
				}
				if( vertexBufferTarget->hasNormals() )
				{
					const int normalOffset = cpuVertexBuffer->getNormalOffset();
					const int normalStride = cpuVertexBuffer->getNormalStride();
					float *normalPointer = basePointer + normalOffset;

					for( int vertexIndex = firstVertex; vertexIndex < lastVertex; ++vertexIndex )
					{
						Vectormath::Aos::Vector3 normal = m_vertexData.getNormal(vertexIndex);
						*(normalPointer + 0) = normal.getX();
						*(normalPointer + 1) = normal.getY();
						*(normalPointer + 2) = normal.getZ();
						normalPointer += normalStride;
					}
				}
			}
		} // if( btVertexBufferDescriptor *vertexBufferTarget = currentCloth->getVertexBufferTarget() )
	} // for( int clothIndex = 0; clothIndex < m_cloths.size(); ++clothIndex )
} // outputToVertexBuffers



btOpenCLSoftBodySolver::KernelDesc btOpenCLSoftBodySolver::compileCLKernelFromString( const char *shaderString, const char *shaderName )
{
	cl_int err;

	context = m_queue.getInfo<CL_QUEUE_CONTEXT>();
	device = m_queue.getInfo<CL_QUEUE_DEVICE>();
	std::vector< cl::Device > devices;
	devices.push_back( device );

	cl::Program::Sources source(1, std::make_pair(shaderString, strlen(shaderString) + 1));
	cl::Program program(context, source, &err);
	if( err != CL_SUCCESS ) 
	{
		btAssert( "program" );
	}
	err = program.build(devices);
	if (err != CL_SUCCESS) {
		//std::string str;
		//str = program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(devices[0]);
		//std::cout << "Program Info: " << str;
		if( err != CL_SUCCESS ) 
		{
			btAssert( "Program::build()" );
		}
	}
	cl::Kernel kernel(program, shaderName, &err);
	if( err != CL_SUCCESS ) 
	{
		btAssert( "kernel" );
	}

	KernelDesc descriptor;
	descriptor.kernel = kernel;
	return descriptor;
}

bool btOpenCLSoftBodySolver::buildShaders()
{
	bool returnVal = true;

	if( m_shadersInitialized )
		return true;
	
	prepareLinksKernel = compileCLKernelFromString( PrepareLinksCLString, "PrepareLinksKernel" );
	updatePositionsFromVelocitiesKernel = compileCLKernelFromString( UpdatePositionsFromVelocitiesCLString, "UpdatePositionsFromVelocitiesKernel" );
	solvePositionsFromLinksKernel = compileCLKernelFromString( SolvePositionsCLString, "SolvePositionsFromLinksKernel" );
	updateVelocitiesFromPositionsWithVelocitiesKernel = compileCLKernelFromString( UpdateNodesCLString, "updateVelocitiesFromPositionsWithVelocitiesKernel" );
	updateVelocitiesFromPositionsWithoutVelocitiesKernel = compileCLKernelFromString( UpdatePositionsCLString, "updateVelocitiesFromPositionsWithoutVelocitiesKernel" );
	updateConstantsKernel = compileCLKernelFromString( UpdateConstantsCLString, "UpdateConstantsKernel" );
	integrateKernel = compileCLKernelFromString( IntegrateCLString, "IntegrateKernel" );
	applyForcesKernel = compileCLKernelFromString( ApplyForcesCLString, "ApplyForcesKernel" );

	// TODO: Rename to UpdateSoftBodies
	resetNormalsAndAreasKernel = compileCLKernelFromString( UpdateNormalsCLString, "ResetNormalsAndAreasKernel" );
	normalizeNormalsAndAreasKernel = compileCLKernelFromString( UpdateNormalsCLString, "NormalizeNormalsAndAreasKernel" );
	updateSoftBodiesKernel = compileCLKernelFromString( UpdateNormalsCLString, "UpdateSoftBodiesKernel" );
	//outputToVertexArrayWithNormalsKernel = compileCLKernelFromString( OutputToVertexArrayCLString, "OutputToVertexArrayWithNormalsKernel" );
	//outputToVertexArrayWithoutNormalsKernel = compileCLKernelFromString( OutputToVertexArrayCLString, "OutputToVertexArrayWithoutNormalsKernel" );


	if( returnVal )
		m_shadersInitialized = true;

	return returnVal;
}

/** Return the btAcceleratedSoftBodyInterface object represented by softBodyIndex */
btAcceleratedSoftBodyInterface *btOpenCLSoftBodySolver::getSoftBody( int softBodyIndex )
{
	return m_cloths[softBodyIndex];
}


void btOpenCLSoftBodySolver::addCollisionObjectForSoftBody( int clothIdentifier, btCollisionObject *collisionObject )
{
	btAssert("Cannot use collision objects with OpenCL solver.");
}

#endif // #ifdef BULLET_USE_OPENCL