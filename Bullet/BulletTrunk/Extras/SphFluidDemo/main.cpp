
#include "project_defs.h"

#include "BulletFluids/btFluidWorld.h"
#include "BulletFluids/btSphFluid.h"

#include <map>
#include <math.h>

#if defined(GL_PIPELINE)
#include "gl_render.h"
#endif

#if defined(DX_PIPELINE)
#include "dx10_render.h"
#endif

#define INITIAL_PARTICLE_COUNT	MAX_PARTICLES //( 32 * 1024 )
#define MAX_PARTICLES			( 128 * 1024 )
#define NEIGHBOR_COUNT			32

// a water-like fluid
#define MU						10.0f//Pa.s (Pascal second)
#define STIFFNESS				0.75f//J
#define RHO0					1000.0f//kg/m^3

#define SIMULATION_SCALE		0.004f
#define	CFL_LIMIT				100.0f
#define BOUNDARY_REFLECTANCE	0.75f


// the boundaries of the domain
#define XMAX 100.0f
#define YMAX 40.0f 
#define ZMAX 40.0f

#define EPSILON 0.1f

#define XMININIT ( EPSILON )
#define XMAXINIT ( (XMAX) / 10 - EPSILON )
#define YMININIT ( EPSILON )
#define YMAXINIT ( YMAX - EPSILON )
#define ZMININIT ( EPSILON )
#define ZMAXINIT ( ZMAX - EPSILON )

static btFluidWorld * world = NULL;
static btfVector3 * initialPositions = NULL;
static btfVector3 * initialVelocities = NULL;
static btParticleData * initialParticles;
static btParticleData * writeParticles;
static btFluidDescriptor * fluidDescriptor = NULL;
static btSphFluid * fluid = NULL;
static unsigned int particleCount = INITIAL_PARTICLE_COUNT;


btSphFluid *
initializeSimulation( btFluidWorld * world )
{
	// create initial particles with random position and zero velocity
	if( initialPositions == NULL ){
		initialPositions = new btfVector3[ INITIAL_PARTICLE_COUNT ];
	}
	if( initialVelocities == NULL ){
		initialVelocities = new btfVector3[ INITIAL_PARTICLE_COUNT ];
	}
	srand( 0 );//use a consistent seed for reproducibility

	for( int i = 0; i < INITIAL_PARTICLE_COUNT; ++i ){
		float x, y, z;
		float r;
		r = ( (float)rand() / (float)RAND_MAX );
#define SCALE( MIN, MAX, X ) ( (MIN) + (X) * ( (MAX) - (MIN) ) )
		x = SCALE( XMININIT, XMAXINIT, r );
		r = ( (float)rand() / (float)RAND_MAX );
		y = SCALE( YMININIT, YMAXINIT, r );
		r = ( (float)rand() / (float)RAND_MAX );
		z = SCALE( ZMININIT, ZMAXINIT, r );
		initialPositions[ i ] = btfVector3( x, y, z );
		initialVelocities[ i ] = btfVector3( 0.0f, 0.0f, 0.0f );
	}//for

	initialParticles = new btParticleData( "initial particles" );
	initialParticles->m_numParticles = &particleCount;
	initialParticles->m_position = initialPositions;
	initialParticles->m_positionStorageClass = BTF_NATIVE;
	initialParticles->m_positionStride = 4 * sizeof( float );
	initialParticles->m_positionOffset = 0;
	initialParticles->m_velocity = initialVelocities;
	initialParticles->m_velocityStorageClass = BTF_NATIVE;
	initialParticles->m_velocityStride = 4 * sizeof( float );
	initialParticles->m_velocityOffset = 0;

	writeParticles = new btParticleData( "write particles" );
	writeParticles->m_numParticles = &particleCount;
	writeParticles->m_position = new btfVector3[ INITIAL_PARTICLE_COUNT ];
	writeParticles->m_positionStorageClass = BTF_NATIVE;
	writeParticles->m_positionStride = 4 * sizeof( float );
	writeParticles->m_positionOffset = 0;

	fluidDescriptor = new btFluidDescriptor();
	fluidDescriptor->m_initialParticleData = initialParticles;
	fluidDescriptor->m_particlesWriteData = writeParticles;
	fluidDescriptor->m_viscosity = MU;
	fluidDescriptor->m_stiffness = STIFFNESS;
	fluidDescriptor->m_restDensity = RHO0;
	fluidDescriptor->m_simulationVolume = btfVector3( XMAX, YMAX, ZMAX );
	fluidDescriptor->m_neighborCount = NEIGHBOR_COUNT;

#ifdef TEST_PIPELINE
	fluidDescriptor->m_simulationType = BTF_SPH_TEST;
#elif defined( CAPTURE_PIPELINE )
	fluidDescriptor->m_simulationType = BTF_SPH_CAPTURE;
#else
	fluidDescriptor->m_simulationType = BTF_SPH_SIMULATION;
#endif

	world->setGravity( btfVector3( 0.0f, -9.8f, 0.0f ) );//y up
	fluid = (btSphFluid *)world->createFluid( fluidDescriptor );	
	return fluid;
}


#ifndef DX_PIPELINE

void clearCollisionGeometry( btFluidWorld * world ){
	btfTriangleList triangles;
	world->specifyCollisionTriangleList( triangles );
}

#define XMIN 0
#define YMIN 0
#define ZMIN 0

void loadCollisionGeometry( btFluidWorld * world ){
	btfTriangleList triangles;
	btTriangle tri;
	// front 
	tri.m_vertex0 = btVector3( XMIN, YMIN, ZMIN );
	tri.m_vertex1 = btVector3( XMAX, YMIN, ZMIN );
	tri.m_vertex2 = btVector3( XMAX, YMAX, ZMIN );
	triangles.push_back( tri );
	tri.m_vertex0 = btVector3( XMIN, YMIN, ZMIN );
	tri.m_vertex1 = btVector3( XMIN, YMAX, ZMIN );
	tri.m_vertex2 = btVector3( XMAX, YMAX, ZMIN );
	triangles.push_back( tri );
	// back 
	tri.m_vertex0 = btVector3( XMIN, YMIN, ZMAX );
	tri.m_vertex1 = btVector3( XMAX, YMIN, ZMAX );
	tri.m_vertex2 = btVector3( XMAX, YMAX, ZMAX );
	triangles.push_back( tri );
	tri.m_vertex0 = btVector3( XMIN, YMIN, ZMAX );
	tri.m_vertex1 = btVector3( XMIN, YMAX, ZMAX );
	tri.m_vertex2 = btVector3( XMAX, YMAX, ZMAX );
	triangles.push_back( tri );
	// top
	tri.m_vertex0 = btVector3( XMIN, YMAX, ZMIN );
	tri.m_vertex1 = btVector3( XMAX, YMAX, ZMAX );
	tri.m_vertex2 = btVector3( XMAX, YMAX, ZMIN );
	triangles.push_back( tri );
	tri.m_vertex0 = btVector3( XMIN, YMAX, ZMIN );
	tri.m_vertex1 = btVector3( XMAX, YMAX, ZMAX );
	tri.m_vertex2 = btVector3( XMIN, YMAX, ZMAX );
	triangles.push_back( tri );
	// bottom
	tri.m_vertex0 = btVector3( XMIN, YMIN, ZMIN );
	tri.m_vertex1 = btVector3( XMAX, YMIN, ZMAX );
	tri.m_vertex2 = btVector3( XMAX, YMIN, ZMIN );
	triangles.push_back( tri );
	tri.m_vertex0 = btVector3( XMIN, YMIN, ZMIN );
	tri.m_vertex1 = btVector3( XMAX, YMIN, ZMAX );
	tri.m_vertex2 = btVector3( XMIN, YMIN, ZMAX );
	triangles.push_back( tri );
	// left
	tri.m_vertex0 = btVector3( XMIN, YMIN, ZMIN );
	tri.m_vertex1 = btVector3( XMIN, YMAX, ZMAX );
	tri.m_vertex2 = btVector3( XMIN, YMIN, ZMAX );
	triangles.push_back( tri );
	tri.m_vertex0 = btVector3( XMIN, YMIN, ZMIN );
	tri.m_vertex1 = btVector3( XMIN, YMAX, ZMAX );
	tri.m_vertex2 = btVector3( XMIN, YMAX, ZMIN );
	triangles.push_back( tri );
	// right
	tri.m_vertex0 = btVector3( XMAX, YMIN, ZMIN );
	tri.m_vertex1 = btVector3( XMAX, YMAX, ZMAX );
	tri.m_vertex2 = btVector3( XMAX, YMIN, ZMAX );
	triangles.push_back( tri );
	tri.m_vertex0 = btVector3( XMAX, YMIN, ZMIN );
	tri.m_vertex1 = btVector3( XMAX, YMAX, ZMAX );
	tri.m_vertex2 = btVector3( XMAX, YMAX, ZMIN );
	triangles.push_back( tri );
	world->specifyCollisionTriangleList( triangles );
}


#endif

int main ( int argc, char **argv )
{
#if defined(GL_PIPELINE)
	preInitGL( argc, argv );
#elif defined(DX_PIPELINE)
	preInitDX10( MAX_PARTICLES );
#endif

	world = new btFluidWorld( "Bullet SPH demo 1", SIMULATION_SCALE, BOUNDARY_REFLECTANCE );
	btSphFluid * fluid = initializeSimulation( world );

#if defined(GL_PIPELINE)
	goGL( fluid, world );
#elif defined(DX_PIPELINE)
	goDX10( fluid, world );
#else
	// if no graphics, run the compute pipeline

	std::cout << "Timing for simulation of " << particleCount << " particles" << std::endl;
	for( int i = 0; i < 1000; ++i ){

		if( i == 0 ){
			loadCollisionGeometry( world );
		}
		else if( i == FRAME_TO_REMOVE_BOX ){
			clearCollisionGeometry( world );
		}

#ifdef _WIN32
		__int64 freq, tStart, tStop;
		unsigned long TimeDiff;
		// Get the frequency of the hi-res timer
		QueryPerformanceFrequency((LARGE_INTEGER*)&freq);
		// Assuming that has worked you can then use hi-res timer
		QueryPerformanceCounter((LARGE_INTEGER*)&tStart);
#endif
		std::cout << "Frame #" << i << std::endl;
		world->step();
#ifdef _WIN32
		// Perform operations that require timing
		QueryPerformanceCounter((LARGE_INTEGER*)&tStop);
		// Calculate time difference
		TimeDiff = (unsigned long)(((tStop - tStart) * 1000000) / freq);
		std::cout << "Elapsed time frame " << i << " is " << TimeDiff << " microseconds" << std::endl;
#endif
	}//for
#endif
}
