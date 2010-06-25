#include <GL/glew.h>

#if defined(GL_PIPELINE)

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <iterator>
#include <math.h>


#ifndef _WIN32
#include <GL/glx.h>
#endif //!_WIN32

#if defined(__APPLE__) || defined(__MACOSX)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

// vbo variables
static GLuint vbo = 0;

#ifdef _WIN32
#include <windows.h>
#endif



#define GL_PIPELINE_POINTS 1 //visualize positions as points
//#define RENDER_VELOCITY // visualize velocity vectors

#include "gl_render.h"

static unsigned int windowWidth  = 1280;
static unsigned int windowHeight = 1024;


#if !defined( HEADLESS_DISPLAY )

#include <cmath>

#include <iostream>
#include <cstring>

// Fonts
void *font = GLUT_BITMAP_8_BY_13;
void *fonts[] = {
	GLUT_BITMAP_9_BY_15,
	GLUT_BITMAP_TIMES_ROMAN_10,
	GLUT_BITMAP_TIMES_ROMAN_24
};

static Sph sph;


// mouse controls
int mouseOldX;
int mouseOldY;
int mouseButtons         = 0;

float rotateX;
float rotateY;

float translateZ;
float translateX;
float translateY;

static bool stepSim              = false;
static bool runSimCon            = false;
static bool displayHelp          = false;
static bool displayCube          = false;

static float _positions[ PARTICLE_COUNT * 4 ];
static float _velocities[ PARTICLE_COUNT * 4 ];

#define LIGHT_NEAR		0.5f
#define LIGHT_FAR		300.0f
#define DEGtoRAD		( 3.141592f / 180.0f )

static GLuint glSphere = 65535;
static float  glRadius = 0.0f;


static GLuint glProgram;

#define STRINGIFY(A) #A

const char * vertexShader = STRINGIFY(
void main()
{
    gl_Position    = ftransform();
}
);

const char * pixelShader = STRINGIFY(
void main()
{
  vec3 color         = vec3(1.0, 0.0, 0.0);
  gl_FragColor       = vec4(color, 1.0);
}
);

/*
const char * gemometryShader = STRINGIFY(
layout(points) in;
layout(points, max_vertices = 10) out;
void main() 
{
	int i;
	for(i=0; i < gl_in.length(); i++){
		gl_Position = gl_in[i].gl_Position;
		EmitVertex();
	}
	EndPrimitive();	 
}
);


layout(points) in;
layout(triangle_strip, max_vertices = 4) out;

void main() 
{
    float3 g_positions[4] = { vec3( -1.0, 1.0, 0.0 ), vec3( 1.0, 1.0, 0.0 ), vec3( -1.0, -1.0, 0.0 ), vec3( 1.0, -1.0, 0.0 ) };
	float g_fParticleRad = 1.0;
	int i;
	for(i=0; i < 4; i++){
		float3 position = g_positions[i]*g_gParticleRad;
		position = position * gl_ModelViewMatrix;
		float3 particlePosition = (vec3(0.0,0.0,0.0) * gl_ModelViewMatrix) + gl_in[0].gl_Position;
		EmitVertex();
	}
	EndPrimitive();	 
}
*/


GLuint
compileProgram(
	const char * vsrc, 
	const char * psrc,
	const char * gsrc)
{
    GLint err = 0;

    GLuint vertexShader    = glCreateShader(GL_VERTEX_SHADER);
	GLuint geometryShader  = glCreateShader(GL_GEOMETRY_SHADER);
    GLuint pixelShader     = glCreateShader(GL_FRAGMENT_SHADER);

    glShaderSource(vertexShader, 1, (const GLchar**)&vsrc, 0);
    glShaderSource(pixelShader, 1, (const GLchar**)&psrc, 0);
    glShaderSource(geometryShader, 1, (const GLchar**)&gsrc, 0);

    glCompileShader(vertexShader);

    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &err);

      if (!err) {
          char temp[256];
           glGetShaderInfoLog(vertexShader, 256, 0, temp);
           std::cout << "Failed to compile shader: " << temp << std::endl;
      }

    glCompileShader(geometryShader);

    glGetShaderiv(geometryShader, GL_COMPILE_STATUS, &err);

     if (!err) {
         char temp[256];
          glGetShaderInfoLog(geometryShader, 256, 0, temp);
          std::cout << "Failed to compile shader: " << temp << std::endl;
     }

    glCompileShader(pixelShader);

    glGetShaderiv(pixelShader, GL_COMPILE_STATUS, &err);

     if (!err) {
         char temp[256];
          glGetShaderInfoLog(pixelShader, 256, 0, temp);
          std::cout << "Failed to compile shader: " << temp << std::endl;
     }

    GLuint program = glCreateProgram();

    glAttachShader(program, vertexShader);
 //   glAttachShader(program, geometryShader);
    glAttachShader(program, pixelShader);

    glLinkProgram(program);

    // check if program linked
    err = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &err);

    if (!err) {
        char temp[256];
         glGetProgramInfoLog(program, 256, 0, temp);
         std::cout << "Failed to link program: " << temp << std::endl;
         glDeleteProgram(program);
         program = 0;
    }

    return program;
}

void setSphereRadius ( float r )
{
  if ( glRadius != r ) {
    glRadius = r;
    
    // GL sphere
    if ( glSphere != 65535 ) {
      glDeleteLists ( glSphere, 1 );
    }
    glSphere = glGenLists ( 1 );
    float x, y, z, x1, y1, z1;	
    glNewList ( glSphere, GL_COMPILE );
    glBegin ( GL_TRIANGLE_STRIP );
    for ( float tilt = -90.0f; tilt <= 90.0f; tilt += 10.0f ) {
	  for ( float ang = 0.0f; ang <= 360.0f; ang += 30.0f ) {
		x = (float)sin( ang * DEGtoRAD ) * cos( tilt * DEGtoRAD );
		y = (float)cos( ang * DEGtoRAD ) * cos( tilt * DEGtoRAD );
		z = (float)sin( tilt * DEGtoRAD ) ;
		x1 = (float)sin( ang * DEGtoRAD ) * cos(( tilt + 10.0f ) * DEGtoRAD ) ;
		y1 = (float)cos( ang * DEGtoRAD ) * cos(( tilt + 10.0f ) * DEGtoRAD ) ;
		z1 = (float)sin(( tilt + 10.0 ) * DEGtoRAD );
		glNormal3f ( x, y, z );		
		glVertex3f ( x * r, y * r, z * r );		
		glNormal3f ( x1, y1, z1 );	
		glVertex3f ( x1 * r, y1 * r, z1 * r );
	  }
    }
    glEnd ();
    glEndList ();
  }
}

inline void drawSphere (float * position)
{
  glPushMatrix();
  
  glTranslatef( position[ 0 ], position[ 1 ], position[ 2 ] );
  glScalef( 0.01f, 0.01f, 0.01f );
  
  setSphereRadius( 1.0 );
  glCallList( glSphere );

  glPopMatrix();
}

inline void drawVector( float * position, float * vector ){
	float v[ 3 ];
#if K==8
#define SCALE 10.0f
#elif K==16
#define SCALE 1.0f
#elif K>=32
#define SCALE 0.1f
#endif
#define SCALE 2.0f
	float lambda = vector[ 3 ] * SCALE;
	v[ 0 ] = vector[ 0 ] * lambda;
	v[ 1 ] = vector[ 1 ] * lambda;
	v[ 2 ] = vector[ 2 ] * lambda;
	glVertex3f( position[ 0 ], position[ 1 ], position[ 2 ] );
	glVertex3f( position[ 0 ] + v[ 0 ],
		position[ 1 ] + v[ 1 ],
		position[ 2 ] + v[ 2 ]
	);
}



void
drawText(GLint x, GLint y, char * s, GLfloat r, GLfloat g, GLfloat b)
{
    int lines;
    char* p;

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    glOrtho(
		0.0, 
		windowWidth,
	    windowHeight, 
		0.0, 
		-1.0, 
		1.0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glColor3f(r,g,b);
    glRasterPos2i(x, y);

	for(p = s, lines = 0; *p; p++) {
		if (*p == '\n') {
			lines++;
			glRasterPos2i(x, y-(lines*18));
		}
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *p);
    }

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

void displayHelpText(void)
{
	glDisable ( GL_LIGHTING );  
	glDisable ( GL_DEPTH_TEST );

	drawText ( 10, 20, (char *)"Press H for help.", 1.0f, 1.0f, 1.0f );  

	if (displayHelp) {
	  drawText (20, 60, (char *)"Keyboard", 1.0f, 1.0f, 1.0f);

		char * s = displayCube ? 
		  (char *)" c   Toggle Cube  off" 
		  : (char *)" c   Toggle Cube  on";
		drawText ( 
			20, 
			80,  
			s,
			1.0f, 
			1.0f, 
			1.0f );
		
		drawText ( 
			20, 
			100, 
			(char *)" l    Step Simulation", 
			1.0f, 
			1.0f, 
			1.0f );

		char * ss = runSimCon ? 
		  (char *)" r    Toggle Simulation off" 
		  : (char *)" r    Toggle Simulation on";
		drawText ( 
			20, 
			120, 
			ss,
			1.0f, 
			1.0f, 
			1.0f );

		drawText (20, 140, (char *)" f    Zoom in", 1.0f, 1.0f, 1.0f);

		drawText (20, 160, (char *)" g   Zoom out", 1.0f, 1.0f, 1.0f);

		drawText (20, 180, (char *)" q   Quit", 1.0f, 1.0f, 1.0f);
	}
}

void drawBoundingBox()
{
		glBegin(GL_QUADS);
			glVertex3f( XMAX, YMIN, ZMAX );			// Top Right Of The Quad (Bottom)
			glVertex3f( XMIN, YMIN, ZMAX );			// Top Left Of The Quad (Bottom)
			glVertex3f( XMIN, YMIN, ZMIN );			// Bottom Left Of The Quad (Bottom)
			glVertex3f( XMAX, YMIN, ZMIN );			// Bottom Right Of The Quad (Bottom)

			glVertex3f( XMAX, YMAX, ZMAX );			// Top Right Of The Quad (Top)
			glVertex3f( XMIN, YMAX, ZMAX );			// Top Left Of The Quad (Top)
			glVertex3f( XMIN, YMAX, ZMIN );			// Bottom Left Of The Quad (Top)
			glVertex3f( XMAX, YMAX, ZMIN );			// Bottom Right Of The Quad (Top)

			glVertex3f( XMAX, YMAX, ZMAX );			// Top Right Of The Quad (Front)
			glVertex3f( XMIN, YMAX, ZMAX );			// Top Left Of The Quad (Front)
			glVertex3f( XMIN, YMIN, ZMAX );			// Bottom Left Of The Quad (Front)
			glVertex3f( XMAX, YMIN, ZMAX );			// Bottom Right Of The Quad (Front)

			glVertex3f( XMIN, YMIN, ZMIN );			// Bottom Left Of The Quad (Back)
			glVertex3f( XMAX, YMIN, ZMIN );			// Bottom Right Of The Quad (Back)
			glVertex3f( XMAX, YMAX, ZMIN );			// Top Right Of The Quad (Back)
			glVertex3f( XMIN, YMAX, ZMIN );			// Top Left Of The Quad (Back)

			glVertex3f( XMIN, YMAX, ZMAX );			// Top Right Of The Quad (Left)
			glVertex3f( XMIN, YMAX, ZMIN );			// Top Left Of The Quad (Left)
			glVertex3f( XMIN, YMIN, ZMIN );			// Bottom Left Of The Quad (Left)
			glVertex3f( XMIN, YMIN, ZMAX );			// Bottom Right Of The Quad (Left)

			glVertex3f( XMAX, YMAX, ZMIN );			// Top Right Of The Quad (Right)
			glVertex3f( XMAX, YMAX, ZMAX );			// Top Left Of The Quad (Right)
			glVertex3f( XMAX, YMIN, ZMAX );			// Bottom Left Of The Quad (Right)
			glVertex3f( XMAX, YMIN, ZMIN );			// Bottom Right Of The Quad (Right)
		glEnd();
}


void colorFromLambda( float lambda ){
	lambda = fabs( lambda );
	float b;
	if( lambda < 0.5 ) b = 1.0f - lambda * 2.0f;
	else b = 0.0f;
	float g;
	g = 1.0f - ( fabs( lambda - 0.5f ) ) * 2.0f;
	float r;
	if( lambda > 0.5 ) r = lambda - 0.5 * 2.0;
	else r = 0.0f;

	glColor4f( r, g, b, 1.0f );
}


void render( Sph * sph )
{
//#define RENDER_FROM_VBO
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	glDisable ( GL_CULL_FACE );

	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	glPushMatrix();

	glTranslatef( translateX, translateY, translateZ );
	glRotatef( rotateX, 0.5f, 0.0f, 0.0f );
	glRotatef( rotateY, 0.0f, 0.5f, 0.0f );

#ifndef RENDER_FROM_VBO
	std::vector<cl::Memory> vec;
#if defined(GL_PIPELINE)
	vec.push_back( sph->position );
	sph->queue.enqueueAcquireGLObjects(&vec);
#elif defined (DX_PIPELINE)
	vec.push_back( sph->position );
	sph->queue.enqueueAcquireD3D10Objects(&vec);
#endif
#endif//RENDER_FROM_VBO

#if defined(RENDER_VELOCITY)
	err = sph->queue.enqueueReadBuffer( 
		sph->velocity, 
		CL_TRUE, 
		0, 
		PARTICLE_COUNT * 4 * sizeof( float ), 
		_velocities );
	if ( err != CL_SUCCESS ) {
		std::cerr << "Failed to read velocity" << std::endl;
		exit( 1 );
	}
#endif
	glDisable (GL_BLEND);

#if defined(GL_PIPELINE_POINTS)

//#define RENDER_GRADP
#define RENDER_PRESSURE
//#define RENDER_DENSITY
//#define RENDER_SURFACE


#ifdef RENDER_GRADP
#if !defined( DEBUG_DATA )
#error RENDER_GRADP requires DEBUG_DATA defined in sphHelpers.h
#endif
#endif

#ifdef RENDER_FROM_VBO
   // render from the vbo
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glVertexPointer(4, GL_FLOAT, 0, NULL);

    glUseProgram(glProgram);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_POINTS, 0, PARTICLE_COUNT * 4);
    glDisableClientState(GL_VERTEX_ARRAY);

    glBindBuffer(GL_ARRAY_BUFFER, 0);

#else

	float * position_ = new float[ PARTICLE_COUNT * 4 ];
	if( position_ != NULL ){
		sph->queue.enqueueReadBuffer( sph->position, CL_TRUE, 0, PARTICLE_COUNT * 4 * sizeof( float ), position_ );

#if defined( DEBUG_DATA ) && defined( RENDER_GRADP )
		// color points according to the magnitude of the pressure gradient
		int debugDataSize = PARTICLE_COUNT * 4;
		float * _debugData = new float[ debugDataSize ];
		sph->queue.enqueueReadBuffer( sph->debugData, CL_TRUE, 0, debugDataSize * sizeof( float ), _debugData );
#elif defined( RENDER_PRESSURE )
		// color points according to pressure
		float *_pressure = new float[ PARTICLE_COUNT ];
		sph->queue.enqueueReadBuffer( sph->pressure, CL_TRUE, 0, PARTICLE_COUNT * sizeof( float ), _pressure );
#elif defined( RENDER_DENSITY )
		// color points according to rho
		float *_rho = new float[ PARTICLE_COUNT ];
		sph->queue.enqueueReadBuffer( sph->rho, CL_TRUE, 0, PARTICLE_COUNT * sizeof( float ), _rho );
#elif defined( RENDER_SURFACE )
		// surface normal vector
		float * _normal = new float[ PARTICLE_COUNT * 4 ];
		sph->queue.enqueueReadBuffer( sph->normal, CL_TRUE, 0, PARTICLE_COUNT * 4 * sizeof( float ), _normal );
#endif

#if defined(GL_PIPELINE)
		sph->queue.enqueueReleaseGLObjects(&vec);
		sph->queue.finish();
#elif defined (DX_PIPELINE)
		sph->queue.enqueueReleaseD3D10Objects(&vec);
		sph->queue.finish();
#else
		sph->queue.finish();
#endif//remove


#if 0//capture data for debug
		{
			std::cout << "Capturing render.position.txt" << std::endl;
			FILE * f = fopen( "render.position.txt", "w" );
			for( int i = 0; i < PARTICLE_COUNT; ++i ){
				float * p = position_ + 4 * i;
				fprintf( f, "%g\t%g\t%g\t%g\n",
					p[ 0 ], p[ 1 ], p[ 2 ], p[ 3 ] );
			}//for
			fclose( f );
		}
#endif

#ifdef RENDER_SURFACE
		glBegin( GL_LINES );
#else
		glBegin ( GL_POINTS );
#endif

		for( int i = 0; i < PARTICLE_COUNT; i++ ){
			int offset = i * 4;
#if defined( DEBUG_DATA ) && defined( RENDER_GRADP )
			float * gradP = _debugData + offset;
			float length = sqrt( gradP[ 0 ] * gradP[ 0 ] + gradP[ 1 ] * gradP[ 1 ] + gradP[ 2 ] * gradP[ 2 ] );
			length *= 0.00001;
			colorFromLambda( length );
#elif defined( RENDER_PRESSURE )
			float length = _pressure[ i ] / 200.0f;
			colorFromLambda( length );
#elif defined( RENDER_DENSITY )
			float rho = _rho[ i ] * 0.001;
			colorFromLambda( rho );
#endif

#ifdef RENDER_SURFACE
			float * n = _normal + offset;
			float lambda = n[ 3 ];
#define THRESHOLD 0.8f
			if( lambda > THRESHOLD ){
				colorFromLambda( lambda );
				float * p = position_ + offset;
#if 0
				glVertex3f( p[ 0 ], p[ 1 ], p[ 2 ] );
#elif 0//crashes
				drawSphere( p );
#else
				drawVector( p, n );
#endif
			}
#else
			float * p = position_ + offset;
			glVertex3f( p[ 0 ], p[ 1 ], p[ 2 ] );
#endif
		}//for
#if defined( DEBUG_DATA ) && defined( RENDER_GRADP )
		delete [] _debugData;
#elif defined( RENDER_PRESSURE )
		delete [] _pressure;
#elif defined( RENDER_DENSITY )
		delete [] _rho;
#elif defined( RENDER_SURFACE )
		delete [] _normal;
#endif
		glEnd();

		delete [] position_;
	}else{
#if defined(GL_PIPELINE)
		sph->queue.enqueueReleaseGLObjects(&vec);
		sph->queue.finish();
#elif defined (DX_PIPELINE)
		sph->queue.enqueueReleaseD3D10Objects(&vec);
		sph->queue.finish();
#endif
	}
#endif
#elif defined(RENDER_VELOCITY)
	glLineWidth( 0.05f );
	glBegin( GL_LINES );
	for( int i = 0; i < PARTICLE_COUNT; ++i ){
		float * p = _positions + 4 * i;
		float * v = _velocities + 4 * i;
		float length = sqrt( v[ 0 ] * v[ 0 ] + v[ 1 ] * v[ 1 ] + v[ 2 ] * v[ 2 ] );
		float vel[ 3 ];
		vel[ 0 ] = v[ 0 ] / length;q
		vel[ 1 ] = v[ 1 ] / length;
		vel[ 2 ] = v[ 2 ] / length;
		colorFromLambda( length );
		glVertex3f( p[ 0 ], p[ 1 ], p[ 2 ] );
		glVertex3f( p[ 0 ] + vel[ 0 ], p[ 1 ] + vel[ 1 ], p[ 2 ] + vel[ 2 ] );
	}//for
	glEnd();
#else
   GLfloat mat_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
   GLfloat mat_shininess[] = { 50.0f };
 
   glEnable ( GL_COLOR_MATERIAL );
   glShadeModel( GL_SMOOTH );
   glEnable( GL_LINE_SMOOTH );

   glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular );
   glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess );

   glEnable( GL_LIGHTING );
   glEnable( GL_LIGHT0 );

	// Now draw the particles
	glEnable( GL_LIGHTING );
	for( int i = 0; i < PARTICLE_COUNT * 4; i +=4 ){
		glColor3f( 1.0f ,1.0f ,0.0f );
		drawSphere( &_positions[ i ] );
	}
#endif

	if (displayCube) {
		glDisable( GL_LIGHTING );

		glColor4f( 0.0f, 0.5f, 0.0f, 0.1f );

		glEnable (GL_BLEND);
		glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		drawBoundingBox();

		glEnable( GL_LIGHTING );
		glEnable( GL_LIGHT0 );
		glEnable ( GL_NORMALIZE );
		glDepthFunc(GL_EQUAL);

#if 1
		glColor4f( 1.0f, 1.0f, 1.0f, 0.1f);
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glColor4f( 1.0f, 1.0f, 1.0f, 0.1f );
		glColorMaterial(GL_FRONT_AND_BACK, GL_SPECULAR);
#endif


#if	1
		//GLfloat black[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
		GLfloat black[ 4 ] = { 0.0f, 0.0f, 0.0f, 1.0f };
		glLightfv(GL_LIGHT0, GL_DIFFUSE, black);
		glLightfv(GL_LIGHT0, GL_SPECULAR, black);
#endif

		drawBoundingBox();

	}
	glPopMatrix();

	displayHelpText();
}

static void initGL(void) 
{
   GLfloat mat_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
   GLfloat mat_shininess[] = { 50.0f };
   GLfloat light_position[] = { 
	   XMIN + 0.5f * XRANGE, 
	   YMIN + 0.5f * YRANGE, 
	   ZMIN + 2.0f * ZRANGE, 
	   1.0f };

   glClearColor( 0.05f, 0.0f, 0.1f, 0.1f );
   glEnable ( GL_COLOR_MATERIAL );
   glShadeModel( GL_SMOOTH );
   glEnable( GL_LINE_SMOOTH );

   glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular );
   glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess );
   glLightfv( GL_LIGHT0, GL_POSITION, light_position );

   glEnable( GL_LIGHTING );
   glEnable( GL_LIGHT0 );
   glEnable( GL_DEPTH_TEST );

   rotateX = rotateY = 0;
   translateX = 0.0f;
   translateY = YMIN + 0.5f * YRANGE;
   translateZ = -2.5f * ZMAX;

   std::ifstream fileGEM("gemometry.glsl");
   if (!fileGEM.is_open()) {
	   std::cout << "Failed to open gemometry.glsl" << std::endl;
	   exit(EXIT_FAILURE);
   }

   std::string progGEM(
        std::istreambuf_iterator<char>(fileGEM),
        (std::istreambuf_iterator<char>()));

   glProgram = compileProgram(vertexShader, pixelShader, progGEM.data());
   if (!glProgram) {
		exit(EXIT_FAILURE);
   }
}


void display(void)
{
	if (stepSim || runSimCon) {
		step( sph );
		stepSim = false;
	}

	render( sph );

	glutSwapBuffers();
	glutPostRedisplay();

}

void keyboard( unsigned char key, int /*x*/, int /*y*/)
{
  switch( key) {
  case('q') :
#ifdef _WIN32
  case VK_ESCAPE:
#endif //_WIN32
	  sph.terminate();
    exit( 0);
  break;
  case('a'):
    translateY += 0.1f;
    break;
  case('z'):
    translateY -= 0.1f;
    break;
  case('d'):
    translateX += 0.1f;
    break;
  case('s'):
    translateX -= 0.1f;
    break;
  case('f'):
    translateZ += 0.1f;
    break;
  case('g'):
    translateZ -= 0.1f;
    break;
  case('l'):
    stepSim = true;
	break;
  case('r'):
    runSimCon = !runSimCon;
	break;
  case('c'):
	  displayCube = !displayCube;
    break;
  case('h'):
	  displayHelp = !displayHelp;
    break;
  }
}

void mouse(int button, int state, int x, int y)
{
  if (state == GLUT_DOWN) {
    mouseButtons |= 1<<button;
  } else if (state == GLUT_UP) {
    mouseButtons = 0;
  }

  mouseOldX = x;
  mouseOldY = y;
  glutPostRedisplay();
}

void motion(int x, int y)
{
  float dx, dy;
  dx = x - mouseOldX;
  dy = y - mouseOldY;
  
  if (mouseButtons & 1) {
    rotateX += dy * 0.2;
    rotateY += dx * 0.2;
  } 
  else if (mouseButtons & 5) {
    translateY -= dy * 0.01;
    translateX -= dx * 0.01;
  }
  else if (mouseButtons & 4) {
    translateZ += dy * 0.01;
  } 

  mouseOldX = x;
  mouseOldY = y;
}


void reshape (int w, int h)
{
  windowWidth  = w;
  windowHeight = h;
  glViewport( 0, 0, windowWidth, windowHeight);

  glMatrixMode( GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(
		 60.0,
		 (GLfloat)windowWidth / (GLfloat) windowHeight,
		 0.1,
		 ZRANGE * 6.0f );
}

void goGL(Sph & sph_)
{
	sph = sph_;

	glutMainLoop();
}

#endif//!defined( HEADLESS_DISPLAY )


void preInitGL(int argc, char ** argv)
{
	glutInit( &argc, argv );

	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH );
	glutInitWindowSize( windowWidth, windowHeight );    
	glutCreateWindow ("SPH 2");

    glewInit();
    if (!glewIsSupported( "GL_VERSION_2_0 " )) {
          std::cerr
              << "Support for necessary OpenGL extensions missing."
              << std::endl;
          exit(EXIT_FAILURE);
    }

#if !defined( HEADLESS_DISPLAY )

	initGL();

	glViewport( 0, 0, windowWidth, windowHeight);

	reshape( windowWidth, windowHeight );

	glutDisplayFunc(display); 
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
#endif
}


//moved from above
int getVBO( std::string )
{
	if (vbo == 0) {
		// Create VBO
		// create buffer object
		glGenBuffers(1, &vbo);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, PARTICLE_COUNT * (sizeof(float)*4), 0, GL_DYNAMIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	return vbo;
}

#endif